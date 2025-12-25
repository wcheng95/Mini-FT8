#include "stream_uac.h"
#include "resample.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "usb/usb_host.h"
#include "usb/uac_host.h"

extern "C" {
#include "ft8/decode.h"
#include "ft8/constants.h"
#include "common/monitor.h"
}

#include "ui.h"
#include <cstring>
#include <cmath>

static const char* TAG = "UAC_STREAM";

// ============================================================================
// TEST TONE MODE: Replace UAC streaming with synthetic 1.5kHz tone for debugging
// Set to 1 to enable test tone, 0 for normal UAC streaming
// ============================================================================
#define USE_TEST_TONE 1

#if USE_TEST_TONE
// Test tone parameters
#define TEST_TONE_FREQ_HZ   1000    // 1 kHz test tone
#define TEST_TONE_PERIOD    48      // 48000 / 1000 = 48 samples per cycle
#define TEST_TONE_DB        (-50.0f)  // Tone amplitude in dB (0 dB = full scale)
#define TEST_NOISE_DB       (-80.0f)  // White noise floor in dB
#define TEST_TONE_AMPLITUDE (powf(10.0f, TEST_TONE_DB / 20.0f))  // -50dB ≈ 0.00316
#define TEST_NOISE_AMPLITUDE (powf(10.0f, TEST_NOISE_DB / 20.0f))  // -80dB ≈ 0.0001

// Pre-computed 1.5kHz tone buffer (32 samples as float, before noise addition)
static float s_test_tone_samples[TEST_TONE_PERIOD];
static bool s_test_tone_initialized = false;

// Fast xorshift32 PRNG for white noise generation
static uint32_t s_noise_state = 0x12345678;

static inline uint32_t xorshift32(void) {
    uint32_t x = s_noise_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_noise_state = x;
    return x;
}

// Generate white noise sample in range [-1, 1]
static inline float white_noise(void) {
    // Convert to float in range [-1, 1]
    return ((float)(int32_t)xorshift32()) / 2147483648.0f;
}

static void init_test_tone_buffer(void) {
    if (s_test_tone_initialized) return;

    // Pre-compute normalized tone samples [-1, 1]
    for (int i = 0; i < TEST_TONE_PERIOD; i++) {
        float phase = (2.0f * M_PI * i) / TEST_TONE_PERIOD;
        s_test_tone_samples[i] = sinf(phase);
    }

    // Seed noise generator with esp_random for better randomness
    s_noise_state = esp_random();

    s_test_tone_initialized = true;
    ESP_LOGI(TAG, "Test tone initialized: %d Hz, tone=%.0fdB, noise=%.0fdB, SNR=%.0fdB",
             TEST_TONE_FREQ_HZ, TEST_TONE_DB, TEST_NOISE_DB, TEST_TONE_DB - TEST_NOISE_DB);
}

// Fill buffer with test tone + white noise samples
static void fill_test_tone_samples(uint8_t* out, int num_stereo_samples, int* phase_idx) {
    const float tone_scale = TEST_TONE_AMPLITUDE * 8388607.0f;
    const float noise_scale = TEST_NOISE_AMPLITUDE * 8388607.0f;

    for (int i = 0; i < num_stereo_samples; i++) {
        // Get tone sample from circular buffer
        float tone = s_test_tone_samples[*phase_idx % TEST_TONE_PERIOD] * tone_scale;
        // Add white noise
        float noise = white_noise() * noise_scale;
        float sample = tone + noise;

        // Clamp to 24-bit range
        if (sample > 8388607.0f) sample = 8388607.0f;
        if (sample < -8388608.0f) sample = -8388608.0f;
        int32_t sample_int = (int32_t)sample;

        // Pack as 24-bit LE stereo (same value for L and R)
        int dst_offset = i * 6;
        out[dst_offset + 0] = (sample_int >>  0) & 0xFF;
        out[dst_offset + 1] = (sample_int >>  8) & 0xFF;
        out[dst_offset + 2] = (sample_int >> 16) & 0xFF;
        out[dst_offset + 3] = (sample_int >>  0) & 0xFF;
        out[dst_offset + 4] = (sample_int >>  8) & 0xFF;
        out[dst_offset + 5] = (sample_int >> 16) & 0xFF;

        (*phase_idx)++;
    }
}
#endif // USE_TEST_TONE

// External references from main.cpp
extern bool g_streaming;
void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui);
int64_t rtc_now_ms();

#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 12000
#endif

// Task priorities and stack sizes
#define USB_HOST_TASK_PRIORITY  5
#define UAC_TASK_PRIORITY       5
#define UAC_STREAM_TASK_PRIORITY 4
#define TASK_STACK_SIZE         4096
#define STREAM_TASK_STACK_SIZE  8192

// UAC read buffer size (bytes)
#define UAC_READ_BUFFER_SIZE    4096

// Event types for internal queue
typedef enum {
    UAC_EVT_DRIVER,
    UAC_EVT_DEVICE,
    UAC_EVT_STOP,
} uac_event_type_t;

typedef struct {
    uac_event_type_t type;
    union {
        struct {
            uint8_t addr;
            uint8_t iface_num;
            uac_host_driver_event_t event;
        } driver;
        struct {
            uac_host_device_handle_t handle;
            uac_host_device_event_t event;
        } device;
    };
} uac_event_t;

// Global state
static uac_stream_state_t s_state = UAC_STATE_IDLE;
static QueueHandle_t s_event_queue = NULL;
static uac_host_device_handle_t s_mic_handle = NULL;
static TaskHandle_t s_usb_task_handle = NULL;
static TaskHandle_t s_uac_task_handle = NULL;
static TaskHandle_t s_stream_task_handle = NULL;
static volatile bool s_stop_requested = false;
static char s_status_string[64] = "Idle";

// Resampler state
static resample_state_t s_resample_state;
static void log_heap_stats(const char* where) {
    size_t free_def = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    size_t largest_def = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    size_t free_int = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t largest_int = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "HEAP[%s] free=%u largest=%u int_free=%u int_largest=%u",
             where, (unsigned)free_def, (unsigned)largest_def,
             (unsigned)free_int, (unsigned)largest_int);
}

// Forward declarations
static void usb_lib_task(void* arg);
static void uac_lib_task(void* arg);
static void stream_uac_task(void* arg);

// Push waterfall row (same as stream_wav.cpp)
static void push_waterfall_latest(const monitor_t& mon) {
    if (mon.wf.num_blocks <= 0 || mon.wf.mag == nullptr) return;
    const int block = mon.wf.num_blocks - 1;
    const int num_bins = mon.wf.num_bins;
    const int freq_osr = mon.wf.freq_osr;
    const uint8_t* base = mon.wf.mag + block * mon.wf.block_stride;

    static uint8_t collapsed[480];  // max num_bins
    memset(collapsed, 0, num_bins);
    for (int b = 0; b < num_bins; ++b) {
        uint8_t v = 0;
        for (int fs = 0; fs < freq_osr; ++fs) {
            uint8_t val = base[fs * num_bins + b];
            if (val > v) v = val;
        }
        collapsed[b] = v;
    }

    constexpr int width = 240;
    static uint8_t scaled[width];
    for (int x = 0; x < width; ++x) {
        int start = (int)((int64_t)x * num_bins / width);
        int end = (int)((int64_t)(x + 1) * num_bins / width);
        if (end <= start) end = start + 1;
        uint8_t maxv = 0;
        for (int s = start; s < end && s < num_bins; ++s) {
            if (collapsed[s] > maxv) maxv = collapsed[s];
        }
        scaled[x] = maxv;
    }

    ui_push_waterfall_row(scaled, width);

    // Debug: log waterfall max occasionally to verify signal presence
    static int wf_log = 0;
    if ((wf_log++ % 40) == 0) { // roughly every ~6s
        uint8_t max_scaled = 0;
        for (int i = 0; i < width; ++i) {
            if (scaled[i] > max_scaled) max_scaled = scaled[i];
        }
        ESP_LOGI(TAG, "Waterfall row max=%u num_bins=%d freq_osr=%d", max_scaled, num_bins, freq_osr);
    }
}

// UAC device callback
static void uac_device_callback(uac_host_device_handle_t handle,
                                 const uac_host_device_event_t event,
                                 void* arg) {
    if (event == UAC_HOST_DRIVER_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "UAC device disconnected");
        if (handle == s_mic_handle) {
            s_mic_handle = NULL;
            s_state = UAC_STATE_WAITING;
            snprintf(s_status_string, sizeof(s_status_string), "Disconnected");
        }
        uac_host_device_close(handle);
        return;
    }

    uac_event_t evt = {
        .type = UAC_EVT_DEVICE,
        .device = {
            .handle = handle,
            .event = event
        }
    };
    xQueueSend(s_event_queue, &evt, 0);
}

// UAC driver callback
static void uac_driver_callback(uint8_t addr, uint8_t iface_num,
                                 const uac_host_driver_event_t event,
                                 void* arg) {
    ESP_LOGI(TAG, "UAC drv evt addr:%d iface:%d evt:%d", addr, iface_num, event);

    uac_event_t evt = {
        .type = UAC_EVT_DRIVER,
        .driver = {
            .addr = addr,
            .iface_num = iface_num,
            .event = event
        }
    };
    xQueueSend(s_event_queue, &evt, 0);
}

// USB host library task
static void usb_lib_task(void* arg) {
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };

    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB host: %s", esp_err_to_name(err));
        s_state = UAC_STATE_ERROR;
        snprintf(s_status_string, sizeof(s_status_string), "USB init failed");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "USB Host installed");
    xTaskNotifyGive((TaskHandle_t)arg);

    while (!s_stop_requested) {
        uint32_t event_flags;
        err = usb_host_lib_handle_events(pdMS_TO_TICKS(100), &event_flags);
        if (err == ESP_OK) {
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
                ESP_LOGI(TAG, "No USB clients");
                usb_host_device_free_all();
            }
        }
    }

    ESP_LOGI(TAG, "USB Host uninstalling");
    usb_host_uninstall();
    s_usb_task_handle = NULL;
    vTaskDelete(NULL);
}

// UAC class driver task
static void uac_lib_task(void* arg) {
    // Wait for USB host to be ready
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uac_host_driver_config_t uac_config = {
        .create_background_task = true,
        .task_priority = UAC_TASK_PRIORITY,
        .stack_size = 4096,
        .core_id = 0,
        .callback = uac_driver_callback,
        .callback_arg = NULL
    };

    esp_err_t err = uac_host_install(&uac_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UAC driver: %s", esp_err_to_name(err));
        s_state = UAC_STATE_ERROR;
        snprintf(s_status_string, sizeof(s_status_string), "UAC init failed");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UAC driver installed");
    s_state = UAC_STATE_WAITING;
    snprintf(s_status_string, sizeof(s_status_string), "Waiting for device");

    uac_event_t evt;
    while (!s_stop_requested) {
        if (xQueueReceive(s_event_queue, &evt, pdMS_TO_TICKS(100))) {
            if (evt.type == UAC_EVT_STOP) {
                break;
            } else if (evt.type == UAC_EVT_DRIVER) {
                if (evt.driver.event == UAC_HOST_DRIVER_EVENT_RX_CONNECTED) {
                    ESP_LOGI(TAG, "Microphone connected - addr:%d, iface:%d",
                             evt.driver.addr, evt.driver.iface_num);

                    if (s_mic_handle != NULL) {
                        ESP_LOGW(TAG, "Already have a mic device, ignoring");
                        continue;
                    }

                    uac_host_device_config_t dev_config = {
                        .addr = evt.driver.addr,
                        .iface_num = evt.driver.iface_num,
                        .buffer_size = UAC_BUFFER_SIZE,
                        .buffer_threshold = UAC_BUFFER_THRESHOLD,
                        .callback = uac_device_callback,
                        .callback_arg = NULL
                    };

                    uac_host_device_handle_t handle = NULL;
                    err = uac_host_device_open(&dev_config, &handle);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to open device: %s", esp_err_to_name(err));
                        snprintf(s_status_string, sizeof(s_status_string), "Open failed");
                        continue;
                    }

                    // Print device info and alts
                    uac_host_dev_info_t dev_info = {};
                    if (uac_host_get_device_info(handle, &dev_info) == ESP_OK) {
                        ESP_LOGI(TAG, "UAC dev addr:%d type:%d iface:%d alts:%d VID:0x%04X PID:0x%04X",
                                 dev_info.addr, dev_info.type, dev_info.iface_num,
                                 dev_info.iface_alt_num, dev_info.VID, dev_info.PID);
                        // Alt settings are numbered from 1..iface_alt_num; alt 0 is typically zero-bandwidth.
                        for (int alt = 1; alt <= dev_info.iface_alt_num; ++alt) {
                            uac_host_dev_alt_param_t altp = {};
                            if (uac_host_get_device_alt_param(handle, alt, &altp) == ESP_OK) {
                                ESP_LOGI(TAG, "  Alt[%d]: fmt=%u ch=%u bits=%u freq_type=%u f0=%u f1=%u",
                                         alt, altp.format, altp.channels, altp.bit_resolution,
                                         altp.sample_freq_type, altp.sample_freq[0], altp.sample_freq[1]);
                            } else {
                                ESP_LOGW(TAG, "  Alt[%d]: get_param failed", alt);
                            }
                        }
                    } else {
                        ESP_LOGW(TAG, "uac_host_get_device_info failed");
                    }

                    // Try to start with required format: 24-bit/48kHz/stereo
                    uac_host_stream_config_t stm_config = {
                        .channels = UAC_CHANNELS,
                        .bit_resolution = UAC_BIT_RESOLUTION,
                        .sample_freq = UAC_SAMPLE_RATE,
                        .flags = 0
                    };

                    ESP_LOGI(TAG, "Starting stream: %dHz, %d-bit, %dch",
                             stm_config.sample_freq, stm_config.bit_resolution, stm_config.channels);

                    err = uac_host_device_start(handle, &stm_config);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to start stream: %s", esp_err_to_name(err));
                        snprintf(s_status_string, sizeof(s_status_string),
                                 "Format not supported");
                        uac_host_device_close(handle);
                        continue;
                    }

                    s_mic_handle = handle;
                    s_state = UAC_STATE_STREAMING;
                    g_streaming = true;
                    snprintf(s_status_string, sizeof(s_status_string),
                             "Streaming 48k/24/2");

                    // Start the audio processing task
                    if (s_stream_task_handle == NULL) {
                        xTaskCreatePinnedToCore(stream_uac_task, "stream_uac",
                                                STREAM_TASK_STACK_SIZE, NULL,
                                                UAC_STREAM_TASK_PRIORITY,
                                                &s_stream_task_handle, 1);
                    }

                } else if (evt.driver.event == UAC_HOST_DRIVER_EVENT_TX_CONNECTED) {
                    ESP_LOGI(TAG, "Speaker connected (ignored)");
                }
            }
        }
    }

    // Cleanup
    if (s_mic_handle) {
        uac_host_device_stop(s_mic_handle);
        uac_host_device_close(s_mic_handle);
        s_mic_handle = NULL;
    }

    ESP_LOGI(TAG, "UAC driver uninstalling");
    uac_host_uninstall();
    s_uac_task_handle = NULL;
    vTaskDelete(NULL);
}

// Audio streaming and processing task
static void stream_uac_task(void* arg) {
    ESP_LOGI(TAG, "Audio streaming task started");

    // Initialize resampler
    resample_init(&s_resample_state);
    log_heap_stats("uac_start");

    // Initialize FT8 monitor (waterfall/decoder)
    monitor_config_t mon_cfg = {
        .f_min = 200.0f,
        .f_max = 3000.0f,
        .sample_rate = FT8_SAMPLE_RATE,
        .time_osr = 1,
        .freq_osr = 2,  // finer bins; relies on freed BLE RAM
        .protocol = FTX_PROTOCOL_FT8
    };

    monitor_t mon;
    monitor_init(&mon, &mon_cfg);
    monitor_reset(&mon);
    log_heap_stats("after_monitor_init");

    const int target_blocks = 81; // full FT8 slot (160 ms * 81 ≈ 12.96 s)
    const int block_samples = mon.block_size; // 160 ms @ 12 kHz

    // Allocate buffers
    uint8_t* usb_buffer = (uint8_t*)heap_caps_malloc(UAC_READ_BUFFER_SIZE, MALLOC_CAP_DEFAULT);
    float* ft8_buffer = (float*)heap_caps_malloc(sizeof(float) * block_samples, MALLOC_CAP_DEFAULT);
    // Intermediate buffer for 48kHz mono samples (max: 4096 bytes / 6 = 682 stereo samples)
    float* temp_12k = (float*)heap_caps_malloc(sizeof(float) * 1024, MALLOC_CAP_DEFAULT);
    log_heap_stats("uac_buffers_allocated");

    if (!usb_buffer || !ft8_buffer || !temp_12k) {
        ESP_LOGE(TAG, "Buffer allocation failed");
        if (usb_buffer) free(usb_buffer);
        if (ft8_buffer) free(ft8_buffer);
        if (temp_12k) free(temp_12k);
        monitor_free(&mon);
        s_stream_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    int ft8_buffer_idx = 0;  // Current position in ft8_buffer
    TickType_t next_wake = xTaskGetTickCount();
    int slot_blocks = 0;
    int64_t slot_start_ms = 0;

    // Align start to the current 15-second boundary using RTC
    {
        int64_t now_ms = rtc_now_ms();
        int64_t rem = now_ms % 15000;
        int64_t wait_ms = (rem < 100) ? 0 : (15000 - rem);
        ESP_LOGI(TAG, "Aligning to slot: now=%lldms rem=%lldms wait=%lldms", (long long)now_ms, (long long)rem, (long long)wait_ms);
        if (wait_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS((uint32_t)wait_ms));
            now_ms = rtc_now_ms();
        }
        slot_start_ms = now_ms - (now_ms % 15000);
    }

    while (!s_stop_requested && s_mic_handle != NULL) {
        // Read USB audio data
        uint32_t bytes_read = 0;
        esp_err_t ret = uac_host_device_read(s_mic_handle, usb_buffer,
                                              UAC_READ_BUFFER_SIZE,
                                              &bytes_read,
                                              pdMS_TO_TICKS(200));

        if (ret != ESP_OK || bytes_read == 0) {
            if (ret != ESP_ERR_TIMEOUT && ret != ESP_FAIL) {
                ESP_LOGW(TAG, "USB read error: %s", esp_err_to_name(ret));
            }
            continue;
        }

        // Convert USB audio to FT8 format
        // 24-bit stereo = 6 bytes per sample pair
        int num_stereo_samples = bytes_read / 6;
        if (num_stereo_samples == 0) continue;

        // Convert and resample: 24-bit/48kHz/stereo -> 12kHz mono float
        int samples_12k = uac_to_ft8_samples(&s_resample_state, usb_buffer,
                                              temp_12k, num_stereo_samples);

        // Accumulate into ft8_buffer
        for (int i = 0; i < samples_12k && !s_stop_requested; i++) {
            ft8_buffer[ft8_buffer_idx++] = temp_12k[i];

            // When we have a full block (1920 samples = 160ms)
            if (ft8_buffer_idx >= block_samples) {
                // Apply gain normalization and log RMS (block-level AGC)
                double acc = 0.0;
                for (int j = 0; j < block_samples; ++j) {
                    acc += fabsf(ft8_buffer[j]);
                }
                float level = (float)(acc / block_samples);
                float gain = (level > 1e-6f) ? 0.5f / level : 1.0f; // target ~0.5 avg abs
                if (gain < 0.1f) gain = 0.1f;
                if (gain > 30.0f) gain = 30.0f;
                for (int j = 0; j < block_samples; ++j) {
                    ft8_buffer[j] *= gain;
                }
                static int rms_log_count = 0;
                if ((rms_log_count++ % 30) == 0) { // every ~5s
                    ESP_LOGI(TAG, "Block RMS avg=%.5f gain=%.2f post=%.3f", level, gain, level * gain);
                }

                if (mon.wf.num_blocks < target_blocks) {
                    monitor_process(&mon, ft8_buffer);
                    push_waterfall_latest(mon);
                }

                ft8_buffer_idx = 0;

                // Keep pacing close to real-time
                vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(160));

                // Track slot progression and decode at 79 blocks; resync to RTC every 15s.
                slot_blocks++;
                int64_t now_ms = rtc_now_ms();
                if (slot_blocks >= 79) {
                    ESP_LOGI(TAG, "Triggering decode slot=%lld blocks=%d", (long long)(slot_start_ms / 15000), slot_blocks);
                    decode_monitor_results(&mon, &mon_cfg, false);
                    monitor_reset(&mon);
                    mon.wf.num_blocks = 0;
                    slot_blocks = 0;
                    slot_start_ms = now_ms - (now_ms % 15000);
                    next_wake = xTaskGetTickCount();
                } else if (now_ms - slot_start_ms >= 15000) {
                    // Slot rolled over unexpectedly; reset buffers and counters
                    ESP_LOGW(TAG, "Slot rollover at %lldms, resetting", (long long)(now_ms - slot_start_ms));
                    monitor_reset(&mon);
                    mon.wf.num_blocks = 0;
                    slot_blocks = 0;
                    slot_start_ms = now_ms - (now_ms % 15000);
                    next_wake = xTaskGetTickCount();
                }
            }
        }
    }

    // Cleanup
    free(usb_buffer);
    free(ft8_buffer);
    free(temp_12k);
    monitor_free(&mon);

    g_streaming = false;
    s_stream_task_handle = NULL;
    ESP_LOGI(TAG, "Audio streaming task stopped");
    vTaskDelete(NULL);
}

#if USE_TEST_TONE
// Test tone streaming task - injects synthetic 1.5kHz tone for debugging
// This bypasses USB/UAC completely and uses precise timing
static void stream_test_tone_task(void* arg) {
    ESP_LOGI(TAG, "*** TEST TONE MODE *** - Injecting synthetic 1.5kHz tone");

    // Initialize test tone buffer
    init_test_tone_buffer();

    // Initialize resampler
    resample_init(&s_resample_state);
    log_heap_stats("test_tone_start");

    // Initialize FT8 monitor (waterfall/decoder)
    monitor_config_t mon_cfg = {
        .f_min = 200.0f,
        .f_max = 3000.0f,
        .sample_rate = FT8_SAMPLE_RATE,
        .time_osr = 1,
        .freq_osr = 2,
        .protocol = FTX_PROTOCOL_FT8
    };

    monitor_t mon;
    monitor_init(&mon, &mon_cfg);
    monitor_reset(&mon);
    log_heap_stats("after_monitor_init");

    const int target_blocks = 81;
    const int block_samples = mon.block_size;  // 1920 @ 12kHz = 160ms

    // Allocate buffers
    // Every 1ms we inject 48 stereo samples = 288 bytes
    const int samples_per_ms = 48;
    const int bytes_per_injection = samples_per_ms * 6;  // 288 bytes

    uint8_t* tone_buffer = (uint8_t*)heap_caps_malloc(bytes_per_injection, MALLOC_CAP_DEFAULT);
    float* ft8_buffer = (float*)heap_caps_malloc(sizeof(float) * block_samples, MALLOC_CAP_DEFAULT);
    float* temp_12k = (float*)heap_caps_malloc(sizeof(float) * 256, MALLOC_CAP_DEFAULT);
    log_heap_stats("test_tone_buffers_allocated");

    if (!tone_buffer || !ft8_buffer || !temp_12k) {
        ESP_LOGE(TAG, "Buffer allocation failed");
        if (tone_buffer) free(tone_buffer);
        if (ft8_buffer) free(ft8_buffer);
        if (temp_12k) free(temp_12k);
        monitor_free(&mon);
        s_stream_task_handle = NULL;
        g_streaming = false;
        vTaskDelete(NULL);
        return;
    }

    int ft8_buffer_idx = 0;
    int tone_phase_idx = 0;  // Track position in tone cycle
    TickType_t next_wake = xTaskGetTickCount();
    int slot_blocks = 0;
    int64_t slot_start_ms = 0;

    // Align start to the current 15-second boundary using RTC
    {
        int64_t now_ms = rtc_now_ms();
        int64_t rem = now_ms % 15000;
        int64_t wait_ms = (rem < 100) ? 0 : (15000 - rem);
        ESP_LOGI(TAG, "Test tone aligning to slot: now=%lldms rem=%lldms wait=%lldms",
                 (long long)now_ms, (long long)rem, (long long)wait_ms);
        if (wait_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS((uint32_t)wait_ms));
            now_ms = rtc_now_ms();
        }
        slot_start_ms = now_ms - (now_ms % 15000);
    }

    ESP_LOGI(TAG, "Test tone streaming started: %d samples/ms, %d bytes/injection",
             samples_per_ms, bytes_per_injection);

    while (!s_stop_requested) {
        // Fill buffer with test tone samples (48 samples @ 48kHz = 1ms of audio)
        fill_test_tone_samples(tone_buffer, samples_per_ms, &tone_phase_idx);

        // Convert and resample: 24-bit/48kHz/stereo -> 12kHz mono float
        int samples_12k = uac_to_ft8_samples(&s_resample_state, tone_buffer,
                                              temp_12k, samples_per_ms);

        // Accumulate into ft8_buffer
        for (int i = 0; i < samples_12k && !s_stop_requested; i++) {
            ft8_buffer[ft8_buffer_idx++] = temp_12k[i];

            // When we have a full block (1920 samples = 160ms)
            if (ft8_buffer_idx >= block_samples) {
                // Apply gain normalization (same as UAC task)
                double acc = 0.0;
                for (int j = 0; j < block_samples; ++j) {
                    acc += fabsf(ft8_buffer[j]);
                }
                float level = (float)(acc / block_samples);
                float gain = (level > 1e-6f) ? 0.5f / level : 1.0f;
                if (gain < 0.1f) gain = 0.1f;
                if (gain > 30.0f) gain = 30.0f;
                for (int j = 0; j < block_samples; ++j) {
                    ft8_buffer[j] *= gain;
                }

                static int rms_log_count = 0;
                if ((rms_log_count++ % 30) == 0) {
                    ESP_LOGI(TAG, "TestTone Block avg=%.5f gain=%.2f post=%.3f phase=%d",
                             level, gain, level * gain, tone_phase_idx % TEST_TONE_PERIOD);
                }

                if (mon.wf.num_blocks < target_blocks) {
                    monitor_process(&mon, ft8_buffer);
                    push_waterfall_latest(mon);
                }

                ft8_buffer_idx = 0;

                // Track slot progression and decode at 79 blocks
                slot_blocks++;
                int64_t now_ms = rtc_now_ms();
                if (slot_blocks >= 79) {
                    ESP_LOGI(TAG, "TestTone triggering decode slot=%lld blocks=%d",
                             (long long)(slot_start_ms / 15000), slot_blocks);
                    decode_monitor_results(&mon, &mon_cfg, false);
                    monitor_reset(&mon);
                    mon.wf.num_blocks = 0;
                    slot_blocks = 0;
                    slot_start_ms = now_ms - (now_ms % 15000);
                    next_wake = xTaskGetTickCount();
                } else if (now_ms - slot_start_ms >= 15000) {
                    ESP_LOGW(TAG, "TestTone slot rollover at %lldms, resetting",
                             (long long)(now_ms - slot_start_ms));
                    monitor_reset(&mon);
                    mon.wf.num_blocks = 0;
                    slot_blocks = 0;
                    slot_start_ms = now_ms - (now_ms % 15000);
                    next_wake = xTaskGetTickCount();
                }
            }
        }

        // Wait 1ms before next injection
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(1));
    }

    // Cleanup
    free(tone_buffer);
    free(ft8_buffer);
    free(temp_12k);
    monitor_free(&mon);

    g_streaming = false;
    s_stream_task_handle = NULL;
    ESP_LOGI(TAG, "Test tone streaming task stopped");
    vTaskDelete(NULL);
}
#endif // USE_TEST_TONE

// Public API implementation
uac_stream_state_t uac_get_state(void) {
    return s_state;
}

bool uac_is_streaming(void) {
#if USE_TEST_TONE
    return s_state == UAC_STATE_STREAMING && s_stream_task_handle != NULL;
#else
    return s_state == UAC_STATE_STREAMING && s_mic_handle != NULL;
#endif
}

bool uac_start(void) {
    if (s_state != UAC_STATE_IDLE) {
        ESP_LOGW(TAG, "UAC already started");
        return false;
    }

    s_stop_requested = false;
    resample_init(&s_resample_state);

#if USE_TEST_TONE
    // TEST TONE MODE: Skip USB/UAC initialization, start test tone task directly
    ESP_LOGI(TAG, "*** TEST TONE MODE ENABLED *** - Starting synthetic tone injection");

    // Start the test tone task directly
    BaseType_t ret = xTaskCreatePinnedToCore(stream_test_tone_task, "test_tone",
                                              STREAM_TASK_STACK_SIZE, NULL,
                                              UAC_STREAM_TASK_PRIORITY,
                                              &s_stream_task_handle, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create test tone task");
        return false;
    }

    s_state = UAC_STATE_STREAMING;
    g_streaming = true;
    snprintf(s_status_string, sizeof(s_status_string), "Test 1.5kHz %.0fdB", TEST_TONE_DB);
    return true;

#else
    // Normal UAC mode
    ESP_LOGI(TAG, "Starting UAC host");

    // Create event queue
    s_event_queue = xQueueCreate(10, sizeof(uac_event_t));
    if (!s_event_queue) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return false;
    }

    // Create UAC task first (it will wait for USB task notification)
    BaseType_t ret = xTaskCreatePinnedToCore(uac_lib_task, "uac_lib",
                                              TASK_STACK_SIZE, NULL,
                                              UAC_TASK_PRIORITY,
                                              &s_uac_task_handle, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UAC task");
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return false;
    }

    // Create USB host task (will notify UAC task when ready)
    ret = xTaskCreatePinnedToCore(usb_lib_task, "usb_lib",
                                   TASK_STACK_SIZE, (void*)s_uac_task_handle,
                                   USB_HOST_TASK_PRIORITY,
                                   &s_usb_task_handle, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create USB task");
        s_stop_requested = true;
        vTaskDelete(s_uac_task_handle);
        s_uac_task_handle = NULL;
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return false;
    }

    s_state = UAC_STATE_WAITING;
    snprintf(s_status_string, sizeof(s_status_string), "Waiting for device");
    return true;
#endif
}

void uac_stop(void) {
    if (s_state == UAC_STATE_IDLE) {
        return;
    }

    s_stop_requested = true;
    g_streaming = false;

#if USE_TEST_TONE
    ESP_LOGI(TAG, "Stopping test tone");

    // Wait for test tone task to finish
    int timeout = 50;  // 5 seconds
    while (s_stream_task_handle && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout--;
    }

    s_state = UAC_STATE_IDLE;
    snprintf(s_status_string, sizeof(s_status_string), "Idle");
    ESP_LOGI(TAG, "Test tone stopped");
#else
    ESP_LOGI(TAG, "Stopping UAC host");

    // Send stop event
    if (s_event_queue) {
        uac_event_t evt = {.type = UAC_EVT_STOP};
        xQueueSend(s_event_queue, &evt, pdMS_TO_TICKS(100));
    }

    // Wait for tasks to finish
    int timeout = 50;  // 5 seconds
    while ((s_stream_task_handle || s_uac_task_handle || s_usb_task_handle) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout--;
    }

    if (s_event_queue) {
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
    }

    s_state = UAC_STATE_IDLE;
    snprintf(s_status_string, sizeof(s_status_string), "Idle");
    ESP_LOGI(TAG, "UAC host stopped");
#endif
}

const char* uac_get_status_string(void) {
    return s_status_string;
}
