#include "stream_uac.h"
#include "resample.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
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
    ESP_LOGI(TAG, "UAC driver callback - addr:%d, iface:%d, event:%d", addr, iface_num, event);

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

                    // Print device info
                    uac_host_printf_device_param(handle);

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

    // Wait until the next 15s boundary
    {
        int64_t now_ms = rtc_now_ms();
        int64_t rem = now_ms % 15000;
        int64_t wait_ms = (rem < 100) ? 0 : (15000 - rem);
        if (wait_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS((uint32_t)wait_ms));
        }
    }

    // Initialize FT8 monitor
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

    // Allocate buffers
    uint8_t* usb_buffer = (uint8_t*)heap_caps_malloc(UAC_READ_BUFFER_SIZE, MALLOC_CAP_DEFAULT);
    float* ft8_buffer = (float*)heap_caps_malloc(sizeof(float) * mon.block_size, MALLOC_CAP_DEFAULT);
    // Intermediate buffer for 48kHz mono samples (max: 4096 bytes / 6 = 682 stereo samples)
    float* temp_12k = (float*)heap_caps_malloc(sizeof(float) * 1024, MALLOC_CAP_DEFAULT);

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

    const int target_blocks = 80;
    int ft8_buffer_idx = 0;  // Current position in ft8_buffer
    TickType_t next_wake = xTaskGetTickCount();

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
            if (ft8_buffer_idx >= mon.block_size) {
                // Apply gain normalization (same as stream_wav.cpp)
                double acc = 0.0;
                for (int j = 0; j < mon.block_size; ++j) {
                    acc += fabsf(ft8_buffer[j]);
                }
                float level = (float)(acc / mon.block_size);
                float gain = (level > 1e-6f) ? 0.1f / level : 1.0f;
                if (gain < 0.1f) gain = 0.1f;
                if (gain > 10.0f) gain = 10.0f;
                for (int j = 0; j < mon.block_size; ++j) {
                    ft8_buffer[j] *= gain;
                }

                // Process through monitor
                if (mon.wf.num_blocks < target_blocks) {
                    monitor_process(&mon, ft8_buffer);
                    push_waterfall_latest(mon);
                }

                ft8_buffer_idx = 0;

                // Maintain 160ms timing
                vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(160));

                // Check if we have enough blocks for decoding
                if (mon.wf.num_blocks >= target_blocks) {
                    if (mon.wf.num_blocks > 0) {
                        ESP_LOGI(TAG, "Triggering decode with %d blocks", mon.wf.num_blocks);

                        // Decode synchronously (or could spawn decode task like stream_wav)
                        decode_monitor_results(&mon, &mon_cfg, false);
                    }

                    // Reset for next slot
                    monitor_reset(&mon);
                    mon.wf.num_blocks = 0;
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

// Public API implementation
uac_stream_state_t uac_get_state(void) {
    return s_state;
}

bool uac_is_streaming(void) {
    return s_state == UAC_STATE_STREAMING && s_mic_handle != NULL;
}

bool uac_start(void) {
    if (s_state != UAC_STATE_IDLE) {
        ESP_LOGW(TAG, "UAC already started");
        return false;
    }

    ESP_LOGI(TAG, "Starting UAC host");
    s_stop_requested = false;
    resample_init(&s_resample_state);

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
}

void uac_stop(void) {
    if (s_state == UAC_STATE_IDLE) {
        return;
    }

    ESP_LOGI(TAG, "Stopping UAC host");
    s_stop_requested = true;
    g_streaming = false;

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
}

const char* uac_get_status_string(void) {
    return s_status_string;
}
