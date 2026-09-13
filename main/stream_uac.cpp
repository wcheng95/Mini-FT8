#include "stream_uac.h"
#include "resample.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "usb/usb_host.h"
#include "usb/uac_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_types_ch9.h"

extern "C" {
#include "ft8/decode.h"
#include "ft8/constants.h"
#include "common/monitor.h"
}

#include "ui.h"
#include "core_api_internal.h"
#include <cstring>
#include <cmath>
#include <inttypes.h>

static const char* TAG = "UAC_STREAM";
extern void log_heap(const char* tag);

// External references from main.cpp
extern bool g_streaming;
extern bool g_decode_enabled;
extern int g_time_osr;
extern int g_freq_osr;
extern int64_t g_decode_slot_idx;
extern volatile bool g_decode_in_progress;
// g_decode_applied_slot_idx is owned by the main loop now (core_api_internal.h
// autoseq owner); this task reports lost slots instead of writing it.
#include "core_api_internal.h"
static constexpr int kMinDecodeBlocks = 72;
extern volatile bool g_cdc_initial_sync_pending;
void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui);
int64_t rtc_now_ms();

#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 6000
#endif

// Task priorities and stack sizes
#define USB_HOST_TASK_PRIORITY  5
#define UAC_TASK_PRIORITY       5
#define UAC_STREAM_TASK_PRIORITY 4
#define TASK_STACK_SIZE         4096
#define STREAM_TASK_STACK_SIZE  8192

// UAC read buffer size (bytes) - must be multiple of 288 (USB transfer size at 48kHz/24bit/stereo)
// 288 bytes = 48 stereo samples per 1ms USB transfer.
// 2304 = 288 * 8 — halved from the original 4608 to free 2304 B of
// DMA-capable BSS for the heap pool. The decoder is offline (FFT runs
// for ~13 s with the sample pump effectively paused), so a smaller
// in-flight USB EP IN buffer only matters during live collection — and
// 8 ms of headroom is still well above what the host scheduler needs.
// Regression canary: waterfall artifacts from the UAC signal generator.
#define UAC_READ_BUFFER_SIZE    2304

// USB host DMAs into this buffer. Placed in DMA-capable BSS via DMA_ATTR so
// task start-up doesn't depend on finding a large DMA-capable heap block —
// previously heap_caps_malloc(MALLOC_CAP_DMA) failed under fragmentation
// even when raw free bytes looked sufficient (alignment eats into runs).
static DMA_ATTR uint8_t s_usb_buffer[UAC_READ_BUFFER_SIZE];

typedef struct {
    uint32_t sample_freq;
    uint8_t bit_resolution;
    uint8_t channels;
} uac_active_format_t;

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
static cdc_acm_dev_hdl_t s_cdc_handle = NULL;
static TaskHandle_t s_usb_task_handle = NULL;
static TaskHandle_t s_uac_task_handle = NULL;
static TaskHandle_t s_stream_task_handle = NULL;
static volatile bool s_stop_requested = false;
static char s_status_string[64] = "Idle";
static uac_stream_profile_t s_profile = UAC_PROFILE_QMX;
static uac_active_format_t s_format = {
    .sample_freq = UAC_SAMPLE_RATE,
    .bit_resolution = UAC_BIT_RESOLUTION,
    .channels = UAC_CHANNELS,
};
static bool s_cdc_installed = false;
static int s_cdc_iface = -1;
static int s_cdc_iface_hint = -1;
static int64_t s_cdc_last_attempt_ms = 0;
static constexpr uint16_t k_qmx_vid = 0x0483;
static constexpr uint16_t k_qmx_pid = 0xA34C;

// Debug display buffers
static char s_debug_line1[64] = "";
static char s_debug_line2[64] = "";

// Resampler state
static resample_state_t s_resample_state;
static uint8_t s_latest_waterfall_row[UAC_WATERFALL_ROW_WIDTH] = {0};
static bool s_latest_waterfall_row_valid = false;
static portMUX_TYPE s_latest_waterfall_row_lock = portMUX_INITIALIZER_UNLOCKED;

// Forward declarations
static void usb_lib_task(void* arg);
static void uac_lib_task(void* arg);
static void stream_uac_task(void* arg);
static void cdc_close(void);
static void cdc_try_open(void);
static void cdc_event_cb(const cdc_acm_host_dev_event_data_t* event, void* user_ctx);
static void cdc_new_dev_cb(usb_device_handle_t usb_dev);

static const char* profile_name(uac_stream_profile_t profile) {
    switch (profile) {
    case UAC_PROFILE_QMX:
        return "qmx_uac";
    case UAC_PROFILE_GENERIC_USB:
        return "usb_uac_generic";
    default:
        return "unknown";
    }
}

// Push waterfall row — zero-copy at freq_osr=1 (our current config).
// `base` points into mon.wf.mag for the most recently written block's
// data. That memory is stable until the next slot boundary (other blocks
// write to different offsets), so downstream consumers (UI scaling here,
// ble_native elsewhere) can read from it safely without us copying.
//
// At freq_osr>1 we still need a scratch buffer to merge the multiple
// frequency-sub bands into one row — but since we're at freq_osr=1, no
// static or stack buffer is allocated here in practice.
static void push_waterfall_latest(const monitor_t& mon) {
    if (mon.wf.num_blocks <= 0 || mon.wf.mag == nullptr) return;
    const int block = mon.wf.num_blocks - 1;
    const int num_bins = mon.wf.num_bins;
    const int freq_osr = mon.wf.freq_osr;
    const uint8_t* base = mon.wf.mag + block * mon.wf.block_stride;

    const uint8_t* row_bins;     // pointer to `num_bins` bytes, one per FFT bin
    if (freq_osr <= 1) {
        row_bins = base;          // already single-band, zero-copy
    } else {
        // Rare path (not used in current 6 kHz config). Keep the merge
        // buffer function-local so freq_osr=1 doesn't pay for BSS.
        static uint8_t collapsed[480];
        for (int b = 0; b < num_bins; ++b) {
            uint8_t v = 0;
            for (int fs = 0; fs < freq_osr; ++fs) {
                uint8_t val = base[fs * num_bins + b];
                if (val > v) v = val;
            }
            collapsed[b] = v;
        }
        row_bins = collapsed;
    }

    constexpr int width = UAC_WATERFALL_ROW_WIDTH;
    static uint8_t scaled[width];
    for (int x = 0; x < width; ++x) {
        int start = (int)((int64_t)x * num_bins / width);
        int end = (int)((int64_t)(x + 1) * num_bins / width);
        if (end <= start) end = start + 1;
        uint8_t maxv = 0;
        for (int s = start; s < end && s < num_bins; ++s) {
            if (row_bins[s] > maxv) maxv = row_bins[s];
        }
        scaled[x] = maxv;
    }

    ui_push_waterfall_row(scaled, width);
    taskENTER_CRITICAL(&s_latest_waterfall_row_lock);
    memcpy(s_latest_waterfall_row, scaled, width);
    s_latest_waterfall_row_valid = true;
    taskEXIT_CRITICAL(&s_latest_waterfall_row_lock);

    // Stubbed swr/pwr/ptt until real polling lands (see
    // NATIVE_CLIENT_ARCHITECTURE.md). row_bins points into mon.wf.mag
    // (zero-copy) so the callback receives a pointer into authoritative
    // waterfall storage — valid until the next slot boundary.
    static int wf_log_counter = 0;
    if ((wf_log_counter++ % 60) == 0) {
        ESP_LOGI(TAG, "push_waterfall_latest: block=%d num_bins=%d", block, num_bins);
    }
    core_fire_waterfall_row(block, row_bins, num_bins,
                            /*swr=*/1.5f, /*pwr=*/2.0f, /*ptt=*/false);
}

// CDC-ACM helpers (CAT TX only)
static void cdc_close(void) {
    if (s_cdc_handle) {
        cdc_acm_host_close(s_cdc_handle);
        s_cdc_handle = NULL;
    }
    s_cdc_iface = -1;
    s_cdc_last_attempt_ms = 0;
    s_cdc_iface_hint = -1;
}

static void cdc_event_cb(const cdc_acm_host_dev_event_data_t* event, void* user_ctx) {
    (void)user_ctx;
    if (!event) return;
    switch (event->type) {
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "CDC device disconnected");
        cdc_close();
        break;
    case CDC_ACM_HOST_ERROR:
        ESP_LOGW(TAG, "CDC device error: %d", event->data.error);
        break;
    default:
        break;
    }
}

static void cdc_try_open(void) {
    if (!s_cdc_installed) return;
    if (s_cdc_handle) return;

    // Throttle attempts
    int64_t now_ms = rtc_now_ms();
    if (s_cdc_last_attempt_ms != 0 && (now_ms - s_cdc_last_attempt_ms) < 1000) return;
    s_cdc_last_attempt_ms = now_ms;

    cdc_acm_host_device_config_t dev_cfg = {
        .connection_timeout_ms = 1000,
        .out_buffer_size = 64,   // small TX buffer; RX disabled
        .in_buffer_size = 0,
        .event_cb = cdc_event_cb,
        .data_cb = NULL,
        .user_arg = NULL,
    };

    const int max_iface_scan = 12;

    // QMX profile: try known QMX CAT (VID/PID, iface 0) first.
    if (s_profile == UAC_PROFILE_QMX) {
        cdc_acm_dev_hdl_t handle = NULL;
        esp_err_t err = cdc_acm_host_open(k_qmx_vid, k_qmx_pid, 0, &dev_cfg, &handle);
        if (err == ESP_OK) {
            s_cdc_handle = handle;
            s_cdc_iface = 0;
            ESP_LOGI(TAG, "CDC-ACM opened (QMX iface 0, VID 0x%04x PID 0x%04x)", k_qmx_vid, k_qmx_pid);
            cdc_acm_host_desc_print(handle);
            g_cdc_initial_sync_pending = true;  // main loop will auto-sync VFO + RX mode
            return;
        } else if (err != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "CDC open QMX iface 0 failed: %s", esp_err_to_name(err));
        }
    }

    // Try hinted CDC interface first (if we saw class 0x02)
    if (s_cdc_iface_hint >= 0) {
        cdc_acm_dev_hdl_t handle = NULL;
        esp_err_t err = cdc_acm_host_open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID,
                                          (uint8_t)s_cdc_iface_hint, &dev_cfg, &handle);
        if (err == ESP_OK) {
            s_cdc_handle = handle;
            s_cdc_iface = s_cdc_iface_hint;
            ESP_LOGI(TAG, "CDC-ACM opened (hint iface %d)", s_cdc_iface_hint);
            cdc_acm_host_desc_print(handle);
            g_cdc_initial_sync_pending = true;  // main loop will auto-sync VFO + RX mode
            return;
        } else {
            ESP_LOGW(TAG, "CDC open hint iface %d failed: %s",
                     s_cdc_iface_hint, esp_err_to_name(err));
        }
    }

    for (int iface = 0; iface < max_iface_scan; ++iface) {
        cdc_acm_dev_hdl_t handle = NULL;
        esp_err_t err = cdc_acm_host_open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID,
                                          (uint8_t)iface, &dev_cfg, &handle);
        if (err == ESP_OK) {
            s_cdc_handle = handle;
            s_cdc_iface = iface;
            ESP_LOGI(TAG, "CDC-ACM opened (iface %d)", iface);
            cdc_acm_host_desc_print(handle);
            g_cdc_initial_sync_pending = true;  // main loop will auto-sync VFO + RX mode
            break;
        } else if (err != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "CDC open iface %d failed: %s", iface, esp_err_to_name(err));
        }
    }

    if (!s_cdc_handle) {
        ESP_LOGD(TAG, "CDC-ACM not found yet (profile=%s attempt at %" PRId64 " ms)",
                 profile_name(s_profile), now_ms);
    }
}

static void cdc_new_dev_cb(usb_device_handle_t usb_dev) {
    usb_device_info_t info = {};
    if (usb_host_device_info(usb_dev, &info) == ESP_OK) {
        ESP_LOGI(TAG, "USB dev addr:%u speed:%d", info.dev_addr, info.speed);
    }

    const usb_device_desc_t* dev_desc = nullptr;
    if (usb_host_get_device_descriptor(usb_dev, &dev_desc) == ESP_OK && dev_desc) {
        ESP_LOGI(TAG, "USB dev attached: VID:0x%04x PID:0x%04x cfgs:%u",
                 dev_desc->idVendor, dev_desc->idProduct, dev_desc->bNumConfigurations);
    }

    const usb_config_desc_t* cfg = nullptr;
    if (usb_host_get_active_config_descriptor(usb_dev, &cfg) == ESP_OK && cfg) {
        const uint8_t* p = (const uint8_t*)cfg;
        int offset = 0;
        while (offset + 2 <= cfg->wTotalLength) {
            uint8_t len = p[offset];
            uint8_t dtype = p[offset + 1];
            if (len == 0) break;
            if (dtype == USB_B_DESCRIPTOR_TYPE_INTERFACE && len >= sizeof(usb_intf_desc_t)) {
                const usb_intf_desc_t* intf = (const usb_intf_desc_t*)(p + offset);
                ESP_LOGI(TAG, "  IF num=%u alt=%u eps=%u class=0x%02x subclass=0x%02x proto=0x%02x",
                         intf->bInterfaceNumber, intf->bAlternateSetting,
                         intf->bNumEndpoints, intf->bInterfaceClass,
                         intf->bInterfaceSubClass, intf->bInterfaceProtocol);
                if (intf->bInterfaceClass == USB_CLASS_COMM && s_cdc_iface_hint < 0) {
                    s_cdc_iface_hint = intf->bInterfaceNumber;
                    ESP_LOGI(TAG, "  -> CDC candidate iface %d", s_cdc_iface_hint);
                }
            }
            offset += len;
        }
    }
}

// UAC device callback
static void uac_device_callback(uac_host_device_handle_t handle,
                                 const uac_host_device_event_t event,
                                 void* arg) {
    if (event == UAC_HOST_DRIVER_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "UAC device disconnected");
        cdc_close();
        if (handle == s_mic_handle) {
            s_mic_handle = NULL;
            s_state = UAC_STATE_WAITING;
            snprintf(s_status_string, sizeof(s_status_string), "Disconnected");
        }
        uac_host_device_close(handle);
        return;
    }

    uac_event_t evt = {};
    evt.type = UAC_EVT_DEVICE;
    evt.device.handle = handle;
    evt.device.event = event;
    xQueueSend(s_event_queue, &evt, 0);
}

// UAC driver callback
static void uac_driver_callback(uint8_t addr, uint8_t iface_num,
                                 const uac_host_driver_event_t event,
                                 void* arg) {
    ESP_LOGI(TAG, "UAC driver callback - addr:%d, iface:%d, event:%d", addr, iface_num, event);

    uac_event_t evt = {};
    evt.type = UAC_EVT_DRIVER;
    evt.driver.addr = addr;
    evt.driver.iface_num = iface_num;
    evt.driver.event = event;
    xQueueSend(s_event_queue, &evt, 0);
}

// USB host library task
static void usb_lib_task(void* arg) {
    usb_host_config_t host_config = {};
    host_config.skip_phy_setup = false;
    host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;

    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB host: %s", esp_err_to_name(err));
        s_state = UAC_STATE_ERROR;
        snprintf(s_status_string, sizeof(s_status_string), "USB init failed");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "USB Host installed");

    // Install CDC-ACM driver (for CAT control) before starting class drivers
    const cdc_acm_host_driver_config_t cdc_cfg = {
        .driver_task_stack_size = 3072,
        .driver_task_priority = 4,
        .xCoreID = 0,
        .new_dev_cb = cdc_new_dev_cb,
    };
    err = cdc_acm_host_install(&cdc_cfg);
    if (err == ESP_OK) {
        s_cdc_installed = true;
        ESP_LOGI(TAG, "CDC-ACM driver installed");
    } else {
        ESP_LOGW(TAG, "CDC-ACM driver install failed: %s", esp_err_to_name(err));
    }

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

    if (s_cdc_installed) {
        cdc_close();
        cdc_acm_host_uninstall();
        s_cdc_installed = false;
        ESP_LOGI(TAG, "CDC-ACM driver uninstalled");
    }

    // Drain pending USB host events. usb_host_uninstall() refuses to
    // run (returns ESP_ERR_INVALID_STATE without releasing the USB
    // PHY) if process_pending_flags / lib_event_flags / flags.val is
    // non-zero. cdc_acm_host_uninstall posts a client-detach event
    // that must be pumped through, otherwise the PHY stays claimed
    // and the subsequent TinyUSB device-mode init fails with
    // "selected PHY is in use" — which is exactly what we hit on the
    // no-QMX-after-10s path where no device-disconnect ever
    // pre-drained the queue. Pump until ALL_FREE fires or we time
    // out (defence against a stuck event we can't service anyway).
    for (int i = 0; i < 50; ++i) {
        uint32_t event_flags = 0;
        usb_host_lib_handle_events(pdMS_TO_TICKS(20), &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) break;
    }

    ESP_LOGI(TAG, "USB Host uninstalling");
    esp_err_t uerr = usb_host_uninstall();
    if (uerr != ESP_OK) {
        ESP_LOGW(TAG, "usb_host_uninstall: %s", esp_err_to_name(uerr));
    }
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

                    // Start stream format negotiation.
                    // QMX profile keeps the existing strict format.
                    // Generic profile tries mono and 16-bit fallbacks.
                    uac_host_stream_config_t candidates[4];
                    int candidate_count = 0;
                    auto add_candidate = [&](uint8_t ch, uint8_t bits) {
                        candidates[candidate_count].channels = ch;
                        candidates[candidate_count].bit_resolution = bits;
                        candidates[candidate_count].sample_freq = UAC_SAMPLE_RATE;
                        candidates[candidate_count].flags = 0;
                        candidate_count++;
                    };
                    if (s_profile == UAC_PROFILE_GENERIC_USB) {
                        add_candidate(1, 24);
                        add_candidate(1, 16);
                        add_candidate(2, 24);
                        add_candidate(2, 16);
                    } else {
                        add_candidate(UAC_CHANNELS, UAC_BIT_RESOLUTION);
                    }

                    bool started = false;
                    for (int i = 0; i < candidate_count; ++i) {
                        const uac_host_stream_config_t* cfg = &candidates[i];
                        ESP_LOGI(TAG, "Starting stream (profile=%s) candidate %d/%d: %dHz, %d-bit, %dch",
                                 profile_name(s_profile), i + 1, candidate_count,
                                 cfg->sample_freq, cfg->bit_resolution, cfg->channels);
                        err = uac_host_device_start(handle, cfg);
                        if (err == ESP_OK) {
                            s_format.sample_freq = cfg->sample_freq;
                            s_format.bit_resolution = cfg->bit_resolution;
                            s_format.channels = cfg->channels;
                            started = true;
                            ESP_LOGI(TAG, "Selected stream format: %dHz, %d-bit, %dch",
                                     s_format.sample_freq, s_format.bit_resolution, s_format.channels);
                            break;
                        }
                        ESP_LOGW(TAG, "Stream candidate failed: %s", esp_err_to_name(err));
                    }

                    if (!started) {
                        ESP_LOGE(TAG, "Failed to start stream for profile=%s", profile_name(s_profile));
                        snprintf(s_status_string, sizeof(s_status_string), "Format not supported");
                        uac_host_device_close(handle);
                        continue;
                    }

                    s_mic_handle = handle;
                    s_state = UAC_STATE_STREAMING;
                    g_streaming = true;
                    snprintf(s_status_string, sizeof(s_status_string),
                             "Streaming %s %luk/%u/%u",
                             profile_name(s_profile),
                             (unsigned long)(s_format.sample_freq / 1000),
                             s_format.bit_resolution,
                             s_format.channels);

                    // Try to open companion CDC-ACM interface (CAT)
                    cdc_try_open();

                    // Start the audio processing task using a STATIC stack
                    // (BSS) so task creation doesn't depend on finding a
                    // contiguous 8 KB block in a fragmented heap.
                    if (s_stream_task_handle == NULL) {
                        static StackType_t  s_stream_task_stack[STREAM_TASK_STACK_SIZE / sizeof(StackType_t)];
                        static StaticTask_t s_stream_task_tcb;
                        size_t free_before = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
                        size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
                        ESP_LOGI(TAG, "Pre-task-create heap: free=%u largest=%u",
                                 (unsigned)free_before, (unsigned)largest);
                        s_stream_task_handle = xTaskCreateStaticPinnedToCore(
                            stream_uac_task, "stream_uac",
                            STREAM_TASK_STACK_SIZE / sizeof(StackType_t), NULL,
                            UAC_STREAM_TASK_PRIORITY,
                            s_stream_task_stack, &s_stream_task_tcb, 1);
                        if (!s_stream_task_handle) {
                            ESP_LOGE(TAG, "stream_uac_task create FAILED "
                                     "(static) free=%u largest=%u",
                                     (unsigned)free_before, (unsigned)largest);
                        }
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

    // Initialize FT8 monitor and grab heap working buffers FIRST, before
    // the slot-boundary wait below. NimBLE init runs deferred in the main
    // loop and races against this task creation; if we malloc'd after the
    // wait, NimBLE's controller pool would already have drained the heap
    // and ft8_buffer would come back NULL.
    monitor_config_t mon_cfg = {
        .f_min = 200.0f,
        .f_max = 2900.0f,
        .sample_rate = FT8_SAMPLE_RATE,
        .time_osr = g_time_osr,
        .freq_osr = g_freq_osr,
        .protocol = FTX_PROTOCOL_FT8
    };

    monitor_t mon;
    monitor_init(&mon, &mon_cfg);
    monitor_reset(&mon);

    // USB buffer lives in DMA-capable BSS (s_usb_buffer above). Only the
    // float working buffers are heap-allocated; they don't need DMA capability.
    uint8_t* usb_buffer = s_usb_buffer;
    float*   ft8_buffer = (float*)heap_caps_malloc(sizeof(float) * mon.block_size,
                                                   MALLOC_CAP_DEFAULT);
    float*   temp_dec   = (float*)heap_caps_malloc(sizeof(float) * 512,
                                                   MALLOC_CAP_DEFAULT);
    log_heap("UAC_AFTER_FFT_ALLOC");
    if (!ft8_buffer || !temp_dec) {
        ESP_LOGE(TAG, "Buffer allocation failed: ft8=%p temp=%p",
                 ft8_buffer, temp_dec);
        if (ft8_buffer) free(ft8_buffer);
        if (temp_dec) free(temp_dec);
        monitor_free(&mon);
        s_stream_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Wait until the next 15s boundary so the sample-pump loop below aligns
    // to FT8 slot edges. Buffers were grabbed above so heap state during
    // this sleep doesn't matter.
    {
        int64_t now_ms = rtc_now_ms();
        int64_t rem = now_ms % 15000;
        int64_t wait_ms = (rem < 100) ? 0 : (15000 - rem);
        if (wait_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS((uint32_t)wait_ms));
        }
    }

    const int target_blocks = 80;
    int ft8_buffer_idx = 0;  // Current position in ft8_buffer
    TickType_t next_wake = xTaskGetTickCount();
    int slot_blocks = 0;
    int64_t slot_idx = rtc_now_ms() / 15000;
    int64_t slot_start_ms = slot_idx * 15000;
    (void)slot_start_ms; // silence unused warning

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

        int frame_bytes = (s_format.bit_resolution / 8) * s_format.channels;
        if (frame_bytes <= 0) {
            ESP_LOGW(TAG, "Invalid stream format bytes/frame=%d", frame_bytes);
            continue;
        }
        int num_frames = bytes_read / frame_bytes;
        int remainder = bytes_read % frame_bytes;

        // Debug display
        if (num_frames > 0) {
            int32_t val = 0;
            if (s_format.bit_resolution == 24) {
                val = usb_buffer[0] | (usb_buffer[1] << 8) | (usb_buffer[2] << 16);
                if (val & 0x800000) val |= 0xFF000000;
            } else { // 16-bit
                int16_t v16 = (int16_t)(usb_buffer[0] | (usb_buffer[1] << 8));
                val = v16;
            }
            snprintf(s_debug_line1, sizeof(s_debug_line1),
                     "fmt=%lu/%u/%u v=%ld",
                     (unsigned long)s_format.sample_freq,
                     s_format.bit_resolution,
                     s_format.channels,
                     (long)val);
            snprintf(s_debug_line2, sizeof(s_debug_line2),
                     "rd=%lu fb=%d rem=%d", (unsigned long)bytes_read, frame_bytes, remainder);
        }

        if (num_frames == 0) continue;

        // Convert and resample selected USB PCM format -> 6kHz mono float.
        int samples_dec = uac_pcm_to_ft8_samples(&s_resample_state, usb_buffer,
                                                 (int)bytes_read, temp_dec,
                                                 s_format.bit_resolution,
                                                 s_format.channels);

        // Accumulate into ft8_buffer
        for (int i = 0; i < samples_dec && !s_stop_requested; i++) {
            ft8_buffer[ft8_buffer_idx++] = temp_dec[i];

            // When we have a full block (960 samples = 160ms at 6kHz)
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

                // Retry CDC open periodically until success
                if (!s_cdc_handle) {
                    cdc_try_open();
                }

                ft8_buffer_idx = 0;

                // Maintain 160ms timing
                vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(160));

                // Align decode to 15s boundaries based on RTC
                slot_blocks++;
                int64_t now_idx = rtc_now_ms() / 15000;
                if (now_idx != slot_idx) {
                    ESP_LOGI(TAG, "Slot boundary %lld->%lld blocks=%d wf=%d",
                             (long long)slot_idx, (long long)now_idx,
                             slot_blocks, mon.wf.num_blocks);
                    // A slot ending short of 79 blocks means this task was busy
                    // elsewhere — decoding the previous slot (and, until the
                    // logger task, writing its log lines to flash). Decode it
                    // anyway if the payload symbols are all there: FT8 carries
                    // its 174 payload bits in symbols 7-35 and 43-71; 72-78 are
                    // only the third sync array, so >= 72 blocks decodes nearly
                    // as well as a full slot. Discarding it — and silently
                    // advancing the TX gate here — is how RT260913 (Pismo)
                    // transmitted a retry over the other station's response.
                    if (g_decode_enabled && mon.wf.num_blocks >= kMinDecodeBlocks &&
                        mon.wf.num_blocks < 79) {
                        ESP_LOGW(TAG, "Short slot %lld: decoding %d/79 blocks",
                                 (long long)slot_idx, mon.wf.num_blocks);
                        g_decode_slot_idx = slot_idx;
                        g_decode_in_progress = true;
                        decode_monitor_results(&mon, &mon_cfg, false);
                    } else if (mon.wf.num_blocks < 79) {
                        // Nothing decodable. The owner advances the TX gate
                        // and logs it — not done here, not in silence.
                        autoseq_owner_post_slot_lost(slot_idx, mon.wf.num_blocks);
                    }
                    // Reset counters at the boundary
                    slot_idx = now_idx;
                    slot_start_ms = slot_idx * 15000;
                    slot_blocks = 0;
                    mon.wf.num_blocks = 0;
                    monitor_reset(&mon);
                    next_wake = xTaskGetTickCount();
                } else if (slot_blocks >= 79 && mon.wf.num_blocks >= 79) {
                    ESP_LOGI(TAG, "Triggering decode at slot %lld blocks=%d wf=%d",
                             (long long)slot_idx, slot_blocks, mon.wf.num_blocks);
                    if (g_decode_enabled) {
                        g_decode_slot_idx = slot_idx;
                        g_decode_in_progress = true;  // Block TX trigger until decode finishes
                        decode_monitor_results(&mon, &mon_cfg, false);
                        // g_decode_in_progress and g_decode_applied_slot_idx are
                        // both updated at the end of decode_monitor_results.
                    } else {
                        ESP_LOGI(TAG, "Decode paused; skipping");
                        // Decode disabled — the owner still has to advance the
                        // TX gate past this slot, or TX blocks forever.
                        autoseq_owner_post_slot_lost(slot_idx, -1);
                    }
                    monitor_reset(&mon);
                    mon.wf.num_blocks = 0;
                    slot_blocks = 0;
                    next_wake = xTaskGetTickCount();
                }
            }
        }
    }

    // Cleanup — usb_buffer is static BSS, no free.
    free(ft8_buffer);
    free(temp_dec);
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

bool uac_get_latest_waterfall_row(uint8_t* out_row, int out_len) {
    if (!out_row || out_len < UAC_WATERFALL_ROW_WIDTH) return false;
    bool valid = false;
    taskENTER_CRITICAL(&s_latest_waterfall_row_lock);
    valid = s_latest_waterfall_row_valid;
    if (valid) {
        memcpy(out_row, s_latest_waterfall_row, UAC_WATERFALL_ROW_WIDTH);
    }
    taskEXIT_CRITICAL(&s_latest_waterfall_row_lock);
    return valid;
}

bool uac_start_with_profile(uac_stream_profile_t profile) {
    if (s_state != UAC_STATE_IDLE) {
        ESP_LOGW(TAG, "UAC already started");
        return false;
    }

    s_profile = profile;
    s_cdc_last_attempt_ms = 0;
    s_cdc_iface = -1;
    s_cdc_iface_hint = -1;
    s_format.sample_freq = UAC_SAMPLE_RATE;
    s_format.bit_resolution = UAC_BIT_RESOLUTION;
    s_format.channels = UAC_CHANNELS;
    taskENTER_CRITICAL(&s_latest_waterfall_row_lock);
    memset(s_latest_waterfall_row, 0, sizeof(s_latest_waterfall_row));
    s_latest_waterfall_row_valid = false;
    taskEXIT_CRITICAL(&s_latest_waterfall_row_lock);

    ESP_LOGI(TAG, "Starting UAC host profile=%s", profile_name(s_profile));
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
    snprintf(s_status_string, sizeof(s_status_string), "Waiting for %s", profile_name(s_profile));
    return true;
}

bool uac_start(void) {
    return uac_start_with_profile(UAC_PROFILE_QMX);
}

bool uac_qmx_detected(void) {
  return s_mic_handle != NULL || s_cdc_handle != NULL;
}

void uac_stop(void) {
    if (s_state == UAC_STATE_IDLE) {
        return;
    }

    ESP_LOGI(TAG, "Stopping UAC host");
    s_stop_requested = true;
    g_streaming = false;
    cdc_close();

    // Send stop event
    if (s_event_queue) {
        uac_event_t evt = {};
        evt.type = UAC_EVT_STOP;
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
    taskENTER_CRITICAL(&s_latest_waterfall_row_lock);
    memset(s_latest_waterfall_row, 0, sizeof(s_latest_waterfall_row));
    s_latest_waterfall_row_valid = false;
    taskEXIT_CRITICAL(&s_latest_waterfall_row_lock);
    snprintf(s_status_string, sizeof(s_status_string), "Idle");
    ESP_LOGI(TAG, "UAC host stopped");
}

const char* uac_get_status_string(void) {
    return s_status_string;
}

const char* uac_get_debug_line1(void) {
    return s_debug_line1;
}

const char* uac_get_debug_line2(void) {
    return s_debug_line2;
}

bool cat_cdc_ready(void) {
    return s_cdc_handle != NULL;
}

esp_err_t cat_cdc_send(const uint8_t* data, size_t len, uint32_t timeout_ms) {
    if (!s_cdc_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    return cdc_acm_host_data_tx_blocking(s_cdc_handle, data, len, timeout_ms);
}
