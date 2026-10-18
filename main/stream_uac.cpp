#include "stream_uac.h"
#include "ft8_audio_pipeline.h"
#include "resample.h"
#include "dds_q15.h"
#include "feature_flags.h"
#include "protocol.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "usb/usb_host.h"
#include "usb/uac_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.h"
#include "usb/vcp_cp210x.h"
#include "usb/usb_types_ch9.h"
#include "usb/usb_helpers.h"

#include <cstring>
#include <cmath>
#include <inttypes.h>

static const char* TAG = "UAC_STREAM";
extern void log_heap(const char* tag);

// External references from main.cpp
extern bool g_streaming;
extern volatile bool g_cdc_initial_sync_pending;
int64_t rtc_now_ms();

// Task priorities and stack sizes
#define USB_HOST_TASK_PRIORITY  5
#define UAC_TASK_PRIORITY       5
// Below the USB host task (5) so CDC-ACM CAT TX to the QMX is never
// preempted by audio-pipeline work.
#define UAC_STREAM_TASK_PRIORITY 4
#define TASK_STACK_SIZE         4096
#define STREAM_TASK_STACK_SIZE  8192

// UAC read buffer size (bytes) - must be multiple of 288 (USB transfer size at 48kHz/24bit/stereo).
// 288 bytes = 48 stereo samples per 1 ms USB transfer; 2304 = 8 transfers = 8 ms of audio.
// Kept at 8 transfers: the DMA buffer lives in BSS (DMA_ATTR), so larger
// values directly increase static internal DRAM use.
#define UAC_READ_BUFFER_SIZE    2304

// USB host DMA buffer — placed in DMA-capable BSS via DMA_ATTR so task
// startup doesn't depend on finding a large DMA-aligned heap block under
// fragmentation (heap_caps_malloc(MALLOC_CAP_DMA) can fail even when raw
// free bytes look sufficient because alignment requirements eat contiguous runs).
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

// Speaker (UAC OUT) is captured and fully allocated during enumeration.
// QDX transmission only resumes or suspends the pre-opened endpoint.
static uint8_t s_spk_addr = 0;
static uint8_t s_spk_iface = 0;
static bool s_spk_known = false;
static uac_host_device_handle_t s_spk_handle = NULL;
static TaskHandle_t s_spk_writer_task = NULL;
static volatile bool s_spk_writer_stop = false;  // permanent shutdown
static volatile bool s_spk_writer_run = false;   // per-TX gate
static uint8_t* s_spk_pump_buf = nullptr;
static uint32_t s_spk_packets_sent = 0;
static uint32_t s_spk_write_errors = 0;
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
static constexpr uint32_t k_fx4cr_cat_baud = 115200;
// FX-4CR: the USB-serial bridge seen behind the radio's hub. Any device
// with a CDC or vendor-specific interface that is not the audio codec or a
// hub is a candidate. Opened by exact VID/PID: CDC_HOST_ANY_* behind a hub
// may pick the codec instead.
static uint16_t s_vcp_vid = 0;
static uint16_t s_vcp_pid = 0;
static int s_vcp_iface = 0;
static char s_cat_status[32] = "CAT none";  // FX-4CR STATUS line text
// FX-4CR host channel budget. The ESP32-S3 USB host has 8 channels and
// ESP-IDF keeps one per open endpoint. The radio has two hubs (control +
// status each) and three devices (codec, CH340, CSR Bluetooth; one control
// endpoint each): 7 channels are held permanently, leaving exactly one.
// So the mic, the speaker and the CAT bridge take turns: CAT is sent in
// short bursts (uac_cat_batch) with the mic paused, and the bridge is
// claimed with its bulk OUT endpoint only (fx_ch340_write).
static bool s_fx_cat_announced = false;
static int64_t s_cat_fail_ms = 0;   // last failed CAT burst
static int s_cat_fail_count = 0;    // consecutive failures, for back-off
static SemaphoreHandle_t s_cat_lock = NULL;

// Debug display buffers
static char s_debug_line1[64] = "";
static char s_debug_line2[64] = "";

// Resampler state
static resample_state_t s_resample_state;

// Forward declarations
static void usb_lib_task(void* arg);
static void uac_lib_task(void* arg);
static void stream_uac_task(void* arg);
static void cdc_close(void);
static void cdc_try_open(void);
static void cdc_event_cb(const cdc_acm_host_dev_event_data_t* event, void* user_ctx);

// Speaker constants and event callback forward declaration. The
// constants are referenced from uac_lib_task's TX_CONNECTED handler
// (which now pre-opens the speaker on enum) as well as from the
// pump task itself further down in the file.
// Speaker ringbuffer must hold one full pump write atomically (push
// is all-or-none in xRingbuffer). All speaker resources are pre-
// allocated at TX_CONNECTED enum time (handle, ringbuffer, xfer URBs,
// pump buffer, writer task) so per-TX is allocation-free.
//   ringbuffer: 4 KB  (~14 ms of audio at 288 B/ms)
//   pump:       1.7 KB (6 packets × 288 B) — must fit in ringbuffer
//   stack:      3 KB  for the writer task
static const uint32_t SPK_BUFFER_SIZE      = 4000;      // driver ringbuffer
static const uint32_t SPK_BUFFER_THRESHOLD = 600;       // ~2 ms at 48k/24/stereo
static const uint32_t SPK_WRITE_TIMEOUT_MS = 200;
static const uint32_t SPK_PACKET_BYTES     = 288;       // 48 stereo frames × 6 B
static const uint32_t SPK_PUMP_PACKETS     = 6;         // 6 packets per write = 1728 B
static const uint32_t SPK_PUMP_BYTES       = SPK_PACKET_BYTES * SPK_PUMP_PACKETS;  // 1728
// Negotiated speaker format. QMX/QDX always use 24-bit stereo; the FX-4CR's
// C-Media codec is 16-bit. SPK_PUMP_BYTES divides evenly by 2, 3, 4 and 6.
static uint8_t s_spk_bits = 24;
static uint8_t s_spk_channels = 2;
// FX-4CR (see the channel budget above): the speaker is only opened at
// enumeration and claimed per TX. Mic stopped + speaker started at TX
// begin, the reverse at TX end.
static bool s_spk_swap_mode = false;
static bool s_spk_fmt_known = false;
static volatile bool s_mic_paused = false;
static volatile bool s_mic_restart_needed = false;  // a restart failed; retry
static void mic_resume_after_tx(void);
static void spk_event_cb(uac_host_device_handle_t dev,
                         const uac_host_device_event_t event,
                         void* arg);
static void spk_writer_task(void* arg);
static void cdc_new_dev_cb(usb_device_handle_t usb_dev);
static int uac_read_ft8_samples(void* ctx, float* out, int max_samples);
static bool uac_should_stop(void* ctx);
static void uac_on_block_processed(void* ctx);

static const char* profile_name(uac_stream_profile_t profile) {
    switch (profile) {
    case UAC_PROFILE_QMX:
        return "qmx_uac";
    case UAC_PROFILE_GENERIC_USB:
        return "usb_uac_generic";
    case UAC_PROFILE_FX4CR:
        return "fx4cr_uac";
    default:
        return "unknown";
    }
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

// Profiles whose radio exposes CAT and accepts TX audio over USB.
static bool profile_has_cat_and_tx_audio(uac_stream_profile_t profile) {
    return profile == UAC_PROFILE_QMX || profile == UAC_PROFILE_FX4CR;
}

// Kenwood CAT on the FX-4CR bridge: 115200 8N1. RTS is wired to PTT in the
// F5BUD firmware ("RTS as PTT"), so both modem lines must stay deasserted.
static void cdc_configure_fx4cr_line(cdc_acm_dev_hdl_t handle) {
    cdc_acm_line_coding_t coding = {};
    coding.dwDTERate = k_fx4cr_cat_baud;
    coding.bCharFormat = 0;  // 1 stop bit
    coding.bParityType = 0;  // none
    coding.bDataBits = 8;
    esp_err_t err = cdc_acm_host_line_coding_set(handle, &coding);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "FX-4CR CAT line coding failed: %s", esp_err_to_name(err));
    }
    err = cdc_acm_host_set_control_line_state(handle, false, false);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "FX-4CR CAT DTR/RTS clear failed: %s", esp_err_to_name(err));
    }
}

static bool is_legacy_ch34x_pid(uint16_t pid) {
    // CH340/CH341 use vendor requests for baud rate. Newer WCH parts with the
    // same VID (e.g. CH9102, PID 0x55D4) are CDC-ACM compliant instead.
    return pid == CH340_PID || pid == CH340_PID_1 || pid == CH341_PID;
}

// FX-4CR fallback for non-CH340 bridges (CDC-ACM, CP210x, CH9102): open the
// bridge seen during enumeration through the CDC driver.
static esp_err_t fx_bridge_open(const cdc_acm_host_device_config_t* dev_cfg,
                                cdc_acm_dev_hdl_t* out) {
    if (s_vcp_vid == 0) return ESP_ERR_NOT_FOUND;
    if (s_vcp_vid == NANJING_QINHENG_MICROE_VID && is_legacy_ch34x_pid(s_vcp_pid)) {
        return ch34x_vcp_open(s_vcp_pid, 0, dev_cfg, out);
    }
    if (s_vcp_vid == SILICON_LABS_VID) {
        return cp210x_vcp_open(s_vcp_pid, 0, dev_cfg, out);
    }
    return cdc_acm_host_open(s_vcp_vid, s_vcp_pid, (uint8_t)s_vcp_iface, dev_cfg, out);
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

    if (s_profile == UAC_PROFILE_FX4CR) {
        // The bridge is opened per command batch (uac_cat_batch), never held.
        // Never use CDC_HOST_ANY_* here: behind the hub it can match the codec.
        if (s_vcp_vid != 0 && !s_fx_cat_announced) {
            s_fx_cat_announced = true;
            snprintf(s_cat_status, sizeof(s_cat_status), "CAT %04x:%04x", s_vcp_vid, s_vcp_pid);
            g_cdc_initial_sync_pending = true;  // main loop syncs band + mode
        }
        return;
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

// ---------------------------------------------------------------------------
// FX-4CR: write-only CH340 CAT
//
// The CH340 interface has bulk IN, bulk OUT and interrupt IN; a normal claim
// would need 3 channels with only 1 free. CAT only needs bulk OUT: with the
// patched usb component (components/espressif__usb) the claim allocates just
// that endpoint. The CDC driver cannot be used (it requires the interrupt
// endpoint), so this is a minimal driver on the usb_host client API.
// ---------------------------------------------------------------------------
static volatile bool s_ep_filter_out_only = false;
static usb_device_handle_t s_ep_filter_dev = NULL;
static uint8_t s_vcp_addr = 0;

// C linkage: overrides the weak default in the patched usb component.
extern "C" bool usb_host_claim_ep_filter(usb_device_handle_t dev_hdl, const usb_ep_desc_t* ep_desc) {
    if (!s_ep_filter_out_only || dev_hdl != s_ep_filter_dev || !ep_desc) return true;
    return USB_EP_DESC_GET_EP_DIR(ep_desc) == 0 &&
           USB_EP_DESC_GET_XFERTYPE(ep_desc) == USB_TRANSFER_TYPE_BULK;
}

static void fx_xfer_cb(usb_transfer_t* transfer) {
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(transfer->context));
}

static void fx_client_event_cb(const usb_host_client_event_msg_t* msg, void* arg) {
    (void)msg;
    (void)arg;
}

// Submit and wait, pumping this client's events (callbacks run there).
static esp_err_t fx_run_xfer(usb_host_client_handle_t client, usb_transfer_t* xfer, bool control) {
    SemaphoreHandle_t done = static_cast<SemaphoreHandle_t>(xfer->context);
    esp_err_t err = control ? usb_host_transfer_submit_control(client, xfer)
                            : usb_host_transfer_submit(xfer);
    if (err != ESP_OK) return err;
    for (int i = 0; i < 100; ++i) {
        usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
        if (xSemaphoreTake(done, 0) == pdTRUE) {
            return xfer->status == USB_TRANSFER_STATUS_COMPLETED ? ESP_OK : ESP_FAIL;
        }
    }
    // Still in flight: keep pumping until the stack hands it back, so the
    // transfer can be freed safely.
    while (xSemaphoreTake(done, 0) != pdTRUE) {
        usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t fx_ch340_vendor_write(usb_host_client_handle_t client, usb_device_handle_t dev,
                                       usb_transfer_t* ctrl, uint8_t request,
                                       uint16_t value, uint16_t index) {
    usb_setup_packet_t* setup = reinterpret_cast<usb_setup_packet_t*>(ctrl->data_buffer);
    setup->bmRequestType = USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_RECIP_DEVICE |
                           USB_BM_REQUEST_TYPE_DIR_OUT;
    setup->bRequest = request;
    setup->wValue = value;
    setup->wIndex = index;
    setup->wLength = 0;
    ctrl->num_bytes = sizeof(usb_setup_packet_t);
    ctrl->device_handle = dev;
    ctrl->bEndpointAddress = 0;
    return fx_run_xfer(client, ctrl, true);
}

static esp_err_t fx_ch340_write(const char* const* cmds, size_t count) {
    if (s_vcp_addr == 0) return ESP_ERR_NOT_FOUND;
    usb_host_client_handle_t client = NULL;
    usb_host_client_config_t ccfg = {};
    ccfg.is_synchronous = false;
    ccfg.max_num_event_msg = 5;
    ccfg.async.client_event_callback = fx_client_event_cb;
    ccfg.async.callback_arg = NULL;
    esp_err_t err = usb_host_client_register(&ccfg, &client);
    if (err != ESP_OK) return err;

    usb_device_handle_t dev = NULL;
    usb_transfer_t* ctrl = NULL;
    usb_transfer_t* bulk = NULL;
    SemaphoreHandle_t done = xSemaphoreCreateBinary();
    bool claimed = false;
    uint8_t out_ep = 0;
    const usb_config_desc_t* cfg_desc = NULL;

    err = done ? usb_host_device_open(client, s_vcp_addr, &dev) : ESP_ERR_NO_MEM;
    if (err == ESP_OK) err = usb_host_get_active_config_descriptor(dev, &cfg_desc);
    if (err == ESP_OK) {
        int offset = 0;
        const usb_intf_desc_t* intf = usb_parse_interface_descriptor(cfg_desc, 0, 0, &offset);
        for (int i = 0; intf && i < intf->bNumEndpoints && out_ep == 0; ++i) {
            int ep_offset = offset;
            const usb_ep_desc_t* ep = usb_parse_endpoint_descriptor_by_index(
                intf, i, cfg_desc->wTotalLength, &ep_offset);
            if (ep && USB_EP_DESC_GET_EP_DIR(ep) == 0 &&
                USB_EP_DESC_GET_XFERTYPE(ep) == USB_TRANSFER_TYPE_BULK) {
                out_ep = ep->bEndpointAddress;
            }
        }
        if (out_ep == 0) err = ESP_ERR_NOT_FOUND;
    }
    if (err == ESP_OK) {
        s_ep_filter_dev = dev;
        s_ep_filter_out_only = true;
        err = usb_host_interface_claim(client, dev, 0, 0);
        s_ep_filter_out_only = false;
        claimed = (err == ESP_OK);
    }
    if (err == ESP_OK) err = usb_host_transfer_alloc(64, 0, &ctrl);
    if (err == ESP_OK) err = usb_host_transfer_alloc(64, 0, &bulk);
    if (err == ESP_OK) {
        ctrl->callback = fx_xfer_cb;
        ctrl->context = done;
        ctrl->timeout_ms = 500;
        bulk->callback = fx_xfer_cb;
        bulk->context = done;
        bulk->timeout_ms = 500;
        bulk->device_handle = dev;
        bulk->bEndpointAddress = out_ep;
        // 115200 8N1 (values from the espressif CH34x driver's divisor math),
        // then DTR/RTS inactive: RTS is PTT on the F5BUD firmware.
        err = fx_ch340_vendor_write(client, dev, ctrl, 0x9A, 0x1312, 0xCC83);
        if (err == ESP_OK) err = fx_ch340_vendor_write(client, dev, ctrl, 0x9A, 0x2518, 0x00C3);
        if (err == ESP_OK) err = fx_ch340_vendor_write(client, dev, ctrl, 0xA4, 0x0060, 0);
    }
    // Reprogramming the UART can put a glitch byte on the line, which would
    // make the radio reject the first command. Let the line settle, then
    // send ';' to flush the radio's command parser.
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(20));
        bulk->data_buffer[0] = ';';
        bulk->num_bytes = 1;
        err = fx_run_xfer(client, bulk, false);
    }
    for (size_t i = 0; err == ESP_OK && i < count; ++i) {
        const size_t len = strlen(cmds[i]);
        if (len > bulk->data_buffer_size) {
            err = ESP_ERR_INVALID_SIZE;
            break;
        }
        // Give the radio time to act on each command: right after MD/DA it
        // drops a TX1; that follows immediately.
        if (i > 0) vTaskDelay(pdMS_TO_TICKS(15));
        memcpy(bulk->data_buffer, cmds[i], len);
        bulk->num_bytes = (int)len;
        err = fx_run_xfer(client, bulk, false);
    }
    if (err == ESP_OK) vTaskDelay(pdMS_TO_TICKS(10));  // UART drains the last bytes

    if (ctrl) usb_host_transfer_free(ctrl);
    if (bulk) usb_host_transfer_free(bulk);
    if (claimed) usb_host_interface_release(client, dev, 0);
    if (dev) usb_host_device_close(client, dev);
    usb_host_client_deregister(client);
    if (done) vSemaphoreDelete(done);
    return err;
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

    bool has_audio = false;
    int comm_iface = -1;
    int vendor_iface = -1;

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
                if (intf->bInterfaceClass == USB_CLASS_AUDIO) has_audio = true;
                if (intf->bInterfaceClass == USB_CLASS_COMM && comm_iface < 0) {
                    comm_iface = intf->bInterfaceNumber;
                }
                if (intf->bInterfaceClass == USB_CLASS_VENDOR_SPEC && vendor_iface < 0) {
                    vendor_iface = intf->bInterfaceNumber;
                }
            }
            offset += len;
        }
    }

    // Remember the serial bridge: not the codec, not a hub, and either a known
    // bridge VID or a CDC / vendor-specific interface.
    if (dev_desc && !has_audio && dev_desc->bDeviceClass != USB_CLASS_HUB) {
        const bool known_vid = dev_desc->idVendor == NANJING_QINHENG_MICROE_VID ||
                               dev_desc->idVendor == SILICON_LABS_VID;
        if (known_vid || comm_iface >= 0 || vendor_iface >= 0) {
            s_vcp_vid = dev_desc->idVendor;
            s_vcp_pid = dev_desc->idProduct;
            s_vcp_addr = info.dev_addr;
            s_vcp_iface = comm_iface >= 0 ? comm_iface : (vendor_iface >= 0 ? vendor_iface : 0);
            snprintf(s_cat_status, sizeof(s_cat_status), "CAT seen %04x:%04x",
                     s_vcp_vid, s_vcp_pid);
            ESP_LOGI(TAG, "  -> USB-serial bridge candidate iface %d", s_vcp_iface);
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

    if (profile_has_cat_and_tx_audio(s_profile)) {
        // Custom FIFO partitioning enables simultaneous bidirectional ISO
        // streaming with QDX or QMX hardware. ESP32-S3 has 200 FIFO lines.
        // Built-in Kconfig biases do not cover 24-bit/48k/stereo MPS=300
        // in both directions at once. This split reserves 364 B for RX,
        // 364 B for PTX, and 72 B for non-periodic OUT (CDC CAT bulk-OUT).
        host_config.fifo_settings_custom.rx_fifo_lines   = 91;   // 364 B
        host_config.fifo_settings_custom.nptx_fifo_lines = 18;   // 72 B
        host_config.fifo_settings_custom.ptx_fifo_lines  = 91;   // 364 B
        ESP_LOGI(TAG, "USB FIFO profile=%s policy=qmx-custom rx=91 nptx=18 ptx=91",
                 profile_name(s_profile));
    } else {
        ESP_LOGI(TAG, "USB FIFO profile=%s policy=default", profile_name(s_profile));
    }

    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB host: %s", esp_err_to_name(err));
        s_state = UAC_STATE_ERROR;
        snprintf(s_status_string, sizeof(s_status_string), "USB init failed");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "USB Host installed");

    if (profile_has_cat_and_tx_audio(s_profile)) {
        // Install CDC-ACM driver for QMX/QDX/FX-4CR CAT control before starting class drivers.
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
    } else {
        ESP_LOGI(TAG, "CDC-ACM driver skipped profile=%s", profile_name(s_profile));
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

    // Drain pending USB host events before uninstall. usb_host_uninstall()
    // refuses to release the PHY while client-detach/all-free events are
    // still pending, which blocks the subsequent TinyUSB device-mode MSC path.
    for (int i = 0; i < 50; ++i) {
        uint32_t event_flags = 0;
        usb_host_lib_handle_events(pdMS_TO_TICKS(20), &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            break;
        }
    }

    ESP_LOGI(TAG, "USB Host uninstalling");
    esp_err_t uerr = usb_host_uninstall();
    if (uerr != ESP_OK) {
        ESP_LOGW(TAG, "usb_host_uninstall: %s", esp_err_to_name(uerr));
    }
    s_usb_task_handle = NULL;
    vTaskDelete(NULL);
}

// FX-4CR: the radio's CSR Bluetooth module (0a12:4007) is also a UAC
// device. Only the C-Media codec carries the radio's USB audio.
static bool is_fx4cr_bluetooth_audio(uac_host_device_handle_t handle) {
    if (s_profile != UAC_PROFILE_FX4CR || !handle) return false;
    uac_host_dev_info_t info = {};
    if (uac_host_get_device_info(handle, &info) != ESP_OK) return false;
    return info.VID == 0x0A12;
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
                    if (is_fx4cr_bluetooth_audio(handle)) {
                        ESP_LOGI(TAG, "Ignoring Bluetooth module mic addr:%d", evt.driver.addr);
                        uac_host_device_close(handle);
                        continue;
                    }

                    // Print device info
                    uac_host_printf_device_param(handle);

                    // Start stream format negotiation.
                    // QMX profile keeps the existing strict format.
                    // Generic profile stays 48 kHz and tries mic-only
                    // mono/stereo 24-bit and 16-bit candidates.
                    uac_host_stream_config_t candidates[4];
                    int candidate_count = 0;
                    auto add_candidate = [&](uint8_t ch, uint8_t bits) {
                        candidates[candidate_count].channels = ch;
                        candidates[candidate_count].bit_resolution = bits;
                        candidates[candidate_count].sample_freq = UAC_SAMPLE_RATE;
                        candidates[candidate_count].flags = 0;
                        candidate_count++;
                    };
                    if (s_profile == UAC_PROFILE_GENERIC_USB ||
                        s_profile == UAC_PROFILE_FX4CR) {
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
                        if (s_profile != UAC_PROFILE_QMX) {
                            ESP_LOGE(TAG, "No 48k UAC mic format for profile=%s",
                                     profile_name(s_profile));
                            snprintf(s_status_string, sizeof(s_status_string), "No 48k UAC mic format");
                        } else {
                            ESP_LOGE(TAG, "Failed to start stream for profile=%s",
                                     profile_name(s_profile));
                            snprintf(s_status_string, sizeof(s_status_string), "Format not supported");
                        }
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

                    if (profile_has_cat_and_tx_audio(s_profile)) {
                        // Try to open companion CAT serial interface.
                        cdc_try_open();
                    }

                    // Start the audio processing task.
                    //
                    // FT4's decoder needs a larger stack than FT8. Keep the
                    // FT8 stack static and allocate the FT4 stack on demand.
                    if (s_stream_task_handle == NULL) {
#if ENABLE_FT4
                        if (g_protocol == &kProtocolFT4) {
                            BaseType_t cr = xTaskCreatePinnedToCore(
                                stream_uac_task, "stream_uac",
                                16384, NULL,
                                UAC_STREAM_TASK_PRIORITY,
                                &s_stream_task_handle, 1);
                            if (cr != pdPASS) {
                                ESP_LOGE(TAG, "stream_uac_task create FAILED (dynamic FT4) free=%u",
                                         (unsigned)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
                                s_stream_task_handle = NULL;
                            }
                        } else {
#endif
                        // FT8 uses the existing static 8 KB stack.
                        static StackType_t  s_stream_task_stack[8192 / sizeof(StackType_t)];
                        static StaticTask_t s_stream_task_tcb;
                        size_t free_before = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
                        size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
                        ESP_LOGI(TAG, "Pre-task-create heap: free=%u largest=%u",
                                 (unsigned)free_before, (unsigned)largest);
                        s_stream_task_handle = xTaskCreateStaticPinnedToCore(
                            stream_uac_task, "stream_uac",
                            8192 / sizeof(StackType_t), NULL,
                            UAC_STREAM_TASK_PRIORITY,
                            s_stream_task_stack, &s_stream_task_tcb, 1);
                        if (!s_stream_task_handle) {
                            ESP_LOGE(TAG, "stream_uac_task create FAILED "
                                     "(static FT8) free=%u largest=%u",
                                     (unsigned)free_before, (unsigned)largest);
                        }
#if ENABLE_FT4
                        }
#endif
                    }

                } else if (evt.driver.event == UAC_HOST_DRIVER_EVENT_TX_CONNECTED) {
                    if (!profile_has_cat_and_tx_audio(s_profile)) {
                        ESP_LOGI(TAG, "Speaker connected ignored profile=%s addr=%u iface=%u",
                                 profile_name(s_profile),
                                 (unsigned)evt.driver.addr,
                                 (unsigned)evt.driver.iface_num);
                        continue;
                    }

                    if (s_profile == UAC_PROFILE_FX4CR && s_spk_known) {
                        // e.g. the FX-4CR Bluetooth module's speaker
                        ESP_LOGI(TAG, "Additional speaker ignored addr=%u",
                                 (unsigned)evt.driver.addr);
                        continue;
                    }
                    s_spk_addr  = evt.driver.addr;
                    s_spk_iface = evt.driver.iface_num;
                    s_spk_known = true;
                    ESP_LOGI(TAG, "Speaker captured: addr=%u iface=%u",
                             (unsigned)s_spk_addr, (unsigned)s_spk_iface);
                    // Open and claim the speaker while enumeration-time heap
                    // is contiguous, then hold it across all TX cycles.
                    if (!s_spk_handle) {
                        uac_host_device_config_t cfg = {};
                        uac_host_device_handle_t probe = NULL;
                        cfg.addr = s_spk_addr;
                        cfg.iface_num = s_spk_iface;
                        cfg.buffer_size = SPK_BUFFER_SIZE;
                        cfg.buffer_threshold = SPK_BUFFER_THRESHOLD;
                        cfg.callback = spk_event_cb;
                        cfg.callback_arg = nullptr;
                        esp_err_t oerr = uac_host_device_open(&cfg, &probe);
                        if (oerr == ESP_OK && is_fx4cr_bluetooth_audio(probe)) {
                            ESP_LOGI(TAG, "Ignoring Bluetooth module speaker addr=%u",
                                     (unsigned)s_spk_addr);
                            uac_host_device_close(probe);
                            s_spk_known = false;
                            continue;
                        }
                        s_spk_handle = probe;
                        if (oerr != ESP_OK) {
                            ESP_LOGE(TAG, "spk pre-open failed at enum: %s",
                                     esp_err_to_name(oerr));
                            s_spk_handle = NULL;
                        } else {
                            // Claim iface + alloc xfer URBs + EP slot now,
                            // but pass FLAG_STREAM_SUSPEND_AFTER_START so
                            // no URBs are queued on the bus yet. The
                            // 512-byte aligned xfer_desc_list (2x 512B
                            // for ISOC double-buffering) requires fresh
                            // heap to satisfy alignment — at TX time the
                            // largest contiguous block is <2 KB and
                            // aligned alloc fails. Resume happens at TX
                            // via uac_host_device_resume (cheap, no
                            // realloc). Suspend at TX-end stops URB flow
                            // but keeps everything allocated.
                            // QMX/QDX: 24-bit stereo only. FX-4CR (C-Media):
                            // usually 16-bit stereo; try the common layouts.
                            static const uint8_t k_spk_fmt_qmx[][2] = {{2, 24}};
                            static const uint8_t k_spk_fmt_fx4cr[][2] = {
                                {2, 16}, {1, 16}, {2, 24}, {1, 24}};
                            const bool fx = (s_profile == UAC_PROFILE_FX4CR);
                            const uint8_t (*fmts)[2] = fx ? k_spk_fmt_fx4cr : k_spk_fmt_qmx;
                            const int fmt_count = fx ? 4 : 1;
                            esp_err_t serr = ESP_FAIL;
                            if (s_spk_swap_mode) {
                                // Opened only; claimed per TX (see s_spk_swap_mode).
                                serr = ESP_OK;
                            }
                            for (int f = 0; f < fmt_count && serr != ESP_OK; ++f) {
                                uac_host_stream_config_t scfg = {};
                                scfg.channels = fmts[f][0];
                                scfg.bit_resolution = fmts[f][1];
                                scfg.sample_freq = 48000;
                                scfg.flags = FLAG_STREAM_SUSPEND_AFTER_START;
                                serr = uac_host_device_start(s_spk_handle, &scfg);
                                if (serr == ESP_OK) {
                                    s_spk_channels = scfg.channels;
                                    s_spk_bits = scfg.bit_resolution;
                                    ESP_LOGI(TAG, "Speaker format 48k/%u-bit/%uch",
                                             (unsigned)s_spk_bits, (unsigned)s_spk_channels);
                                }
                            }
                            if (serr != ESP_OK) {
                                ESP_LOGE(TAG, "spk pre-start failed at enum: %s",
                                         esp_err_to_name(serr));
                                uac_host_device_close(s_spk_handle);
                                s_spk_handle = NULL;
                            } else {
                                ESP_LOGI(TAG, "Speaker pre-opened%s (handle=%p, ringbuf=%u B)",
                                         s_spk_swap_mode ? ", claimed per TX" : "+claimed, suspended",
                                         s_spk_handle, (unsigned)SPK_BUFFER_SIZE);
                                // Allocate the pump buffer + permanent
                                // writer task NOW too, while heap is
                                // healthy. Per-TX cost is just flipping
                                // s_spk_writer_run — no allocs.
                                if (!s_spk_pump_buf) {
                                    s_spk_pump_buf = (uint8_t*)heap_caps_malloc(
                                        SPK_PUMP_BYTES, MALLOC_CAP_DEFAULT);
                                    if (!s_spk_pump_buf) {
                                        ESP_LOGE(TAG, "spk pump buf alloc failed at enum");
                                    }
                                }
                                if (s_spk_pump_buf && !s_spk_writer_task) {
                                    s_spk_writer_stop = false;
                                    s_spk_writer_run = false;
                                    BaseType_t ok = xTaskCreatePinnedToCore(
                                        spk_writer_task, "uac_tx_writer",
                                        3072, NULL, 5, &s_spk_writer_task, 1);
                                    if (ok != pdPASS) {
                                        ESP_LOGE(TAG, "spk writer task create failed at enum");
                                        s_spk_writer_task = NULL;
                                    } else {
                                        ESP_LOGI(TAG, "Speaker writer task ready (idle)");
                                    }
                                }
                            }
                        }
                    }
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
    // Stop the permanent writer task so it exits its loop and frees
    // its TCB before we tear down the stream / pump buffer.
    if (s_spk_writer_task) {
        s_spk_writer_run = false;
        s_spk_writer_stop = true;
        for (int i = 0; i < 50 && s_spk_writer_task; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    if (s_spk_pump_buf) {
        free(s_spk_pump_buf);
        s_spk_pump_buf = nullptr;
    }
    if (s_spk_handle) {
        // Speaker was opened + claimed (suspended) at TX_CONNECTED
        // enum and held for the lifetime of the host driver. Tear it
        // down here before uac_host_uninstall() to release xfer URBs
        // and ringbuffer cleanly.
        uac_host_device_stop(s_spk_handle);
        uac_host_device_close(s_spk_handle);
        s_spk_handle = NULL;
        s_spk_known = false;
    }

    ESP_LOGI(TAG, "UAC driver uninstalling");
    uac_host_uninstall();
    s_uac_task_handle = NULL;
    vTaskDelete(NULL);
}

// Audio streaming and processing task
static void stream_uac_task(void* arg) {
    (void)arg;
    ESP_LOGI(TAG, "Audio streaming task started");

    resample_init(&s_resample_state);
    uint8_t* usb_buffer = s_usb_buffer;

    ft8_audio_pipeline_config_t pipe_cfg = {
        .tag = TAG,
        .ctx = usb_buffer,
        .read = uac_read_ft8_samples,
        .should_stop = uac_should_stop,
        .on_block_processed = uac_on_block_processed,
    };
    ft8_audio_pipeline_run(&pipe_cfg);

    g_streaming = false;
    s_stream_task_handle = NULL;
    ESP_LOGI(TAG, "Audio streaming task stopped");
    vTaskDelete(NULL);
}

static int uac_read_ft8_samples(void* ctx, float* out, int max_samples) {
    (void)max_samples;
    uint8_t* usb_buffer = (uint8_t*)ctx;
    if (!usb_buffer || !out) return 0;

    while (!s_stop_requested && s_mic_handle != NULL) {
        if (s_mic_paused) {
            vTaskDelay(pdMS_TO_TICKS(20));  // FX-4CR TX: mic released
            continue;
        }
        uint32_t bytes_read = 0;
        esp_err_t ret = uac_host_device_read(s_mic_handle, usb_buffer,
                                              UAC_READ_BUFFER_SIZE,
                                              &bytes_read,
                                              pdMS_TO_TICKS(200));
        if (ret != ESP_OK || bytes_read == 0) {
            if (ret == ESP_ERR_INVALID_STATE) {
                vTaskDelay(pdMS_TO_TICKS(20));  // stream not active (FX-4CR TX/CAT)
                // Safety net: a failed restart must not freeze RX for good.
                static int64_t last_retry_ms = 0;
                const int64_t now = rtc_now_ms();
                if (s_mic_restart_needed && !s_spk_writer_run && now - last_retry_ms > 1000 &&
                    s_cat_lock && xSemaphoreTake(s_cat_lock, 0) == pdTRUE) {
                    last_retry_ms = now;
                    s_mic_paused = true;  // mic_resume_after_tx() acts only when paused
                    mic_resume_after_tx();
                    xSemaphoreGive(s_cat_lock);
                }
            } else if (ret != ESP_ERR_TIMEOUT && ret != ESP_FAIL) {
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
        if (num_frames == 0) continue;

        int32_t val = 0;
        if (s_format.bit_resolution == 24) {
            val = usb_buffer[0] | (usb_buffer[1] << 8) | (usb_buffer[2] << 16);
            if (val & 0x800000) val |= 0xFF000000;
        } else {
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

        return uac_pcm_to_ft8_samples(&s_resample_state, usb_buffer,
                                      (int)bytes_read, out,
                                      s_format.bit_resolution,
                                      s_format.channels);
    }
    return 0;
}

static bool uac_should_stop(void* ctx) {
    (void)ctx;
    return s_stop_requested || s_mic_handle == NULL;
}

static void uac_on_block_processed(void* ctx) {
    (void)ctx;
    if (!s_cdc_handle) {
        cdc_try_open();
    }
}

// Public API implementation
uac_stream_state_t uac_get_state(void) {
    return s_state;
}

bool uac_is_streaming(void) {
    return s_state == UAC_STATE_STREAMING && s_mic_handle != NULL;
}

bool uac_get_latest_waterfall_row(uint8_t* out_row, int out_len) {
    return ft8_audio_pipeline_get_latest_waterfall_row(out_row, out_len);
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
    s_vcp_vid = 0;
    s_vcp_pid = 0;
    s_vcp_iface = 0;
    s_vcp_addr = 0;
    s_fx_cat_announced = false;
    s_cat_fail_ms = 0;
    s_cat_fail_count = 0;
    if (!s_cat_lock) s_cat_lock = xSemaphoreCreateMutex();
    snprintf(s_cat_status, sizeof(s_cat_status), "CAT none");
    s_spk_bits = 24;
    s_spk_channels = 2;
    s_spk_swap_mode = (profile == UAC_PROFILE_FX4CR);
    s_spk_fmt_known = false;
    s_mic_paused = false;
    s_mic_restart_needed = false;
    s_format.sample_freq = UAC_SAMPLE_RATE;
    s_format.bit_resolution = UAC_BIT_RESOLUTION;
    s_format.channels = UAC_CHANNELS;
    ft8_audio_pipeline_clear_latest_waterfall_row();

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
    ft8_audio_pipeline_clear_latest_waterfall_row();
    snprintf(s_status_string, sizeof(s_status_string), "Idle");
    ESP_LOGI(TAG, "UAC host stopped");
}

// UAC OUT synthesis for QDX mode.

static void spk_event_cb(uac_host_device_handle_t /*dev*/,
                         const uac_host_device_event_t event,
                         void* /*arg*/) {
    if (event == UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR) {
        s_spk_write_errors++;
        ESP_LOGW(TAG, "spk transfer error");
    }
}

static void spk_writer_task(void* /*arg*/) {
    // Permanent task, gated per transmission. The DDS owns symbol timing.
    while (!s_spk_writer_stop) {
        if (!s_spk_writer_run) {
            // Idle between TX cycles. Sleep cheaply.
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        // Read per block: in swap mode the format is negotiated at TX start.
        const uint32_t frame_bytes = (s_spk_bits / 8u) * s_spk_channels;
        const uint32_t frames_per_block = SPK_PUMP_BYTES / frame_bytes;
        dds_render_pcm(s_spk_pump_buf, frames_per_block, s_spk_bits, s_spk_channels);
        esp_err_t err = uac_host_device_write(s_spk_handle, s_spk_pump_buf, SPK_PUMP_BYTES,
                                              pdMS_TO_TICKS(SPK_WRITE_TIMEOUT_MS));
        if (err == ESP_OK) {
            s_spk_packets_sent += SPK_PUMP_PACKETS;
        } else if (err != ESP_ERR_TIMEOUT) {
            s_spk_write_errors++;
            if (s_spk_write_errors <= 5) {
                ESP_LOGW(TAG, "spk write err=%s", esp_err_to_name(err));
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        // Yield so IDLE0 can run (watchdog appeasement).
        vTaskDelay(1);
    }

    s_spk_writer_task = NULL;
    vTaskDelete(NULL);
}

// Stop a UAC stream and really release its interface (= its host channel).
// uac_host_device_stop() suspends, then releases. The release is refused
// (ESP_ERR_INVALID_STATE) while the just-cancelled ISO transfers are still
// in flight / unprocessed by the UAC client task, leaving the channel
// claimed (field logs, mic and speaker alike). Retry: once suspended, a
// further call only releases, which succeeds after the transfers drain.
static esp_err_t uac_stop_and_release(uac_host_device_handle_t handle, const char* what) {
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < 10; ++attempt) {
        err = uac_host_device_stop(handle);
        if (err == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s stop failed: %s", what, esp_err_to_name(err));
    }
    return err;
}

// Stop the mic stream to free its host channel (FX-4CR TX / CAT batch).
static void mic_pause_for_usb(void) {
    if (s_mic_paused || !s_mic_handle) return;
    s_mic_paused = true;
    uac_stop_and_release(s_mic_handle, "mic");
}

static void mic_resume_after_tx(void) {
    if (!s_mic_paused) return;
    if (s_mic_handle) {
        uac_host_stream_config_t mcfg = {};
        mcfg.channels = s_format.channels;
        mcfg.bit_resolution = s_format.bit_resolution;
        mcfg.sample_freq = s_format.sample_freq;
        mcfg.flags = 0;
        esp_err_t err = uac_host_device_start(s_mic_handle, &mcfg);
        // start() returns OK without streaming if a failed stop left the
        // interface claimed but suspended (READY); resume() covers that and
        // is a no-op when already ACTIVE.
        if (err == ESP_OK) err = uac_host_device_resume(s_mic_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "mic restart after TX failed: %s", esp_err_to_name(err));
        }
        s_mic_restart_needed = (err != ESP_OK);  // read loop retries
    }
    s_mic_paused = false;
}

// Swap mode TX start: release the mic's channel, then claim the speaker.
static bool spk_swap_start(void) {
    mic_pause_for_usb();  // may already be paused by the TX; CAT batch

    static const uint8_t k_fmts[][2] = {{2, 16}, {1, 16}, {2, 24}, {1, 24}};
    esp_err_t serr = ESP_FAIL;
    // Last good format first, then the common C-Media layouts.
    for (int f = -1; f < 4 && serr != ESP_OK; ++f) {
        if (f == -1 && !s_spk_fmt_known) continue;
        uac_host_stream_config_t scfg = {};
        scfg.channels = (f == -1) ? s_spk_channels : k_fmts[f][0];
        scfg.bit_resolution = (f == -1) ? s_spk_bits : k_fmts[f][1];
        scfg.sample_freq = 48000;
        scfg.flags = 0;
        serr = uac_host_device_start(s_spk_handle, &scfg);
        if (serr == ESP_OK) {
            s_spk_channels = scfg.channels;
            s_spk_bits = scfg.bit_resolution;
            if (!s_spk_fmt_known) {
                ESP_LOGI(TAG, "Speaker format 48k/%u-bit/%uch",
                         (unsigned)s_spk_bits, (unsigned)s_spk_channels);
            }
            s_spk_fmt_known = true;
        }
    }
    if (serr != ESP_OK) {
        ESP_LOGE(TAG, "UAC OUT start failed: %s", esp_err_to_name(serr));
        mic_resume_after_tx();
        return false;
    }
    s_spk_writer_run = true;
    return true;
}

static bool uac_tx_start_writer(void) {
    if (s_spk_writer_run) {
        ESP_LOGW(TAG, "UAC OUT already active");
        return false;
    }
    if (!s_spk_known || !s_spk_handle || !s_spk_writer_task) {
        ESP_LOGW(TAG, "UAC OUT speaker not ready");
        return false;
    }

    s_spk_packets_sent = 0;
    s_spk_write_errors = 0;

    if (s_spk_swap_mode) {
        return spk_swap_start();
    }

    esp_err_t err = uac_host_device_resume(s_spk_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UAC OUT resume failed: %s", esp_err_to_name(err));
        return false;
    }

    s_spk_writer_run = true;
    return true;
}

bool uac_tx_begin_cpfsk(float base_hz,
                        const uint8_t* symbols,
                        size_t symbol_count,
                        float tone_spacing_hz,
                        uint32_t samples_per_symbol) {
    if (!dds_cpfsk_begin(static_cast<double>(base_hz), symbols, symbol_count,
                         static_cast<double>(tone_spacing_hz),
                         samples_per_symbol)) {
        ESP_LOGE(TAG, "Invalid CPFSK schedule count=%u samples/symbol=%u",
                 (unsigned)symbol_count, (unsigned)samples_per_symbol);
        return false;
    }

    if (!uac_tx_start_writer()) {
        dds_cpfsk_end();
        return false;
    }

    ESP_LOGI(TAG,
             "UAC OUT CPFSK start base=%.2f count=%u spacing=%.4f samples/symbol=%u",
             (double)base_hz, (unsigned)symbol_count, (double)tone_spacing_hz,
             (unsigned)samples_per_symbol);
    return true;
}

bool uac_tx_begin_tune(float tone_hz) {
    dds_cpfsk_end();
    dds_reset_phase();
    dds_set_freq_hz(static_cast<double>(tone_hz));
    if (!uac_tx_start_writer()) {
        dds_set_freq_hz(0.0);
        return false;
    }

    ESP_LOGI(TAG, "UAC OUT tune start tone=%.2f", (double)tone_hz);
    return true;
}

void uac_tx_end(void) {
    uac_tx_end_ex(true);
}

void uac_tx_end_ex(bool resume_rx) {
    if (!s_spk_writer_run) {
        dds_cpfsk_end();
        dds_set_freq_hz(0.0);
        return;
    }

    s_spk_writer_run = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    dds_cpfsk_end();
    dds_set_freq_hz(0.0);

    if (s_spk_handle) {
        if (s_spk_swap_mode) {
            uac_stop_and_release(s_spk_handle, "speaker");  // free the channel
            if (resume_rx) mic_resume_after_tx();
        } else {
            uac_host_device_suspend(s_spk_handle);
        }
    }

    ESP_LOGI(TAG, "UAC OUT stopped packets=%u errors=%u",
             (unsigned)s_spk_packets_sent, (unsigned)s_spk_write_errors);
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

const char* uac_get_cat_status(void) {
    return s_cat_status;
}

bool cat_cdc_ready(void) {
    if (s_profile == UAC_PROFILE_FX4CR) return s_fx_cat_announced;
    return s_cdc_handle != NULL;
}

void uac_rx_resume(void) {
    mic_resume_after_tx();
}

esp_err_t uac_cat_batch(const char* const* cmds, size_t count, bool resume_rx) {
    return uac_cat_batch_ex(cmds, count, resume_rx, false);
}

esp_err_t uac_cat_batch_ex(const char* const* cmds, size_t count, bool resume_rx, bool urgent) {
    if (s_profile != UAC_PROFILE_FX4CR) {
        esp_err_t err = ESP_OK;
        for (size_t i = 0; i < count && err == ESP_OK; ++i) {
            err = cat_cdc_send(reinterpret_cast<const uint8_t*>(cmds[i]), strlen(cmds[i]), 200);
        }
        return err;
    }
    if (!s_cdc_installed || s_vcp_vid == 0) return ESP_ERR_INVALID_STATE;
    if (s_spk_writer_run) return ESP_ERR_INVALID_STATE;  // speaker holds a channel
    // Every burst pauses RX audio; a failing link retried often makes the
    // waterfall choppy and kills decodes. Back off 3, 6, 12, 24 s ... 60 s.
    if (s_cat_fail_count > 0 && !urgent) {
        int64_t backoff_ms = 3000LL << (s_cat_fail_count > 5 ? 5 : s_cat_fail_count - 1);
        if (backoff_ms > 60000) backoff_ms = 60000;
        if (rtc_now_ms() - s_cat_fail_ms < backoff_ms) return ESP_ERR_INVALID_STATE;
    }
    if (s_cat_lock) xSemaphoreTake(s_cat_lock, portMAX_DELAY);

    mic_pause_for_usb();
    esp_err_t err;
    if (s_vcp_vid == NANJING_QINHENG_MICROE_VID && is_legacy_ch34x_pid(s_vcp_pid)) {
        err = fx_ch340_write(cmds, count);
    } else {
        cdc_acm_host_device_config_t dev_cfg = {
            .connection_timeout_ms = 500,
            .out_buffer_size = 64,
            .in_buffer_size = 0,
            .event_cb = NULL,
            .data_cb = NULL,
            .user_arg = NULL,
        };
        cdc_acm_dev_hdl_t h = NULL;
        err = fx_bridge_open(&dev_cfg, &h);
        if (err == ESP_OK) {
            cdc_configure_fx4cr_line(h);
            for (size_t i = 0; i < count && err == ESP_OK; ++i) {
                err = cdc_acm_host_data_tx_blocking(h, reinterpret_cast<const uint8_t*>(cmds[i]),
                                                    strlen(cmds[i]), 200);
            }
            vTaskDelay(pdMS_TO_TICKS(10));  // let the bridge shift the last bytes out
            cdc_acm_host_close(h);
        }
    }
    if (err == ESP_OK) {
        snprintf(s_cat_status, sizeof(s_cat_status), "CAT ok %04x:%04x", s_vcp_vid, s_vcp_pid);
        s_cat_fail_ms = 0;
        s_cat_fail_count = 0;
    } else {
        snprintf(s_cat_status, sizeof(s_cat_status), "CATerr %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "FX-4CR CAT burst failed: %s", esp_err_to_name(err));
        s_cat_fail_ms = rtc_now_ms();
        s_cat_fail_count++;
    }
    if (resume_rx) mic_resume_after_tx();
    if (s_cat_lock) xSemaphoreGive(s_cat_lock);
    return err;
}

esp_err_t cat_cdc_send(const uint8_t* data, size_t len, uint32_t timeout_ms) {
    if (s_profile == UAC_PROFILE_FX4CR) {
        char cmd[64];
        if (len >= sizeof(cmd)) return ESP_ERR_INVALID_SIZE;
        memcpy(cmd, data, len);
        cmd[len] = '\0';
        const char* one[] = {cmd};
        return uac_cat_batch(one, 1, true);
    }
    if (!s_cdc_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    return cdc_acm_host_data_tx_blocking(s_cdc_handle, data, len, timeout_ms);
}
