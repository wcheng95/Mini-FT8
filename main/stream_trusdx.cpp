#include "stream_trusdx.h"

#include "ft8_audio_pipeline.h"
#include "protocol.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"
#include "usb/vcp_ch34x.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char* TAG = "TRUSDX";

extern bool g_streaming;
extern volatile bool g_cdc_initial_sync_pending;

// ---------------------------------------------------------------------------
// Link and audio constants (see dl2man.de "truSDX details", trusdx-txrx.py)
// ---------------------------------------------------------------------------
static constexpr uint32_t k_cat_baud = 115200;      // firmware >= 2.00t
static constexpr int k_reset_wait_ms = 3000;        // CH340 DTR toggles the ATmega reset
static constexpr float k_rx_rate_hz = 7812.5f;      // 62500 / 8, nominal
// Real rates differ between firmware builds (7936 Hz measured on 2.00x), and
// a 1-2 % error smears FT8 symbol timing over a 12.6 s frame. The resampler
// uses a running estimate measured from the byte rate while in RX.
static constexpr float k_rx_rate_min_hz = 7600.0f;
static constexpr float k_rx_rate_max_hz = 8200.0f;
static constexpr float k_tx_rate_hz = 11520.0f;     // = 115200 baud / 10 bits
static constexpr float k_out_rate_hz = 6000.0f;     // FT8 pipeline rate
static constexpr uint8_t k_semicolon = 0x3B;
static constexpr size_t k_rx_sb_size = 16384;       // ~2 s: survives a slot decode
static constexpr size_t k_rx_backlog_max = 2400;    // trim above ~300 ms latency
static constexpr size_t k_rx_backlog_keep = 800;    // ...down to ~100 ms
static constexpr int k_rx_read_wait_ms = 60;
static constexpr int k_rx_watchdog_ms = 3000;       // re-arm UA if audio stops in RX
static constexpr int k_tx_lead_samples = 576;       // ~50 ms queued ahead of the UART
static constexpr int k_tx_chunk = 128;
static constexpr int k_tx_period_ms = 4;
static constexpr float k_tx_amplitude = 110.0f;     // of 127, keeps clear of clipping

#define USB_HOST_TASK_PRIORITY 5
#define CTRL_TASK_PRIORITY 4
#define STREAM_TASK_PRIORITY 4
#define TX_TASK_PRIORITY 5

typedef enum {
    TRUSDX_IDLE,
    TRUSDX_WAITING,
    TRUSDX_RESET_WAIT,
    TRUSDX_STREAMING,
    TRUSDX_ERROR,
} trusdx_state_t;

static volatile trusdx_state_t s_state = TRUSDX_IDLE;
static volatile bool s_stop_requested = false;
static TaskHandle_t s_usb_task = NULL;
static TaskHandle_t s_ctrl_task = NULL;
static TaskHandle_t s_stream_task = NULL;
static TaskHandle_t s_tx_task = NULL;

static cdc_acm_dev_hdl_t s_cdc = NULL;
static SemaphoreHandle_t s_cdc_lock = NULL;  // serializes all writes to the radio
static volatile bool s_cdc_lost = false;
static volatile bool s_cat_ready = false;
static uint16_t s_ch34x_pid = CH34X_PID_AUTO;
static bool s_cdc_installed = false;

static char s_status[64] = "Idle";
static char s_dbg1[64] = "";
static char s_dbg2[64] = "";

// ---------------------------------------------------------------------------
// RX: CAT stream parser (runs in the CDC driver task)
// ---------------------------------------------------------------------------
static StreamBufferHandle_t s_rx_sb = NULL;
static bool s_rx_in_audio = false;
static char s_reply[32];
static size_t s_reply_len = 0;
static volatile int64_t s_rx_last_audio_ms = 0;
static volatile uint32_t s_rx_bytes_total = 0;
static volatile uint32_t s_rx_dropped = 0;
static volatile uint32_t s_rx_rearms = 0;
static volatile float s_rx_rate_est = k_rx_rate_hz;
static uint32_t s_rx_rate_windows = 0;  // watchdog recoveries (stuck TX / stream off)

static int64_t now_ms(void) {
    return esp_timer_get_time() / 1000;
}

static void rx_flush_audio(const uint8_t* run, size_t len) {
    if (len == 0 || !s_rx_sb) return;
    size_t sent = xStreamBufferSend(s_rx_sb, run, len, 0);
    if (sent < len) s_rx_dropped += (uint32_t)(len - sent);
    s_rx_bytes_total += (uint32_t)len;
    s_rx_last_audio_ms = now_ms();
}

// Byte stream looks like: US<audio...>;<CAT reply>;US<audio...>
static bool cdc_rx_cb(const uint8_t* data, size_t len, void* arg) {
    (void)arg;
    size_t run_start = 0;
    for (size_t i = 0; i < len; ++i) {
        const uint8_t b = data[i];
        if (s_rx_in_audio) {
            if (b == k_semicolon) {
                rx_flush_audio(data + run_start, i - run_start);
                s_rx_in_audio = false;
                s_reply_len = 0;
            }
            continue;
        }
        if (b == k_semicolon) {
            if (s_reply_len > 0) {
                s_reply[s_reply_len] = '\0';
                ESP_LOGD(TAG, "CAT<%s;", s_reply);
            }
            s_reply_len = 0;
            continue;
        }
        if (s_reply_len < sizeof(s_reply) - 1) {
            s_reply[s_reply_len++] = (char)b;
        }
        if (s_reply_len == 2 && s_reply[0] == 'U' && s_reply[1] == 'S') {
            s_rx_in_audio = true;
            s_reply_len = 0;
            run_start = i + 1;
        }
    }
    if (s_rx_in_audio) rx_flush_audio(data + run_start, len - run_start);
    return true;
}

static void cdc_event_cb(const cdc_acm_host_dev_event_data_t* event, void* user_ctx) {
    (void)user_ctx;
    if (event->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        ESP_LOGI(TAG, "truSDX disconnected");
        s_cat_ready = false;
        s_cdc_lost = true;  // ctrl task closes the handle under the write lock
    } else if (event->type == CDC_ACM_HOST_ERROR) {
        ESP_LOGW(TAG, "CDC error %d", event->data.error);
    }
}

static void new_dev_cb(usb_device_handle_t usb_dev) {
    const usb_device_desc_t* desc = nullptr;
    if (usb_host_get_device_descriptor(usb_dev, &desc) == ESP_OK && desc) {
        ESP_LOGI(TAG, "USB dev attached VID:0x%04x PID:0x%04x",
                 desc->idVendor, desc->idProduct);
        if (desc->idVendor == NANJING_QINHENG_MICROE_VID) {
            s_ch34x_pid = desc->idProduct;
        }
    }
}

// ---------------------------------------------------------------------------
// RX: U8 @ ~7812.5 Hz -> float @ 6 kHz
// DC block, 15-tap windowed-sinc low-pass at 2.8 kHz, linear interpolation.
// ---------------------------------------------------------------------------
static constexpr int k_fir_taps = 15;
static float s_fir_coef[k_fir_taps];

typedef struct {
    float hist[k_fir_taps];
    int hist_pos;
    float dc_x1;
    float dc_y1;
    float prev;  // previous low-passed input sample
    float t;     // output position between prev and current, in input samples
} rx_resampler_t;

static rx_resampler_t s_rs;

static void rx_resampler_init(void) {
    const float fc = 2800.0f / k_rx_rate_hz;  // cycles per input sample
    const int mid = k_fir_taps / 2;
    float sum = 0.0f;
    for (int n = 0; n < k_fir_taps; ++n) {
        const int m = n - mid;
        float sinc = (m == 0) ? 2.0f * fc
                              : sinf(2.0f * (float)M_PI * fc * m) / ((float)M_PI * m);
        float win = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * n / (k_fir_taps - 1));
        s_fir_coef[n] = sinc * win;
        sum += s_fir_coef[n];
    }
    for (int n = 0; n < k_fir_taps; ++n) s_fir_coef[n] /= sum;
    memset(&s_rs, 0, sizeof(s_rs));
}

static int rx_resample(const uint8_t* in, int n_in, float* out, int max_out) {
    const float step = s_rx_rate_est / k_out_rate_hz;  // input samples per output
    int n_out = 0;
    for (int i = 0; i < n_in; ++i) {
        const float x = ((float)in[i] - 128.0f) / 128.0f;
        const float y = x - s_rs.dc_x1 + 0.995f * s_rs.dc_y1;
        s_rs.dc_x1 = x;
        s_rs.dc_y1 = y;

        s_rs.hist[s_rs.hist_pos] = y;
        s_rs.hist_pos = (s_rs.hist_pos + 1) % k_fir_taps;
        float acc = 0.0f;
        int idx = s_rs.hist_pos;
        for (int k = 0; k < k_fir_taps; ++k) {
            acc += s_fir_coef[k] * s_rs.hist[idx];
            idx = (idx + 1) % k_fir_taps;
        }

        while (s_rs.t <= 1.0f && n_out < max_out) {
            out[n_out++] = s_rs.prev + (acc - s_rs.prev) * s_rs.t;
            s_rs.t += step;
        }
        s_rs.t -= 1.0f;
        s_rs.prev = acc;
    }
    return n_out;
}

static int trusdx_read_ft8_samples(void* ctx, float* out, int max_samples) {
    (void)ctx;
    static uint8_t raw[384];
    if (!s_rx_sb || max_samples <= 0) return 0;

    // Bound latency: the pipeline is paced by wall clock, so a radio clock
    // slightly faster than nominal would otherwise grow the backlog forever.
    size_t avail = xStreamBufferBytesAvailable(s_rx_sb);
    if (avail > k_rx_backlog_max) {
        size_t drop = avail - k_rx_backlog_keep;
        while (drop > 0) {
            size_t n = xStreamBufferReceive(s_rx_sb, raw, drop < sizeof(raw) ? drop : sizeof(raw), 0);
            if (n == 0) break;
            drop -= n;
        }
    }

    size_t got = xStreamBufferReceive(s_rx_sb, raw, sizeof(raw), pdMS_TO_TICKS(k_rx_read_wait_ms));
    if (got == 0) {
        // No audio (own TX, CAT exchange or link down): feed silence at
        // roughly real-time so slot timing in the pipeline keeps running.
        int n = (int)(k_out_rate_hz * k_rx_read_wait_ms / 1000.0f);
        if (n > max_samples) n = max_samples;
        memset(out, 0, sizeof(float) * n);
        return n;
    }

    // max_samples is 512; 384 input bytes yield at most ~296 outputs.
    return rx_resample(raw, (int)got, out, max_samples);
}

static bool trusdx_should_stop(void* ctx) {
    (void)ctx;
    return s_stop_requested;
}

static void stream_task(void* arg) {
    (void)arg;
    ESP_LOGI(TAG, "RX pipeline task started");
    rx_resampler_init();
    g_streaming = true;
    ft8_audio_pipeline_config_t cfg = {
        .tag = TAG,
        .ctx = NULL,
        .read = trusdx_read_ft8_samples,
        .should_stop = trusdx_should_stop,
        .on_block_processed = NULL,
    };
    ft8_audio_pipeline_run(&cfg);
    g_streaming = false;
    s_stream_task = NULL;
    ESP_LOGI(TAG, "RX pipeline task stopped");
    vTaskDelete(NULL);
}

static bool start_stream_task(void) {
    if (s_stream_task) return true;
#if ENABLE_FT4
    if (g_protocol == &kProtocolFT4) {
        // FT4's decoder needs a larger stack than FT8 (see stream_uac.cpp).
        if (xTaskCreatePinnedToCore(stream_task, "stream_trusdx", 16384, NULL,
                                    STREAM_TASK_PRIORITY, &s_stream_task, 1) != pdPASS) {
            s_stream_task = NULL;
        }
        return s_stream_task != NULL;
    }
#endif
    static StackType_t s_stack[8192 / sizeof(StackType_t)];
    static StaticTask_t s_tcb;
    s_stream_task = xTaskCreateStaticPinnedToCore(stream_task, "stream_trusdx",
                                                  8192 / sizeof(StackType_t), NULL,
                                                  STREAM_TASK_PRIORITY, s_stack, &s_tcb, 1);
    return s_stream_task != NULL;
}

// ---------------------------------------------------------------------------
// Writes to the radio
// ---------------------------------------------------------------------------
static esp_err_t write_locked(const uint8_t* data, size_t len, uint32_t timeout_ms) {
    if (!s_cdc_lock) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_cdc_lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t err = s_cdc ? cdc_acm_host_data_tx_blocking(s_cdc, data, len, timeout_ms)
                          : ESP_ERR_INVALID_STATE;
    xSemaphoreGive(s_cdc_lock);
    return err;
}

static esp_err_t send_raw_cmd(const char* cmd, uint32_t timeout_ms) {
    ESP_LOGD(TAG, "CAT>%s", cmd);
    return write_locked(reinterpret_cast<const uint8_t*>(cmd), strlen(cmd), timeout_ms);
}

bool trusdx_cat_ready(void) {
    return s_cat_ready && s_cdc != NULL;
}

static volatile bool s_tx_active = false;  // radio keyed by us

esp_err_t trusdx_cat_send(const char* cmd, uint32_t timeout_ms) {
    if (!trusdx_cat_ready() || !cmd) return ESP_ERR_INVALID_STATE;
    if (!s_tx_active) return send_raw_cmd(cmd, timeout_ms);
    // While TX samples are streaming, ';' ends the audio before the command.
    char buf[40];
    snprintf(buf, sizeof(buf), ";%s", cmd);
    return send_raw_cmd(buf, timeout_ms);
}

// ---------------------------------------------------------------------------
// TX: CPFSK / tone synthesis at 11520 Hz, paced against esp_timer
// ---------------------------------------------------------------------------
static constexpr size_t k_tx_max_symbols = 105;
static float s_tx_freqs[k_tx_max_symbols];
static size_t s_tx_symbol_count = 0;      // 0 = continuous tone (tune)
static double s_tx_samples_per_symbol = 0.0;
static float s_tx_tone_hz = 0.0f;
static volatile bool s_tx_run = false;    // writer gate
static volatile bool s_tx_done = false;   // waveform fully queued
static int64_t s_tx_t0_us = 0;
static uint64_t s_tx_sent = 0;
static double s_tx_phase = 0.0;
static uint32_t s_tx_write_errors = 0;

static void tx_task(void* arg) {
    (void)arg;
    uint8_t chunk[k_tx_chunk];
    while (!s_stop_requested) {
        if (!s_tx_run || s_tx_done) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        const int64_t elapsed_us = esp_timer_get_time() - s_tx_t0_us;
        const uint64_t due = (uint64_t)((double)elapsed_us * k_tx_rate_hz / 1e6) + k_tx_lead_samples;
        while (s_tx_run && !s_tx_done && s_tx_sent < due) {
            int n = 0;
            while (n < k_tx_chunk && s_tx_sent < due) {
                float f = s_tx_tone_hz;
                if (s_tx_symbol_count > 0) {
                    size_t sym = (size_t)((double)s_tx_sent / s_tx_samples_per_symbol);
                    if (sym >= s_tx_symbol_count) {
                        s_tx_done = true;
                        break;
                    }
                    f = s_tx_freqs[sym];
                }
                s_tx_phase += 2.0 * M_PI * (double)f / (double)k_tx_rate_hz;
                if (s_tx_phase > 2.0 * M_PI) s_tx_phase -= 2.0 * M_PI;
                int v = 128 + (int)lrintf(k_tx_amplitude * sinf((float)s_tx_phase));
                uint8_t b = (uint8_t)v;
                if (b == k_semicolon) b = 0x3A;  // ';' would end the stream
                chunk[n++] = b;
                s_tx_sent++;
            }
            if (n > 0 && write_locked(chunk, (size_t)n, 200) != ESP_OK) {
                s_tx_write_errors++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(k_tx_period_ms));
    }
    s_tx_task = NULL;
    vTaskDelete(NULL);
}

static bool tx_start(void) {
    if (!trusdx_cat_ready() || !s_tx_task) return false;
    if (s_tx_active) {
        ESP_LOGW(TAG, "TX already active");
        return false;
    }
    s_tx_sent = 0;
    s_tx_phase = 0.0;
    s_tx_done = false;
    s_tx_write_errors = 0;
    // The radio stops streaming RX audio and starts consuming samples.
    if (send_raw_cmd("TX0;", 200) != ESP_OK) return false;
    s_tx_active = true;
    s_tx_t0_us = esp_timer_get_time();
    s_tx_run = true;
    return true;
}

bool trusdx_tx_begin_cpfsk(float base_hz,
                           const uint8_t* symbols,
                           size_t symbol_count,
                           float tone_spacing_hz,
                           float symbol_period_s) {
    if (!symbols || symbol_count == 0 || symbol_count > k_tx_max_symbols ||
        symbol_period_s <= 0.0f) {
        return false;
    }
    for (size_t i = 0; i < symbol_count; ++i) {
        s_tx_freqs[i] = base_hz + tone_spacing_hz * (float)symbols[i];
    }
    s_tx_symbol_count = symbol_count;
    s_tx_samples_per_symbol = (double)symbol_period_s * (double)k_tx_rate_hz;
    if (!tx_start()) return false;
    ESP_LOGI(TAG, "TX CPFSK base=%.1f symbols=%u", (double)base_hz, (unsigned)symbol_count);
    return true;
}

bool trusdx_tx_begin_tune(float tone_hz) {
    s_tx_symbol_count = 0;
    s_tx_tone_hz = tone_hz;
    if (!tx_start()) return false;
    ESP_LOGI(TAG, "TX tune tone=%.1f", (double)tone_hz);
    return true;
}

esp_err_t trusdx_tx_end(void) {
    if (!s_tx_active) return ESP_OK;
    // Let the last symbol finish queueing, then give the radio time to play
    // out what is buffered (lead + CH340 FIFO) before switching to RX.
    if (s_tx_symbol_count > 0) {
        for (int i = 0; i < 40 && s_tx_run && !s_tx_done; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    s_tx_run = false;
    vTaskDelay(pdMS_TO_TICKS(k_tx_lead_samples * 1000 / (int)k_tx_rate_hz + 10));
    s_tx_active = false;
    s_rx_last_audio_ms = now_ms();  // RX stream resumes after RX;
    // Sent twice (as the original trusdx-audio script did): a lost RX; leaves
    // the radio keyed with no audio — zero output, "nan" SWR, no RX stream.
    esp_err_t err = s_cdc ? send_raw_cmd(";RX;", 200) : ESP_ERR_INVALID_STATE;
    vTaskDelay(pdMS_TO_TICKS(30));
    if (s_cdc) send_raw_cmd(";RX;", 200);
    ESP_LOGI(TAG, "TX end sent=%llu errors=%u rc=%s",
             (unsigned long long)s_tx_sent, (unsigned)s_tx_write_errors, esp_err_to_name(err));
    return err;
}

// ---------------------------------------------------------------------------
// Connection management
// ---------------------------------------------------------------------------
static void close_cdc(void) {
    s_cat_ready = false;
    s_tx_run = false;
    s_tx_active = false;
    if (s_cdc_lock) xSemaphoreTake(s_cdc_lock, portMAX_DELAY);
    if (s_cdc) {
        cdc_acm_host_close(s_cdc);
        s_cdc = NULL;
    }
    if (s_cdc_lock) xSemaphoreGive(s_cdc_lock);
    s_rx_in_audio = false;
    s_reply_len = 0;
}

static bool try_open(void) {
    cdc_acm_host_device_config_t cfg = {
        .connection_timeout_ms = 500,
        .out_buffer_size = 256,
        .in_buffer_size = 256,
        .event_cb = cdc_event_cb,
        .data_cb = cdc_rx_cb,
        .user_arg = NULL,
    };
    cdc_acm_dev_hdl_t h = NULL;
    esp_err_t err = ch34x_vcp_open(s_ch34x_pid, 0, &cfg, &h);
    if (err != ESP_OK) return false;

    cdc_acm_line_coding_t coding = {};
    coding.dwDTERate = k_cat_baud;
    coding.bCharFormat = 0;
    coding.bParityType = 0;
    coding.bDataBits = 8;
    err = cdc_acm_host_line_coding_set(h, &coding);
    if (err != ESP_OK) ESP_LOGW(TAG, "line coding failed: %s", esp_err_to_name(err));
    // Per DL2MAN: DTR high, RTS low in RX (RTS high would key the rig).
    err = cdc_acm_host_set_control_line_state(h, true, false);
    if (err != ESP_OK) ESP_LOGW(TAG, "DTR/RTS set failed: %s", esp_err_to_name(err));

    s_cdc = h;
    s_cdc_lost = false;
    ESP_LOGI(TAG, "truSDX CH34x opened (PID 0x%04x), %lu 8N1",
             s_ch34x_pid, (unsigned long)k_cat_baud);
    return true;
}

// Called ~once per second from the ctrl task: measures the RX byte rate and
// updates the resampler's rate estimate from gap-free RX windows.
static void update_rate_and_debug(void) {
    static int64_t last_ms = 0;
    static uint32_t last_bytes = 0;
    static bool window_had_tx = false;
    int64_t t = now_ms();
    if (s_tx_active) window_had_tx = true;
    if (t - last_ms < 1000) return;
    uint32_t bytes = s_rx_bytes_total;
    float rate = (last_ms > 0) ? (float)(bytes - last_bytes) * 1000.0f / (float)(t - last_ms) : 0.0f;
    const bool usable = last_ms > 0 && !window_had_tx && !s_tx_active &&
                        rate >= k_rx_rate_min_hz && rate <= k_rx_rate_max_hz;
    last_ms = t;
    last_bytes = bytes;
    window_had_tx = false;
    if (usable) {
        // First window seeds the estimate; then a ~20 s exponential average
        // smooths out USB packet granularity.
        s_rx_rate_est = (s_rx_rate_windows == 0) ? rate : s_rx_rate_est + 0.05f * (rate - s_rx_rate_est);
        s_rx_rate_windows++;
    }
    // Shown on the STATUS line (19 characters): measured RX sample rate.
    snprintf(s_dbg1, sizeof(s_dbg1), "truSDX rx %.0fHz", (double)s_rx_rate_est);
    if (s_rx_dropped || s_rx_rearms) {
        ESP_LOGD(TAG, "rx drops=%lu rearms=%lu", (unsigned long)s_rx_dropped,
                 (unsigned long)s_rx_rearms);
    }
    snprintf(s_dbg2, sizeof(s_dbg2), "tx=%s sent=%llu err=%u", s_tx_active ? "on" : "off",
             (unsigned long long)s_tx_sent, (unsigned)s_tx_write_errors);
}

static void ctrl_task(void* arg) {
    (void)arg;
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // USB host + CDC driver ready
    int64_t opened_ms = 0;
    while (!s_stop_requested) {
        if (s_cdc_lost && s_cdc) {
            close_cdc();
            s_state = TRUSDX_WAITING;
            snprintf(s_status, sizeof(s_status), "truSDX disconnected");
        }
        if (!s_cdc) {
            if (try_open()) {
                opened_ms = now_ms();
                s_state = TRUSDX_RESET_WAIT;
                snprintf(s_status, sizeof(s_status), "truSDX found, wait");
            } else {
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
        }
        if (s_state == TRUSDX_RESET_WAIT && now_ms() - opened_ms >= k_reset_wait_ms) {
            // USB mode, then CAT streaming with the radio speaker muted.
            esp_err_t err = send_raw_cmd(";MD2;UA2;", 200);
            if (err == ESP_OK) {
                s_cat_ready = true;
                s_rx_last_audio_ms = now_ms();
                s_state = TRUSDX_STREAMING;
                snprintf(s_status, sizeof(s_status), "Streaming truSDX 7.8k/8");
                g_cdc_initial_sync_pending = true;  // main loop pushes band FA
                if (!start_stream_task()) {
                    ESP_LOGE(TAG, "RX pipeline task create failed");
                }
            } else {
                ESP_LOGW(TAG, "stream enable failed: %s", esp_err_to_name(err));
                opened_ms = now_ms();
            }
        }
        if (s_state == TRUSDX_STREAMING && !s_tx_active &&
            now_ms() - s_rx_last_audio_ms > k_rx_watchdog_ms) {
            // Radio stuck in TX (missed RX;), rebooted, or dropped streaming:
            // force RX and re-arm streaming.
            ESP_LOGW(TAG, "No RX audio for %d ms, forcing RX + stream", k_rx_watchdog_ms);
            send_raw_cmd(";RX;UA2;", 200);
            s_rx_rearms = s_rx_rearms + 1;
            s_rx_last_audio_ms = now_ms();
        }
        update_rate_and_debug();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    close_cdc();
    s_ctrl_task = NULL;
    vTaskDelete(NULL);
}

static void usb_task(void* arg) {
    usb_host_config_t host_config = {};
    host_config.skip_phy_setup = false;
    host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install: %s", esp_err_to_name(err));
        s_state = TRUSDX_ERROR;
        snprintf(s_status, sizeof(s_status), "USB init failed");
        s_stop_requested = true;
        xTaskNotifyGive((TaskHandle_t)arg);  // let ctrl task exit
        s_usb_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    const cdc_acm_host_driver_config_t cdc_cfg = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 5,
        .xCoreID = 0,
        .new_dev_cb = new_dev_cb,
    };
    err = cdc_acm_host_install(&cdc_cfg);
    s_cdc_installed = (err == ESP_OK);
    if (!s_cdc_installed) ESP_LOGE(TAG, "cdc_acm_host_install: %s", esp_err_to_name(err));
    xTaskNotifyGive((TaskHandle_t)arg);

    while (!s_stop_requested) {
        uint32_t flags = 0;
        if (usb_host_lib_handle_events(pdMS_TO_TICKS(100), &flags) == ESP_OK &&
            (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)) {
            usb_host_device_free_all();
        }
    }

    // ctrl task closes the device; wait for it before uninstalling.
    for (int i = 0; i < 50 && s_ctrl_task; ++i) vTaskDelay(pdMS_TO_TICKS(20));
    if (s_cdc_installed) {
        cdc_acm_host_uninstall();
        s_cdc_installed = false;
    }
    for (int i = 0; i < 50; ++i) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(pdMS_TO_TICKS(20), &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
        if (flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) break;
    }
    err = usb_host_uninstall();
    if (err != ESP_OK) ESP_LOGW(TAG, "usb_host_uninstall: %s", esp_err_to_name(err));
    s_usb_task = NULL;
    vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
bool trusdx_stream_start(void) {
    if (s_state != TRUSDX_IDLE) {
        ESP_LOGW(TAG, "already started");
        return false;
    }
    s_stop_requested = false;
    s_cdc_lost = false;
    s_cat_ready = false;
    s_ch34x_pid = CH34X_PID_AUTO;
    s_rx_bytes_total = 0;
    s_rx_dropped = 0;
    s_rx_rearms = 0;
    s_rx_rate_est = k_rx_rate_hz;
    s_rx_rate_windows = 0;
    ft8_audio_pipeline_clear_latest_waterfall_row();

    if (!s_cdc_lock) s_cdc_lock = xSemaphoreCreateMutex();
    if (!s_rx_sb) s_rx_sb = xStreamBufferCreate(k_rx_sb_size, 1);
    if (!s_cdc_lock || !s_rx_sb) {
        ESP_LOGE(TAG, "alloc failed");
        return false;
    }
    xStreamBufferReset(s_rx_sb);

    if (xTaskCreatePinnedToCore(tx_task, "trusdx_tx", 3072, NULL, TX_TASK_PRIORITY,
                                &s_tx_task, 1) != pdPASS) {
        s_tx_task = NULL;
        return false;
    }
    if (xTaskCreatePinnedToCore(ctrl_task, "trusdx_ctrl", 4096, NULL, CTRL_TASK_PRIORITY,
                                &s_ctrl_task, 0) != pdPASS) {
        s_ctrl_task = NULL;
        s_stop_requested = true;
        return false;
    }
    if (xTaskCreatePinnedToCore(usb_task, "trusdx_usb", 4096, (void*)s_ctrl_task,
                                USB_HOST_TASK_PRIORITY, &s_usb_task, 0) != pdPASS) {
        s_usb_task = NULL;
        s_stop_requested = true;
        xTaskNotifyGive(s_ctrl_task);
        return false;
    }
    s_state = TRUSDX_WAITING;
    snprintf(s_status, sizeof(s_status), "Waiting for truSDX");
    return true;
}

void trusdx_stream_stop(void) {
    if (s_state == TRUSDX_IDLE) return;
    ESP_LOGI(TAG, "stopping");
    if (s_tx_active) trusdx_tx_end();
    s_stop_requested = true;
    g_streaming = false;
    for (int i = 0; i < 50 && (s_stream_task || s_ctrl_task || s_usb_task || s_tx_task); ++i) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    s_state = TRUSDX_IDLE;
    ft8_audio_pipeline_clear_latest_waterfall_row();
    snprintf(s_status, sizeof(s_status), "Idle");
}

bool trusdx_stream_is_streaming(void) {
    return s_state == TRUSDX_STREAMING && s_stream_task != NULL;
}

bool trusdx_stream_detected(void) {
    return s_cdc != NULL;
}

const char* trusdx_get_status_string(void) { return s_status; }
const char* trusdx_get_debug_line1(void) { return s_dbg1; }
const char* trusdx_get_debug_line2(void) { return s_dbg2; }
