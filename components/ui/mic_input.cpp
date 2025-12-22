#include "mic_input.h"
#include <M5Cardputer.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <algorithm>

static const char* TAG_MIC = "MIC_INPUT";
#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 12000
#endif
// Capture at 17 kHz (matches M5 Mic example) and resample down to 12 kHz.
static const int MIC_CAPTURE_RATE = 17000;

static bool s_inited = false;
static bool s_running = false;
static int16_t* s_buf = nullptr;
static size_t s_buf_len = 0;

esp_err_t mic_input_init() {
    if (!s_inited) {
        M5Cardputer.Speaker.end(); // share codec; keep speaker off
        M5Cardputer.Mic.begin();
        s_running = true;
        s_inited = true;
        ESP_LOGI(TAG_MIC, "Mic initialized (capture %d Hz -> FT8 %d Hz)", MIC_CAPTURE_RATE, FT8_SAMPLE_RATE);
    }
    return ESP_OK;
}

void mic_input_start() {
    if (!s_inited) mic_input_init();
    if (!s_running) {
        M5Cardputer.Speaker.end();
        M5Cardputer.Mic.begin();
        s_running = true;
        ESP_LOGI(TAG_MIC, "Mic start");
    }
}

void mic_input_stop() {
    if (s_running) {
        M5Cardputer.Mic.end();
        s_running = false;
        ESP_LOGI(TAG_MIC, "Mic stop");
    }
}

bool mic_input_get_block(float* out, size_t n) {
    if (!out) return false;
    if (!s_running) mic_input_start();

    // How many input samples at capture rate to produce n at FT8 sample rate
    size_t cap_len = (size_t)((uint64_t)n * MIC_CAPTURE_RATE / FT8_SAMPLE_RATE + 1);
    if (s_buf_len < cap_len) {
        free(s_buf);
        s_buf = (int16_t*)heap_caps_malloc(cap_len * sizeof(int16_t), MALLOC_CAP_8BIT);
        s_buf_len = s_buf ? cap_len : 0;
        if (!s_buf) {
            ESP_LOGE(TAG_MIC, "alloc %u samples failed", (unsigned)cap_len);
            return false;
        }
    }

    auto do_record = [&](int16_t* buf, size_t len) {
        bool ok = M5Cardputer.Mic.record(buf, len, MIC_CAPTURE_RATE);
        if (ok && len > 0) {
            int16_t minv = buf[0], maxv = buf[0];
            for (size_t i = 1; i < len; ++i) {
                if (buf[i] < minv) minv = buf[i];
                if (buf[i] > maxv) maxv = buf[i];
            }
            ESP_LOGI(TAG_MIC, "min=%d max=%d", (int)minv, (int)maxv);
        }
        return ok;
    };

    bool ok = do_record(s_buf, cap_len);
    if (!ok) {
        ESP_LOGW(TAG_MIC, "Mic.record failed, restarting mic");
        M5Cardputer.Mic.end();
        M5Cardputer.Mic.begin();
        s_running = true;
        ok = do_record(s_buf, cap_len);
        if (!ok) return false;
    }

    // Simple nearest-neighbor resample to 12 kHz
    float step = (float)MIC_CAPTURE_RATE / (float)FT8_SAMPLE_RATE;
    for (size_t i = 0; i < n; ++i) {
        size_t idx = (size_t)(i * step);
        if (idx >= cap_len) { out[i] = 0.0f; continue; }
        out[i] = s_buf[idx] / 32768.0f;
    }
    return true;
}
