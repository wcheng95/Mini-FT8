#include "mic_selftest.h"
#include <M5Cardputer.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG_MIC_TEST = "MIC_TEST";
static TaskHandle_t s_test_task = nullptr;
static const int CAPTURE_RATE = 17000; // Hz
static const int CAPTURE_MS = 1000;    // 1 second

static void draw_waveform(const int16_t* buf, size_t len) {
    const int w = M5Cardputer.Display.width();
    const int h = M5Cardputer.Display.height();
    M5Cardputer.Display.startWrite();
    M5Cardputer.Display.fillScreen(TFT_BLACK);
    // Grid line center
    M5Cardputer.Display.drawFastHLine(0, h/2, w, M5Cardputer.Display.color565(40,40,40));
    int samples_per_px = (int)(len / w);
    if (samples_per_px < 1) samples_per_px = 1;
    for (int x = 0; x < w; ++x) {
        int start = x * samples_per_px;
        int end = start + samples_per_px;
        if (end > (int)len) end = (int)len;
        int16_t minv = buf[start], maxv = buf[start];
        for (int i = start + 1; i < end; ++i) {
            if (buf[i] < minv) minv = buf[i];
            if (buf[i] > maxv) maxv = buf[i];
        }
        int y1 = h/2 - (minv >> 7); // scale
        int y2 = h/2 - (maxv >> 7);
        if (y1 < 0) y1 = 0; if (y1 >= h) y1 = h-1;
        if (y2 < 0) y2 = 0; if (y2 >= h) y2 = h-1;
        if (y2 < y1) std::swap(y1, y2);
        M5Cardputer.Display.drawFastVLine(x, y1, (y2 - y1 + 1), TFT_YELLOW);
    }
    M5Cardputer.Display.endWrite();
}

static void mic_selftest_task(void* arg) {
    const size_t samples = (CAPTURE_RATE * CAPTURE_MS) / 1000;
    int16_t* buf = (int16_t*)heap_caps_malloc(samples * sizeof(int16_t), MALLOC_CAP_8BIT);
    if (!buf) {
        ESP_LOGE(TAG_MIC_TEST, "alloc %u samples failed", (unsigned)samples);
        s_test_task = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    // Ensure speaker off, mic on
    M5Cardputer.Speaker.end();
    M5Cardputer.Mic.begin();
    bool ok = M5Cardputer.Mic.record(buf, samples, CAPTURE_RATE);
    if (!ok) {
        ESP_LOGE(TAG_MIC_TEST, "Mic.record failed");
        free(buf);
        s_test_task = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    int16_t minv = buf[0], maxv = buf[0];
    int64_t acc = 0;
    for (size_t i = 0; i < samples; ++i) {
        if (buf[i] < minv) minv = buf[i];
        if (buf[i] > maxv) maxv = buf[i];
        acc += std::abs(buf[i]);
    }
    float avg = (float)acc / samples;
    ESP_LOGI(TAG_MIC_TEST, "capture done min=%d max=%d avg_abs=%.1f", (int)minv, (int)maxv, avg);

    draw_waveform(buf, samples);

    // Playback
    M5Cardputer.Mic.end();
    M5Cardputer.Speaker.begin();
    M5Cardputer.Speaker.playRaw(buf, samples, CAPTURE_RATE, false, 1, 0);
    while (M5Cardputer.Speaker.isPlaying()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    M5Cardputer.Speaker.end();
    M5Cardputer.Mic.begin();

    free(buf);
    s_test_task = nullptr;
    vTaskDelete(nullptr);
}

void mic_selftest_start() {
    if (s_test_task) return;
    xTaskCreatePinnedToCore(mic_selftest_task, "mic_selftest", 4096, nullptr, 5, &s_test_task, 1);
}
