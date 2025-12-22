#include "stream_mic.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ui.h"
#include "mic_input.h"
#include <vector>
#include <cmath>
#include <cstring>

extern "C" {
  #include "ft8/decode.h"
  #include "ft8/constants.h"
  #include "common/monitor.h"
}

static const char* TAG_MIC = "FT8_MIC";
extern bool g_streaming;  // true when WAV playback is running
void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui);
int64_t rtc_now_ms();

#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 12000
#endif

static void push_waterfall_latest(const monitor_t& mon) {
  if (mon.wf.num_blocks <= 0 || mon.wf.mag == nullptr) return;
  const int block = mon.wf.num_blocks - 1;
  const int num_bins = mon.wf.num_bins;
  const int freq_osr = mon.wf.freq_osr;
  const uint8_t* base = mon.wf.mag + block * mon.wf.block_stride;

  static std::vector<uint8_t> collapsed;
  collapsed.assign(num_bins, 0);
  for (int b = 0; b < num_bins; ++b) {
    uint8_t v = 0;
    for (int fs = 0; fs < freq_osr; ++fs) {
      uint8_t val = base[fs * num_bins + b];
      if (val > v) v = val;
    }
    collapsed[b] = v;
  }

  constexpr int width = 240;
  static std::vector<uint8_t> scaled;
  scaled.assign(width, 0);
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

  ui_push_waterfall_row(scaled.data(), width);
}

void stream_mic_task(void* /*arg*/) {
  if (mic_input_init() != ESP_OK) {
    ESP_LOGE(TAG_MIC, "mic init failed");
    vTaskDelete(nullptr);
    return;
  }

  monitor_config_t mon_cfg;
  mon_cfg.f_min = 200.0f;
  mon_cfg.f_max = 3000.0f;
  mon_cfg.sample_rate = FT8_SAMPLE_RATE;
  mon_cfg.time_osr = 1;
  mon_cfg.freq_osr = 2;
  mon_cfg.protocol = FTX_PROTOCOL_FT8;

  monitor_t mon;
  monitor_init(&mon, &mon_cfg);
  monitor_reset(&mon);

  float* chunk = (float*)heap_caps_malloc(sizeof(float) * mon.block_size, MALLOC_CAP_DEFAULT);
  if (!chunk) {
    ESP_LOGE(TAG_MIC, "Chunk alloc failed");
    monitor_free(&mon);
    vTaskDelete(nullptr);
    return;
  }

  const int target_blocks = 80; // 79 symbols ~=12.64s; one-frame margin

  while (true) {
    if (g_streaming) {
      mic_input_stop();
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    mic_input_start();

    // Wait to slot boundary (00/15/30/45s)
    {
      int64_t now_ms = rtc_now_ms();
      int64_t rem = now_ms % 15000;
      int64_t wait_ms = (rem < 100) ? 0 : (15000 - rem);
      if (wait_ms > 0) vTaskDelay(pdMS_TO_TICKS((uint32_t)wait_ms));
    }

    monitor_reset(&mon);
    TickType_t next_wake = xTaskGetTickCount();

        for (int blk = 0; blk < target_blocks && !g_streaming; ++blk) {
      if (!mic_input_get_block(chunk, mon.block_size)) {
        ESP_LOGW(TAG_MIC, "mic read failed");
        break;
      }
      double acc = 0.0;
      for (int i = 0; i < mon.block_size; ++i) acc += fabsf(chunk[i]);
      float level = (float)(acc / mon.block_size);
      float gain = (level > 1e-6f) ? 0.1f / level : 1.0f;
      if (gain < 0.1f) gain = 0.1f;
      if (gain > 10.0f) gain = 10.0f;
      if ((blk % 10) == 0) {
        ESP_LOGI(TAG_MIC, "blk %d level=%.5f gain=%.2f", blk, level, gain);
      }
      for (int i = 0; i < mon.block_size; ++i) {
        chunk[i] *= gain;
      }

            // Debug: show raw waveform energy instead of spectrum
      {
        const int width = 240;
        uint8_t row[width];
        int samples_per_px = mon.block_size / width;
        if (samples_per_px < 1) samples_per_px = 1;
        for (int x = 0; x < width; ++x) {
          int start = x * samples_per_px;
          int end = start + samples_per_px;
          if (end > mon.block_size) end = mon.block_size;
          float acc = 0.0f;
          for (int i = start; i < end; ++i) acc += fabsf(chunk[i]);
          float avg = acc / (end - start + 1e-6f);
          int v = (int)(avg * 4000.0f); // scale roughly to 0-255
          if (v > 255) v = 255;
          row[x] = (uint8_t)v;
        }
        ui_push_waterfall_row(row, width);
      }

      monitor_process(&mon, chunk);
      vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(160));
    }

    // If WAV streaming started, skip decode and continue
    if (g_streaming) {
      mic_input_stop();
      continue;
    }

    if (mon.wf.num_blocks > 0) {
      struct DecodeParam { monitor_t* mon; monitor_config_t cfg; TaskHandle_t waiter; };
      auto* p = new DecodeParam{&mon, mon_cfg, xTaskGetCurrentTaskHandle()};
      auto decode_task = [](void* a) {
        auto* d = static_cast<DecodeParam*>(a);
        decode_monitor_results(d->mon, &d->cfg, false);
        xTaskNotifyGive(d->waiter);
        delete d;
        vTaskDelete(nullptr);
      };
      if (xTaskCreatePinnedToCore(decode_task, "decode_mic", 8192, p, 4, nullptr, 1) == pdPASS) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      } else {
        ESP_LOGE(TAG_MIC, "decode task start failed");
        delete p;
      }
    }
  }

  // never reached
  mic_input_stop();
  free(chunk);
  monitor_free(&mon);
  vTaskDelete(nullptr);
}


