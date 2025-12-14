#include "stream_wav.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_ipc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ui.h"
#include <vector>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <cstdint>
#include <sys/time.h>

extern "C" {
  #include "ft8/decode.h"
  #include "ft8/constants.h"
  #include "common/monitor.h"
}

static const char* TAG_STREAM = "FT8_STREAM";
extern bool g_streaming;
void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui);
int64_t rtc_now_ms();

#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 12000
#endif

struct WAVHeader {
  char riff[4];
  uint32_t file_size;
  char wave[4];
  char fmt[4];
  uint32_t fmt_size;
  uint16_t audio_format;
  uint16_t num_channels;
  uint32_t sample_rate;
  uint32_t byte_rate;
  uint16_t block_align;
  uint16_t bits_per_sample;
  char data[4];
  uint32_t data_size;
};

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

void stream_wav_task(void* arg) {
  const char* path = (const char*)arg;
  FILE* f = fopen(path, "rb");
  if (!f) {
    ESP_LOGE(TAG_STREAM, "Failed to open %s", path);
    g_streaming = false;
    vTaskDelete(NULL);
    return;
  }

  WAVHeader hdr;
  if (fread(&hdr, 1, sizeof(hdr), f) != sizeof(hdr) ||
      memcmp(hdr.riff, "RIFF", 4) != 0 || memcmp(hdr.wave, "WAVE", 4) != 0 ||
      hdr.sample_rate != FT8_SAMPLE_RATE || hdr.num_channels != 1) {
    ESP_LOGE(TAG_STREAM, "Invalid WAV header");
    fclose(f);
    g_streaming = false;
    vTaskDelete(NULL);
    return;
  }

  const int bytes_per_sample = hdr.bits_per_sample / 8;

  // Wait until the next 15 s boundary (00, 15, 30, 45) before starting,
  // but keep a small tolerance so we don't overshoot a just-passed edge.
  {
    int64_t now_ms = rtc_now_ms();
    int64_t rem = now_ms % 15000;
    // If we are within 100 ms after a boundary, start immediately; otherwise wait to the next boundary.
    int64_t wait_ms = (rem < 100) ? 0 : (15000 - rem);
    if (wait_ms > 0) {
      vTaskDelay(pdMS_TO_TICKS((uint32_t)wait_ms));
    }
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

  // Allocate chunk in internal heap.
  float* chunk = (float*)heap_caps_malloc(sizeof(float) * mon.block_size, MALLOC_CAP_DEFAULT);
  if (!chunk) {
    ESP_LOGE(TAG_STREAM, "Chunk alloc failed");
    fclose(f);
    monitor_free(&mon);
    g_streaming = false;
    vTaskDelete(NULL);
    return;
  }

  TickType_t next_wake = xTaskGetTickCount();

  const int target_blocks = 80; // 79 symbols ~=12.64s; take one-frame margin
  while (!feof(f)) {
    int read_samples = 0;
    while (read_samples < mon.block_size && !feof(f)) {
      float sample_value = 0.0f;
      if (bytes_per_sample == 1) {
        int s = fgetc(f);
        if (s == EOF) break;
        sample_value = ((float)s - 128.0f) / 128.0f;
      } else if (bytes_per_sample == 2) {
        int low = fgetc(f);
        int high = fgetc(f);
        if (low == EOF || high == EOF) break;
        int16_t s = (int16_t)((high << 8) | low);
        sample_value = (float)s / 32768.0f;
      }
      chunk[read_samples++] = sample_value;
    }
    if (read_samples == 0) break;
    for (int i = read_samples; i < mon.block_size; ++i) {
      chunk[i] = 0.0f;
    }

    double acc = 0.0;
    for (int i = 0; i < mon.block_size; ++i) acc += fabsf(chunk[i]);
    float level = (float)(acc / mon.block_size);
    float gain = (level > 1e-6f) ? 0.1f / level : 1.0f;
    if (gain < 0.1f) gain = 0.1f;
    if (gain > 10.0f) gain = 10.0f;
    for (int i = 0; i < mon.block_size; ++i) {
      chunk[i] *= gain;
    }

    if (mon.wf.num_blocks < target_blocks) {
      monitor_process(&mon, chunk);
      push_waterfall_latest(mon);
    }
    vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(160));

    if (mon.wf.num_blocks >= target_blocks) {
      if (mon.wf.num_blocks > 0) {
        struct DecodeTaskParam {
          monitor_t* mon;
          monitor_config_t cfg;
          TaskHandle_t waiter;
        };
        auto* p = new DecodeTaskParam{&mon, mon_cfg, xTaskGetCurrentTaskHandle()};
        auto decode_task = [](void* a) {
          auto* d = static_cast<DecodeTaskParam*>(a);
          decode_monitor_results(d->mon, &d->cfg, false); // decode on core0, defer UI
          xTaskNotifyGive(d->waiter);
          delete d;
          vTaskDelete(nullptr);
        };
        if (xTaskCreatePinnedToCore(decode_task, "decode_core0", 8192, p, 6, nullptr, 0) != pdPASS) {
          ESP_LOGE(TAG_STREAM, "Failed to start decode task");
          delete p;
          break;
        } else {
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
      }
      // prepare for next slot if more data remains
      monitor_reset(&mon);
      mon.wf.num_blocks = 0;
      next_wake = xTaskGetTickCount();
    }
  }

  free(chunk);
  fclose(f);

  // If we never decoded a slot, log a warning
  if (false) { // placeholder to satisfy references after loop
    (void)target_blocks;
  }
  if (mon.wf.num_blocks == 0) {
    ESP_LOGW(TAG_STREAM, "No audio blocks processed");
  }
  monitor_free(&mon);
  g_streaming = false;
  vTaskDelete(NULL);
}
