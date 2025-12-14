#include <cstdio>
#include <cmath>
#include "esp_log.h"
#include "esp_spiffs.h"
extern "C" {
  #include "ft8/decode.h"
  #include "ft8/constants.h"
  #include "ft8/message.h"
  #include "ft8/encode.h"
  #include "common/monitor.h"
}

#include "ui.h"
#include <vector>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "tx_queue.h"
#include <M5Cardputer.h>
#include <sstream>
#include <iterator>
#include <cstdio>
#include <string>
#include <cstdint>
#include <vector>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>
#include "driver/usb_serial_jtag.h"
#include <cctype>
#include <cstdlib>
#include <ctime>
#include "esp_timer.h"
#include "stream_wav.h"
#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 12000
#endif

static const char* TAG = "FT8";
enum class UIMode { RX, TX, BAND, MENU, HOST, DEBUG, LIST, STATUS };
static UIMode ui_mode = UIMode::RX;
static int tx_page = 0;
static int tx_selected_idx = -1;  // index into tx_queue (not including tx_next)
static std::vector<UiRxLine> g_rx_lines;
static const char* STATION_FILE = "/spiffs/StationData.ini";

enum class BeaconMode { OFF = 0, EVEN, EVEN2, ODD, ODD2 };
struct BandItem {
  const char* name;
  int freq;
};
static std::vector<BandItem> g_bands = {
    {"160m", 1840},   {"80m", 3573},   {"60m", 5357},   {"40m", 7074},
    {"30m", 10136},   {"20m", 14074},  {"17m", 18100},  {"15m", 21074},
    {"12m", 24915},   {"10m", 28074},  {"6m", 50313},   {"2m", 144174},
};
static int band_page = 0;
static int band_edit_idx = -1;       // absolute index into g_bands
static std::string band_edit_buffer; // text while editing
static BeaconMode g_beacon = BeaconMode::OFF;
static int g_offset_hz = 1500;
static int g_band_sel = 1; // default 80m
static bool g_tune = false;
static std::string g_date = "2025-12-11";
static std::string g_time = "10:10:00";
static int status_edit_idx = -1;     // 0-5
static std::string status_edit_buffer;
static int status_cursor_pos = -1;
static std::vector<std::string> g_debug_lines;
static int debug_page = 0;
static const size_t DEBUG_MAX_LINES = 18; // 3 pages
static void debug_log_line(const std::string& msg);
static void host_handle_line(const std::string& line);
static void poll_host_uart();
static void enter_mode(UIMode new_mode);
static bool g_rx_dirty = false;
static std::vector<std::string> g_list_lines = {
    "10:34 20m WA4HR",
    "10:36 40m K4ABC",
    "10:40 17m DL2XYZ",
    "10:45 30m JA1ZZZ",
    "10:50 15m VK2AAA",
    "10:52 10m KH6BBB"
};
static int list_page = 0;
static std::vector<std::string> g_host_lines = {
    "HOST MODE: USB serial",
    "Commands:",
    "WRITEBIN <file> <size> <crc32_hex>",
    "WRITE/APPEND",
    "READ/DELETE",
    "LIST/INFO/HELP",
    "EXIT to leave"
};
static std::string host_input;
static const char* HOST_PROMPT = "MINIFT8> ";
static bool usb_ready = false;
static bool host_bin_active = false;
static size_t host_bin_remaining = 0;
static FILE* host_bin_fp = nullptr;
static uint32_t host_bin_crc = 0;
static uint32_t host_bin_expected_crc = 0;
static size_t host_bin_received = 0;
static std::vector<uint8_t> host_bin_buf;
static const size_t HOST_BIN_CHUNK = 512;
static size_t host_bin_chunk_expect = 0; // payload bytes this chunk (excludes CRC trailer)
static uint8_t host_bin_first8[8] = {0};
static uint8_t host_bin_last8[8] = {0};
static size_t host_bin_first_filled = 0;
static std::string host_bin_path;

// Software RTC
static time_t rtc_epoch_base = 0;
static int64_t rtc_ms_start = 0;
static int64_t rtc_last_update = 0;
static bool rtc_valid = false;

enum class CqType { CQ, CQSOTA, CQPOTA, CQQRP, CQFD, CQFREETEXT };
enum class OffsetSrc { RANDOM, CURSOR, RX };
enum class RadioType { NONE, TRUSDX, QMX, KH1 };
static CqType g_cq_type = CqType::CQ;
static std::string g_cq_freetext = "FreeText";
static bool g_skip_tx1 = false;
static std::string g_free_text = "DE AG6AQ CM97";
static std::string g_call = "AG6AQ";
static std::string g_grid = "CM97";
static OffsetSrc g_offset_src = OffsetSrc::RANDOM;
static RadioType g_radio = RadioType::NONE;
static std::string g_ant = "EFHW";
static std::string g_comment1 = "MiniFT8 /Radio /Ant";
static std::string g_comment2;
static int menu_page = 0;
static int menu_edit_idx = -1;
static std::string menu_edit_buf;
static bool menu_long_edit = false;
static enum { LONG_NONE, LONG_FT, LONG_COMMENT } menu_long_kind = LONG_NONE;
static std::string menu_long_buf;
static std::string menu_long_backup;
static int menu_flash_idx = -1;          // absolute index to flash highlight
static int64_t menu_flash_deadline = 0;  // ms timestamp when flash ends
static int rx_flash_idx = -1;
static int64_t rx_flash_deadline = 0;
bool g_streaming = false;
static void draw_menu_view();
static void draw_battery_icon(int x, int y, int w, int h, int level, bool charging);
static void draw_status_view();
static void draw_status_line(int idx, const std::string& text, bool highlight);
void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui);
int64_t rtc_now_ms();
static void update_countdown();
static void menu_flash_tick();
static void rx_flash_tick();

static void ensure_usb() {
  if (usb_ready) return;
  usb_serial_jtag_driver_config_t cfg = {
    .tx_buffer_size = 1024,
    .rx_buffer_size = 4096,
  };
  if (usb_serial_jtag_driver_install(&cfg) == ESP_OK) {
    usb_ready = true;
  }
}

static void host_write_str(const std::string& s) {
  ensure_usb();
  if (!usb_ready) return;
  usb_serial_jtag_write_bytes((const uint8_t*)s.data(), s.size(), 0);
}

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

[[maybe_unused]] static esp_err_t decode_wav(const char* path) {
  ESP_LOGI(TAG, "Decoding %s", path);
  FILE* f = fopen(path, "rb");
  if (!f) {
    ESP_LOGE(TAG, "Failed to open %s", path);
    return ESP_FAIL;
  }

  WAVHeader hdr;
  if (fread(&hdr, 1, sizeof(hdr), f) != sizeof(hdr)) {
    ESP_LOGE(TAG, "Failed to read WAV header");
    fclose(f);
    return ESP_FAIL;
  }
  if (memcmp(hdr.riff, "RIFF", 4) != 0 || memcmp(hdr.wave, "WAVE", 4) != 0) {
    ESP_LOGE(TAG, "Invalid WAV header");
    fclose(f);
    return ESP_FAIL;
  }
  if (hdr.sample_rate != FT8_SAMPLE_RATE || hdr.num_channels != 1) {
    ESP_LOGE(TAG, "WAV must be mono %d Hz (got %u Hz, %u ch)", FT8_SAMPLE_RATE, hdr.sample_rate, hdr.num_channels);
    fclose(f);
    return ESP_FAIL;
  }

  const int bytes_per_sample = hdr.bits_per_sample / 8;

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

  float* chunk = (float*)malloc(sizeof(float) * mon.block_size);
  if (!chunk) {
    ESP_LOGE(TAG, "Chunk alloc failed");
    fclose(f);
    monitor_free(&mon);
    return ESP_ERR_NO_MEM;
  }

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

    // Simple per-block AGC to ~0.1 target level
    double acc = 0.0;
    for (int i = 0; i < mon.block_size; ++i) acc += fabsf(chunk[i]);
    float level = (float)(acc / mon.block_size);
    float gain = (level > 1e-6f) ? 0.1f / level : 1.0f;
    if (gain < 0.1f) gain = 0.1f;
    if (gain > 10.0f) gain = 10.0f;
    for (int i = 0; i < mon.block_size; ++i) {
      chunk[i] *= gain;
    }

    monitor_process(&mon, chunk);
  }

  free(chunk);
  fclose(f);

  if (mon.wf.num_blocks == 0) {
    ESP_LOGW(TAG, "No audio blocks processed");
    monitor_free(&mon);
    return ESP_FAIL;
  }
  decode_monitor_results(&mon, &mon_cfg, false); // defer UI to main loop on core1
  monitor_free(&mon);

  return ESP_OK;
}

static void redraw_tx_view() {
  std::vector<std::string> qtext;
  std::vector<bool> marks;
  qtext.reserve(tx_queue.size());
  marks.reserve(tx_queue.size());
  for (auto &e : tx_queue) {
    qtext.push_back(e.text);
    marks.push_back(e.mark_delete);
  }
  ui_draw_tx(tx_next.text, qtext, tx_page, -1, marks);
}

static void draw_band_view() {
  std::vector<std::string> lines;
  lines.reserve(g_bands.size());
  for (size_t i = 0; i < g_bands.size(); ++i) {
    std::string freq_str;
    if ((int)i == band_edit_idx && !band_edit_buffer.empty()) {
      freq_str = band_edit_buffer;
    } else {
      freq_str = std::to_string(g_bands[i].freq);
    }
    lines.push_back(std::string(g_bands[i].name) + ": " + freq_str);
  }
  ui_draw_list(lines, band_page, band_edit_idx);
}

static const char* beacon_name(BeaconMode m) {
  switch (m) {
    case BeaconMode::OFF: return "OFF";
    case BeaconMode::EVEN: return "EVEN";
    case BeaconMode::EVEN2: return "EVEN2";
    case BeaconMode::ODD: return "ODD";
    case BeaconMode::ODD2: return "ODD2";
  }
  return "OFF";
}

static const char* cq_type_name(CqType t) {
  switch (t) {
    case CqType::CQ: return "CQ";
    case CqType::CQSOTA: return "CQ SOTA";
    case CqType::CQPOTA: return "CQ POTA";
    case CqType::CQQRP: return "CQ QRP";
    case CqType::CQFD: return "CQ FD";
    case CqType::CQFREETEXT: return "FreeText";
  }
  return "CQ";
}

static const char* offset_name(OffsetSrc o) {
  switch (o) {
    case OffsetSrc::RANDOM: return "Random";
    case OffsetSrc::CURSOR: return "Cursor";
    case OffsetSrc::RX: return "RX";
  }
  return "Random";
}

static const char* radio_name(RadioType r) {
  switch (r) {
    case RadioType::NONE: return "None";
    case RadioType::TRUSDX: return "truSDX";
    case RadioType::QMX: return "QMX";
    case RadioType::KH1: return "KH1";
  }
  return "None";
}

static std::string expand_comment1() {
  std::string out = g_comment1;
  auto repl = [](std::string& s, const std::string& from, const std::string& to) {
    size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
      s.replace(pos, from.size(), to);
      pos += to.size();
    }
  };
  repl(out, "/Radio", radio_name(g_radio));
  repl(out, "/Ant", g_ant);
  return out;
}

static std::string battery_status_line() {
  int level = (int)M5.Power.getBatteryLevel();
  bool charging = M5.Power.isCharging();
  if (level < 0 || level > 100) level = 0;
  char buf[32];
  snprintf(buf, sizeof(buf), "Batt:%3d%%%s", level, charging ? " CHG" : "");
  return buf;
}

static std::string elide_right(const std::string& s, size_t max_len = 22) {
  if (s.size() <= max_len) return s;
  if (max_len <= 3) return s.substr(s.size() - max_len);
  return std::string("...") + s.substr(s.size() - (max_len - 3));
}

static std::string head_trim(const std::string& s, size_t max_len = 16) {
  if (s.size() <= max_len) return s;
  if (max_len == 0) return "";
  if (max_len == 1) return ">";
  return s.substr(0, max_len - 1) + ">";
}

static std::string highlight_pos(const std::string& s, int pos) {
  if (pos < 0 || pos >= (int)s.size()) return s;
  std::string out;
  out.reserve(s.size() + 2);
  out.append(s, 0, pos);
  out.push_back('[');
  out.push_back(s[pos]);
  out.push_back(']');
  out.append(s, pos + 1, std::string::npos);
  return out;
}

static void draw_status_view();

static bool rtc_set_from_strings() {
  int y, M, d, h, m, s;
  if (sscanf(g_date.c_str(), "%d-%d-%d", &y, &M, &d) != 3) return false;
  if (sscanf(g_time.c_str(), "%d:%d:%d", &h, &m, &s) != 3) return false;
  struct tm t = {};
  t.tm_year = y - 1900;
  t.tm_mon = M - 1;
  t.tm_mday = d;
  t.tm_hour = h;
  t.tm_min = m;
  t.tm_sec = s;
  time_t epoch = mktime(&t);
  if (epoch == (time_t)-1) return false;
  rtc_epoch_base = epoch;
  rtc_ms_start = esp_timer_get_time() / 1000;
  rtc_last_update = rtc_ms_start;
  rtc_valid = true;
  return true;
}

static void rtc_update_strings() {
  if (!rtc_valid) return;
  struct tm t;
  time_t now = rtc_epoch_base + (esp_timer_get_time() / 1000 - rtc_ms_start) / 1000;
  localtime_r(&now, &t);
  char buf_date[32];
  snprintf(buf_date, sizeof(buf_date), "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
  g_date = buf_date;
  char buf_time[16];
  snprintf(buf_time, sizeof(buf_time), "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
  g_time = buf_time;
}

int64_t rtc_now_ms() {
  if (!rtc_valid) {
    return esp_timer_get_time() / 1000;
  }
  return (int64_t)rtc_epoch_base * 1000 + (esp_timer_get_time() / 1000 - rtc_ms_start);
}

static void rtc_tick() {
  if (!rtc_valid) {
    rtc_set_from_strings();
    if (!rtc_valid) return;
  }
  int64_t now_ms = esp_timer_get_time() / 1000;
  if (now_ms - rtc_last_update >= 1000) {
    rtc_last_update += 1000; // Increment by interval to prevent drift accumulation
    if (status_edit_idx != 5) { // keep time ticking unless editing time
      std::string old_date = g_date;
      std::string old_time = g_time;
      rtc_update_strings();
      if (ui_mode == UIMode::STATUS && status_edit_idx == -1) {
        if (old_date != g_date) {
          draw_status_line(4, std::string("Date: ") + g_date, false);
        }
        if (old_time != g_time) {
          draw_status_line(5, std::string("Time: ") + g_time, false);
        }
      }
    }
  }
}

static void update_countdown() {
  int64_t now_ms = rtc_now_ms();
  int64_t slot_idx = now_ms / 15000;
  int64_t slot_ms = now_ms % 15000;
  static int64_t last_slot_idx = -1;
  static int last_sec = -1;
  int sec = (int)(slot_ms / 1000);
  if (slot_idx != last_slot_idx || sec != last_sec) {
    float frac = (float)slot_ms / 15000.0f;
    bool even = (slot_idx % 2) == 0;
    ui_draw_countdown(frac, even);
    last_slot_idx = slot_idx;
    last_sec = sec;
  }
}

static void menu_flash_tick() {
  if (menu_flash_idx < 0) return;
  int64_t now = rtc_now_ms();
  if (now >= menu_flash_deadline) {
    menu_flash_idx = -1;
    if (ui_mode == UIMode::MENU && !menu_long_edit && menu_edit_idx < 0) {
      draw_menu_view();
    }
  }
}

static void rx_flash_tick() {
  if (rx_flash_idx < 0) return;
  int64_t now = rtc_now_ms();
  if (now >= rx_flash_deadline) {
    rx_flash_idx = -1;
    rx_flash_deadline = 0;
    if (ui_mode == UIMode::RX) {
      ui_draw_rx();
    }
  }
}

void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui) {
  const int max_cand = 50;
  ftx_candidate_t candidates[max_cand];
  int num_candidates = ftx_find_candidates(&mon->wf, max_cand, candidates, 5);
  ESP_LOGI(TAG, "Candidates found: %d", num_candidates);

  std::vector<UiRxLine> ui_lines;
  if (num_candidates > 0) {
    int decodedCount = 0;
    std::vector<std::string> seen;
    for (int i = 0; i < num_candidates; ++i) {
      ftx_message_t message;
      ftx_decode_status_t status;
      memset(&message, 0, sizeof(message));
      memset(&status, 0, sizeof(status));

      if (!ftx_decode_candidate(&mon->wf, &candidates[i], 25, &message, &status)) {
        continue;
      }

      char text[40] = {0};
      char call_to[14] = {0};
      char call_de[14] = {0};
      char extra[10] = {0};
      ftx_field_t fields[FTX_MAX_MESSAGE_FIELDS];
      ftx_message_rc_t rc = ftx_message_decode_std(&message, NULL, call_to, call_de, extra, fields);
      if (rc == FTX_MESSAGE_RC_OK) {
        snprintf(text, sizeof(text), "%s %s %s", call_to, call_de, extra);
      } else {
        ftx_message_decode_free(&message, text);
      }

      if (text[0] != '\0') {
        float freq_hz = (mon->min_bin + candidates[i].freq_offset +
                         candidates[i].freq_sub / (float)cfg->freq_osr) / mon->symbol_period;
        float time_s = (candidates[i].time_offset +
                        candidates[i].time_sub / (float)cfg->time_osr) * mon->symbol_period;
        ESP_LOGI(TAG, "Decoded[%d] t=%.2fs f=%.1fHz snr(est)=%d : %s",
                 decodedCount, time_s, freq_hz, candidates[i].score, text);
        // Deduplicate exact text
        if (std::find(seen.begin(), seen.end(), std::string(text)) != seen.end()) {
          continue;
        }
        seen.push_back(std::string(text));

        UiRxLine line;
        line.text = text;
        line.snr = candidates[i].score;
        if (line.text.rfind("CQ ", 0) == 0 || line.text == "CQ") {
          line.is_cq = true;
        }
        ui_lines.push_back(line);
        decodedCount++;
        if (decodedCount >= 32) break; // safety cap
      }
    }
    if (decodedCount == 0) {
      ESP_LOGW(TAG, "Candidates present but no messages decoded");
    }
  } else {
    ESP_LOGW(TAG, "No candidates found");
  }

  // Sort into reply-to-me, CQ, other; preserve order within each group
  std::vector<UiRxLine> to_me;
  std::vector<UiRxLine> cqs;
  std::vector<UiRxLine> others;
  for (auto& l : ui_lines) {
    std::string up = l.text;
    for (auto& ch : up) ch = toupper((unsigned char)ch);
    std::string mycall = g_call;
    for (auto& ch : mycall) ch = toupper((unsigned char)ch);
    if (!mycall.empty() && up.find(mycall) != std::string::npos) {
      l.is_to_me = true;
      to_me.push_back(l);
    } else if (l.is_cq) {
      cqs.push_back(l);
    } else {
      others.push_back(l);
    }
  }
  std::vector<UiRxLine> merged;
  merged.reserve(to_me.size() + cqs.size() + others.size());
  merged.insert(merged.end(), to_me.begin(), to_me.end());
  merged.insert(merged.end(), cqs.begin(), cqs.end());
  merged.insert(merged.end(), others.begin(), others.end());
  if (merged.size() > 12) merged.resize(12);

  g_rx_lines = merged;
  if (update_ui) {
    ui_set_rx_list(g_rx_lines);
    ui_draw_rx();
    char buf[64];
    snprintf(buf, sizeof(buf), "Heap %u", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    debug_log_line(buf);
  } else {
    g_rx_dirty = true;
  }
}

static void draw_menu_long_edit() {
  std::vector<std::string> lines(6, "");
  std::string text = menu_long_buf;
  size_t idx = 0;
  int line = 0;
  while (idx < text.size() && line < 6) {
    size_t chunk = std::min<size_t>(18, text.size() - idx);
    lines[line] = text.substr(idx, chunk);
    idx += chunk;
    line++;
  }
  // cursor indicator on the last line
  if (line == 0) {
    lines[0] = "_";
  } else {
    if (lines[line - 1].size() < 20) lines[line - 1].push_back('_');
    else if (line < 6) lines[line] = "_";
  }
  ui_draw_list(lines, 0, -1);
}

static void log_tones(const uint8_t* tones, size_t n) {
  std::string line;
  for (size_t i = 0; i < n; ++i) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%u", (unsigned)tones[i]);
    line += buf;
    if ((i + 1) % 20 == 0 || i + 1 == n) {
      debug_log_line(line);
      line.clear();
    }
  }
}

static void encode_and_log_tx_next() {
  ftx_message_t msg;
  ftx_message_rc_t rc = ftx_message_encode(&msg, NULL, tx_next.text.c_str());
  if (rc != FTX_MESSAGE_RC_OK) {
    debug_log_line("Encode failed");
    return;
  }
  uint8_t tones[79] = {0};
  ft8_encode(msg.payload, tones);
  debug_log_line(std::string("Tones for '") + tx_next.text + "'");
  log_tones(tones, 79);
}

static void draw_menu_view() {
  if (menu_long_edit) {
    draw_menu_long_edit();
    return;
  }
  std::vector<std::string> lines;
  lines.reserve(12);

  std::string cq_line = std::string("CQ Type:");
  if (g_cq_type == CqType::CQFREETEXT) cq_line += g_cq_freetext;
  else cq_line += cq_type_name(g_cq_type);
  lines.push_back(cq_line);
  lines.push_back(std::string("SkipTX1:") + (g_skip_tx1 ? "ON" : "OFF"));
  lines.push_back("Send FreeText");
  lines.push_back(std::string("F:") + head_trim(menu_edit_idx == 3 ? menu_edit_buf : g_free_text, 16));
  lines.push_back(std::string("Call:") + elide_right(menu_edit_idx == 4 ? menu_edit_buf : g_call));
  lines.push_back(std::string("Grid:") + elide_right(menu_edit_idx == 5 ? menu_edit_buf : g_grid));

  lines.push_back(std::string("Offset:") + offset_name(g_offset_src));
  lines.push_back(std::string("Radio:") + radio_name(g_radio));
  lines.push_back(std::string("Antenna:") + elide_right(menu_edit_idx == 8 ? menu_edit_buf : g_ant));
  // Combine comments into one line; Comment2 hidden (placeholder NA)
  lines.push_back(std::string("C:") + head_trim(menu_edit_idx == 9 ? menu_edit_buf : expand_comment1(), 16));
  lines.push_back("");
  lines.push_back(battery_status_line());

  int highlight_abs = -1;
  int64_t now = rtc_now_ms();
  if (menu_edit_idx >= 0) {
    highlight_abs = menu_edit_idx;
  } else if (menu_flash_idx >= 0 && now < menu_flash_deadline) {
    highlight_abs = menu_flash_idx;
  } else {
    menu_flash_idx = -1;
  }
  // Auto-clear flash after timeout
  if (menu_flash_idx >= 0 && now >= menu_flash_deadline) {
    menu_flash_idx = -1;
  }
  ui_draw_list(lines, menu_page, highlight_abs);
  // Draw battery icon on visible battery line
  int battery_abs_idx = (int)lines.size() - 1;
  if (menu_page == (battery_abs_idx / 6)) {
    int line_on_page = battery_abs_idx % 6;
    const int line_h = 19;
    const int start_y = 18 + 3 + 3; // WATERFALL_H + COUNTDOWN_H + gap
    int y = start_y + line_on_page * line_h + 3;
    int level = (int)M5.Power.getBatteryLevel();
    bool charging = M5.Power.isCharging();
    draw_battery_icon(190, y, 24, 12, level, charging);
  }
}

static void draw_status_view() {
  std::string lines[6];
  lines[0] = std::string("Beacon: ") + beacon_name(g_beacon);
  lines[1] = "Offset: " + std::to_string(g_offset_hz);
  lines[2] = std::string("Band: ") + g_bands[g_band_sel].name + " " + std::to_string(g_bands[g_band_sel].freq);
  lines[3] = std::string("Tune: ") + (g_tune ? "ON" : "OFF");
  if (status_edit_idx == 4 && !status_edit_buffer.empty()) {
    lines[4] = std::string("Date: ") + highlight_pos(status_edit_buffer, status_cursor_pos);
  } else {
    lines[4] = std::string("Date: ") + g_date;
  }
  if (status_edit_idx == 5 && !status_edit_buffer.empty()) {
    lines[5] = std::string("Time: ") + highlight_pos(status_edit_buffer, status_cursor_pos);
  } else {
    lines[5] = std::string("Time: ") + g_time;
  }
  for (int i = 0; i < 6; ++i) {
    bool hl = (status_edit_idx == i);
    draw_status_line(i, lines[i], hl);
  }
}

static void debug_log_line(const std::string& msg) {
  if (g_debug_lines.size() >= DEBUG_MAX_LINES) {
    g_debug_lines.erase(g_debug_lines.begin());
  }
  g_debug_lines.push_back(msg);
  debug_page = (int)((g_debug_lines.size() - 1) / 6);
  if (ui_mode == UIMode::DEBUG) {
    ui_draw_debug(g_debug_lines, debug_page);
  }
}

static std::string trim_copy(const std::string& s) {
  size_t b = 0, e = s.size();
  while (b < e && isspace((unsigned char)s[b])) ++b;
  while (e > b && isspace((unsigned char)s[e - 1])) --e;
  return s.substr(b, e - b);
}

static uint32_t parse_crc_hex(const std::string& hex) {
  if (hex.empty()) return 0;
  char* end = nullptr;
  unsigned long v = strtoul(hex.c_str(), &end, 16);
  if (end == hex.c_str() || *end != '\0') return 0;
  return (uint32_t)v;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = crc ^ 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

static void host_debug_hex8(const char* prefix, const uint8_t* b) {
  char buf[64];
  int n = snprintf(buf, sizeof(buf), "%s ", prefix);
  for (int i = 0; i < 8 && n + 3 < (int)sizeof(buf); ++i) {
    n += snprintf(buf + n, sizeof(buf) - n, "%02X ", b[i]);
  }
  if (n > 0 && buf[n - 1] == ' ') buf[n - 1] = 0;
  host_write_str(std::string(buf) + "\r\n");
}

static void host_handle_line(const std::string& line_in) {
  bool send_prompt = true;
  std::string line = trim_copy(line_in);
  if (line.empty()) { host_write_str(HOST_PROMPT); return; }
  debug_log_line(std::string("[USB RX] ") + line);
  std::string echo = std::string("ECHO: ") + line + "\r\n";
  host_write_str(echo);

  auto to_upper = [](std::string s) {
    for (auto& c : s) c = toupper((unsigned char)c);
    return s;
  };
  std::istringstream iss(line);
  std::string cmd;
  iss >> cmd;
  std::string cmd_up = to_upper(cmd);
  std::string rest;
  std::getline(iss, rest);
  rest = trim_copy(rest);

  auto send = [](const std::string& msg) { host_write_str(msg + "\r\n"); };

  if (cmd_up == "WRITE" || cmd_up == "APPEND") {
    std::istringstream rs(rest);
    std::string fname;
    rs >> fname;
    std::string content;
    std::getline(rs, content);
    content = trim_copy(content);
    if (fname.empty()) {
      send("ERROR: filename required");
    } else {
      std::string path = std::string("/spiffs/") + fname;
      const char* mode = (cmd_up == "WRITE") ? "w" : "a";
      FILE* f = fopen(path.c_str(), mode);
      if (!f) send("ERROR: open failed");
      else { fwrite(content.data(), 1, content.size(), f); fclose(f); send("OK"); }
    }
  } else if (cmd_up == "READ") {
    if (rest.empty()) send("ERROR: filename required");
    else {
      std::string path = std::string("/spiffs/") + rest;
      FILE* f = fopen(path.c_str(), "r");
      if (!f) send("ERROR: open failed");
      else {
        char buf[128];
        while (fgets(buf, sizeof(buf), f)) host_write_str(std::string(buf));
        fclose(f);
        send("OK");
      }
    }
  } else if (cmd_up == "DELETE") {
    if (rest.empty()) send("ERROR: filename required");
    else {
      std::string path = std::string("/spiffs/") + rest;
      if (unlink(path.c_str()) == 0) send("OK"); else send("ERROR: delete failed");
    }
  } else if (cmd_up == "LIST") {
    DIR* d = opendir("/spiffs");
    if (!d) send("ERROR: opendir failed");
    else {
      struct dirent* ent;
      while ((ent = readdir(d)) != nullptr) {
        send(ent->d_name);
      }
      closedir(d);
      send("OK");
    }
  } else if (cmd_up == "WRITEBIN") {
    std::istringstream rs(rest);
    std::string fname;
    size_t size = 0;
    std::string crc_hex;
    rs >> fname >> size >> crc_hex;
    uint32_t crc_exp = parse_crc_hex(crc_hex);
    if (fname.empty() || size == 0 || crc_hex.empty()) {
      send("ERROR: filename, size, crc32_hex required");
    } else if (host_bin_active) {
      send("ERROR: binary upload in progress");
    } else {
      std::string path = std::string("/spiffs/") + fname;
      FILE* f = fopen(path.c_str(), "wb");
      if (!f) {
        send("ERROR: open failed");
      } else {
        host_bin_path = path;
        host_bin_active = true;
        host_bin_remaining = size;
        host_bin_fp = f;
        host_bin_crc = 0;
        host_bin_expected_crc = crc_exp;
        host_bin_received = 0;
        host_bin_buf.clear();
        host_bin_buf.reserve(HOST_BIN_CHUNK);
        host_bin_chunk_expect = (host_bin_remaining < HOST_BIN_CHUNK) ? host_bin_remaining : HOST_BIN_CHUNK;
        host_bin_first_filled = 0;
        memset(host_bin_first8, 0, sizeof(host_bin_first8));
        memset(host_bin_last8, 0, sizeof(host_bin_last8));
        host_write_str("OK: send " + std::to_string(size) + " bytes, chunk " + std::to_string(HOST_BIN_CHUNK) + " +4crc\r\n");
        send_prompt = false; // prompt after binary upload completes
      }
    }
  } else if (cmd_up == "INFO") {
    send("Heap: " + std::to_string(heap_caps_get_free_size(MALLOC_CAP_DEFAULT)));
    send("OK");
  } else if (cmd_up == "HELP") {
    for (auto& l : g_host_lines) send(l);
    send("RX decode: press 'x' in RX to stream /kfs40m.wav");
  } else if (cmd_up == "EXIT") {
    send("OK: exit host");
    enter_mode(UIMode::RX);
    return;
  } else {
    send("ERROR: Unknown command. Type HELP.");
  }

  if (send_prompt) host_write_str(std::string(HOST_PROMPT));
}

static void poll_host_uart() {
  ensure_usb();
  if (!usb_ready) return;
  uint8_t buf[512];
  while (true) {
    int r = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
    if (r <= 0) break;
    for (int i = 0; i < r; ) {
      if (host_bin_active) {
        // Skip any stray CR/LF before first payload byte
        if (host_bin_received == 0 && host_bin_buf.empty() && (buf[i] == '\r' || buf[i] == '\n')) {
          ++i;
          continue;
        }
        size_t payload_need = host_bin_chunk_expect;
        size_t total_need = payload_need + 4; // payload + crc32 trailer
        size_t avail = (size_t)(r - i);
        size_t copy = total_need - host_bin_buf.size();
        if (copy > avail) copy = avail;
        host_bin_buf.insert(host_bin_buf.end(), buf + i, buf + i + copy);
        i += (int)copy;

        if (host_bin_buf.size() >= total_need) {
          size_t payload_len = payload_need;
          uint32_t recv_crc = (uint32_t(host_bin_buf[payload_len])) |
                              (uint32_t(host_bin_buf[payload_len + 1]) << 8) |
                              (uint32_t(host_bin_buf[payload_len + 2]) << 16) |
                              (uint32_t(host_bin_buf[payload_len + 3]) << 24);
          uint32_t calc_crc = crc32_update(0, host_bin_buf.data(), payload_len);
          if (calc_crc != recv_crc) {
            char dbg[128];
            snprintf(dbg, sizeof(dbg), "ERROR: chunk crc off=%u len=%u calc=%08X recv=%08X\r\n",
                     (unsigned)(host_bin_received + payload_len), (unsigned)payload_len,
                     (unsigned)calc_crc, (unsigned)recv_crc);
            host_write_str(std::string(dbg));
            // Send first/last bytes of the chunk to compare
            if (payload_len >= 8) host_debug_hex8("DBG CHUNK FIRST8", host_bin_buf.data());
            if (payload_len >= 8) host_debug_hex8("DBG CHUNK LAST8", host_bin_buf.data() + payload_len - 8);
            if (payload_len < 8) host_debug_hex8("DBG CHUNK PART", host_bin_buf.data());
            // Also report the CRC trailer bytes as seen
            uint8_t crc_bytes[4] = {
              host_bin_buf[payload_len],
              host_bin_buf[payload_len + 1],
              host_bin_buf[payload_len + 2],
              host_bin_buf[payload_len + 3]
            };
            host_debug_hex8("DBG CRC BYTES", crc_bytes);
            fclose(host_bin_fp);
            host_bin_fp = nullptr;
            host_bin_active = false;
            host_bin_remaining = 0;
            host_bin_buf.clear();
            host_write_str(std::string(HOST_PROMPT));
            continue;
          }

          // Capture first/last bytes for debugging
          if (host_bin_first_filled < 8) {
            size_t need = 8 - host_bin_first_filled;
            if (need > payload_len) need = payload_len;
            memcpy(host_bin_first8 + host_bin_first_filled, host_bin_buf.data(), need);
            host_bin_first_filled += need;
          }
          // update last8 buffer
          if (payload_len >= 8) {
            memcpy(host_bin_last8, host_bin_buf.data() + payload_len - 8, 8);
          } else {
            // shift existing and append
            size_t shift = (payload_len + 8 > 8) ? (payload_len) : payload_len;
            if (shift > 0) {
              memmove(host_bin_last8, host_bin_last8 + shift, 8 - shift);
              memcpy(host_bin_last8 + (8 - payload_len), host_bin_buf.data(), payload_len);
            }
          }

          size_t written = fwrite(host_bin_buf.data(), 1, payload_len, host_bin_fp);
          if (written != payload_len) {
            host_write_str("ERROR: write failed\r\n");
            fclose(host_bin_fp);
            host_bin_fp = nullptr;
            host_bin_active = false;
            host_bin_remaining = 0;
            host_bin_buf.clear();
            host_write_str(std::string(HOST_PROMPT));
            continue;
          }
          host_bin_crc = crc32_update(host_bin_crc, host_bin_buf.data(), payload_len);
          host_bin_remaining -= payload_len;
          host_bin_received += payload_len;
          host_bin_buf.clear();
          host_write_str("ACK " + std::to_string(host_bin_received) + "\r\n");

        if (host_bin_remaining == 0) {
          fclose(host_bin_fp);
          host_bin_fp = nullptr;
          host_bin_active = false;
          uint32_t crc_final = host_bin_crc;
          // Reopen file to send first/last 8 bytes for debugging
          host_debug_hex8("DBG FIRST8", host_bin_first8);
          host_debug_hex8("DBG LAST8", host_bin_last8);
          char crc_line[64];
          snprintf(crc_line, sizeof(crc_line), "DBG CRC %08X EXPECT %08X\r\n",
                   (unsigned)crc_final, (unsigned)host_bin_expected_crc);
          host_write_str(std::string(crc_line));
          if (crc_final != host_bin_expected_crc) {
            host_write_str("ERROR: crc mismatch\r\n");
          } else {
            host_write_str("OK crc " + std::to_string(crc_final) + "\r\n");
          }
          host_write_str(std::string(HOST_PROMPT));
        } else {
          host_bin_chunk_expect = (host_bin_remaining < HOST_BIN_CHUNK) ? host_bin_remaining : HOST_BIN_CHUNK;
        }
      }
      continue;
      }
      char ch = (char)buf[i++];
      if (ch == '\r' || ch == '\n') {
        if (!host_input.empty()) {
          host_handle_line(host_input);
          host_input.clear();
        } else {
          host_write_str(std::string(HOST_PROMPT));
        }
      } else if (ch == 0x08 || ch == 0x7f) {
        if (!host_input.empty()) host_input.pop_back();
      } else if (ch >= 32 && ch < 127) {
        host_input.push_back(ch);
      }
    }
  }
}


static void load_station_data() {
  FILE* f = fopen(STATION_FILE, "r");
  if (!f) return;
  char line[64];
  while (fgets(line, sizeof(line), f)) {
    int idx = -1;
    int val = 0;
    if (sscanf(line, "band%d=%d", &idx, &val) == 2) {
      if (idx >= 0 && idx < (int)g_bands.size()) {
        g_bands[idx].freq = val;
      }
    } else if (sscanf(line, "beacon=%d", &val) == 1) {
      if (val >= 0 && val <= 4) g_beacon = (BeaconMode)val;
    } else if (sscanf(line, "offset=%d", &val) == 1) {
      g_offset_hz = val;
    } else if (sscanf(line, "band_sel=%d", &val) == 1) {
      if (val >= 0 && val < (int)g_bands.size()) g_band_sel = val;
    } else if (sscanf(line, "date=%63s", line) == 1) {
      g_date = line;
    } else if (sscanf(line, "time=%63s", line) == 1) {
      g_time = line;
    } else if (sscanf(line, "cq_type=%d", &val) == 1) {
      if (val >= 0 && val <= 5) g_cq_type = (CqType)val;
    } else if (sscanf(line, "skiptx1=%d", &val) == 1) {
      g_skip_tx1 = (val != 0);
    } else if (sscanf(line, "offset_src=%d", &val) == 1) {
      if (val >= 0 && val <= 2) g_offset_src = (OffsetSrc)val;
    } else if (sscanf(line, "radio=%d", &val) == 1) {
      if (val >= 0 && val <= 3) g_radio = (RadioType)val;
    } else if (strncmp(line, "cq_ft=", 6) == 0) {
      g_cq_freetext = trim_copy(line + 6);
    } else if (strncmp(line, "free_text=", 10) == 0) {
      g_free_text = trim_copy(line + 10);
    } else if (strncmp(line, "call=", 5) == 0) {
      g_call = trim_copy(line + 5);
    } else if (strncmp(line, "grid=", 5) == 0) {
      g_grid = trim_copy(line + 5);
    } else if (strncmp(line, "ant=", 4) == 0) {
      g_ant = trim_copy(line + 4);
    } else if (strncmp(line, "comment1=", 9) == 0) {
      g_comment1 = trim_copy(line + 9);
    } else if (strncmp(line, "comment2=", 9) == 0) {
      g_comment2 = trim_copy(line + 9);
    }
  }
  fclose(f);
  rtc_set_from_strings();
}

static void save_station_data() {
  FILE* f = fopen(STATION_FILE, "w");
  if (!f) {
    ESP_LOGE(TAG, "Failed to open %s for write", STATION_FILE);
    return;
  }
  for (size_t i = 0; i < g_bands.size(); ++i) {
    fprintf(f, "band%u=%d\n", (unsigned)i, g_bands[i].freq);
  }
  fprintf(f, "beacon=%d\n", (int)g_beacon);
  fprintf(f, "offset=%d\n", g_offset_hz);
  fprintf(f, "band_sel=%d\n", g_band_sel);
  fprintf(f, "date=%s\n", g_date.c_str());
  fprintf(f, "time=%s\n", g_time.c_str());
  fprintf(f, "cq_type=%d\n", (int)g_cq_type);
  fprintf(f, "cq_ft=%s\n", g_cq_freetext.c_str());
  fprintf(f, "skiptx1=%d\n", g_skip_tx1 ? 1 : 0);
  fprintf(f, "free_text=%s\n", g_free_text.c_str());
  fprintf(f, "call=%s\n", g_call.c_str());
  fprintf(f, "grid=%s\n", g_grid.c_str());
  fprintf(f, "offset_src=%d\n", (int)g_offset_src);
  fprintf(f, "radio=%d\n", (int)g_radio);
  fprintf(f, "ant=%s\n", g_ant.c_str());
  fprintf(f, "comment1=%s\n", g_comment1.c_str());
  fprintf(f, "comment2=%s\n", g_comment2.c_str());
  fclose(f);
}

static void enter_mode(UIMode new_mode) {
  ESP_LOGI("MODE", "Switching %d -> %d", (int)ui_mode, (int)new_mode);
  {
    char buf[32];
    snprintf(buf, sizeof(buf), "MODE %d->%d", (int)ui_mode, (int)new_mode);
    debug_log_line(buf);
  }
  if (ui_mode == UIMode::TX && new_mode != UIMode::TX) {
    tx_commit_deletions();
  }
  if (ui_mode == UIMode::STATUS && new_mode != UIMode::STATUS) {
    status_edit_idx = -1;
    status_edit_buffer.clear();
  }
  ui_mode = new_mode;
  rx_flash_idx = -1;
  switch (ui_mode) {
    case UIMode::RX:
      // Force RX list redraw
      ui_force_redraw_rx();
      ui_draw_rx();
      break;
    case UIMode::TX:
      tx_page = 0;
      tx_selected_idx = -1;
      redraw_tx_view();
      break;
    case UIMode::BAND:
      band_page = 0;
      band_edit_idx = -1;
      draw_band_view();
      break;
    case UIMode::MENU:
      menu_page = 0;
      menu_edit_idx = -1;
      menu_edit_buf.clear();
      draw_menu_view();
      break;
    case UIMode::DEBUG:
      debug_page = (int)((g_debug_lines.size() - 1) / 6);
      ui_draw_debug(g_debug_lines, debug_page);
      break;
    case UIMode::HOST:
      ui_draw_list(g_host_lines, 0, -1);
      host_input.clear();
      ensure_usb();
      if (usb_ready) {
        host_write_str("READY\r\n");
        host_write_str(std::string(HOST_PROMPT));
      } else {
        debug_log_line("USB serial not ready");
      }
      break;
    case UIMode::LIST:
      list_page = 0;
      ui_draw_list(g_list_lines, list_page, -1);
      break;
    case UIMode::STATUS:
      status_edit_idx = -1;
      status_cursor_pos = -1;
      draw_status_view();
      break;
  }
}

static void app_task_core1(void* /*param*/) {
  esp_vfs_spiffs_conf_t conf = {
    .base_path = "/spiffs",
    .partition_label = NULL,
    .max_files = 5,
    .format_if_mount_failed = false
  };
  ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

  ui_init();
  ui_draw_waterfall();
  std::vector<UiRxLine> empty;
  ui_set_rx_list(empty);
  ui_draw_rx();
  tx_init();
  ui_mode = UIMode::RX;
  load_station_data();

  ESP_LOGI(TAG, "Free heap: %u, internal: %u, 8bit: %u",
           heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           heap_caps_get_free_size(MALLOC_CAP_8BIT));
  {
    char buf[64];
    snprintf(buf, sizeof(buf), "Heap %u", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    debug_log_line(buf);
  }

  // UI loop
  char last_key = 0;
  while (true) {
    M5Cardputer.update();
    M5Cardputer.Keyboard.updateKeysState();
    auto &state = M5Cardputer.Keyboard.keysState();
    char c = 0;
    if (!state.word.empty()) {
      c = state.word.back();
      state.word.clear();  // consume key
    } else if (state.del) {
      c = 0x7f;  // treat delete/backspace
    } else if (state.enter) {
      c = '\n';  // enter/return
    }

    rtc_tick();
    update_countdown();

    // HOST mode: always service USB input; only react to 'h'/'H' to exit
  if (ui_mode == UIMode::HOST) {
    poll_host_uart();
    if (host_bin_active) { // block keyboard exits during binary upload
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
      }
      // Keyboard path uses normal repeat suppression; USB path stays always-on
      if (c == 0) {
        last_key = 0;
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
      if (c == last_key) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
      last_key = c;
      if (c == 'h' || c == 'H') {
        enter_mode(UIMode::RX);
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (c == 0) {
      if (g_rx_dirty && ui_mode == UIMode::RX) {
        ui_set_rx_list(g_rx_lines);
        ui_draw_rx(rx_flash_idx);
        g_rx_dirty = false;
      }
      menu_flash_tick();
      rx_flash_tick();
      last_key = 0;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
  if (c == last_key) {
    vTaskDelay(pdMS_TO_TICKS(10));
    continue;
  }
  last_key = c;

  rtc_tick();
  update_countdown();
  menu_flash_tick();
  rx_flash_tick();

  auto try_start_stream = [&](char key) {
    if (!g_streaming && (key == 'x' || key == 'X') && ui_mode != UIMode::HOST) {
      g_streaming = true;
      xTaskCreatePinnedToCore(stream_wav_task, "stream_wav", 8192, (void*)"/spiffs/kfs40m.wav", 4, NULL, 1);
      return true;
    }
    return false;
  };

  try_start_stream(c);

  if (g_rx_dirty && ui_mode == UIMode::RX) {
    ui_set_rx_list(g_rx_lines);
    ui_draw_rx(rx_flash_idx);
    g_rx_dirty = false;
    rx_flash_idx = -1;
    rx_flash_deadline = 0;
  }

  bool switched = false;
  auto cancel_status_edit = []() {
    if (status_edit_idx != -1) {
      status_edit_idx = -1;
      status_edit_buffer.clear();
      status_cursor_pos = -1;
    }
  };
  if (!(ui_mode == UIMode::MENU && (menu_edit_idx >= 0 || menu_long_edit))) {
      // Mode switch keys (disabled while editing in MENU)
      if (c == 'r' || c == 'R') { cancel_status_edit(); enter_mode(UIMode::RX); ui_force_redraw_rx(); ui_draw_rx(); switched = true; }
      else if (c == 't' || c == 'T') { cancel_status_edit(); enter_mode(ui_mode == UIMode::TX ? UIMode::RX : UIMode::TX); switched = true; }
      else if (c == 'b' || c == 'B') { cancel_status_edit(); enter_mode(ui_mode == UIMode::BAND ? UIMode::RX : UIMode::BAND); switched = true; }
      else if (c == 'm' || c == 'M') { cancel_status_edit(); enter_mode(ui_mode == UIMode::MENU ? UIMode::RX : UIMode::MENU); switched = true; }
      else if (c == 'h' || c == 'H') { cancel_status_edit(); enter_mode(ui_mode == UIMode::HOST ? UIMode::RX : UIMode::HOST); switched = true; }
      else if (c == 'd' || c == 'D') { cancel_status_edit(); enter_mode(ui_mode == UIMode::DEBUG ? UIMode::RX : UIMode::DEBUG); switched = true; }
      else if (c == 'l' || c == 'L') { cancel_status_edit(); enter_mode(ui_mode == UIMode::LIST ? UIMode::RX : UIMode::LIST); switched = true; }
      else if (c == 's' || c == 'S') { cancel_status_edit(); enter_mode(ui_mode == UIMode::STATUS ? UIMode::RX : UIMode::STATUS); switched = true; }
    }

  if (!switched && c) {
    // Mode-specific handling
    switch (ui_mode) {
      case UIMode::RX: {
        int sel = ui_handle_rx_key(c);
        if (sel >= 0 && sel < (int)g_rx_lines.size()) {
          std::istringstream iss(g_rx_lines[sel].text);
          std::vector<std::string> tok{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
          std::string call2;
          if (tok.size() >= 2) call2 = tok[1];
          else if (!tok.empty()) call2 = tok[0];
          if (!call2.empty()) {
            tx_enqueue(call2);
            tx_next.text = call2 + " AG6AQ CM97";
            tx_next.mark_delete = false;
          }
          rx_flash_idx = sel;
          rx_flash_deadline = rtc_now_ms() + 500;
          ui_draw_rx(rx_flash_idx);
        }
    if (c == 'x' || c == 'X') {
      if (!g_streaming) {
        g_streaming = true;
        xTaskCreatePinnedToCore(stream_wav_task, "stream_wav", 8192, (void*)"/spiffs/kfs40m.wav", 5, NULL, 1);
            }
          }
          break;
        }
        case UIMode::TX: {
          int start_idx = tx_page * 5;
          if (c == ';') {
            if (tx_page > 0) { tx_page--; redraw_tx_view(); }
          } else if (c == '.') {
            if (start_idx + 5 < (int)tx_queue.size()) { tx_page++; redraw_tx_view(); }
          } else if (c == '1') {
            if (start_idx < (int)tx_queue.size()) { tx_next = tx_queue[start_idx]; redraw_tx_view(); }
          } else if (c == 'e' || c == 'E') {
            encode_and_log_tx_next();
          } else if (c >= '2' && c <= '6') {
            int line = c - '2'; // 0..4
            int idx = start_idx + line;
            if (idx >= 0 && idx < (int)tx_queue.size()) {
              tx_queue[idx].mark_delete = !tx_queue[idx].mark_delete;
              redraw_tx_view();
            }
          }
          break;
        }
        case UIMode::BAND: {
          if (band_edit_idx >= 0) {
            if (c >= '0' && c <= '9') { band_edit_buffer.push_back(c); draw_band_view(); }
            else if (c == 0x08 || c == 0x7f) {
              if (!band_edit_buffer.empty()) { band_edit_buffer.pop_back(); draw_band_view(); }
            } else if (c == '\r' || c == '\n') {
              if (!band_edit_buffer.empty()) {
                int val = std::stoi(band_edit_buffer);
                g_bands[band_edit_idx].freq = val;
                save_station_data();
              }
              band_edit_idx = -1;
              band_edit_buffer.clear();
              draw_band_view();
            }
          } else {
            if (c == ';') {
              if (band_page > 0) { band_page--; draw_band_view(); }
            } else if (c == '.') {
              if ((band_page + 1) * 6 < (int)g_bands.size()) { band_page++; draw_band_view(); }
            } else if (c >= '1' && c <= '6') {
              int idx = band_page * 6 + (c - '1');
              if (idx >= 0 && idx < (int)g_bands.size()) {
                band_edit_idx = idx;
                band_edit_buffer = std::to_string(g_bands[idx].freq);
                draw_band_view();
              }
            }
          }
          break;
        }
        case UIMode::STATUS: {
            if (status_edit_idx == -1) {
              if (c == '1') { g_beacon = (BeaconMode)(((int)g_beacon + 1) % 5); save_station_data(); draw_status_view(); }
              else if (c == '2') { status_edit_idx = 1; status_edit_buffer.clear(); draw_status_view(); }
              else if (c == '3') { g_band_sel = (g_band_sel + 1) % g_bands.size(); save_station_data(); draw_status_view(); }
              else if (c == '4') { g_tune = !g_tune; draw_status_view(); }
              else if (c == '5') { status_edit_idx = 4; status_edit_buffer = g_date; status_cursor_pos = 0; while (status_cursor_pos < (int)status_edit_buffer.size() && (status_edit_buffer[status_cursor_pos] == '-')) status_cursor_pos++; draw_status_view(); }
              else if (c == '6') { status_edit_idx = 5; status_edit_buffer = g_time; status_cursor_pos = 0; while (status_cursor_pos < (int)status_edit_buffer.size() && (status_edit_buffer[status_cursor_pos] == ':')) status_cursor_pos++; draw_status_view(); }
            } else {
              if (status_edit_idx == 1) {
                if (c == '`') { status_edit_idx = -1; status_edit_buffer.clear(); draw_status_view(); }
                if (c == ';') { g_offset_hz += 100; draw_status_view(); }
                else if (c == '.') { g_offset_hz -= 100; draw_status_view(); }
                else if (c == ',') { g_offset_hz -= 10; draw_status_view(); }
                else if (c == '/') { g_offset_hz += 10; draw_status_view(); }
                else if (c == '\n') { save_station_data(); status_edit_idx = -1; draw_status_view(); }
              } else if (status_edit_idx == 4 || status_edit_idx == 5) {
                if (c == '`') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
                else if (c == ',') { // left
                  int pos = status_cursor_pos - 1;
                  while (pos >= 0 && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos--;
                  if (pos >= 0) status_cursor_pos = pos;
                  draw_status_view();
                } else if (c == '/') { // right
                  int pos = status_cursor_pos + 1;
                  while (pos < (int)status_edit_buffer.size() && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos++;
                  if (pos < (int)status_edit_buffer.size()) status_cursor_pos = pos;
                  draw_status_view();
                } else if (c >= '0' && c <= '9') {
                  if (status_cursor_pos >= 0 && status_cursor_pos < (int)status_edit_buffer.size()) {
                    status_edit_buffer[status_cursor_pos] = c;
                    int pos = status_cursor_pos + 1;
                    while (pos < (int)status_edit_buffer.size() && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos++;
                    if (pos < (int)status_edit_buffer.size()) status_cursor_pos = pos;
                  }
                  draw_status_view();
                } else if (c == '\n') {
                  if (status_edit_idx == 4) g_date = status_edit_buffer;
                  else g_time = status_edit_buffer;
                  save_station_data();
                  rtc_set_from_strings();
                  status_edit_idx = -1;
                  status_cursor_pos = -1;
                  status_edit_buffer.clear();
                  draw_status_view();
                }
              } else {
                if (c == '`') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
                else if (c == '\n') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
              }
            }
            break;
          }
        case UIMode::DEBUG: {
          if (c == ';') {
            if (debug_page > 0) { debug_page--; ui_draw_debug(g_debug_lines, debug_page); }
          } else if (c == '.') {
            if ((debug_page + 1) * 6 < (int)g_debug_lines.size()) { debug_page++; ui_draw_debug(g_debug_lines, debug_page); }
          }
          break;
        }
        case UIMode::LIST: {
          if (c == ';') {
            if (list_page > 0) { list_page--; ui_draw_list(g_list_lines, list_page, -1); }
          } else if (c == '.') {
            if ((list_page + 1) * 6 < (int)g_list_lines.size()) { list_page++; ui_draw_list(g_list_lines, list_page, -1); }
          }
          break;
        }
        case UIMode::HOST:
        case UIMode::MENU: {
          if (ui_mode == UIMode::MENU) {
            if (menu_long_edit) {
              if (c == '\n' || c == '\r') {
                if (menu_long_kind == LONG_FT) {
                  g_free_text = menu_long_buf;
                  if (g_cq_type == CqType::CQFREETEXT) g_cq_freetext = g_free_text;
                } else if (menu_long_kind == LONG_COMMENT) {
                  g_comment1 = menu_long_buf;
                }
                save_station_data();
                menu_long_edit = false;
                menu_long_kind = LONG_NONE;
                menu_long_buf.clear();
                menu_long_backup.clear();
                draw_menu_view();
              } else if (c == '`') {
                menu_long_edit = false;
                menu_long_kind = LONG_NONE;
                menu_long_buf.clear();
                menu_long_backup.clear();
                draw_menu_view();
              } else if (c == 0x08 || c == 0x7f) {
                if (!menu_long_buf.empty()) menu_long_buf.pop_back();
                draw_menu_view();
              } else if (c >= 32 && c < 127) {
                char ch = c;
                if (menu_long_kind == LONG_FT) ch = toupper((unsigned char)ch);
                menu_long_buf.push_back(ch);
                draw_menu_view();
              }
              break;
            } else if (menu_edit_idx >= 0) {
              if (c == '\n' || c == '\r') {
                int idx = menu_edit_idx % 6;
                int page = menu_edit_idx / 6;
                if (page == 0) {
                  if (idx == 3) { g_free_text = menu_edit_buf; if (g_cq_type == CqType::CQFREETEXT) g_cq_freetext = g_free_text; }
                  else if (idx == 4) g_call = menu_edit_buf;
                  else if (idx == 5) g_grid = menu_edit_buf;
                } else {
                  if (idx == 2) g_ant = menu_edit_buf;
                  else if (idx == 3) g_comment1 = menu_edit_buf;
                  else if (idx == 4) g_comment2 = menu_edit_buf;
                }
                save_station_data();
                menu_edit_idx = -1;
                menu_edit_buf.clear();
                draw_menu_view();
              } else if (c == 0x08 || c == 0x7f) {
                if (!menu_edit_buf.empty()) menu_edit_buf.pop_back();
                draw_menu_view();
              } else if (c == '`') {
                menu_edit_idx = -1;
                menu_edit_buf.clear();
                draw_menu_view();
              } else if (c >= 32 && c < 127) {
                char ch = c;
                if (menu_edit_idx % 6 == 3 || menu_edit_idx % 6 == 4 || menu_edit_idx % 6 == 5) {
                  ch = toupper((unsigned char)ch);
                }
                menu_edit_buf.push_back(ch);
                draw_menu_view();
              }
              break;
            }
              if (c == ';') {
                if (menu_page > 0) { menu_page--; draw_menu_view(); }
              } else if (c == '.') {
                if (menu_page < 1) { menu_page++; draw_menu_view(); }
            } else if (menu_page == 0) {
              if (c == '1') {
                g_cq_type = (CqType)(((int)g_cq_type + 1) % 6);
                save_station_data();
                draw_menu_view();
              } else if (c == '2') {
                g_skip_tx1 = !g_skip_tx1;
                save_station_data();
                draw_menu_view();
              } else if (c == '3') {
                tx_enqueue(g_free_text);
                tx_next.text = g_free_text;
                tx_next.mark_delete = false;
                menu_flash_idx = 2; // absolute index of "Send FreeText"
                menu_flash_deadline = rtc_now_ms() + 500;
                draw_menu_view();
                debug_log_line(std::string("Queued: ") + g_free_text);
              } else if (c == '4') {
                menu_long_edit = true;
                menu_long_kind = LONG_FT;
                menu_long_buf = g_free_text;
                menu_long_backup = g_free_text;
                draw_menu_view();
              } else if (c == '5') {
                menu_edit_idx = 4;
                menu_edit_buf = g_call;
                draw_menu_view();
              } else if (c == '6') {
                menu_edit_idx = 5;
                menu_edit_buf = g_grid;
                draw_menu_view();
              }
              } else if (menu_page == 1) {
                if (c == '1') {
                  g_offset_src = (OffsetSrc)(((int)g_offset_src + 1) % 3);
                  save_station_data();
                  draw_menu_view();
                } else if (c == '2') {
                  g_radio = (RadioType)(((int)g_radio + 1) % 4);
                  save_station_data();
                  draw_menu_view();
                } else if (c == '3') {
                  menu_edit_idx = 6 + 2;
                  menu_edit_buf = g_ant;
                  draw_menu_view();
                } else if (c == '4') {
                  menu_long_edit = true;
                  menu_long_kind = LONG_COMMENT;
                  menu_long_buf = g_comment1;
                  menu_long_backup = g_comment1;
                  draw_menu_view();
                }
              }
          }
          break;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

extern "C" void app_main(void) {
  // Run the full application loop on core1; decoder work stays on core0.
  xTaskCreatePinnedToCore(app_task_core1, "app_core1", 12288, nullptr, 5, nullptr, 1);
}
static void draw_status_line(int idx, const std::string& text, bool highlight) {
  const int line_h = 19;
  const int start_y = 18 + 3 + 3; // WATERFALL_H + COUNTDOWN_H + gap
  int y = start_y + idx * line_h;
  uint16_t bg = highlight ? M5.Display.color565(30, 30, 60) : TFT_BLACK;
  M5.Display.startWrite();
  M5.Display.setTextSize(2);
  M5.Display.fillRect(0, y, 240, line_h, bg);
  M5.Display.setTextColor(TFT_WHITE, bg);
  M5.Display.setCursor(0, y);
  M5.Display.printf("%d %s", idx + 1, text.c_str());
  M5.Display.endWrite();
}
static void draw_battery_icon(int x, int y, int w, int h, int level, bool charging) {
  if (level < 0) level = 0;
  if (level > 100) level = 100;
  // Outline
  M5.Display.startWrite();
  M5.Display.fillRect(x, y, w, h, TFT_BLACK);
  M5.Display.drawRect(x, y, w - 3, h, TFT_WHITE);
  M5.Display.fillRect(x + w - 3, y + h / 4, 3, h / 2, TFT_WHITE); // tab
  // Fill
  int inner_w = w - 5;
  int inner_h = h - 4;
  int fill_w = (inner_w * level) / 100;
  uint16_t fill_color = (level > 30) ? M5.Display.color565(0, 200, 0)
                        : (level > 15) ? M5.Display.color565(200, 180, 0)
                                        : M5.Display.color565(200, 0, 0);
  M5.Display.fillRect(x + 2, y + 2, fill_w, inner_h, fill_color);
  // Charging bolt
  if (charging) {
    int bx = x + w / 2 - 2;
    int by = y + 2;
    M5.Display.fillTriangle(bx, by, bx + 4, by + h / 2, bx + 2, by, M5.Display.color565(255, 255, 0));
    M5.Display.fillTriangle(bx + 2, by + h / 2, bx + 6, by + h - 2, bx + 4, by + h - 2, M5.Display.color565(255, 255, 0));
  }
  M5.Display.endWrite();
}
