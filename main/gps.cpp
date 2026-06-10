#include "gps.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr int kGpsBaudFast = 115200;
constexpr int kGpsBaudSlow = 9600;
constexpr size_t kGpsLineMax = 128;
constexpr uint32_t kProbeWindowMs = 2500;
// Self-heal threshold: if the UART has been silent (zero RX bytes) for this
// long since the most recent (re)start, reinstall the UART driver. Recovers
// from the ESP32-S3 issue where usb_host_install (KH1-USBC / QMX paths)
// clobbers the GPIO matrix routing for UART2 RX on the CAP-1262 (G15).
constexpr uint32_t kSelfHealSilentMs = 20000;
constexpr uint32_t kSelfHealCooldownMs = 20000;

const char* kTag = "GPS";

gps_state_t s_state = {};
gps_pins_t s_pins = {};
std::string s_line_buffer;
uint32_t s_probe_start_ms = 0;
uint32_t s_probe_rx_bytes = 0;
bool s_probe_decodable = false;
int s_reported_good_baud = kGpsBaudFast;
bool s_pending_baud_update = false;
int s_pending_baud_value = 0;
uint32_t s_last_self_heal_ms = 0;

// Per-talker GSV bookkeeping. Multi-constellation receivers (the ATGM336H
// emits GP/BD/GL) report sats-in-view separately per constellation, so we
// keep the most recent count for each and sum only entries seen within
// kGsvStaleMs to avoid double-counting after a talker goes silent.
constexpr size_t kMaxGsvTalkers = 4;
constexpr uint32_t kGsvStaleMs = 15000;
struct gsv_talker_t {
  char tag[3];        // "GP", "BD", "GL", "GA", "GN"; empty slot = tag[0]==0
  int in_view;
  uint32_t at_ms;
};
gsv_talker_t s_gsv[kMaxGsvTalkers] = {};

inline uint32_t now_ms() {
  return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

int normalize_baud(int baud) {
  return (baud == kGpsBaudSlow) ? kGpsBaudSlow : kGpsBaudFast;
}

int other_baud(int baud) {
  return (normalize_baud(baud) == kGpsBaudFast) ? kGpsBaudSlow : kGpsBaudFast;
}

std::vector<std::string> split_csv(const std::string& s) {
  std::vector<std::string> out;
  size_t start = 0;
  while (start <= s.size()) {
    size_t comma = s.find(',', start);
    if (comma == std::string::npos) {
      out.push_back(s.substr(start));
      break;
    }
    out.push_back(s.substr(start, comma - start));
    start = comma + 1;
  }
  return out;
}

int from_hex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool nmea_checksum_ok(const std::string& line, std::string* payload_out) {
  if (line.size() < 9 || line[0] != '$') return false;
  size_t star = line.find('*');
  if (star == std::string::npos || star + 2 >= line.size()) return false;
  const int h0 = from_hex(line[star + 1]);
  const int h1 = from_hex(line[star + 2]);
  if (h0 < 0 || h1 < 0) return false;
  uint8_t expected = (uint8_t)((h0 << 4) | h1);
  uint8_t parity = 0;
  for (size_t i = 1; i < star; ++i) parity ^= (uint8_t)line[i];
  if (parity != expected) return false;
  if (payload_out) *payload_out = line.substr(1, star - 1);
  return true;
}

double parse_nmea_coord(const std::string& val, const std::string& dir) {
  if (val.empty() || dir.empty()) return 0.0;
  double raw = std::strtod(val.c_str(), nullptr);
  int deg = (int)(raw / 100.0);
  double mins = raw - (double)(deg * 100);
  double out = (double)deg + mins / 60.0;
  if (dir[0] == 'S' || dir[0] == 'W') out = -out;
  return out;
}

std::string lat_lon_to_grid(double lat, double lon) {
  if (lon < -180.0 || lon > 180.0 || lat < -90.0 || lat > 90.0) return "";
  lon += 180.0;
  lat += 90.0;
  std::string grid = "    ";
  grid[0] = (char)('A' + (int)(lon / 20.0));
  grid[1] = (char)('A' + (int)(lat / 10.0));
  lon = std::fmod(lon, 20.0);
  lat = std::fmod(lat, 10.0);
  grid[2] = (char)('0' + (int)(lon / 2.0));
  grid[3] = (char)('0' + (int)(lat / 1.0));
  return grid;
}

void rearm_probe_window() {
  s_probe_start_ms = now_ms();
  s_probe_rx_bytes = 0;
  s_probe_decodable = false;
  s_line_buffer.clear();
}

void reset_gsv_tracking() {
  for (auto& t : s_gsv) { t.tag[0] = 0; t.in_view = 0; t.at_ms = 0; }
  s_state.satellites_in_view = 0;
}

void update_gsv_talker(const std::string& tag, int in_view) {
  if (tag.empty() || tag.size() > 2) return;
  const uint32_t now = now_ms();
  // Find existing slot for this tag, or the oldest slot to replace.
  gsv_talker_t* victim = nullptr;
  uint32_t oldest = UINT32_MAX;
  for (auto& t : s_gsv) {
    if (t.tag[0] != 0 && tag == t.tag) { victim = &t; break; }
    if (t.tag[0] == 0) { victim = &t; break; }
    if (t.at_ms < oldest) { oldest = t.at_ms; victim = &t; }
  }
  if (!victim) return;
  std::strncpy(victim->tag, tag.c_str(), sizeof(victim->tag) - 1);
  victim->tag[sizeof(victim->tag) - 1] = 0;
  victim->in_view = in_view;
  victim->at_ms = now;

  int total = 0;
  for (const auto& t : s_gsv) {
    if (t.tag[0] != 0 && (now - t.at_ms) <= kGsvStaleMs) total += t.in_view;
  }
  s_state.satellites_in_view = total;
}

bool configure_uart(int baud) {
  uart_config_t cfg = {};
  cfg.baud_rate = normalize_baud(baud);
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#ifdef UART_SCLK_REF_TICK
  cfg.source_clk = UART_SCLK_REF_TICK;
#else
  cfg.source_clk = UART_SCLK_DEFAULT;
#endif
  esp_err_t err = uart_param_config(s_pins.uart, &cfg);
  if (err != ESP_OK) return false;
  err = uart_set_pin(s_pins.uart, s_pins.tx, s_pins.rx,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (err != ESP_OK) return false;
  err = uart_set_line_inverse(s_pins.uart, (uart_signal_inv_t)0);
  if (err != ESP_OK) return false;
  err = uart_set_baudrate(s_pins.uart, normalize_baud(baud));
  if (err != ESP_OK) return false;
  uart_flush_input(s_pins.uart);
  return true;
}

bool ensure_uart_driver() {
  esp_err_t err = uart_driver_install(s_pins.uart, 2048, 0, 0, nullptr, 0);
  if (err == ESP_ERR_INVALID_STATE) {
    uart_driver_delete(s_pins.uart);
    err = uart_driver_install(s_pins.uart, 2048, 0, 0, nullptr, 0);
  }
  return err == ESP_OK;
}

void try_self_heal_uart() {
  if (!s_state.running) return;
  if (s_state.total_rx_bytes > 0) return;  // bytes flowing — nothing to heal
  const uint32_t now = now_ms();
  const uint32_t uptime = now - s_state.started_at_ms;
  if (uptime < kSelfHealSilentMs) return;
  if (s_last_self_heal_ms != 0 && (now - s_last_self_heal_ms) < kSelfHealCooldownMs) return;
  s_last_self_heal_ms = now;

  ESP_LOGW(kTag, "GPS self-heal: %ums silent on UART%d → reinstall driver",
           (unsigned)uptime, (int)s_pins.uart);

  // Reinstall the UART driver and re-mux the pins. configure_uart() calls
  // uart_set_pin(), which rewrites the GPIO matrix — that's what we need
  // to undo whatever clobbered the routing.
  const int baud = s_state.active_baud;
  uart_driver_delete(s_pins.uart);
  if (!ensure_uart_driver()) {
    ESP_LOGW(kTag, "GPS self-heal: driver install failed");
    return;
  }
  if (!configure_uart(baud)) {
    ESP_LOGW(kTag, "GPS self-heal: UART config failed");
    uart_driver_delete(s_pins.uart);
    s_state.running = false;
    return;
  }
  s_state.started_at_ms = now;
}

void switch_baud_internal(int baud) {
  baud = normalize_baud(baud);
  if (uart_set_baudrate(s_pins.uart, baud) != ESP_OK) return;
  uart_flush_input(s_pins.uart);
  s_state.active_baud = baud;
  s_state.baud_locked = false;
  rearm_probe_window();
  ESP_LOGI(kTag, "GPS probe switch baud=%d", baud);
}

void lock_current_baud_if_needed() {
  if (s_state.baud_locked) return;
  s_state.baud_locked = true;
  if (s_state.active_baud != s_reported_good_baud) {
    s_reported_good_baud = s_state.active_baud;
    s_pending_baud_value = s_state.active_baud;
    s_pending_baud_update = true;
  }
  ESP_LOGI(kTag, "GPS baud lock=%d", s_state.active_baud);
}

bool parse_sentence(const std::string& raw_line) {
  std::string payload;
  if (!nmea_checksum_ok(raw_line, &payload)) return false;

  std::vector<std::string> parts = split_csv(payload);
  if (parts.empty()) return false;

  bool recognized = false;
  const std::string& type = parts[0];
  if (type.size() >= 3 && type.compare(type.size() - 3, 3, "RMC") == 0 && parts.size() >= 10) {
    recognized = true;
    if (parts[2] == "A") {
      s_state.valid_fix = true;
      s_state.latitude = parse_nmea_coord(parts[3], parts[4]);
      s_state.longitude = parse_nmea_coord(parts[5], parts[6]);
      s_state.grid_square = lat_lon_to_grid(s_state.latitude, s_state.longitude);
      if (parts[1].size() >= 6) {
        s_state.time_utc = parts[1].substr(0, 2) + ":" + parts[1].substr(2, 2) + ":" + parts[1].substr(4, 2);
      }
      if (parts[9].size() >= 6) {
        s_state.date_utc = "20" + parts[9].substr(4, 2) + "-" + parts[9].substr(2, 2) + "-" + parts[9].substr(0, 2);
      }
    } else {
      s_state.valid_fix = false;
    }
  } else if (type.size() >= 3 && type.compare(type.size() - 3, 3, "GGA") == 0 && parts.size() >= 8) {
    recognized = true;
    if (!parts[6].empty()) s_state.gga_fix_quality = std::atoi(parts[6].c_str());
    if (!parts[7].empty()) s_state.satellites = std::atoi(parts[7].c_str());
  } else if (type.size() >= 3 && type.compare(type.size() - 3, 3, "GSV") == 0 && parts.size() >= 4) {
    recognized = true;
    const int in_view = std::atoi(parts[3].c_str());
    update_gsv_talker(type.substr(0, type.size() - 3), in_view);
  }

  s_state.last_rx_ms = now_ms();
  s_probe_decodable = true;
  (void)recognized;
  lock_current_baud_if_needed();
  return true;
}

void ingest_uart_bytes(const uint8_t* data, int len) {
  if (len <= 0) return;
  s_probe_rx_bytes += (uint32_t)len;
  s_state.total_rx_bytes += (uint32_t)len;
  for (int i = 0; i < len; ++i) {
    char c = (char)data[i];
    if (c == '\r' || c == '\n') {
      if (!s_line_buffer.empty()) {
        parse_sentence(s_line_buffer);
        s_line_buffer.clear();
      }
      continue;
    }
    if ((unsigned char)c < 32 || (unsigned char)c > 126) continue;
    s_line_buffer.push_back(c);
    if (s_line_buffer.size() > kGpsLineMax) s_line_buffer.clear();
  }
}

}  // namespace

void gps_start(const gps_pins_t& pins) {
  if (s_state.running) return;

  s_pins = pins;
  const int preload_baud = normalize_baud(s_pins.default_baud);

  if (!ensure_uart_driver()) {
    ESP_LOGW(kTag, "GPS UART driver install failed");
    return;
  }
  if (!configure_uart(preload_baud)) {
    ESP_LOGW(kTag, "GPS UART config failed");
    uart_driver_delete(s_pins.uart);
    return;
  }

  s_state = {};
  s_state.running = true;
  s_state.active_baud = preload_baud;
  s_state.baud_locked = false;
  s_state.started_at_ms = now_ms();
  s_reported_good_baud = preload_baud;
  s_pending_baud_update = false;
  s_pending_baud_value = 0;
  s_last_self_heal_ms = 0;
  reset_gsv_tracking();
  rearm_probe_window();
  ESP_LOGI(kTag, "GPS started on UART%d TX=G%d RX=G%d preload=%d auto=%d",
           (int)s_pins.uart, (int)s_pins.tx, (int)s_pins.rx, preload_baud,
           s_pins.auto_baud ? 1 : 0);
}

void gps_stop() {
  if (!s_state.running) return;
  uart_flush_input(s_pins.uart);
  uart_driver_delete(s_pins.uart);
  s_state = {};
  s_pending_baud_update = false;
  s_pending_baud_value = 0;
  s_line_buffer.clear();
  s_probe_start_ms = 0;
  s_probe_rx_bytes = 0;
  s_probe_decodable = false;
  reset_gsv_tracking();
  ESP_LOGI(kTag, "GPS stopped");
}

void gps_tick() {
  if (!s_state.running) return;

  uint8_t buf[256];
  int len = uart_read_bytes(s_pins.uart, buf, sizeof(buf), 0);
  if (len > 0) ingest_uart_bytes(buf, len);

  if (s_pins.auto_baud && !s_state.baud_locked) {
    uint32_t now = now_ms();
    if (s_probe_rx_bytes > 0 &&
        (now - s_probe_start_ms) >= kProbeWindowMs &&
        !s_probe_decodable) {
      switch_baud_internal(other_baud(s_state.active_baud));
    }
  }

  try_self_heal_uart();
}

gps_state_t gps_get_state() {
  return s_state;
}

bool gps_take_baud_update(int* out_baud) {
  if (!s_pending_baud_update) return false;
  if (out_baud) *out_baud = s_pending_baud_value;
  s_pending_baud_update = false;
  s_pending_baud_value = 0;
  return true;
}
