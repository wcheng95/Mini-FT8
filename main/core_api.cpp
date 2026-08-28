// ============================================================================
// core_api.cpp — functional-core implementation
//
// Thin adapter over the existing main.cpp / autoseq.cpp / ui.cpp internals.
// No behavioral change; just exposes a UI-agnostic surface to consumers
// (Cardputer local UI + future BLE server).
//
// See core_api.h for the public contract and docs/NATIVE_CLIENT_ARCHITECTURE.md
// for the high-level design.
// ============================================================================

#include "core_api.h"
#include "core_api_internal.h"
#include "station_types.h"
#include "autoseq.h"

#include <atomic>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <strings.h>   // strncasecmp, for case-insensitive ADIF field names
#include <sys/stat.h>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_system.h"

#include "radio_control.h"   // for radio_control_{ready,end_tx}()

// ---------------------------------------------------------------------------
// Access to main.cpp's globals (un-staticized for this purpose).
// ---------------------------------------------------------------------------
extern std::string       g_call;
extern std::string       g_grid;
extern std::string       g_comment1;
extern RadioType         g_radio;
extern std::vector<BandItem> g_bands;
extern int               g_band_sel;
extern CqType            g_cq_type;
extern std::string       g_cq_freetext;
extern BeaconMode        g_beacon;
extern OffsetSrc         g_offset_src;
extern int               g_offset_hz;
extern bool              g_skip_tx1;
extern int               g_autoseq_max_retry;
extern int               g_rtc_comp;
extern std::string       g_date;
extern std::string       g_time;
extern std::vector<std::string> g_ignore_prefixes;
extern volatile bool     g_tx_cancel_requested;

// Functions from main.cpp that core_api delegates to.
void save_station_data();
void apply_radio_profile_binding();
void update_autoseq_cq_type();
void rebuild_active_bands();
void rebuild_ignore_prefixes();
bool rtc_set_from_strings();
void rtc_sync_to_hw();

// Access to ui.cpp (RX list live in ui.cpp's static array).
int  ui_get_rx_count();
bool ui_get_rx_entry(int idx, RxDecodeEntry* out);

// ---------------------------------------------------------------------------
// Callback registry
// ---------------------------------------------------------------------------
// Single-slot per event. Multiple simultaneous consumers is a step 3 concern.
// ---------------------------------------------------------------------------

namespace {
// Multi-slot registry so Cardputer and BLE server can coexist.
// No remove API — consumers live for the app lifetime.
constexpr int kMaxConsumers = 4;
CoreChangeCb    g_cb_rx_changed     [kMaxConsumers] = {};
CoreChangeCb    g_cb_qso_changed    [kMaxConsumers] = {};
CoreChangeCb    g_cb_config_changed [kMaxConsumers] = {};
CoreWaterfallCb g_cb_waterfall_row  [kMaxConsumers] = {};
int             g_cb_rx_n = 0, g_cb_qso_n = 0, g_cb_config_n = 0, g_cb_wf_n = 0;

// Fake static values for SWR/PWR/PTT until real polling is wired up.
std::atomic<bool>  g_ptt_state{false};
constexpr float    kStubSwr = 1.5f;
constexpr float    kStubPwr = 2.0f;

SemaphoreHandle_t g_config_mutex = nullptr;

void config_lock()   { if (g_config_mutex) xSemaphoreTake(g_config_mutex, portMAX_DELAY); }
void config_unlock() { if (g_config_mutex) xSemaphoreGive(g_config_mutex); }

struct ConfigGuard {
  ConfigGuard()  { config_lock(); }
  ~ConfigGuard() { config_unlock(); }
};

// ---------------------------------------------------------------------------
// Enum mapping helpers (core_api.h <-> station_types.h)
// ---------------------------------------------------------------------------

CoreBeaconMode map_out(BeaconMode m) {
  switch (m) {
    case BeaconMode::OFF:  return CoreBeaconMode::OFF;
    case BeaconMode::EVEN: return CoreBeaconMode::EVEN;
    case BeaconMode::ODD:  return CoreBeaconMode::ODD;
  }
  return CoreBeaconMode::OFF;
}
BeaconMode map_in(CoreBeaconMode m) {
  switch (m) {
    case CoreBeaconMode::OFF:  return BeaconMode::OFF;
    case CoreBeaconMode::EVEN: return BeaconMode::EVEN;
    case CoreBeaconMode::ODD:  return BeaconMode::ODD;
  }
  return BeaconMode::OFF;
}

CoreCqType map_out(CqType t) {
  switch (t) {
    case CqType::CQ:         return CoreCqType::CQ;
    case CqType::CQSOTA:     return CoreCqType::SOTA;
    case CqType::CQPOTA:     return CoreCqType::POTA;
    case CqType::CQQRP:      return CoreCqType::QRP;
    case CqType::CQFD:       return CoreCqType::FD;
    case CqType::CQFREETEXT: return CoreCqType::FREETEXT;
  }
  return CoreCqType::CQ;
}
CqType map_in(CoreCqType t) {
  switch (t) {
    case CoreCqType::CQ:       return CqType::CQ;
    case CoreCqType::SOTA:     return CqType::CQSOTA;
    case CoreCqType::POTA:     return CqType::CQPOTA;
    case CoreCqType::QRP:      return CqType::CQQRP;
    case CoreCqType::FD:       return CqType::CQFD;
    case CoreCqType::FREETEXT: return CqType::CQFREETEXT;
  }
  return CqType::CQ;
}

CoreOffsetSrc map_out(OffsetSrc s) {
  switch (s) {
    case OffsetSrc::RX:     return CoreOffsetSrc::RX;
    case OffsetSrc::CURSOR: return CoreOffsetSrc::CURSOR;
    case OffsetSrc::RANDOM: return CoreOffsetSrc::RANDOM;
  }
  return CoreOffsetSrc::RANDOM;
}
OffsetSrc map_in(CoreOffsetSrc s) {
  switch (s) {
    case CoreOffsetSrc::RX:     return OffsetSrc::RX;
    case CoreOffsetSrc::CURSOR: return OffsetSrc::CURSOR;
    case CoreOffsetSrc::RANDOM: return OffsetSrc::RANDOM;
  }
  return OffsetSrc::RANDOM;
}

CoreRadioType map_out(RadioType r) {
  return (r == RadioType::KH1) ? CoreRadioType::KH1 : CoreRadioType::QMX;
}
RadioType map_in(CoreRadioType r) {
  return (r == CoreRadioType::KH1) ? RadioType::KH1 : RadioType::QMX;
}

CoreQsoState map_out(AutoseqState s) {
  switch (s) {
    case AutoseqState::CALLING:      return CoreQsoState::CALLING;
    case AutoseqState::REPLYING:     return CoreQsoState::REPLYING;
    case AutoseqState::REPORT:       return CoreQsoState::REPORT;
    case AutoseqState::ROGER_REPORT: return CoreQsoState::ROGER_REPORT;
    case AutoseqState::ROGERS:       return CoreQsoState::ROGERS;
    case AutoseqState::SIGNOFF:      return CoreQsoState::SIGNOFF;
    case AutoseqState::IDLE:         return CoreQsoState::IDLE;
  }
  return CoreQsoState::IDLE;
}

CoreTxMsg map_out(TxMsgType t) {
  switch (t) {
    case TxMsgType::TX_NONE: return CoreTxMsg::NONE;
    case TxMsgType::TX1:     return CoreTxMsg::TX1;
    case TxMsgType::TX2:     return CoreTxMsg::TX2;
    case TxMsgType::TX3:     return CoreTxMsg::TX3;
    case TxMsgType::TX4:     return CoreTxMsg::TX4;
    case TxMsgType::TX5:     return CoreTxMsg::TX5;
    case TxMsgType::TX6:     return CoreTxMsg::TX6;
  }
  return CoreTxMsg::NONE;
}

}  // namespace

// ---------------------------------------------------------------------------
// Version + lifecycle
// ---------------------------------------------------------------------------

const char* core_api_version() { return "1.0.0"; }

void core_init() {
  if (!g_config_mutex) {
    g_config_mutex = xSemaphoreCreateMutex();
  }
}

// ---------------------------------------------------------------------------
// Snapshot accessors
// ---------------------------------------------------------------------------

void core_get_rx_list(std::vector<RxDecodeEntry>& out) {
  const int n = ui_get_rx_count();
  out.clear();
  out.reserve(n);
  for (int i = 0; i < n; ++i) {
    RxDecodeEntry e{};
    if (ui_get_rx_entry(i, &e)) out.push_back(e);
  }
}

// Defined in main.cpp. Read by core_get_qso so the BLE snapshot reflects
// the firmware's resolved offset, not autoseq's pre-resolution placeholder.
extern AutoseqTxEntry g_pending_tx;
extern bool           g_pending_tx_valid;

int core_qso_active_count() {
  return autoseq_active_count();
}

bool core_qso_get_active(int idx, QsoEntry& out) {
  QsoContext c;
  if (!autoseq_get_active_context(idx, &c)) return false;
  out.dxcall        = c.dxcall;
  out.dxgrid        = c.dxgrid;
  out.state         = map_out(c.state);
  out.next_tx       = map_out(c.next_tx);
  out.retry_counter = c.retry_counter;
  out.retry_limit   = c.retry_limit;
  out.slot_parity   = c.slot_id & 1;
  out.snr_tx        = c.snr_tx;
  out.snr_rx        = c.snr_rx;
  out.is_fd         = c.is_fd;
  out.logged        = c.logged;
  return true;
}

bool core_qso_get_next_tx(NextTxEntry& out) {
  // Prefer g_pending_tx when arm_pending_tx has fired — it carries the
  // resolved offset (matching the actual TX, including the random roll
  // for RANDOM mode and beacon CQ). autoseq's own pending entry only
  // holds the *unresolved* offset (often 0 for fresh CQs), which is
  // what was reaching BLE before and pinning the marker at the config
  // default.
  if (g_pending_tx_valid && !g_pending_tx.text.empty()) {
    out.valid             = true;
    out.text              = g_pending_tx.text;
    out.dxcall            = g_pending_tx.dxcall;
    out.slot_parity       = g_pending_tx.slot_id & 1;
    out.offset_hz         = g_pending_tx.offset_hz;
    out.retries_remaining = g_pending_tx.repeat_counter;
    return true;
  }
  // Not yet armed — surface autoseq's intent so the client at least
  // knows a TX is queued, even if the offset is still placeholder.
  AutoseqTxEntry pending{};
  if (autoseq_fetch_pending_tx(pending)) {
    out.valid             = true;
    out.text              = pending.text;
    out.dxcall            = pending.dxcall;
    out.slot_parity       = pending.slot_id & 1;
    out.offset_hz         = pending.offset_hz;
    out.retries_remaining = pending.repeat_counter;
    return true;
  }
  out.valid = false;
  return false;
}

void core_get_qso(QsoSnapshot& out) {
  out.active.clear();
  out.next_tx = NextTxEntry{};

  const int n = core_qso_active_count();
  out.active.reserve(n);
  for (int i = 0; i < n; ++i) {
    QsoEntry e;
    if (core_qso_get_active(i, e)) {
      out.active.push_back(std::move(e));
    }
  }
  core_qso_get_next_tx(out.next_tx);
}

void core_get_config(StationConfig& out) {
  ConfigGuard g;
  out.call        = g_call;
  out.grid        = g_grid;
  out.comment     = g_comment1;

  out.radio       = map_out(g_radio);
  out.bands_hz.clear();
  out.bands_hz.reserve(g_bands.size());
  for (const auto& b : g_bands) {
    out.bands_hz.push_back((uint32_t)b.freq * 1000u);  // BandItem.freq is kHz
  }
  out.band_idx    = g_band_sel;

  out.cq_type     = map_out(g_cq_type);
  out.cq_freetext = g_cq_freetext;
  out.beacon      = map_out(g_beacon);

  out.offset_src  = map_out(g_offset_src);
  out.offset_hz   = g_offset_hz;

  out.skip_tx1    = g_skip_tx1;
  out.max_retry   = g_autoseq_max_retry;

  out.rtc_comp    = g_rtc_comp;
  out.date        = g_date;
  out.time        = g_time;

  out.ignore_prefixes = g_ignore_prefixes;
}

// ---------------------------------------------------------------------------
// Callback registration
// ---------------------------------------------------------------------------

void core_on_rx_changed(CoreChangeCb cb) {
  if (g_cb_rx_n < kMaxConsumers) g_cb_rx_changed[g_cb_rx_n++] = cb;
}
void core_on_qso_changed(CoreChangeCb cb) {
  if (g_cb_qso_n < kMaxConsumers) g_cb_qso_changed[g_cb_qso_n++] = cb;
}
void core_on_config_changed(CoreChangeCb cb) {
  if (g_cb_config_n < kMaxConsumers) g_cb_config_changed[g_cb_config_n++] = cb;
}
void core_on_waterfall_row(CoreWaterfallCb cb) {
  if (g_cb_wf_n < kMaxConsumers) g_cb_waterfall_row[g_cb_wf_n++] = cb;
}

// ---------------------------------------------------------------------------
// Internal fire helpers (called by main.cpp / stream_uac.cpp on mutations)
// ---------------------------------------------------------------------------

void core_fire_rx_changed() {
  for (int i = 0; i < g_cb_rx_n; ++i) if (g_cb_rx_changed[i]) g_cb_rx_changed[i]();
}
void core_fire_qso_changed() {
  for (int i = 0; i < g_cb_qso_n; ++i) if (g_cb_qso_changed[i]) g_cb_qso_changed[i]();
}
void core_fire_config_changed() {
  for (int i = 0; i < g_cb_config_n; ++i) if (g_cb_config_changed[i]) g_cb_config_changed[i]();
}

void core_fire_waterfall_row(int sym,
                             const uint8_t* mag, int num_bins,
                             float swr, float pwr, bool ptt) {
  if (g_cb_wf_n == 0) return;
  WaterfallRow row;
  row.sym      = sym;
  row.mag      = mag;
  row.num_bins = num_bins;
  row.swr      = swr;
  row.pwr      = pwr;
  row.ptt      = ptt;
  for (int i = 0; i < g_cb_wf_n; ++i) if (g_cb_waterfall_row[i]) g_cb_waterfall_row[i](row);
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

// Save deferral: every save_station_data call below would otherwise run
// on the ble_native task, whose 4 KB stack can't accommodate the 22-
// fprintf chain in save_station_data plus SPIFFS internals. Set the
// flag instead — the main UI loop on the deeper app_task_core0 stack
// drains it within ~10 ms.
extern volatile bool g_config_save_pending;

// Helper: apply a string setter + request save + fire config event.
namespace {
template <typename T>
bool apply_config_write(T&& mutator) {
  {
    ConfigGuard g;
    mutator();
  }
  g_config_save_pending = true;
  core_fire_config_changed();
  return true;
}
}  // namespace

// Defined in main.cpp; resolves the offset_src semantics and writes
// g_qso_xmit / g_target_slot_parity / g_pending_tx[.offset_hz] /
// g_pending_tx_valid in one shot.
extern void arm_pending_tx(const AutoseqTxEntry& pending);

bool core_cmd_tap_rx(int rx_list_idx) {
  RxDecodeEntry entry{};
  if (!ui_get_rx_entry(rx_list_idx, &entry)) return false;

  // Build a UiRxLine from the static entry (autoseq API still takes it).
  UiRxLine msg;
  msg.text      = entry.text;
  msg.field1    = entry.field1;
  msg.field2    = entry.field2;
  msg.field3    = entry.field3;
  msg.snr       = entry.snr;
  msg.offset_hz = entry.offset_hz;
  msg.slot_id   = entry.slot_id;
  msg.is_cq     = entry.is_cq;
  msg.is_to_me  = entry.is_to_me;
  autoseq_on_touch(msg);

  // Arm the TX state machine for the next matching slot boundary so the
  // user's pick is honoured immediately instead of waiting for the next
  // autoseq tick to pull in the pending TX (which would delay by 1-2
  // slots). Mirrors the Cardputer RX-key handler.
  AutoseqTxEntry pending{};
  if (autoseq_fetch_pending_tx(pending)) {
    arm_pending_tx(pending);
  }
  core_fire_qso_changed();
  return true;
}

bool core_cmd_cancel_tx() {
  // Mirror of the Cardputer's backtick cancel key path. The tx_tick()
  // state machine reads g_tx_cancel_requested on its next iteration and
  // aborts the in-flight TX; radio_control_end_tx() PTTs down immediately.
  g_tx_cancel_requested = true;
  if (radio_control_ready()) radio_control_end_tx();
  core_fire_qso_changed();
  return true;
}

bool core_cmd_clear_qso_queue() {
  autoseq_clear();
  core_fire_qso_changed();
  return true;
}

bool core_cmd_drop_qso(int idx) {
  const bool ok = autoseq_drop_index(idx);
  if (ok) core_fire_qso_changed();
  return ok;
}

bool core_cmd_queue_freetext(const std::string& text) {
  // fallback parity: opposite of whatever slot_id we last saw makes a safe
  // "next slot" default. For step 2, use 0; main.cpp's actual path applies
  // its own fallback when the queue is empty.
  const bool ok = autoseq_schedule_freetext(text, 0);
  if (ok) core_fire_qso_changed();
  return ok;
}

extern bool sync_radio_to_current_band(const char* reason);

bool core_cmd_set_band(int band_idx) {
  if (band_idx < 0 || band_idx >= (int)g_bands.size()) return false;
  if (!apply_config_write([&]{ g_band_sel = band_idx; })) return false;
  // The Cardputer defers the CAT push to STATUS exit because S->3 is a
  // tap-cycle through bands (each press would otherwise click the KH1
  // antenna relay). The BLE client picks a band from a dropdown — one
  // intentional change — so commit to the radio immediately.
  sync_radio_to_current_band("BLE set_band");
  return true;
}

bool core_cmd_set_radio(CoreRadioType r) {
  const RadioType nr = map_in(r);
  if (nr == g_radio) return true;
  {
    ConfigGuard g;
    g_radio = nr;
  }
  apply_radio_profile_binding();
  g_config_save_pending = true;
  core_fire_config_changed();
  return true;
}

// Defined in main.cpp; trims/uppercases the grid to its 4-char FT8 form.
extern std::string grid_ft8_4(const std::string& grid);

// autoseq holds its own snapshot of (my_call, my_grid4) used to generate
// CQ text and TX1/TX2/etc. Without re-pushing after a config edit, the
// next CQ would still go out with the previous callsign — the on-device
// MENU/STATUS edit handlers in main.cpp already do this; mirror it here.
bool core_cmd_set_call(const std::string& call) {
  if (!apply_config_write([&]{ g_call = call; })) return false;
  autoseq_set_station(g_call, grid_ft8_4(g_grid));
  return true;
}
bool core_cmd_set_grid(const std::string& grid) {
  if (!apply_config_write([&]{ g_grid = grid; })) return false;
  autoseq_set_station(g_call, grid_ft8_4(g_grid));
  return true;
}
bool core_cmd_set_comment(const std::string& comment) {
  return apply_config_write([&]{ g_comment1 = comment; });
}

bool core_cmd_set_cq_type(CoreCqType t) {
  {
    ConfigGuard g;
    g_cq_type = map_in(t);
  }
  update_autoseq_cq_type();
  g_config_save_pending = true;
  core_fire_config_changed();
  return true;
}
bool core_cmd_set_cq_freetext(const std::string& text) {
  {
    ConfigGuard g;
    g_cq_freetext = text;
  }
  update_autoseq_cq_type();
  g_config_save_pending = true;
  core_fire_config_changed();
  return true;
}
bool core_cmd_set_beacon(CoreBeaconMode m) {
  return apply_config_write([&]{ g_beacon = map_in(m); });
}

bool core_cmd_set_offset_src(CoreOffsetSrc s) {
  return apply_config_write([&]{ g_offset_src = map_in(s); });
}
bool core_cmd_set_offset_hz(int hz) {
  if (hz < 0 || hz > 4000) return false;
  return apply_config_write([&]{ g_offset_hz = hz; });
}

bool core_cmd_set_skip_tx1(bool skip) {
  {
    ConfigGuard g;
    g_skip_tx1 = skip;
  }
  autoseq_set_skip_tx1(skip);
  g_config_save_pending = true;
  core_fire_config_changed();
  return true;
}
bool core_cmd_set_max_retry(int n) {
  if (n < 0 || n > 99) return false;
  {
    ConfigGuard g;
    g_autoseq_max_retry = n;
  }
  autoseq_set_max_retry(n);
  g_config_save_pending = true;
  core_fire_config_changed();
  return true;
}

bool core_cmd_set_rtc(int64_t epoch_ms) {
  // Convert epoch_ms to date/time strings.
  time_t t = (time_t)(epoch_ms / 1000);
  struct tm tm_utc;
  gmtime_r(&t, &tm_utc);
  char dbuf[16], tbuf[16];
  snprintf(dbuf, sizeof(dbuf), "%04d-%02d-%02d",
           (tm_utc.tm_year + 1900) % 10000,
           (tm_utc.tm_mon + 1) % 100,
           tm_utc.tm_mday % 100);
  snprintf(tbuf, sizeof(tbuf), "%02d:%02d:%02d",
           tm_utc.tm_hour % 100,
           tm_utc.tm_min % 100,
           tm_utc.tm_sec % 100);
  {
    ConfigGuard g;
    g_date = dbuf;
    g_time = tbuf;
  }
  if (!rtc_set_from_strings()) return false;
  rtc_sync_to_hw();
  g_config_save_pending = true;
  core_fire_config_changed();
  return true;
}
bool core_cmd_set_rtc_comp(int32_t ppm_like) {
  return apply_config_write([&]{ g_rtc_comp = ppm_like; });
}

bool core_cmd_ignore_add(const std::string& prefix) {
  if (prefix.empty()) return false;
  {
    ConfigGuard g;
    for (const auto& p : g_ignore_prefixes) {
      if (p == prefix) return true;  // already present
    }
    g_ignore_prefixes.push_back(prefix);
  }
  rebuild_ignore_prefixes();
  g_config_save_pending = true;
  core_fire_config_changed();
  return true;
}
bool core_cmd_ignore_remove(const std::string& prefix) {
  bool removed = false;
  {
    ConfigGuard g;
    for (auto it = g_ignore_prefixes.begin(); it != g_ignore_prefixes.end(); ++it) {
      if (*it == prefix) { g_ignore_prefixes.erase(it); removed = true; break; }
    }
  }
  if (!removed) return false;
  rebuild_ignore_prefixes();
  g_config_save_pending = true;
  core_fire_config_changed();
  return true;
}
bool core_cmd_ignore_clear() {
  {
    ConfigGuard g;
    g_ignore_prefixes.clear();
  }
  rebuild_ignore_prefixes();
  g_config_save_pending = true;
  core_fire_config_changed();
  return true;
}

bool core_cmd_save_config() {
  g_config_save_pending = true;
  return true;
}

// ---------------------------------------------------------------------------
// Worked-QSO log — the one ADIF reader (see core_api.h)
// ---------------------------------------------------------------------------

namespace {

// Day-log filenames are exactly "YYYYMMDD.txt", matching what
// log_adif_entry() writes. Everything else in /storage (Station.txt,
// Cabrillo files) is not a day log.
bool log_is_day_file(const char* name) {
  if (!name) return false;
  if (strlen(name) != 12) return false;
  for (int i = 0; i < 8; ++i) {
    if (!std::isdigit(static_cast<unsigned char>(name[i]))) return false;
  }
  // Case-insensitive on the extension: with LFN disabled
  // (CONFIG_FATFS_LFN_NONE, this project's config) FatFs stores the 8.3
  // name created as "20260828.txt" — and hands it back from readdir — as
  // "20260828.TXT". fopen matches either case, which is why the writer and
  // the direct readers worked while the listing came back empty. That was
  // the first on-hardware symptom of the BLE log viewer: a freshly logged
  // QSO, and log_days returning no days. SPIFFS preserved case, so the old
  // viewer's lowercase strcmp broke silently in the SPIFFS -> FATFS
  // migration too.
  return strcasecmp(name + 8, ".txt") == 0;
}

// Builds "/storage/YYYYMMDD.txt". A null or empty date means today, taken
// from g_date ("YYYY-MM-DD"). Returns false on a malformed date — this can
// arrive off the wire, so it is validated rather than trusted.
bool log_day_path(const char* yyyymmdd, char* out, size_t out_sz) {
  char date[9];
  if (yyyymmdd && yyyymmdd[0]) {
    if (strnlen(yyyymmdd, 9) != 8) return false;
    for (int i = 0; i < 8; ++i) {
      if (!std::isdigit(static_cast<unsigned char>(yyyymmdd[i]))) return false;
    }
    memcpy(date, yyyymmdd, 8);
  } else {
    const char* d = g_date.c_str();
    int o = 0;
    for (int i = 0; d[i] && o < 8; ++i) {
      if (d[i] >= '0' && d[i] <= '9') date[o++] = d[i];
    }
    if (o != 8) return false;
  }
  date[8] = '\0';
  snprintf(out, out_sz, "/storage/%s.txt", date);
  return true;
}

constexpr size_t kLogRecMax = 512;

// Reads bytes up to and including the next "<eor>" (case-insensitive).
// Returns false at EOF, which also discards any trailing partial record —
// a half-written entry is not a record.
//
// Scanning for the terminator rather than reading lines means a record may
// span newlines and an over-long one is truncated at kLogRecMax instead of
// corrupting the records after it.
bool log_next_record(FILE* f, char* rec, size_t* out_len) {
  static const char kEor[] = "<eor>";
  size_t n = 0;
  int    m = 0;                       // chars of "<eor>" matched so far
  int    c;
  while ((c = fgetc(f)) != EOF) {
    if (n + 1 < kLogRecMax) rec[n++] = (char)c;
    const char lc = (char)std::tolower(static_cast<unsigned char>(c));
    if (lc == kEor[m]) {
      if (++m == 5) { rec[n] = '\0'; *out_len = n; return true; }
    } else {
      m = (lc == '<') ? 1 : 0;        // a fresh '<' restarts the match
    }
  }
  rec[n] = '\0';
  *out_len = n;
  return false;
}

// Extracts one field by name. ADIF declares a byte count in the tag
// ("<call:5>W1ABC"), so the length is authoritative and the value may
// legally contain spaces, '<' or '>'. Scanning for a delimiter instead —
// which the old Cardputer viewer did — truncates any value with a space in
// it, and the default comment "MiniFT8 /Radio" has one.
bool log_field(const char* rec, size_t len, const char* name,
               char* out, size_t out_sz) {
  out[0] = '\0';
  const size_t nlen = strlen(name);
  size_t i = 0;
  while (i < len) {
    const char* lt = (const char*)memchr(rec + i, '<', len - i);
    if (!lt) return false;
    const size_t p = (size_t)(lt - rec);
    const char* gt = (const char*)memchr(rec + p, '>', len - p);
    if (!gt) return false;
    const size_t q = (size_t)(gt - rec);

    // "<eor>" / "<eoh>" carry no length — skip them.
    const char* colon = (const char*)memchr(rec + p + 1, ':', q - p - 1);
    if (!colon) { i = q + 1; continue; }

    const size_t fname_len = (size_t)(colon - (rec + p + 1));
    int vlen = atoi(colon + 1);       // stops at ':' (type suffix) or '>'
    if (vlen < 0) vlen = 0;
    const size_t vstart = q + 1;
    if (vstart + (size_t)vlen > len) return false;   // runs past the record

    if (fname_len == nlen && strncasecmp(rec + p + 1, name, nlen) == 0) {
      size_t n = (size_t)vlen;
      if (n > out_sz - 1) n = out_sz - 1;            // truncate, don't drop
      memcpy(out, rec + vstart, n);
      out[n] = '\0';
      return true;
    }
    i = vstart + (size_t)vlen;
  }
  return false;
}

bool log_record_has_call(const char* rec, size_t len) {
  char buf[16];
  return log_field(rec, len, "call", buf, sizeof(buf)) && buf[0] != '\0';
}

// Same crude ±0.1 MHz match the Cardputer viewer has always used. Falls back
// to the raw frequency when it doesn't land in a configured band.
void log_band_name(const char* freq, char* out, size_t out_sz) {
  out[0] = '\0';
  if (!freq || !freq[0]) return;
  const double mhz = atof(freq);
  for (const auto& b : g_bands) {
    if (fabs(b.freq * 0.001 - mhz) < 0.1) {
      snprintf(out, out_sz, "%s", b.name ? b.name : "");
      return;
    }
  }
  snprintf(out, out_sz, "%s", freq);
}

void log_fill_entry(const char* rec, size_t len, int index, CoreLogEntry& e) {
  e = CoreLogEntry{};
  e.index = index;

  log_field(rec, len, "call",       e.call,    sizeof(e.call));
  log_field(rec, len, "gridsquare", e.grid,    sizeof(e.grid));
  log_field(rec, len, "mode",       e.mode,    sizeof(e.mode));
  log_field(rec, len, "time_on",    e.time_on, sizeof(e.time_on));
  log_field(rec, len, "freq",       e.freq,    sizeof(e.freq));
  log_field(rec, len, "comment",    e.comment, sizeof(e.comment));
  log_band_name(e.freq, e.band, sizeof(e.band));

  char buf[16];
  // Absent means the node had no report to log (it omits -99 rather than
  // writing a number that would read as a real signal report).
  e.has_rst_sent = log_field(rec, len, "rst_sent", buf, sizeof(buf)) && buf[0];
  if (e.has_rst_sent) e.rst_sent = atoi(buf);
  e.has_rst_rcvd = log_field(rec, len, "rst_rcvd", buf, sizeof(buf)) && buf[0];
  if (e.has_rst_rcvd) e.rst_rcvd = atoi(buf);
}

}  // namespace

int core_log_count(const char* yyyymmdd) {
  char path[64];
  if (!log_day_path(yyyymmdd, path, sizeof(path))) return -1;
  FILE* f = fopen(path, "rb");
  if (!f) return -1;

  // Counts records carrying a call, which is what core_log_read indexes —
  // the two must agree or paging drifts.
  char   rec[kLogRecMax];
  size_t len = 0;
  int    n = 0;
  while (log_next_record(f, rec, &len)) {
    if (log_record_has_call(rec, len)) ++n;
  }
  fclose(f);
  return n;
}

int core_log_list_days(CoreLogDay* out, int max_out) {
  DIR* dir = opendir("/storage");
  if (!dir) return 0;

  // Lexicographic order on YYYYMMDD is chronological, so a plain reverse
  // sort gives newest-first.
  std::vector<std::string> names;
  struct dirent* ent;
  while ((ent = readdir(dir)) != nullptr) {
    if (log_is_day_file(ent->d_name)) names.emplace_back(ent->d_name);
  }
  closedir(dir);
  std::sort(names.begin(), names.end(), std::greater<std::string>());

  const int total = (int)names.size();
  if (!out || max_out <= 0) return total;

  int n = 0;
  for (int i = 0; i < total && n < max_out; ++i) {
    char date[9];
    memcpy(date, names[i].c_str(), 8);
    date[8] = '\0';
    const int count = core_log_count(date);   // reads the file; days are small
    if (count < 0) continue;                  // vanished mid-listing
    memcpy(out[n].date, date, sizeof(date));
    out[n].entries = count;
    ++n;
  }
  return total;
}

int core_log_read(const char* yyyymmdd, int from, int max_out,
                  CoreLogEntry* out) {
  if (!out || max_out <= 0) return 0;
  if (from < 0) from = 0;

  char path[64];
  if (!log_day_path(yyyymmdd, path, sizeof(path))) return -1;
  FILE* f = fopen(path, "rb");
  if (!f) return -1;

  char   rec[kLogRecMax];
  size_t len = 0;
  int    index = 0;
  int    n = 0;
  while (n < max_out && log_next_record(f, rec, &len)) {
    if (!log_record_has_call(rec, len)) continue;   // header, or junk
    if (index >= from) log_fill_entry(rec, len, index, out[n++]);
    ++index;
  }
  fclose(f);
  return n;
}
