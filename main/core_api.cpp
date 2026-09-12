// ============================================================================
// core_api.cpp — functional-core implementation
//
// Thin adapter over the existing main.cpp / autoseq.cpp / ui.cpp internals.
// No behavioral change; just exposes a UI-agnostic surface to consumers
// (Cardputer local UI and future control surfaces).
//
// See core_api.h for the public contract and docs/NATIVE_CLIENT_ARCHITECTURE.md
// for the high-level design.
// ============================================================================

#include "core_api.h"
#include "core_api_internal.h"
#include "station_types.h"
#include "autoseq.h"
#include "storage_service.h"

#include <atomic>

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
void sync_ignore_prefix_text_from_list();
bool rtc_apply_manual_time_from_strings();
bool set_manual_grid_config(const std::string& grid);

// Access to ui.cpp (RX list live in ui.cpp's static array).
int  ui_get_rx_count();
bool ui_get_rx_entry(int idx, RxDecodeEntry* out);

// ---------------------------------------------------------------------------
// Callback registry
// ---------------------------------------------------------------------------
// Single-slot per event. Multiple simultaneous consumers is a step 3 concern.
// ---------------------------------------------------------------------------

namespace {
// Multi-slot registry so multiple consumers can coexist.
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
  if (r == RadioType::KH1_USBC || r == RadioType::KH1_MIC) {
    return CoreRadioType::KH1;
  }
  return (r == RadioType::QDX) ? CoreRadioType::QDX : CoreRadioType::QMX;
}
RadioType map_in(CoreRadioType r) {
  if (r == CoreRadioType::KH1) return RadioType::KH1;
  return (r == CoreRadioType::QDX) ? RadioType::QDX : RadioType::QMX;
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

// Defined in main.cpp. Read by core_get_qso so snapshots reflect
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
  // what was reaching external consumers before and pinning the marker at the config
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
    out.bands_hz.push_back((uint32_t)(b.freq * 1000.0f + 0.5f));  // BandItem.freq is kHz (float)
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
// on a shallow caller task whose stack may not accommodate the 22-
// fprintf chain in save_station_data plus filesystem internals. Set the
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
  // antenna relay). External clients pick a band in one operation, so
  // intentional change — so commit to the radio immediately.
  sync_radio_to_current_band("core set_band");
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
  bool ok = false;
  {
    ConfigGuard g;
    ok = set_manual_grid_config(grid);
  }
  if (!ok) return false;
  g_config_save_pending = true;
  core_fire_config_changed();
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
  if (!rtc_apply_manual_time_from_strings()) return false;
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
    sync_ignore_prefix_text_from_list();
  }
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
    if (removed) sync_ignore_prefix_text_from_list();
  }
  if (!removed) return false;
  g_config_save_pending = true;
  core_fire_config_changed();
  return true;
}
bool core_cmd_ignore_clear() {
  {
    ConfigGuard g;
    g_ignore_prefixes.clear();
    sync_ignore_prefix_text_from_list();
  }
  g_config_save_pending = true;
  core_fire_config_changed();
  return true;
}

bool core_cmd_save_config() {
  g_config_save_pending = true;
  return true;
}

// ---------------------------------------------------------------------------
// ADIF streaming (handle-based, single-reader)
// ---------------------------------------------------------------------------

namespace {
struct AdifSession {
  int            id     = -1;
  StorageStream* stream = nullptr;
  long           total  = -1;
};
AdifSession g_adif;
int         g_adif_next_id = 1;
}  // namespace

CoreAdifHandle core_adif_open() {
  CoreAdifHandle h{-1, -1};
  if (g_adif.stream) return h;  // single-reader invariant

  // Find most recent .adi file in internal storage.
  // For simplicity, we just try today's filename; step 3 can add the
  // full "pick most recent from listing" logic.
  // Build "YYYYMMDD.adi" from g_date.
  char path[64];
  const char* d = g_date.c_str();
  // strip dashes
  char yyyymmdd[9] = {0};
  int o = 0;
  for (int i = 0; d[i] && o < 8; ++i) {
    if (d[i] >= '0' && d[i] <= '9') yyyymmdd[o++] = d[i];
  }
  snprintf(path, sizeof(path), "%s.adi", yyyymmdd);

  StorageStream* stream = storage_stream_open(path, StorageOpenMode::READ);
  if (!stream) return h;
  const long size = storage_stream_size(stream);
  if (size < 0 || !storage_stream_seek(stream, 0, SEEK_SET)) {
    storage_stream_close(stream);
    return h;
  }

  g_adif.id    = g_adif_next_id++;
  g_adif.stream = stream;
  g_adif.total = size;

  h.id    = g_adif.id;
  h.total = (int)size;
  return h;
}

bool core_adif_read(int handle, std::vector<uint8_t>& out, size_t max_bytes) {
  out.clear();
  if (handle != g_adif.id || !g_adif.stream) return false;
  if (max_bytes == 0) max_bytes = 256;
  out.resize(max_bytes);
  size_t n = storage_stream_read(g_adif.stream, out.data(), max_bytes);
  out.resize(n);
  if (n == 0) {
    // EOF or error — caller should close.
    return false;
  }
  return true;
}

void core_adif_close(int handle) {
  if (handle != g_adif.id) return;
  if (g_adif.stream) {
    storage_stream_close(g_adif.stream);
    g_adif.stream = nullptr;
  }
  g_adif.id = -1;
  g_adif.total = -1;
}
