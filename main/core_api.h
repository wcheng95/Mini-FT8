#pragma once

// ============================================================================
// core_api.h
//
// Functional-core API for Mini-FT8. UI-agnostic — consumed by both the
// Cardputer local UI and the BLE server. No screen-rendering or terminal
// concepts leak through this boundary.
//
// Consumers follow the "notify + pull" pattern:
//   1. On init, pull a snapshot with core_get_*().
//   2. Register a callback with core_on_*_changed().
//   3. When the callback fires, pull a fresh snapshot and update your view.
//
// Streaming data (waterfall) is push-only: no snapshot, subscribe and
// render rows as they arrive.
//
// Thread safety:
//   - All core_get_*() accessors are safe to call from any task.
//   - Callbacks fire from whatever task the change happened on
//     (audio task for RX, main task for config, etc.).
//     Handlers MUST be fast and non-blocking; queue work to your own task.
// ============================================================================

#include <cstdint>
#include <string>
#include <vector>

#include "ui.h"  // for RxDecodeEntry  (shared with UI layer)

// ---------------------------------------------------------------------------
// Version
// ---------------------------------------------------------------------------
const char* core_api_version();   // "major.minor.patch"

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------
enum class CoreBeaconMode : uint8_t { OFF = 0, EVEN = 1, ODD = 2 };
enum class CoreCqType     : uint8_t { CQ = 0, SOTA, POTA, QRP, FD, FREETEXT };
enum class CoreOffsetSrc  : uint8_t { RX = 0, CURSOR = 1, RANDOM = 2 };
enum class CoreRadioType  : uint8_t { QMX = 0, KH1 = 1 };

enum class CoreQsoState : uint8_t {
  CALLING, REPLYING, REPORT, ROGER_REPORT, ROGERS, SIGNOFF, IDLE
};

enum class CoreTxMsg : uint8_t {
  NONE, TX1, TX2, TX3, TX4, TX5, TX6, FREETEXT
};

// ---------------------------------------------------------------------------
// Snapshot types
// ---------------------------------------------------------------------------

// One active QSO context from the autoseq queue.
struct QsoEntry {
  std::string  dxcall;
  std::string  dxgrid;
  CoreQsoState state;
  CoreTxMsg    next_tx;
  int          retry_counter;
  int          retry_limit;
  int          slot_parity;   // 0 = even, 1 = odd
  int          snr_tx;        // -99 if unset
  int          snr_rx;        // -99 if unset
  bool         is_fd;
  bool         logged;
};

// Pending TX ready to transmit at the next boundary.
struct NextTxEntry {
  bool        valid;
  std::string text;
  std::string dxcall;
  int         slot_parity;
  int         offset_hz;
  int         retries_remaining;
};

// Snapshot of autoseq state: active queue + pending TX.
struct QsoSnapshot {
  std::vector<QsoEntry> active;
  NextTxEntry           next_tx;
};

// Full station configuration.
struct StationConfig {
  // Identity
  std::string      call;
  std::string      grid;
  std::string      comment;

  // Radio
  CoreRadioType         radio;
  std::vector<uint32_t> bands_hz;   // band frequencies in Hz
  int                   band_idx;

  // CQ / beacon
  CoreCqType       cq_type;
  std::string      cq_freetext;
  CoreBeaconMode   beacon;

  // Offset
  CoreOffsetSrc    offset_src;
  int              offset_hz;

  // TX policy
  bool             skip_tx1;
  int              max_retry;

  // RTC
  int32_t          rtc_comp;       // ppm-like drift comp
  std::string      date;           // "YYYY-MM-DD"
  std::string      time;           // "HH:MM:SS"

  // Filters
  std::vector<std::string> ignore_prefixes;
};

// One row from the live waterfall stream.
struct WaterfallRow {
  int                  sym;       // 0..92 within current 15s slot
  const uint8_t*       mag;       // num_bins bytes; null during TX
  int                  num_bins;  // matches config (~433 at 6 kHz, f_max=2900)
  float                swr;       // stubbed 1.5 until real polling
  float                pwr;       // stubbed 2.0 until real polling
  bool                 ptt;       // true during TX (mag is null)
};

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

// Initialize the core. Must be called once at startup before any other
// core_* call. Loads config from flash, creates synchronization primitives.
void core_init();

// ---------------------------------------------------------------------------
// Snapshot accessors (thread-safe pulls)
// ---------------------------------------------------------------------------

// Decoded RX list from the most recent slot.
void core_get_rx_list(std::vector<RxDecodeEntry>& out);

// Autoseq state (active QSOs + pending TX).
void core_get_qso(QsoSnapshot& out);

// Non-allocating QSO accessors. Consumers that can't tolerate the
// std::vector<QsoEntry>::reserve inside core_get_qso (e.g. the BLE
// QSO_QUEUE read handler running under heap fragmentation) iterate
// active entries via core_qso_active_count + core_qso_get_active. The
// strings inside QsoEntry / NextTxEntry are FT8 callsigns and grids
// short enough to live in std::string's SSO buffer — no heap there.
int  core_qso_active_count();
bool core_qso_get_active(int idx, QsoEntry& out);
bool core_qso_get_next_tx(NextTxEntry& out);

// Full station config.
void core_get_config(StationConfig& out);

// ---------------------------------------------------------------------------
// Change notifications
// ---------------------------------------------------------------------------
//
// These fire on the task where the mutation happened. Keep handlers trivial
// (set a flag, enqueue a message) and do the real work in your own task.
// ---------------------------------------------------------------------------

using CoreChangeCb    = void (*)(void);
using CoreWaterfallCb = void (*)(const WaterfallRow& row);

void core_on_rx_changed    (CoreChangeCb cb);
void core_on_qso_changed   (CoreChangeCb cb);
void core_on_config_changed(CoreChangeCb cb);
void core_on_waterfall_row (CoreWaterfallCb cb);

// ---------------------------------------------------------------------------
// Commands (mutations)
//
// All return true on accept, false on reject (bad argument, etc.).
// Each successful command fires the matching on_*_changed notification.
// ---------------------------------------------------------------------------

// RX interaction
bool core_cmd_tap_rx(int rx_list_idx);     // reply to the decoded message at idx

// Autoseq / TX
bool core_cmd_cancel_tx();                 // abort in-flight TX, clear pending
bool core_cmd_clear_qso_queue();
bool core_cmd_drop_qso(int idx);           // drop one active QSO by queue index
bool core_cmd_queue_freetext(const std::string& text);

// Band / radio
bool core_cmd_set_band(int band_idx);
bool core_cmd_set_radio(CoreRadioType r);

// Identity
bool core_cmd_set_call(const std::string& call);
bool core_cmd_set_grid(const std::string& grid);
bool core_cmd_set_comment(const std::string& comment);

// CQ / beacon
bool core_cmd_set_cq_type(CoreCqType t);
bool core_cmd_set_cq_freetext(const std::string& text);
bool core_cmd_set_beacon(CoreBeaconMode m);

// Offset
bool core_cmd_set_offset_src(CoreOffsetSrc s);
bool core_cmd_set_offset_hz(int hz);

// TX policy
bool core_cmd_set_skip_tx1(bool skip);
bool core_cmd_set_max_retry(int n);

// RTC
bool core_cmd_set_rtc(int64_t epoch_ms);
bool core_cmd_set_rtc_comp(int32_t ppm_like);

// Ignore list
bool core_cmd_ignore_add   (const std::string& prefix);
bool core_cmd_ignore_remove(const std::string& prefix);
bool core_cmd_ignore_clear();

// System
bool core_cmd_save_config();   // persist to flash (most setters do this auto)

// ---------------------------------------------------------------------------
// ADIF bulk stream
//
// Handle-based streaming read of a day's ADIF log. Caller polls
// core_adif_read() repeatedly; returns false when EOF reached.
// Multiple concurrent readers are not supported (single-reader invariant).
//
// Logs are stored one file per UTC day, "/storage/YYYYMMDD.txt", written by
// log_adif_entry() in main.cpp. A day's file is append-only while that day
// is current and immutable thereafter, so (date, byte offset) is a stable
// resume cursor: a client that has already read N bytes of a given day can
// reopen at N and receive only what was appended since.
//
// The offset must land on a record boundary. Records end with "<eor>\n", so
// a client that tracks "bytes up to and including the last complete <eor>"
// always has a valid offset. The firmware does not validate this — a bogus
// offset yields a partial first record, which the client's parser drops.
// ---------------------------------------------------------------------------

struct CoreAdifHandle {
  int  id;       // >= 0 on success, -1 on failure
  int  total;    // total file size in bytes, or -1 if unknown
  int  offset;   // byte position this session starts at
};

// One day-log file. Enumerated newest-first by core_adif_list().
struct CoreAdifFileInfo {
  char date[9];  // "YYYYMMDD", NUL-terminated
  int  size;     // bytes
};

// Fills up to `max_out` entries, newest date first. Returns the total number
// of day-logs present, which may exceed `max_out` — the caller can detect
// truncation by comparing the return value against max_out.
int core_adif_list(CoreAdifFileInfo* out, int max_out);

// `yyyymmdd` selects the day-log; nullptr or "" means today. `offset` is the
// byte position to resume from (0 = whole file). Fails if offset is past EOF.
CoreAdifHandle core_adif_open(const char* yyyymmdd = nullptr, int offset = 0);
// Reads the next chunk. Fills `out` with up to `max_bytes` bytes.
// Returns true while more data is expected; false when EOF or error.
bool           core_adif_read(int handle, std::vector<uint8_t>& out,
                              size_t max_bytes = 256);
void           core_adif_close(int handle);
