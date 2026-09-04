#pragma once

// ============================================================================
// core_api_internal.h
//
// Internal hooks called by the functional-core implementation and by the
// existing mutation sites in main.cpp / stream_uac.cpp. NOT a public API —
// external consumers (Cardputer UI, BLE server) use core_api.h.
//
// These helpers exist so that changes produced inside main.cpp's legacy
// code paths can also wake up registered core_on_*_changed() callbacks
// without the core API having to monkey-patch every write site.
// ============================================================================

#include <cstdint>

// Called by any code that mutates RX list state.
// Fires the registered core_on_rx_changed callback (if any).
void core_fire_rx_changed();

// Called by any code that mutates the autoseq / QSO queue / next-TX state.
// Fires the registered core_on_qso_changed callback (if any).
void core_fire_qso_changed();

// Called by any code that mutates station configuration.
// Fires the registered core_on_config_changed callback (if any).
void core_fire_config_changed();

// Called by the DSP task once per symbol (~6.25 Hz) with a fresh waterfall row.
// mag may be null during TX; num_bins is zero in that case.
// Fires the registered core_on_waterfall_row callback (if any).
void core_fire_waterfall_row(int sym,
                             const uint8_t* mag, int num_bins,
                             float swr, float pwr, bool ptt);

#include <string>
#include <vector>
#include "autoseq.h"

// ---------------------------------------------------------------------------
// Autoseq owner task (main.cpp, core-0 main loop).
//
// autoseq has no locks and never will: every call into it happens on the
// main loop, the way DXFT8 ran it single-threaded. Other tasks — the decode
// task on core 1, the BLE RPC task — post work here and never touch autoseq
// or the armed-TX state directly. Posting is fire-and-forget: an RPC "ok"
// means accepted, and the result shows up in the next QSO_QUEUE snapshot.
// ---------------------------------------------------------------------------
enum class AutoseqOwnerCfg { STATION, CQ_TYPE };
void autoseq_owner_post_touch(const UiRxLine& msg);
void autoseq_owner_post_clear();
void autoseq_owner_post_drop(int idx);
void autoseq_owner_post_freetext(const std::string& text);
void autoseq_owner_post_config(AutoseqOwnerCfg what);
void autoseq_owner_post_skip_tx1(bool skip);
void autoseq_owner_post_max_retry(int n);

// Read-only snapshot published by the owner whenever autoseq changes.
// This is what other tasks read instead of autoseq itself.
struct AutoseqOwnerSnapshot {
  std::vector<QsoContext> active;
  AutoseqTxEntry          next{};
  bool                    next_valid = false;
};
void autoseq_owner_get_snapshot(AutoseqOwnerSnapshot& out);
