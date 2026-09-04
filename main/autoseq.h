#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <functional>
#include "ui.h"

// Queue size: active + inactive entries.
constexpr int AUTOSEQ_MAX_QUEUE = 30;
// Maximum retries before moving to inactive zone
constexpr int AUTOSEQ_MAX_RETRY = 5;

// High-level auto-sequencer states
// Order matters for priority sorting (higher = more advanced in QSO)
enum class AutoseqState {
    CALLING = 0,   // We sent CQ (TX6)
    REPLYING,      // We sent TX1 (grid)
    REPORT,        // We sent TX2 (SNR report)
    ROGER_REPORT,  // We sent TX3 (R+SNR)
    ROGERS,        // We sent TX4 (RR73)
    SIGNOFF,       // We sent TX5 (73)
    IDLE           // QSO complete (auto-removed)
};

// FT8 message types.
//
// Typing invariant for QsoContext::next_tx:
//
//     next_tx ∈ {TX_NONE, TX1, TX2, TX3, TX4, TX5}
//
// TX6 is the FT8 CQ message but is NEVER assigned to next_tx. CQ ctxs
// (and the Free Text variant that piggy-backs on the CQ infrastructure)
// use next_tx = TX_NONE — text comes from the singleton s_tx_msg_buffer,
// populated at refresh time by either generate_cq_text_into() (for CQ)
// or from the s_pending_ft_text sidecar (for FT).
//
// Semantics of TX_NONE on next_tx:
//   - On a CALLING ctx: "one-shot — text from s_tx_msg_buffer, evict
//     after one TX (tick CALLING → IDLE → pop)."
//   - On an IDLE ctx: "no TX, evict immediately by sort_and_clean."
enum class TxMsgType {
    TX_NONE = 0,
    TX1,  // <DXCALL> <MYCALL> <GRID>
    TX2,  // <DXCALL> <MYCALL> ##
    TX3,  // <DXCALL> <MYCALL> R##
    TX4,  // <DXCALL> <MYCALL> RR73
    TX5,  // <DXCALL> <MYCALL> 73
    TX6   // CQ <MYCALL> <GRID> — never used for next_tx (one-shots use TX_NONE)
};

// QSO context - one per active contact
struct QsoContext {
    AutoseqState state = AutoseqState::IDLE;
    TxMsgType next_tx = TxMsgType::TX_NONE;
    TxMsgType rcvd_msg_type = TxMsgType::TX_NONE;

    std::string dxcall;     // Remote station callsign
    std::string dxgrid;     // Remote grid (preserved from initial exchange!)

    int snr_tx = -99;       // What we report to them (our measurement of their signal)
    int snr_rx = -99;       // What they reported about us

    int retry_counter = 0;
    int retry_limit = AUTOSEQ_MAX_RETRY;
    bool logged = false;    // Prevents duplicate ADIF logging
    bool is_fd = false;
    std::string fd_rx_exchange; // Last received FD exchange (for Cabrillo logging)
    // SIGNOFF handling:
    // true  -> after sending TX5, park context in inactive for possible late RR73
    // false -> after sending TX5, finish immediately (IDLE/pop)
    bool park_after_signoff_tx = false;
    // Timestamp when the context was moved to inactive zone (ms, monotonic).
    // 0 means currently active / not yet parked.
    int64_t inactive_since_ms = 0;

    int offset_hz = 1500;   // TX audio offset
    int slot_id = 0;        // TX slot (0=even, 1=odd)

    // One-shot entries (CQ and Free Text) share the CALLING state but differ
    // in sort priority: FT must TX first (preempts QSOs), CQ stays at bottom
    // (QSOs take priority over beacon CQ). is_freetext is the priority flag
    // consulted in compare_ctx.
    //
    // Text storage: CQ text is regenerated from template at refresh; FT text
    // is held in the s_pending_ft_text singleton (only one FT pending at a
    // time, so one sidecar suffices). No per-ctx text field is needed.
    bool is_freetext = false;
};

// TX entry for scheduling
struct AutoseqTxEntry {
    std::string text;       // Full FT8 message text
    std::string dxcall;     // Target callsign
    int offset_hz = 1500;
    int slot_id = 0;
    int repeat_counter = 5;
    bool is_signoff = false; // True for TX4/TX5 (priority scheduling)
};

// ADIF logging callback type
using AdifLogCallback = std::function<void(const std::string& dxcall,
                                            const std::string& dxgrid,
                                            int rst_sent, int rst_rcvd)>;

// ============== Public API ==============

// Initialize/reset the autoseq engine
void autoseq_init();

// Clear all active QSOs
void autoseq_clear();

// Drop a QSO by index (0-based in display order).
// For active QSOs, this moves the context to inactive; CQ entries are removed.
bool autoseq_drop_index(int idx);

// Rotate to the next QSO with the same slot parity as the current head.
// Returns true if a rotation occurred.
bool autoseq_rotate_same_parity();

// Start a CQ call (adds CQ to queue). One-shot: transmits once then evicts.
// slot_parity: 0 for even slots, 1 for odd slots
void autoseq_start_cq(int slot_parity);

// Remove a pending beacon CQ (CALLING, non-freetext) if one is queued.
// Called when the beacon turns off, so a CQ armed for a future slot can't
// fire after the operator asked for silence.
void autoseq_cancel_cq();

// Schedule a Free Text one-shot transmission.
// - Inherits slot parity from queue[0] if queue is non-empty, so FT joins the
//   current activation period instead of colliding with other QSOs' slots.
// - If queue is empty, uses fallback_slot_parity (caller should pass the next
//   TX slot's parity based on wall-clock).
// Returns false if no room or text is empty.
bool autoseq_schedule_freetext(const std::string& text, int fallback_slot_parity);

// Manual response: user taps on a decoded message
void autoseq_on_touch(const UiRxLine& msg);

// Automatic response: process all decoded messages addressed to us
void autoseq_on_decodes(const std::vector<UiRxLine>& to_me_messages);

// TX retry tick - call AFTER TX completes to set up retry
// This advances retry counter and sets next_tx for the next attempt
void autoseq_tick(int64_t slot_idx, int slot_parity, int ms_to_boundary);

// Get next TX text based on current state (does NOT modify state)
bool autoseq_get_next_tx(std::string& out_text);

// Fetch pending TX entry based on current state (does NOT modify state)
bool autoseq_fetch_pending_tx(AutoseqTxEntry& out);

// Mark TX as sent (called after transmission completes)
void autoseq_mark_sent(int64_t slot_idx);

// Called from tx_start() immediately before TX emission begins.
// Logs the QSO if we're about to emit TX4 (RR73) or TX5 (73) — this is the
// single logging trigger. ctx->logged flag prevents duplicate logs across
// retries or re-emissions.
void autoseq_on_tx_starting();

// Get display strings for active QSOs
void autoseq_get_qso_states(std::vector<std::string>& out);

// Queue snapshot for the RxTx log, one line per context, DXFT8-style. The
// firmware prefixes each with "Q [ts][band] "; the host harness prints them
// per period. Field key:
//   a|i        zone: active / inactive (parked)
//   s<n>       state      (CALLING=0 REPLYING=1 REPORT=2 ROGER_REPORT=3 ROGERS=4 SIGNOFF=5 IDLE=6)
//   t<n>       next_tx    (0=none 1..5=TX1..TX5)
//   r<n>       rcvd_msg_type, last classified message from DX
//   dxcall grid snr_tx snr_rx retry/limit p<parity>  [L]=logged  [FT]=free text
// A context in the active zone with s>1 and t0 is unschedulable: that is
// the "dead head" that silenced the beacon for 18 minutes on 2026-08-28.
void autoseq_get_queue_log_lines(std::vector<std::string>& out);

// Copy the full active-zone QsoContext entries for structured consumers
// (e.g. the core_api / BLE server). Returns active contexts only; inactive
// zone is not included. Thread-safe snapshot.
void autoseq_get_active_contexts(std::vector<QsoContext>& out);

// Non-allocating accessors for hot paths that can't tolerate
// std::vector::reserve under heap fragmentation (BLE QSO_QUEUE read).
int  autoseq_active_count();
bool autoseq_get_active_context(int idx, QsoContext* out);

// Check if there's any active QSO (not IDLE)
bool autoseq_has_active_qso();

// Get current queue size
int autoseq_queue_size();

// Set the ADIF logging callback
void autoseq_set_adif_callback(AdifLogCallback cb);

// Cabrillo Field Day callback type (for ARRL-FD logging)
using CabrilloFdLogCallback = void (*)(const std::string& dxcall, const std::string& their_fd_exchange);
void autoseq_set_cabrillo_fd_callback(CabrilloFdLogCallback cb);

// Configuration setters (called when station data changes)
void autoseq_set_station(const std::string& call, const std::string& grid);
void autoseq_set_skip_tx1(bool skip);  // Skip TX1 and start with TX2
void autoseq_set_max_retry(int retry); // Runtime retry limit for new/retried TX states

// CQ type configuration
enum class AutoseqCqType { CQ = 0, SOTA, POTA, QRP, FD, FREETEXT };
void autoseq_set_cq_type(AutoseqCqType type, const std::string& freetext = "");
