/*
 * autoseq.cpp - FT8 CQ/QSO auto-sequencing engine for Mini-FT8
 *
 * Ported from DX-FT8-MULTIBAND-TABLET-TRANSCEIVER autoseq_engine.c
 * Adapted for ESP32/C++ with integrated TX scheduling.
 */

#include "autoseq.h"
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include "esp_log.h"
#include "esp_timer.h"
#include <string>

//void debug_log_line_public(const std::string& msg);
static const char* TAG = "AUTOSEQ";

// ============== Internal state ==============

static QsoContext s_queue[AUTOSEQ_MAX_QUEUE];
// Active zone: s_queue[0 .. s_active_count-1], grows from front
// Inactive zone: s_queue[s_inactive_start .. AUTOSEQ_MAX_QUEUE-1], grows from back
// Free space: s_queue[s_active_count .. s_inactive_start-1]
static int s_active_count = 0;
static int s_inactive_start = AUTOSEQ_MAX_QUEUE;  // no inactive entries

// Singleton TX message buffer — holds the TX text for queue[0].
// Invariant: (s_active_count == 0) ⇔ s_tx_msg_buffer.empty()
// Refreshed by refresh_tx_msg_buffer() whenever queue[0] or its text
// source may have changed. fetch_pending_tx() and autoseq_get_next_tx()
// are pure reads of this buffer — they do not consult next_tx.
static std::string s_tx_msg_buffer;

// Singleton sidecar for Free Text content. Holds the user-provided text
// while an FT one-shot is in the queue. Overwritten on each injection
// (only one FT pending at a time by construction). Unused/stale when no
// FT ctx is in the queue — safe because refresh consults it only when
// queue[0].is_freetext is true.
static std::string s_pending_ft_text;

// Station configuration
static std::string s_my_call;
static std::string s_my_grid;
static bool s_skip_tx1 = false;
static int s_max_retry = AUTOSEQ_MAX_RETRY;

// Forward declarations (must appear before format_tx_text uses them)
static std::string trim_copy(const std::string& s);
static bool parse_fd_exchange(const std::string& in, std::string& out);

// CQ configuration
static AutoseqCqType s_cq_type = AutoseqCqType::CQ;
static std::string s_cq_freetext;

// ADIF callback
static AdifLogCallback s_adif_callback;

// Cabrillo Field Day callback (for ARRL-FD logging)
static CabrilloFdLogCallback s_cabrillo_fd_callback = nullptr;


// TX scheduling state
static bool s_pending_valid = false;
static AutoseqTxEntry s_pending;
static int s_pending_ctx_idx = -1;
static int64_t s_last_tx_slot_idx = -1000;
static int s_last_tx_parity = -1;

// ============== Forward declarations ==============

static void set_state(QsoContext* ctx, AutoseqState s, TxMsgType first_tx, int limit);
static void generate_cq_text_into(std::string& out);
static void refresh_tx_msg_buffer();
static void format_tx_text(QsoContext* ctx, TxMsgType id, std::string& out);
static TxMsgType parse_rcvd_msg(QsoContext* ctx, const UiRxLine& msg);
static bool generate_response(QsoContext* ctx, const UiRxLine& msg, bool override);
static void on_decode(const UiRxLine& msg);
static bool compare_ctx(const QsoContext& left, const QsoContext& right);
static void pop_front();
static QsoContext* append_ctx();
static void move_to_inactive(int idx);
static void evict_oldest_inactive();
static int find_inactive_by_dxcall(const std::string& dxcall);
static void reactivate(int inactive_idx);
static void sort_and_clean();
static bool looks_like_grid(const std::string& s);
static bool looks_like_report(const std::string& s, int& out);
static void log_qso_if_needed(QsoContext* ctx);
static std::string normalize_call_token(const std::string& s);

// Returns true if we've exchanged reports with DX — meaning we sent TX2/TX3
// (a report or R+report) so the metadata (snr_tx) is what we actually transmitted.
// REPLYING (sent TX1 = grid only) is safe to evict: no reports exchanged yet,
// and if DX retries with TX2, a fresh context gets correct snr_tx from msg.snr.
static bool has_exchanged(const QsoContext* ctx) {
    if (!ctx) return false;
    if (ctx->dxcall.empty() || ctx->dxcall == "CQ") return false;
    return ctx->state > AutoseqState::REPLYING;
}

static inline int64_t mono_ms() {
    return esp_timer_get_time() / 1000;
}

static inline int clamp_retry_limit(int retry) {
    if (retry < 0) return 0;
    return retry;
}

// ============== Public API ==============

void autoseq_init() {
    s_active_count = 0;
    s_inactive_start = AUTOSEQ_MAX_QUEUE;
    s_pending_valid = false;
    s_pending_ctx_idx = -1;
    s_last_tx_slot_idx = -1000;
    s_last_tx_parity = -1;
    s_tx_msg_buffer.clear();
    s_pending_ft_text.clear();
}

void autoseq_clear() {
    autoseq_init();
}

/*
static void dlogf(const char* fmt, ...) {
  char b[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  debug_log_line_public(b);
}
*/

bool autoseq_drop_index(int idx) {
    if (idx < 0 || idx >= s_active_count) return false;
    // TX-page "drop" should preserve QSO metadata for late retries.
    // Keep CQ/transient entries removable as before.
    if (s_queue[idx].state == AutoseqState::CALLING || s_queue[idx].dxcall == "CQ") {
        for (int i = idx; i + 1 < s_active_count; ++i) {
            s_queue[i] = s_queue[i + 1];
        }
        --s_active_count;
        refresh_tx_msg_buffer();
        return true;
    }
    move_to_inactive(idx);
    refresh_tx_msg_buffer();
    return true;
}

bool autoseq_rotate_same_parity() {
    if (s_active_count < 2) return false;

    int parity = s_queue[0].slot_id & 1;

    int last = -1;
    for (int i = 1; i < s_active_count; ++i) {
        if ((s_queue[i].slot_id & 1) == parity) last = i;
        else break; // optional: only rotate within the front same-parity run
    }
    if (last == -1) return false;

    QsoContext head = s_queue[0];
    for (int i = 0; i < last; ++i) {
        s_queue[i] = s_queue[i + 1];
    }
    s_queue[last] = head;
    refresh_tx_msg_buffer();
    return true;
}

// Shared helper for injecting one-shot CALLING entries (CQ and Free Text).
// Both types are single-transmission: emit once, tick moves CALLING → IDLE → pop.
// Returns the appended ctx, or nullptr if no room.
// Inject a one-shot CALLING entry. The caller is responsible for:
//   - Choosing the dxcall marker ("CQ" for beacon CQ, "(FT)" for Free Text)
//   - Setting is_freetext (sort priority — FT above QSOs, CQ below)
//   - Picking the slot parity
// Text is NOT stored on the ctx; it's derived at refresh time from either
// the CQ template (for CQ) or the s_pending_ft_text sidecar (for FT).
// Returns the appended ctx, or nullptr if no room.
static QsoContext* enqueue_one_shot(const std::string& dxcall,
                                    bool is_freetext,
                                    int slot_parity) {
    if (s_inactive_start <= s_active_count) {
        evict_oldest_inactive();
        if (s_inactive_start <= s_active_count) return nullptr;
    }
    QsoContext* ctx = append_ctx();
    ctx->dxcall = dxcall;
    ctx->dxgrid.clear();
    ctx->snr_tx = -99;
    ctx->snr_rx = -99;
    ctx->slot_id = slot_parity;
    ctx->is_freetext = is_freetext;  // compare_ctx priority (FT above CQ)
    // next_tx = TX_NONE marks this ctx as "evict after tick." Text source
    // is determined by is_freetext at refresh time — decoupled from next_tx.
    set_state(ctx, AutoseqState::CALLING, TxMsgType::TX_NONE, 0);
    return ctx;
}

void autoseq_start_cq(int slot_parity) {
    // One CQ at a time — but a repeat call is a statement of intent, not a
    // duplicate: the beacon mode may have changed since the existing CQ was
    // enqueued (the UI cycles OFF->EVEN->ODD, so reaching ODD passes through
    // EVEN). Dropping the request here is how a CQ armed with stale parity
    // fired on the wrong slot (RT260828: first CQ at :30 with beacon ODD).
    // Retarget the existing entry instead.
    for (int i = 0; i < s_active_count; ++i) {
        if (s_queue[i].state == AutoseqState::CALLING && !s_queue[i].is_freetext) {
            if ((s_queue[i].slot_id & 1) != (slot_parity & 1)) {
                s_queue[i].slot_id = slot_parity & 1;
                ESP_LOGI(TAG, "Retargeted CQ to slot %d", slot_parity & 1);
                refresh_tx_msg_buffer();
            }
            return;
        }
    }
    if (enqueue_one_shot("CQ", false, slot_parity)) {
        ESP_LOGI(TAG, "Started CQ on slot %d", slot_parity);
        refresh_tx_msg_buffer();
    }
}

void autoseq_cancel_cq() {
    for (int i = 0; i < s_active_count; ++i) {
        if (s_queue[i].state == AutoseqState::CALLING && !s_queue[i].is_freetext) {
            autoseq_drop_index(i);
            ESP_LOGI(TAG, "Canceled pending CQ");
            return;
        }
    }
}

bool autoseq_schedule_freetext(const std::string& text, int fallback_slot_parity) {
    if (text.empty()) return false;
    // Don't duplicate an already-pending Free Text.
    for (int i = 0; i < s_active_count; ++i) {
        if (s_queue[i].is_freetext) return false;
    }
    // Inherit parity from queue head if non-empty (preserves queue parity
    // invariant — FT joins the current activation period). Otherwise use
    // caller-provided fallback.
    int slot_parity = (s_active_count > 0)
                        ? (s_queue[0].slot_id & 1)
                        : (fallback_slot_parity & 1);
    // Write the user text to the sidecar BEFORE enqueuing, so refresh can
    // find it as soon as the ctx appears at queue[0].
    s_pending_ft_text = text;
    QsoContext* ctx = enqueue_one_shot("(FT)", true, slot_parity);
    if (ctx) {
        ESP_LOGI(TAG, "Scheduled Free Text on slot %d: %s", slot_parity, text.c_str());
        // Sort so FT lands at the correct position (top, above QSOs).
        sort_and_clean();
        refresh_tx_msg_buffer();
        return true;
    }
    return false;
}

void autoseq_on_touch(const UiRxLine& msg) {
    // If no free space, evict an inactive entry to make room
    if (s_inactive_start <= s_active_count) {
        evict_oldest_inactive();
        if (s_inactive_start <= s_active_count) return;  // Still no room
    }

    QsoContext* ctx = append_ctx();

    // Determine the DX callsign from the message (normalize to handle <> wrapped hashed calls)
    std::string dxcall;
    if (!msg.field2.empty()) {
        dxcall = normalize_call_token(msg.field2);  // field2 is the sender
    } else if (!msg.field1.empty() && msg.field1 != "CQ") {
        dxcall = normalize_call_token(msg.field1);
    }

    // Check if it's addressed to me (normalize to handle <> wrapped hashed calls)
    std::string f1_norm = normalize_call_token(msg.field1);
    std::string my_norm = normalize_call_token(s_my_call);

    if (!my_norm.empty() && f1_norm == my_norm) {
        generate_response(ctx, msg, true);
        sort_and_clean();
        refresh_tx_msg_buffer();
        return;
    }

    // Treat as calling CQ - we're initiating contact
    ctx->dxcall = dxcall;
    if (looks_like_grid(msg.field3)) {
        ctx->dxgrid = msg.field3;
    }
    //dlogf("r:=%d %s %s %s",
    //  msg.snr, msg.field1.c_str(), msg.field2.c_str(), msg.field3.c_str());
    //dlogf("s:=%d", ctx->snr_tx);

    ctx->snr_tx = msg.snr;  // Our measurement of their signal
    ctx->offset_hz = msg.offset_hz;
    ctx->slot_id = msg.slot_id ^ 1;  // TX on opposite slot

    // Mark this QSO as Field Day only if the *received* message is CQ FD
    // (This avoids sending FD exchange when answering a normal CQ while our CQType is FD.)
    {
        std::string f1 = msg.field1;
        for (auto& ch : f1) ch = toupper((unsigned char)ch);
        // Handle both "FD" (decoder strips CQ prefix) and "CQ FD" (raw format)
        ctx->is_fd = (f1 == "FD" || f1 == "CQ FD" || f1.rfind("CQ FD", 0) == 0);
    }

    //dlogf("TH: %s %s %s snr=%d",
    //  msg.field1.c_str(), msg.field2.c_str(), msg.field3.c_str(), msg.snr);

    //dlogf("TH: bf snr_tx=%d skip=%d",
    //  ctx->snr_tx, (int)s_skip_tx1);

    set_state(ctx, s_skip_tx1 ? AutoseqState::REPORT : AutoseqState::REPLYING,
              s_skip_tx1 ? TxMsgType::TX2 : TxMsgType::TX1, s_max_retry);
    sort_and_clean();
    refresh_tx_msg_buffer();

    //dlogf("TH: af snr_tx=%d state=%d",
    //  ctx->snr_tx, (int)ctx->state);

    //ESP_LOGI(TAG, "Touch: %s grid=%s snr=%d", ctx->dxcall.c_str(),
    //         ctx->dxgrid.c_str(), ctx->snr_tx);
}

void autoseq_on_decodes(const std::vector<UiRxLine>& to_me_messages) {
    ESP_LOGI(TAG, "on_decodes: %d messages, active=%d inactive=%d",
             (int)to_me_messages.size(), s_active_count,
             AUTOSEQ_MAX_QUEUE - s_inactive_start);
    for (const auto& msg : to_me_messages) {
        ESP_LOGI(TAG, "  msg: %s %s %s snr=%d",
                 msg.field1.c_str(), msg.field2.c_str(), msg.field3.c_str(), msg.snr);
        on_decode(msg);
    }
    sort_and_clean();
    refresh_tx_msg_buffer();
    if (s_active_count > 0) {
        ESP_LOGI(TAG, "on_decodes done: queue[0] state=%d, next_tx=%d, dxcall=%s",
                 (int)s_queue[0].state, (int)s_queue[0].next_tx, s_queue[0].dxcall.c_str());
    }
}

// Called AFTER TX completes to set up retry for next attempt
// This is the reference architecture - tick is for retry management, not scheduling
void autoseq_tick(int64_t slot_idx, int slot_parity, int ms_to_boundary) {
    (void)slot_idx; (void)slot_parity; (void)ms_to_boundary;  // unused for now

    if (s_active_count == 0) return;

    QsoContext* ctx = &s_queue[0];

    // Advance retry counter or move to inactive zone
    switch (ctx->state) {
        case AutoseqState::REPLYING:
        case AutoseqState::REPORT:
        case AutoseqState::ROGER_REPORT:
        case AutoseqState::ROGERS: {
            if (ctx->retry_counter < ctx->retry_limit) {
                // Set up next retry based on current state
                switch (ctx->state) {
                    case AutoseqState::REPLYING:     ctx->next_tx = TxMsgType::TX1; break;
                    case AutoseqState::REPORT:       ctx->next_tx = TxMsgType::TX2; break;
                    case AutoseqState::ROGER_REPORT: ctx->next_tx = TxMsgType::TX3; break;
                    case AutoseqState::ROGERS:       ctx->next_tx = TxMsgType::TX4; break;
                    default: break;
                }
                ctx->retry_counter++;
            } else if (has_exchanged(ctx)) {
                // Retries exhausted but we exchanged data with DX — move to
                // inactive zone to preserve metadata in case DX retries.
                move_to_inactive(0);
            } else {
                // No exchange happened (e.g. sent TX1 but got nothing back).
                // Safe to evict.
                ctx->state = AutoseqState::IDLE;
                ctx->next_tx = TxMsgType::TX_NONE;
            }
            break;
        }
        case AutoseqState::CALLING:  // CQ only once (controlled by beacon)
            // CQ with no response — safe to evict (dxcall is "CQ")
            ctx->state = AutoseqState::IDLE;
            ctx->next_tx = TxMsgType::TX_NONE;
            break;
        case AutoseqState::SIGNOFF:
            // After TX5:
            // - First pass: park in inactive so late RR73 can be answered once.
            // - Late-ack pass: finish immediately.
            if (ctx->park_after_signoff_tx) {
                move_to_inactive(0);
            } else {
                ctx->state = AutoseqState::IDLE;
                ctx->next_tx = TxMsgType::TX_NONE;
            }
            break;
        default:
            break;
    }

    if (s_active_count > 0 && s_queue[0].state == AutoseqState::IDLE) {
        pop_front();
    }
    refresh_tx_msg_buffer();

    ESP_LOGI(TAG, "Tick: active=%d, state=%d, next_tx=%d, retry=%d/%d",
             s_active_count, s_active_count > 0 ? (int)s_queue[0].state : -1,
             s_active_count > 0 ? (int)s_queue[0].next_tx : -1,
             s_active_count > 0 ? s_queue[0].retry_counter : 0,
             s_active_count > 0 ? s_queue[0].retry_limit : 0);
}

// Pure reads of the singleton s_tx_msg_buffer. No inspection of next_tx.
// The invariant "queue empty ⇔ buffer empty" is maintained by
// refresh_tx_msg_buffer() being called at the end of every public API
// that may change queue[0]'s identity or text source.
bool autoseq_get_next_tx(std::string& out_text) {
    out_text = s_tx_msg_buffer;
    return !out_text.empty();
}

bool autoseq_fetch_pending_tx(AutoseqTxEntry& out) {
    if (s_tx_msg_buffer.empty()) return false;
    // Queue must be non-empty for the buffer to be non-empty (invariant).
    QsoContext* ctx = &s_queue[0];
    out.text = s_tx_msg_buffer;
    out.dxcall = ctx->dxcall;
    out.offset_hz = ctx->offset_hz;
    out.slot_id = ctx->slot_id;
    out.repeat_counter = ctx->retry_limit - ctx->retry_counter;
    out.is_signoff = (ctx->next_tx == TxMsgType::TX4 ||
                      ctx->next_tx == TxMsgType::TX5);

    ESP_LOGI(TAG, "Fetch TX: %s (state=%d, next_tx=%d)",
             out.text.c_str(), (int)ctx->state, (int)ctx->next_tx);
    return true;
}

void autoseq_mark_sent(int64_t slot_idx) {
    if (s_active_count == 0) return;

    s_last_tx_slot_idx = slot_idx;
    s_last_tx_parity = s_queue[0].slot_id & 1;

    // Logging now happens in autoseq_on_tx_starting() when TX4/TX5 is emitted
    ESP_LOGI(TAG, "TX sent on slot %lld", slot_idx);
}

// Called by tx_start() immediately before TX emission. If we're about to emit
// TX4 (RR73) or TX5 (73), this is the point where the QSO is logged to ADIF.
// The ctx->logged flag prevents duplicate logs on retries.
//
// This is the single canonical logging trigger. It replaces the scattered
// log_qso_if_needed() calls that used to live in state machine transitions.
// Logging at TX emission (rather than RX state transition) guarantees:
//   (a) We only log QSOs we actually attempted to complete (emitted RR73/73).
//   (b) Log timestamps are unique per TX slot (only one TX per slot).
//   (c) snr_tx/snr_rx are both latched by the time we reach ROGERS/SIGNOFF
//       (we reached those states via parse_rcvd_msg updates on received TX2/TX3).
void autoseq_on_tx_starting() {
    if (s_active_count == 0) return;
    QsoContext* ctx = &s_queue[0];
    if (ctx->next_tx == TxMsgType::TX4 || ctx->next_tx == TxMsgType::TX5) {
        log_qso_if_needed(ctx);
    }
}

void autoseq_get_queue_log_lines(std::vector<std::string>& out) {
    out.clear();
    char buf[96];
    auto emit = [&](const QsoContext& c, char zone) {
        snprintf(buf, sizeof(buf), "%c s%d t%d r%d %-9.9s %-6.6s %+3d %+3d %d/%d p%d%s%s",
                 zone, (int)c.state, (int)c.next_tx, (int)c.rcvd_msg_type,
                 c.dxcall.c_str(), c.dxgrid.c_str(), c.snr_tx, c.snr_rx,
                 c.retry_counter, c.retry_limit, c.slot_id & 1,
                 c.logged ? " L" : "", c.is_freetext ? " FT" : "");
        out.emplace_back(buf);
    };
    for (int i = 0; i < s_active_count; ++i)                 emit(s_queue[i], 'a');
    for (int i = s_inactive_start; i < AUTOSEQ_MAX_QUEUE; ++i) emit(s_queue[i], 'i');
}

void autoseq_get_qso_states(std::vector<std::string>& out) {
    out.clear();
    static const char* state_names[] = {
        "CALL", "RPLY", "RPRT", "RRPT", "RGRS", "SOFF", "", "ZZZ"
    };

    for (int i = 0; i < s_active_count; ++i) {
        const QsoContext* ctx = &s_queue[i];
        if (ctx->state == AutoseqState::IDLE) continue;

        char buf[32];
        snprintf(buf, sizeof(buf), "%-8.8s %.4s %d/%d",
                 ctx->dxcall.c_str(),
                 state_names[(int)ctx->state],
                 ctx->retry_counter, ctx->retry_limit);
        out.push_back(buf);
    }
}

bool autoseq_has_active_qso() {
    for (int i = 0; i < s_active_count; ++i) {
        if (s_queue[i].state != AutoseqState::IDLE &&
            s_queue[i].state != AutoseqState::CALLING) {
            return true;
        }
    }
    return false;
}

int autoseq_queue_size() {
    return s_active_count;
}

void autoseq_get_active_contexts(std::vector<QsoContext>& out) {
    out.clear();
    out.reserve(s_active_count);
    for (int i = 0; i < s_active_count; ++i) {
        out.push_back(s_queue[i]);
    }
}

int autoseq_active_count() {
    return s_active_count;
}

bool autoseq_get_active_context(int idx, QsoContext* out) {
    if (!out || idx < 0 || idx >= s_active_count) return false;
    *out = s_queue[idx];
    return true;
}

void autoseq_set_adif_callback(AdifLogCallback cb) {
    s_adif_callback = cb;
}


void autoseq_set_cabrillo_fd_callback(CabrilloFdLogCallback cb) {
    s_cabrillo_fd_callback = cb;
}

void autoseq_set_station(const std::string& call, const std::string& grid) {
    s_my_call = call;
    s_my_grid = grid;
    // CQ template is regenerated on every refresh, so just re-derive the
    // buffer if a CQ ctx is currently at queue[0].
    refresh_tx_msg_buffer();
}

void autoseq_set_skip_tx1(bool skip) {
    s_skip_tx1 = skip;
}

void autoseq_set_max_retry(int retry) {
    s_max_retry = clamp_retry_limit(retry);
    for (int i = 0; i < s_active_count; ++i) {
        QsoContext& ctx = s_queue[i];
        if (ctx.retry_limit > 0) {
            ctx.retry_limit = s_max_retry;
            if (ctx.retry_counter > ctx.retry_limit) {
                ctx.retry_counter = ctx.retry_limit;
            }
        }
    }
}

void autoseq_set_cq_type(AutoseqCqType type, const std::string& freetext) {
    s_cq_type = type;
    s_cq_freetext = freetext;
    refresh_tx_msg_buffer();  // re-derive CQ template if CQ is at queue[0]
}

// ============== Internal helpers ==============

// The message a context in state `s` should be sending — "keep sending what
// matches our current state". One-shots (CALLING) use TX_NONE as their
// evict-after-tick marker, so they map to TX_NONE on purpose.
static TxMsgType canonical_next_tx(AutoseqState s) {
    switch (s) {
        case AutoseqState::REPLYING:     return TxMsgType::TX1;
        case AutoseqState::REPORT:       return TxMsgType::TX2;
        case AutoseqState::ROGER_REPORT: return TxMsgType::TX3;
        case AutoseqState::ROGERS:       return TxMsgType::TX4;
        case AutoseqState::SIGNOFF:      return TxMsgType::TX5;
        default:                         return TxMsgType::TX_NONE;
    }
}

static void set_state(QsoContext* ctx, AutoseqState s, TxMsgType first_tx, int limit) {
    ctx->state = s;
    ctx->next_tx = first_tx;
    ctx->retry_counter = 0;
    ctx->retry_limit = limit;
}

// Recompute s_tx_msg_buffer from queue[0]. Sole writer of the TX text
// buffer. Called at the end of every public API that may change queue[0]'s
// identity or text source.
//
// Sources of text:
//   queue empty            → buffer cleared
//   queue[0] state IDLE    → buffer cleared (defensive; sort_and_clean should
//                            have popped IDLE entries first)
//   queue[0] state CALLING → text comes from one of two sources, picked by
//                            is_freetext:
//                              - is_freetext=true:  s_pending_ft_text (sidecar)
//                              - is_freetext=false: regenerated CQ template
//   queue[0] other state   → derive via format_tx_text from ctx->next_tx
//                            (the QSO state machine path)
//
// Text storage rationale: QSO text is derivable from state every time. CQ
// template is cheap to regenerate (and picks up any cq_type/station changes
// automatically). FT text is user-supplied and not derivable, so it lives
// in the s_pending_ft_text sidecar (only one FT pending at a time, so a
// single sidecar suffices).
static uint32_t s_generation = 0;
uint32_t autoseq_generation() { return s_generation; }

static void refresh_tx_msg_buffer() {
    ++s_generation;   // every public API that can change the queue ends here
    s_tx_msg_buffer.clear();
    if (s_active_count == 0) return;
    QsoContext* ctx = &s_queue[0];
    if (ctx->state == AutoseqState::IDLE) return;
    if (ctx->state == AutoseqState::CALLING) {
        if (ctx->is_freetext) {
            s_tx_msg_buffer = s_pending_ft_text;
        } else {
            generate_cq_text_into(s_tx_msg_buffer);
        }
        return;
    }
    // QSO ctx: derive from state machine.
    format_tx_text(ctx, ctx->next_tx, s_tx_msg_buffer);
}

// Generate CQ template text into `out`. Called explicitly by the caller
// when it knows it's handling CQ — no type inference from ctx flags.
static void generate_cq_text_into(std::string& out) {
    char buf[64];
    const char* cq_prefix = "CQ";
    switch (s_cq_type) {
        case AutoseqCqType::SOTA: cq_prefix = "CQ SOTA"; break;
        case AutoseqCqType::POTA: cq_prefix = "CQ POTA"; break;
        case AutoseqCqType::QRP:  cq_prefix = "CQ QRP";  break;
        case AutoseqCqType::FD:   cq_prefix = "CQ FD";   break;
        case AutoseqCqType::FREETEXT:
            out = s_cq_freetext;
            return;
        default: break;
    }
    snprintf(buf, sizeof(buf), "%s %s %s",
             cq_prefix, s_my_call.c_str(), s_my_grid.c_str());
    out = buf;
}

// Derive TX text for a QSO ctx from its state-machine fields.
// Called only for states REPLYING..SIGNOFF where next_tx ∈ {TX1..TX5}.
// One-shot CALLING ctxs never reach this function — their text lives
// in ctx->pending_text, populated at enqueue.
static void format_tx_text(QsoContext* ctx, TxMsgType id, std::string& out) {
    out.clear();
    if (!ctx || ctx->state == AutoseqState::IDLE) return;

    char buf[64];

    switch (id) {
        case TxMsgType::TX1:
            snprintf(buf, sizeof(buf), "%s %s %s",
                     ctx->dxcall.c_str(), s_my_call.c_str(), s_my_grid.c_str());
            out = buf;
            break;

        case TxMsgType::TX2:
            if (ctx->is_fd) {
                // Field Day: send our exchange (from CQ freetext) instead of SNR report
                std::string myex;
                if (!parse_fd_exchange(s_cq_freetext, myex)) {
                    myex = trim_copy(s_cq_freetext);
                }
                snprintf(buf, sizeof(buf), "%s %s %s",
                         ctx->dxcall.c_str(), s_my_call.c_str(), myex.c_str());
                out = buf;
            } else {
                snprintf(buf, sizeof(buf), "%s %s %+d",
                         ctx->dxcall.c_str(), s_my_call.c_str(), ctx->snr_tx);
                out = buf;
            }
            break;

        case TxMsgType::TX3:
            if (ctx->is_fd) {
                // Field Day: acknowledge with 'R' + our exchange
                std::string myex;
                if (!parse_fd_exchange(s_cq_freetext, myex)) {
                    myex = trim_copy(s_cq_freetext);
                }
                snprintf(buf, sizeof(buf), "%s %s R %s",
                         ctx->dxcall.c_str(), s_my_call.c_str(), myex.c_str());
                out = buf;
            } else {
                snprintf(buf, sizeof(buf), "%s %s R%+d",
                         ctx->dxcall.c_str(), s_my_call.c_str(), ctx->snr_tx);
                out = buf;
            }
            break;

        case TxMsgType::TX4:
            snprintf(buf, sizeof(buf), "%s %s RR73",
                     ctx->dxcall.c_str(), s_my_call.c_str());
            out = buf;
            // Logging moved to generate_response() state transitions to avoid
            // dual-core race: format_tx_text is called from both core 0 (UI)
            // and core 1 (decode), causing concurrent ADIF writes.
            break;

        case TxMsgType::TX5:
            snprintf(buf, sizeof(buf), "%s %s 73",
                     ctx->dxcall.c_str(), s_my_call.c_str());
            out = buf;
            break;

        // TX6 is no longer assigned to next_tx — one-shot CQ uses TX_NONE
        // and goes through format_one_shot_text() above. The TX6 enum value
        // remains for FT8 protocol completeness but is not a valid next_tx.

        default:
            break;
    }
}

static inline std::string trim_copy(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) ++a;
    while (b > a && (s[b-1] == ' ' || s[b-1] == '\t' || s[b-1] == '\r' || s[b-1] == '\n')) --b;
    return s.substr(a, b - a);
}

// Parse "1B SCV" or "R 1B SCV". Returns normalized exchange without leading "R " (e.g. "1B SCV").
static bool parse_fd_exchange(const std::string& in, std::string& out_norm) {
    std::string s = trim_copy(in);
    if (s.empty()) return false;
    if (s.size() >= 2 && (s[0] == 'R') && (s[1] == ' ')) {
        s = trim_copy(s.substr(2));
    }
    // Expect: "<num><class> <section>"
    size_t sp = s.find(' ');
    if (sp == std::string::npos) return false;
    std::string tok1 = s.substr(0, sp);
    std::string tok2 = trim_copy(s.substr(sp + 1));
    if (tok1.size() < 2 || tok2.empty()) return false;

    char cls = tok1.back();
    if (cls < 'A' || cls > 'F') return false;

    int num = 0;
    for (size_t i = 0; i + 1 < tok1.size(); ++i) {
        char c = tok1[i];
        if (c < '0' || c > '9') return false;
        num = num * 10 + (c - '0');
        if (num > 99) return false;
    }
    if (num < 1 || num > 32) return false;

    // Section: typically 2-3 chars (or "DX"), accept 2-4 alnum.
    if (tok2.size() < 2 || tok2.size() > 4) return false;
    for (char c : tok2) {
        if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) return false;
    }

    out_norm = tok1 + " " + tok2;
    return true;
}


static TxMsgType parse_rcvd_msg(QsoContext* ctx, const UiRxLine& msg) {



    TxMsgType rcvd = TxMsgType::TX_NONE;

    std::string f3 = msg.field3;
    for (auto& ch : f3) ch = toupper((unsigned char)ch);

    // Keywords first
    if (f3 == "RR73" || f3 == "RRR") {
        rcvd = TxMsgType::TX4;
    } else if (f3 == "73") {
        rcvd = TxMsgType::TX5;
    } else {
        // FD exchange shortcut
        std::string norm;
        if (ctx && parse_fd_exchange(msg.field3, norm) && !ctx->dxcall.empty()) {
            ctx->fd_rx_exchange = norm;

            std::string t = trim_copy(msg.field3);
            if (!t.empty() && (t[0] == 'R' || t[0] == 'r')) rcvd = TxMsgType::TX3;
            else rcvd = TxMsgType::TX2;

            ctx->rcvd_msg_type = rcvd;
            return rcvd;
        }

        // Normal FT8
        if (looks_like_grid(f3)) {
            rcvd = TxMsgType::TX1;
            if (ctx && ctx->dxgrid.empty()) ctx->dxgrid = f3;
        } else if (f3.size() > 2 && f3[0] == 'R' && f3[1] == ' ' &&
                   looks_like_grid(f3.substr(2))) {
            // Contest-style TX3 — "R <grid>" from WSJT-X grid-exchange modes
            // (NA VHF etc.): DX acknowledges our report with their grid
            // instead of a numeric report. It is a roger. snr_rx stays unset
            // so the ADIF omits rst_rcvd rather than inventing one.
            // RT260828: VE7MGU sent this seven times while we retried TX2
            // into the ground because this read as "unknown".
            rcvd = TxMsgType::TX3;
            if (ctx && ctx->dxgrid.empty()) ctx->dxgrid = f3.substr(2);
        } else if (!f3.empty() && f3[0] == 'R' && f3.size() > 1) {
            int rpt = 0;
            if (looks_like_report(f3.substr(1), rpt)) {
                rcvd = TxMsgType::TX3;
                if (ctx) ctx->snr_rx = rpt;
            }
        } else {
            int rpt = 0;
            if (looks_like_report(f3, rpt)) {
                rcvd = TxMsgType::TX2;
                if (ctx) ctx->snr_rx = rpt;
            }
        }
    }

    if (ctx) ctx->rcvd_msg_type = rcvd;
    return rcvd;
}

static void log_qso_if_needed(QsoContext* ctx) {
    if (!ctx || ctx->logged) return;

    // Cabrillo Field Day log (optional, independent of ADIF)
    if (ctx->is_fd && s_cabrillo_fd_callback &&
        !ctx->dxcall.empty() && !ctx->fd_rx_exchange.empty()) {
        s_cabrillo_fd_callback(ctx->dxcall, ctx->fd_rx_exchange);
    }

    if (!s_adif_callback) return;

    ctx->logged = true;
    s_adif_callback(ctx->dxcall, ctx->dxgrid, ctx->snr_tx, ctx->snr_rx);

    ESP_LOGI(TAG, "Logged QSO: %s grid=%s rst_sent=%d rst_rcvd=%d",
             ctx->dxcall.c_str(), ctx->dxgrid.c_str(), ctx->snr_tx, ctx->snr_rx);
}

static bool generate_response(QsoContext* ctx, const UiRxLine& msg, bool override) {
    // Get DX callsign from field2 (the sender), normalize to handle <> wrapped hashed calls
    std::string dxcall = normalize_call_token(msg.field2);
    if (dxcall.empty()) dxcall = normalize_call_token(msg.field1);

    // Set dxcall BEFORE parse_rcvd_msg so FD exchange check sees a non-empty dxcall
    if (override && ctx->dxcall.empty()) {
        ctx->dxcall = dxcall;
    }

    TxMsgType rcvd = parse_rcvd_msg(ctx, msg);

    ESP_LOGI(TAG, "generate_response: override=%d, rcvd=%d, ctx->state=%d",
             override, (int)rcvd, (int)ctx->state);

    if (rcvd == TxMsgType::TX_NONE) {
        // The state machine below is "locally total": every (state, rcvd)
        // pair refreshes next_tx to the state's canonical message. This early
        // return used to skip it, so a context that reactivate() brought back
        // with next_tx=TX_NONE stayed unschedulable at the head of the queue
        // and starved the beacon (RT260828, 18 minutes of silence). Keep the
        // early return, keep the invariant.
        if (ctx->next_tx == TxMsgType::TX_NONE) {
            ctx->next_tx = canonical_next_tx(ctx->state);
        }
        ESP_LOGW(TAG, "generate_response: rcvd=TX_NONE, returning false");
        return false;
    }

    // Latch rst_sent once at first TX1/TX2 evidence; preserve for full QSO.
    if ((rcvd == TxMsgType::TX1 || rcvd == TxMsgType::TX2) && ctx->snr_tx == -99) {
        ctx->snr_tx = msg.snr;
    }

    if (override) {
        ctx->dxcall = dxcall;
        ctx->offset_hz = msg.offset_hz;
        ctx->slot_id = msg.slot_id ^ 1;  // TX on opposite slot

        // Determine if this is a Field Day QSO
        {
            std::string f1 = msg.field1;
            for (auto& ch : f1) ch = toupper((unsigned char)ch);
            // Accept "CQ FD" in field1 (touch on CQ FD message)
            if (f1 == "CQ FD" || f1 == "CQFD" || (f1.rfind("CQ FD", 0) == 0)) {
                ctx->is_fd = true;
            }
            // Also mark as FD when we're in FD mode and received a valid FD exchange
            // (handles DX replying to our CQ FD — field1 is our callsign, not "CQ FD")
            else if (s_cq_type == AutoseqCqType::FD) {
                std::string norm;
                if (parse_fd_exchange(msg.field3, norm)) {
                    ctx->is_fd = true;
                }
            }
        }

        // Reset state based on received message type
        switch (rcvd) {
            case TxMsgType::TX1:
                set_state(ctx, AutoseqState::CALLING, TxMsgType::TX_NONE, 0);
                break;
            case TxMsgType::TX2:
                set_state(ctx, AutoseqState::REPLYING, TxMsgType::TX_NONE, 0);
                break;
            case TxMsgType::TX3:
                set_state(ctx, AutoseqState::REPORT, TxMsgType::TX_NONE, 0);
                break;
            case TxMsgType::TX4:
                set_state(ctx, AutoseqState::ROGER_REPORT, TxMsgType::TX_NONE, 0);
                break;
            case TxMsgType::TX5:
                set_state(ctx, AutoseqState::ROGERS, TxMsgType::TX_NONE, 0);
                break;
            default:
                break;
        }
    }

    // State machine transitions.
    //
    // INVARIANT: Every active ctx with state != IDLE must have next_tx != TX_NONE.
    // Each state handler explicitly handles every rcvd type. The "no-op" default
    // case (DX sent something that doesn't advance our state) refreshes next_tx
    // to the canonical value for the current state — i.e. "keep sending what we
    // were sending before." This makes generate_response a total function over
    // (state, rcvd) and repairs the invariant even if a caller (like reactivate)
    // passes in a ctx with a stale TX_NONE.
    switch (ctx->state) {
        case AutoseqState::CALLING:  // We sent CQ
            // CQ is short-lived by design: autoseq_start_cq sets next_tx=TX6,
            // we TX once, and tick() transitions CALLING→IDLE→pop. The beacon
            // re-enqueues a fresh CQ on the next cycle if still on.
            //
            // The default case here is unreachable in normal operation:
            //  - CALLING ctx always has dxcall="CQ" which never matches a DX
            //    sender, so on_decode never calls generate_response(false) on
            //    an existing CALLING ctx.
            //  - The override path sets CALLING only when rcvd=TX1, which
            //    immediately transitions via the TX1 case below.
            //  - CALLING ctxs never enter the inactive zone (tick pops them
            //    directly), so reactivation never produces CALLING.
            switch (rcvd) {
                case TxMsgType::TX1:
                    set_state(ctx, AutoseqState::REPORT, TxMsgType::TX2, s_max_retry);
                    return true;
                case TxMsgType::TX2:
                    set_state(ctx, AutoseqState::ROGER_REPORT, TxMsgType::TX3, s_max_retry);
                    return true;
                case TxMsgType::TX3:
                    // Advance to ROGERS; logging happens when TX4 is emitted.
                    set_state(ctx, AutoseqState::ROGERS, TxMsgType::TX4, s_max_retry);
                    return true;
                case TxMsgType::TX5:
                    // Receive 73 = terminal. Evict immediately. No log (never
                    // reached ROGERS/SIGNOFF; we never emitted TX4/TX5 ourselves).
                    set_state(ctx, AutoseqState::IDLE, TxMsgType::TX_NONE, 0);
                    return false;
                default:
                    return false;
            }

        case AutoseqState::REPLYING:  // We sent TX1 (grid)
            switch (rcvd) {
                case TxMsgType::TX2:
                    set_state(ctx, AutoseqState::ROGER_REPORT, TxMsgType::TX3, s_max_retry);
                    return true;
                case TxMsgType::TX3:
                    set_state(ctx, AutoseqState::ROGERS, TxMsgType::TX4, s_max_retry);
                    return true;
                case TxMsgType::TX4:
                    // DX sent RR73 skipping normal exchange. Advance to SIGNOFF;
                    // we'll emit TX5 (73) and log on that emission.
                    set_state(ctx, AutoseqState::SIGNOFF, TxMsgType::TX5, 0);
                    ctx->park_after_signoff_tx = true;
                    return true;
                case TxMsgType::TX5:
                    // Receive 73 = terminal. Evict immediately, no log.
                    set_state(ctx, AutoseqState::IDLE, TxMsgType::TX_NONE, 0);
                    return false;
                default:
                    // No-op (e.g. DX resent TX1): keep sending our grid
                    ctx->next_tx = TxMsgType::TX1;
                    return false;
            }

        case AutoseqState::REPORT:  // We sent TX2 (report)
            switch (rcvd) {
                case TxMsgType::TX2:
                    // DX sent their own report (no R prefix) — they either
                    // didn't copy our TX2 or changed from TX1 to TX2.
                    // Both sides have exchanged reports; advance to TX3.
                    set_state(ctx, AutoseqState::ROGER_REPORT, TxMsgType::TX3, s_max_retry);
                    return true;
                case TxMsgType::TX3:
                    set_state(ctx, AutoseqState::ROGERS, TxMsgType::TX4, s_max_retry);
                    return true;
                case TxMsgType::TX4:
                    set_state(ctx, AutoseqState::SIGNOFF, TxMsgType::TX5, 0);
                    ctx->park_after_signoff_tx = true;
                    return true;
                case TxMsgType::TX5:
                    // Receive 73 = terminal. Evict immediately, no log.
                    set_state(ctx, AutoseqState::IDLE, TxMsgType::TX_NONE, 0);
                    return false;
                default:
                    // No-op (e.g. DX resent TX1 grid): keep sending our report
                    ctx->next_tx = TxMsgType::TX2;
                    return false;
            }

        case AutoseqState::ROGER_REPORT:  // We sent TX3 (R+report)
            switch (rcvd) {
                case TxMsgType::TX4:
                    set_state(ctx, AutoseqState::SIGNOFF, TxMsgType::TX5, s_max_retry);
                    ctx->park_after_signoff_tx = true;
                    return true;
                case TxMsgType::TX5:
                    // Receive 73 = terminal. We never emitted TX4/TX5 → no log.
                    set_state(ctx, AutoseqState::IDLE, TxMsgType::TX_NONE, 0);
                    return false;
                default:
                    // No-op (DX resent TX1/TX2/TX3): keep sending R+report
                    ctx->next_tx = TxMsgType::TX3;
                    return false;
            }

        case AutoseqState::ROGERS:  // We sent TX4 (RR73) — already logged on emission
            switch (rcvd) {
                case TxMsgType::TX3:
                    // DX didn't get our RR73 — re-send with fresh retries.
                    set_state(ctx, AutoseqState::ROGERS, TxMsgType::TX4, s_max_retry);
                    return true;
                case TxMsgType::TX4:
                case TxMsgType::TX5:
                    // Receive RR73/73 after we sent RR73 = terminal.
                    // Already logged on TX4 emission; no extra action needed.
                    set_state(ctx, AutoseqState::IDLE, TxMsgType::TX_NONE, 0);
                    return false;
                default:
                    // No-op (DX resent TX1/TX2): keep sending RR73
                    ctx->next_tx = TxMsgType::TX4;
                    return false;
            }

        case AutoseqState::SIGNOFF:  // We sent TX5 (73) — already logged on emission
            switch (rcvd) {
                case TxMsgType::TX4:
                    // Late RR73: send another 73 and park inactive again.
                    set_state(ctx, AutoseqState::SIGNOFF, TxMsgType::TX5, 0);
                    ctx->park_after_signoff_tx = true;
                    return true;
                case TxMsgType::TX5:
                    // Receive 73 = terminal. Already logged.
                    set_state(ctx, AutoseqState::IDLE, TxMsgType::TX_NONE, 0);
                    ctx->park_after_signoff_tx = false;
                    return false;
                default:
                    // No-op (DX resent earlier message): keep sending 73
                    ctx->next_tx = TxMsgType::TX5;
                    return false;
            }

        default:
            break;
    }

    return false;
}

static void on_decode(const UiRxLine& msg) {
    // Check if it's addressed to us (normalize to handle <> wrapped hashed calls)
    std::string f1_norm = normalize_call_token(msg.field1);
    std::string my_norm = normalize_call_token(s_my_call);

    if (my_norm.empty() || f1_norm != my_norm) {
        return;
    }

    // Get DX call from field2 (normalize to handle <> wrapped hashed calls)
    std::string dxcall = normalize_call_token(msg.field2);
    if (dxcall.empty()) return;

    // Check active zone for matching context (case-insensitive)
    for (int i = 0; i < s_active_count; ++i) {
        QsoContext* ctx = &s_queue[i];
        std::string ctx_dxcall = ctx->dxcall;
        for (auto& ch : ctx_dxcall) ch = toupper((unsigned char)ch);
        if (ctx_dxcall == dxcall) {
            // Re-anchor slot_id to the current RX slot's opposite parity.
            // This preserves the queue-parity invariant across any edge cases
            // where parity might drift (band switch, external intervention).
            ctx->slot_id = msg.slot_id ^ 1;
            ESP_LOGI(TAG, "on_decode: found active ctx for %s, state=%d, next_tx=%d",
                     dxcall.c_str(), (int)ctx->state, (int)ctx->next_tx);
            generate_response(ctx, msg, false);
            ESP_LOGI(TAG, "on_decode: after response, state=%d, next_tx=%d",
                     (int)ctx->state, (int)ctx->next_tx);
            return;
        }
    }

    // Check inactive zone for matching context — reactivate if found
    int inact_idx = find_inactive_by_dxcall(dxcall);
    if (inact_idx >= 0) {
        // Only wake a parked context for a message the state machine can act
        // on. Reactivation resets the retry counter, so waking on noise would
        // let a station that never decodes us drive an unbounded retry storm
        // (exhaust -> park -> wake -> exhaust...). It stays parked, metadata
        // intact, until something parseable arrives. Probe on a copy:
        // parse_rcvd_msg writes grid/report fields into the ctx.
        QsoContext probe = s_queue[inact_idx];
        if (parse_rcvd_msg(&probe, msg) == TxMsgType::TX_NONE) {
            ESP_LOGW(TAG, "on_decode: %s is parked; '%s' is not actionable — leaving it parked",
                     dxcall.c_str(), msg.field3.c_str());
            return;
        }
        ESP_LOGI(TAG, "on_decode: reactivating inactive ctx for %s", dxcall.c_str());
        reactivate(inact_idx);
        // After reactivation, the entry is at s_queue[s_active_count - 1].
        // Re-anchor slot_id to match the current RX — the parked ctx's
        // slot_id may be stale if parity shifted while dormant.
        QsoContext* ctx = &s_queue[s_active_count - 1];
        ctx->slot_id = msg.slot_id ^ 1;
        generate_response(ctx, msg, false);
        sort_and_clean();
        return;
    }

    // No matching context (active or inactive) for this DX.
    // Reject messages that would produce wrong metadata if a fresh context
    // were created. TX3 (R+report) skips the snr_tx latch — that's the
    // reincarnation bug. Signoff messages (RR73/73) are also rejected.
    // TX1 (grid), TX2 (report), and FD exchanges are allowed — they set
    // snr_tx correctly via msg.snr in generate_response().
    std::string f3 = msg.field3;
    for (auto& ch : f3) ch = toupper((unsigned char)ch);
    if (f3 == "RR73" || f3 == "RRR" || f3 == "73") {
        ESP_LOGW(TAG, "on_decode: ignoring late signoff from %s (no ctx)",
                 dxcall.c_str());
        return;
    }
    // Reject R+report (TX3) from unknown DX — snr_tx would be -99
    if (!f3.empty() && f3[0] == 'R' && f3.size() > 1) {
        int rpt = 0;
        if (looks_like_report(f3.substr(1), rpt)) {
            ESP_LOGW(TAG, "on_decode: rejecting R+report from %s (no ctx, would lose snr_tx)",
                     dxcall.c_str());
            return;
        }
        // Contest-style "R <grid>" is TX3 too — same hazard.
        if (f3.size() > 2 && f3[1] == ' ' && looks_like_grid(f3.substr(2))) {
            ESP_LOGW(TAG, "on_decode: rejecting R+grid from %s (no ctx, would lose snr_tx)",
                     dxcall.c_str());
            return;
        }
    }

    ESP_LOGI(TAG, "on_decode: new QSO from %s (field3=%s, active=%d)",
             dxcall.c_str(), f3.c_str(), s_active_count);

    // No matching context — create new if there's room
    if (s_inactive_start <= s_active_count) {
        evict_oldest_inactive();
        if (s_inactive_start <= s_active_count) return;  // Still no room
    }

    QsoContext* ctx = append_ctx();
    generate_response(ctx, msg, true);
    ESP_LOGI(TAG, "on_decode: NEW ctx %s, state=%d, next_tx=%d",
             ctx->dxcall.c_str(), (int)ctx->state, (int)ctx->next_tx);
}

// Comparison for std::sort: IDLE at top (to be popped), CALLING at bottom
// Only used on the active zone [0 .. s_active_count)
// Returns true if left should come before right
static bool compare_ctx(const QsoContext& left, const QsoContext& right) {
    // IDLE always sorts first (will be popped by sort_and_clean).
    if (left.state == AutoseqState::IDLE && right.state != AutoseqState::IDLE) return true;
    if (right.state == AutoseqState::IDLE && left.state != AutoseqState::IDLE) return false;

    // Free Text is "highest priority non-IDLE" — when the user presses
    // Send Free Text, the next TX must be the FT, preempting any active
    // QSO. The QSO's ctx is preserved (FT is one-shot, popped after TX),
    // so the QSO resumes on the slot after FT fires.
    if (left.is_freetext != right.is_freetext) {
        return left.is_freetext;  // FT comes first
    }

    // Same state? Lower retry count gets priority — round-robin among
    // contacts so we probe for viable propagation paths rather than
    // burning all retries on one potentially dead contact.
    if (left.state == right.state) {
        return left.retry_counter < right.retry_counter;
    }

    // Higher state value wins (more advanced in QSO)
    // DESCENDING order: SIGNOFF(5) > ROGERS(4) > ROGER_REPORT(3) > ...
    // More advanced QSOs processed first
    return left.state > right.state;  // Higher state comes first
}

static void pop_front() {
    if (s_active_count <= 0) return;

    // Shift active zone left
    for (int i = 0; i + 1 < s_active_count; ++i) {
        s_queue[i] = s_queue[i + 1];
    }
    --s_active_count;
}

static QsoContext* append_ctx() {
    if (s_inactive_start <= s_active_count) return nullptr;  // No free space

    QsoContext* ctx = &s_queue[s_active_count++];
    *ctx = QsoContext{};  // Reset to defaults
    return ctx;
}

// Move active entry at index idx to the inactive zone (back of array)
static void move_to_inactive(int idx) {
    if (idx < 0 || idx >= s_active_count) return;

    // Make room in inactive zone if needed
    if (s_inactive_start <= s_active_count) {
        evict_oldest_inactive();
        // If still no room (queue is entirely active), just drop the entry
        if (s_inactive_start <= s_active_count) {
            // Remove from active by shifting, entry is lost
            for (int i = idx; i + 1 < s_active_count; ++i) {
                s_queue[i] = s_queue[i + 1];
            }
            --s_active_count;
            return;
        }
    }

    // Copy to inactive zone (grows leftward)
    QsoContext saved = s_queue[idx];
    saved.next_tx = TxMsgType::TX_NONE;
    saved.inactive_since_ms = mono_ms();

    // Remove from active zone by shifting
    for (int i = idx; i + 1 < s_active_count; ++i) {
        s_queue[i] = s_queue[i + 1];
    }
    --s_active_count;

    // Place into inactive zone
    s_queue[--s_inactive_start] = saved;
}

// Evict an inactive entry to free one slot.
// Evicts the oldest inactive entry by inactive_since_ms.
static void evict_oldest_inactive() {
    if (s_inactive_start >= AUTOSEQ_MAX_QUEUE) return;  // No inactive entries

    int oldest_idx = s_inactive_start;
    int64_t oldest_ts = s_queue[oldest_idx].inactive_since_ms;
    for (int i = s_inactive_start + 1; i < AUTOSEQ_MAX_QUEUE; ++i) {
        int64_t ts = s_queue[i].inactive_since_ms;
        // For equal timestamps, prefer rightmost entry as older.
        if (ts < oldest_ts || (ts == oldest_ts && i > oldest_idx)) {
            oldest_ts = ts;
            oldest_idx = i;
        }
    }

    // Remove oldest_idx from inactive zone and keep the zone contiguous.
    for (int i = oldest_idx; i > s_inactive_start; --i) {
        s_queue[i] = s_queue[i - 1];
    }
    ++s_inactive_start;
}

// Find an inactive entry by dxcall (case-insensitive). Returns array index or -1.
static int find_inactive_by_dxcall(const std::string& dxcall) {
    for (int i = s_inactive_start; i < AUTOSEQ_MAX_QUEUE; ++i) {
        std::string ctx_dxcall = s_queue[i].dxcall;
        for (auto& ch : ctx_dxcall) ch = toupper((unsigned char)ch);
        if (ctx_dxcall == dxcall) return i;
    }
    return -1;
}

// Move an inactive entry back to the active zone (appended at end of active zone).
// Always succeeds because removing the entry from inactive frees one slot for active.
static void reactivate(int inactive_idx) {
    if (inactive_idx < s_inactive_start || inactive_idx >= AUTOSEQ_MAX_QUEUE) return;

    QsoContext saved = s_queue[inactive_idx];

    // Remove from inactive zone by shifting remaining inactive entries right
    for (int i = inactive_idx; i > s_inactive_start; --i) {
        s_queue[i] = s_queue[i - 1];
    }
    ++s_inactive_start;  // Shrink inactive zone from left

    // Append to active zone — there's now room because we freed one slot
    s_queue[s_active_count++] = saved;
    // Reset retry counter for fresh attempts
    s_queue[s_active_count - 1].retry_counter = 0;
    s_queue[s_active_count - 1].inactive_since_ms = 0;
    ESP_LOGI(TAG, "Reactivated %s into active zone (active=%d)",
             saved.dxcall.c_str(), s_active_count);
}

static void sort_and_clean() {
    if (s_active_count == 0) return;

    // Sort only the active zone
    // Use std::sort instead of qsort - qsort does byte-wise swap which
    // corrupts std::string members in QsoContext
    std::sort(s_queue, s_queue + s_active_count, compare_ctx);

    // Pop IDLE entries from front
    while (s_active_count > 0 && s_queue[0].state == AutoseqState::IDLE) {
        pop_front();
    }
}

static bool looks_like_grid(const std::string& s) {
    if (s.size() != 4) return false;
    // Pattern: AA00 (letters, letters, digits, digits)
    return isalpha((unsigned char)s[0]) && isalpha((unsigned char)s[1]) &&
           isdigit((unsigned char)s[2]) && isdigit((unsigned char)s[3]);
}

static bool looks_like_report(const std::string& s, int& out) {
    if (s.empty()) return false;

    // Parse optional sign and digits
    size_t idx = 0;
    bool neg = false;

    if (s[idx] == '+') {
        idx++;
    } else if (s[idx] == '-') {
        neg = true;
        idx++;
    }

    if (idx >= s.size() || !isdigit((unsigned char)s[idx])) return false;

    int val = 0;
    while (idx < s.size() && isdigit((unsigned char)s[idx])) {
        val = val * 10 + (s[idx] - '0');
        idx++;
    }

    // Must consume entire string
    if (idx != s.size()) return false;

    // Valid FT8 report range: -30 to +30
    if (val > 30) return false;

    out = neg ? -val : val;
    return true;
}

// Normalize a callsign token: strip <> wrappers used for hashed nonstd calls
// and convert to uppercase
static std::string normalize_call_token(const std::string& s) {
    std::string out = s;
    // Trim <> wrappers used for hashed nonstd calls
    if (!out.empty() && out.front() == '<') out.erase(out.begin());
    if (!out.empty() && out.back()  == '>') out.pop_back();
    // Convert to uppercase
    for (auto& ch : out) ch = (char)toupper((unsigned char)ch);
    return out;
}
