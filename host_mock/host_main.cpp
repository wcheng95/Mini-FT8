/*
 * host_main.cpp — Mini-FT8 autoseq host-mock test harness
 *
 * Loads a JSON scenario, drives autoseq through CQ/QSO cycles,
 * and prints state transitions for verification.
 *
 * Usage:  ./host_test <test_file.json>
 */

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "host_mocks.h"
#include "json_parser.h"
#include "autoseq.h"

// ---------- Beacon mode (mirrors main.cpp enum) ----------
enum class BeaconMode { OFF = 0, EVEN, ODD };

// ---------- Globals (subset of main.cpp) ----------
static BeaconMode g_beacon = BeaconMode::OFF;
static int g_violations = 0;

// ---------- ADIF callback ----------
static void adif_callback(const std::string& dxcall, const std::string& dxgrid,
                           int rst_sent, int rst_rcvd) {
    printf(">>> ADIF log: %s %s rst_sent=%+d rst_rcvd=%+d\n",
           dxcall.c_str(), dxgrid.c_str(), rst_sent, rst_rcvd);
}

// ---------- Cabrillo FD callback ----------
static void cabrillo_fd_callback(const std::string& dxcall, const std::string& their_exchange) {
    printf(">>> Cabrillo FD log: %s exchange=%s\n",
           dxcall.c_str(), their_exchange.c_str());
}

// ---------- Helpers ----------

static const char* beacon_mode_str(BeaconMode m) {
    switch (m) {
        case BeaconMode::OFF:  return "OFF";
        case BeaconMode::EVEN: return "EVEN";
        case BeaconMode::ODD:  return "ODD";
    }
    return "?";
}

static void print_separator(int period) {
    printf("\n========== Period %d ==========\n", period);
}

// ---------- Main ----------

int main(int argc, char* argv[]) {
    const char* test_file = (argc > 1) ? argv[1] : "test_qso.json";

    TestData td;
    if (!load_test_data(test_file, td)) {
        fprintf(stderr, "Failed to load test data from '%s'\n", test_file);
        return 1;
    }

    // Initialise autoseq
    autoseq_init();
    autoseq_set_station(td.config.my_callsign, td.config.my_grid);
    autoseq_set_adif_callback(adif_callback);
    autoseq_set_cabrillo_fd_callback(cabrillo_fd_callback);
    autoseq_set_max_retry(td.config.max_retry);

    // Configure CQ type from test config
    if (!td.config.cq_type.empty()) {
        AutoseqCqType cq = AutoseqCqType::CQ;
        if (td.config.cq_type == "FD")       cq = AutoseqCqType::FD;
        else if (td.config.cq_type == "SOTA") cq = AutoseqCqType::SOTA;
        else if (td.config.cq_type == "POTA") cq = AutoseqCqType::POTA;
        else if (td.config.cq_type == "QRP")  cq = AutoseqCqType::QRP;
        else if (td.config.cq_type == "FREETEXT") cq = AutoseqCqType::FREETEXT;
        autoseq_set_cq_type(cq, td.config.free_text);
    }

    printf("Station: %s / %s\n", td.config.my_callsign.c_str(),
           td.config.my_grid.c_str());
    printf("TX_ON_EVEN: %s\n", td.config.tx_on_even ? "true" : "false");
    if (!td.config.cq_type.empty()) {
        printf("CQ_TYPE: %s  FREE_TEXT: %s\n",
               td.config.cq_type.c_str(), td.config.free_text.c_str());
    }
    printf("Periods: %d\n\n", (int)td.periods.size());

    // Determine which parity we transmit on
    // tx_on_even=true  -> we TX on even slots (parity 0)
    // tx_on_even=false -> we TX on odd slots  (parity 1)
    int my_tx_parity = td.config.tx_on_even ? 0 : 1;
    int my_rx_parity = my_tx_parity ^ 1;  // RX on opposite parity

    // Each period is 15 seconds.  Two periods = one 30-second FT8 cycle.
    // Period 0,2,4,... are even slots (parity 0)
    // Period 1,3,5,... are odd  slots (parity 1)
    int64_t slot_idx = 0;

    for (int p = 0; p < (int)td.periods.size(); ++p, ++slot_idx) {
        print_separator(p);

        int slot_parity = p & 1;
        int64_t period_base_ms = (int64_t)p * 15000;
        mock_clock_set(period_base_ms);

        TestPeriod& tp = td.periods[p];

        // --- Beacon change (at specified time offset within this period) ---
        if (tp.has_beacon_change) {
            BeaconMode old_beacon = g_beacon;
            if (tp.beacon_change.beacon_on == 0) {
                g_beacon = BeaconMode::OFF;
            } else if (tp.beacon_change.beacon_on == 1) {
                g_beacon = BeaconMode::EVEN;
            } else {
                g_beacon = BeaconMode::ODD;
            }
            printf("[Period %d @ %.1fs] Beacon: %s -> %s\n",
                   p, tp.beacon_change.time_offset,
                   beacon_mode_str(old_beacon), beacon_mode_str(g_beacon));
        }

        // --- RX phase: inject decoded messages ---
        // Filter to only messages addressed to us (is_to_me)
        std::vector<UiRxLine> to_me;
        for (auto& rx : tp.messages) {
            // Assign slot_id based on the period parity (RX messages arrive on
            // the slot opposite to where we TX; but the message's slot_id tells
            // autoseq what parity the *sender* was on so it can TX on the
            // opposite).  If the JSON specifies slot explicitly, honour it;
            // otherwise default to this period's parity.
            if (rx.slot_id == 0 && slot_parity != 0) {
                // JSON didn't set slot — use period parity
                rx.slot_id = slot_parity;
            }

            printf("[Period %d] RX: %s %s %s  snr=%d slot=%d\n",
                   p, rx.field1.c_str(), rx.field2.c_str(), rx.field3.c_str(),
                   rx.snr, rx.slot_id);

            if (rx.is_to_me) {
                to_me.push_back(rx);
            }
        }

        if (!to_me.empty()) {
            autoseq_on_decodes(to_me);
        }

        // --- Touch event ---
        if (tp.has_touch_event) {
            int idx = tp.touch_event.message_index;
            if (idx >= 0 && idx < (int)tp.messages.size()) {
                printf("[Period %d @ %.1fs] Touch: message %d (%s)\n",
                       p, tp.touch_event.time_offset, idx,
                       tp.messages[idx].text.c_str());
                autoseq_on_touch(tp.messages[idx]);
            }
        }

        // --- Free Text event ---
        if (tp.has_freetext_event) {
            int fallback_parity = (int)((slot_idx + 1) & 1);
            bool ok = autoseq_schedule_freetext(tp.freetext_event.text, fallback_parity);
            printf("[Period %d @ %.1fs] FreeText: %s (scheduled=%d)\n",
                   p, tp.freetext_event.time_offset,
                   tp.freetext_event.text.c_str(), ok ? 1 : 0);
        }

        // --- TX phase ---
        // Mirrors main.cpp decode_monitor_results + check_slot_boundary:
        // ARMING can happen in any period (main.cpp enqueues a beacon CQ at
        // every no-pending decode boundary, parity taken from g_beacon);
        // FIRING is parity-gated, like check_slot_boundary. Keep this block
        // in sync with main.cpp — the old harness derived beacon parity from
        // the test config instead of g_beacon and fired without the parity
        // gate, which is exactly why the stale-parity beacon bug (RT260828)
        // never showed up here.
        AutoseqTxEntry pending;
        bool has_tx = autoseq_fetch_pending_tx(pending);

        if (g_beacon != BeaconMode::OFF) {
            int target = (g_beacon == BeaconMode::EVEN) ? 0 : 1;
            if (!has_tx) {
                autoseq_start_cq(target);                    // enqueue fresh
                has_tx = autoseq_fetch_pending_tx(pending);
            } else if (pending.dxcall == "CQ") {
                autoseq_start_cq(target);                    // retarget stale parity
                has_tx = autoseq_fetch_pending_tx(pending);  // re-fetch fresh
            }
        } else if (has_tx && pending.dxcall == "CQ") {
            autoseq_cancel_cq();                             // beacon off — drop it
            has_tx = autoseq_fetch_pending_tx(pending);
        }

        bool held = false;
        if (has_tx && (pending.slot_id & 1) != slot_parity) {
            // Armed for the other parity — check_slot_boundary would not
            // fire it in this slot. Hold it for the matching period.
            printf("[Period %d] (armed '%s' for parity %d — holding)\n",
                   p, pending.text.c_str(), pending.slot_id & 1);
            has_tx = false;
            held = true;
        }

        if (has_tx) {
            printf("[Period %d] TX: %s  (slot=%d offset=%dHz)\n",
                   p, pending.text.c_str(), pending.slot_id, pending.offset_hz);

            // Invariant: a beacon CQ must fire on the parity the operator
            // configured, and never while the beacon is off.
            if (pending.dxcall == "CQ") {
                const int fired = pending.slot_id & 1;
                const bool ok =
                    (g_beacon == BeaconMode::EVEN && fired == 0) ||
                    (g_beacon == BeaconMode::ODD  && fired == 1);
                if (!ok) {
                    printf("*** VIOLATION: beacon CQ fired on parity %d with beacon=%s\n",
                           fired, beacon_mode_str(g_beacon));
                    ++g_violations;
                }
            }

            // Notify autoseq — this is where ADIF logging fires for TX4/TX5
            autoseq_on_tx_starting();

            // Simulate transmission
            autoseq_mark_sent(slot_idx);

            // Post-TX tick for retry management
            int ms_to_boundary = 15000 - (int)(mock_clock_get() - period_base_ms);
            autoseq_tick(slot_idx, slot_parity, ms_to_boundary);
        } else if (!held) {
            printf("[Period %d] (no TX)\n", p);
        }

        // Invariant: with the beacon on, its own parity never goes silent.
        // Either a QSO transmits in this slot or the beacon CQ does; a held
        // entry (armed for the other parity) is the only excuse.
        if (g_beacon != BeaconMode::OFF && !has_tx && !held) {
            const int bp = (g_beacon == BeaconMode::EVEN) ? 0 : 1;
            if (slot_parity == bp) {
                printf("*** VIOLATION: beacon=%s but nothing transmitted on its parity %d\n",
                       beacon_mode_str(g_beacon), bp);
                ++g_violations;
            }
        }

        // Invariant: an active QSO at the head of the queue is always
        // schedulable (next_tx != TX_NONE). One-shots (CALLING) legitimately
        // carry TX_NONE as their evict-after-tick marker.
        {
            QsoContext head;
            if (autoseq_get_active_context(0, &head) &&
                head.state != AutoseqState::IDLE &&
                head.state != AutoseqState::CALLING &&
                head.next_tx == TxMsgType::TX_NONE) {
                printf("*** VIOLATION: dead head — %s at queue[0] in state %d with next_tx=TX_NONE\n",
                       head.dxcall.c_str(), (int)head.state);
                ++g_violations;
            }
        }

        // --- Queue snapshot, same lines the firmware writes to the RxTx log ---
        std::vector<std::string> qlines;
        autoseq_get_queue_log_lines(qlines);
        for (auto& q : qlines) printf("[Period %d] Q %s\n", p, q.c_str());
    }

    if (g_violations) {
        printf("\n========== Test complete: %d VIOLATION%s ==========\n",
               g_violations, g_violations == 1 ? "" : "S");
        return 1;
    }
    printf("\n========== Test complete: no violations ==========\n");
    return 0;
}
