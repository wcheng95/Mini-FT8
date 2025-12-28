#include "tx_queue.h"
#include <algorithm>
#include "esp_log.h"

TxEntry tx_next;
std::vector<TxEntry> tx_queue;

void tx_init() {
    tx_queue.clear();
    tx_next.text.clear();
    tx_next.mark_delete = false;
}

void tx_enqueue(const TxEntry& entry) {
    if (tx_queue.size() >= 10) return;
    tx_queue.push_back(entry);
}

void tx_commit_deletions() {
    std::vector<TxEntry> filtered;
    filtered.reserve(tx_queue.size());
    for (auto& e : tx_queue) {
        if (!e.mark_delete) filtered.push_back(e);
    }
    tx_queue.swap(filtered);
}

std::string tx_entry_display(const TxEntry& e, bool for_queue) {
    std::string base;
    if (e.dxcall == "FreeText") {
        base = !e.text.empty() ? e.text : e.field3;
    } else {
        base = e.dxcall;
        if (!e.field3.empty()) {
            base += " ";
            base += e.field3;
        }
    }
    // Append repeat counter on queue lines for normal entries
    if (for_queue && e.dxcall != "FreeText" && e.repeat_counter > 0) {
        base += " [";
        base += std::to_string(e.repeat_counter);
        base += "]";
    }
    return base;
}

// ---------------- TX Engine placeholder ----------------
// Temporarily unhooked; scheduling/transmit handled elsewhere.
