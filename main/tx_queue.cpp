#include "tx_queue.h"

TxEntry tx_next;
std::vector<TxEntry> tx_queue;

void tx_init() {
    tx_queue.clear();
    tx_next.text.clear();
    tx_next.mark_delete = false;
}

void tx_enqueue(const std::string& msg) {
    if (tx_queue.size() >= 10) return;
    tx_queue.push_back({msg, false});
}

void tx_commit_deletions() {
    std::vector<TxEntry> filtered;
    filtered.reserve(tx_queue.size());
    for (auto& e : tx_queue) {
        if (!e.mark_delete) filtered.push_back(e);
    }
    tx_queue.swap(filtered);
}
