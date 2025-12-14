#pragma once
#include <vector>
#include <string>

struct TxEntry {
    std::string text;
    bool mark_delete = false;
};

// Shared TX state
extern TxEntry tx_next;
extern std::vector<TxEntry> tx_queue;  // max size 10

void tx_init();
void tx_enqueue(const std::string& msg);
void tx_commit_deletions();
