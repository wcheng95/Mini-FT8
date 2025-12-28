#include "active_calls.h"
#include <algorithm>
#include "esp_log.h"

static constexpr size_t kMaxActive = 32;
// Evict after 80 slots (~20 minutes)
static constexpr int kMaxAge = 80;

static std::vector<ActiveCall> g_calls;
static const char* TAG = "ACTIVE";

void active_calls_init() {
    g_calls.clear();
}

// Linear scan is fine for 32 entries; clearer than maintaining a heap with
// frequent updates from the decoder and slot ticks.
static size_t find_oldest_index() {
    size_t oldest = 0;
    int max_age = -1;
    for (size_t i = 0; i < g_calls.size(); ++i) {
        if (g_calls[i].age > max_age) {
            max_age = g_calls[i].age;
            oldest = i;
        }
    }
    return oldest;
}

void active_calls_tick() {
    for (auto& c : g_calls) {
        c.age++;
    }
    // Evict stale entries
    g_calls.erase(std::remove_if(g_calls.begin(), g_calls.end(),
        [](const ActiveCall& c){ return c.age >= kMaxAge; }),
        g_calls.end());
}

ActiveCall* active_calls_find(const std::string& dxcall) {
    for (auto& c : g_calls) {
        if (c.dxcall == dxcall) return &c;
    }
    return nullptr;
}

ActiveCall* active_calls_touch(const std::string& dxcall,
                               const std::string& dxgrid,
                               int rx_snr,
                               int rpt_snr,
                               int offset_hz,
                               int slot_id,
                               int sequence,
                               bool signoff) {
    ActiveCall* c = active_calls_find(dxcall);
    bool is_new = false;
    if (!c) {
        if (g_calls.size() >= kMaxActive) {
            size_t idx = find_oldest_index();
            g_calls[idx] = ActiveCall{};
            c = &g_calls[idx];
        } else {
            g_calls.push_back(ActiveCall{});
            c = &g_calls.back();
        }
        c->dxcall = dxcall;
        c->rpt_snr = rpt_snr;
        is_new = true;
    }
    c->dxgrid = dxgrid;
    c->rx_snr = rx_snr;
    c->offset_hz = offset_hz;
    c->slot_id = slot_id;
    c->sequence = sequence;
    c->signoff = signoff ? true : c->signoff;
    c->age = 0; // reset on update

    // Debug print concise state
    ESP_LOGI(TAG, "%s: grid=%s rx_snr=%d rpt_snr=%d off=%d slot=%d seq=%d signoff=%d %s",
             dxcall.c_str(), dxgrid.c_str(), rx_snr, c->rpt_snr,
             offset_hz, slot_id, sequence, c->signoff ? 1 : 0,
             is_new ? "(new)" : "(upd)");

    return c;
}

const std::vector<ActiveCall>& active_calls_list() {
    return g_calls;
}
