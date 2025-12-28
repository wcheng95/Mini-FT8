#pragma once
#include <string>
#include <vector>

struct ActiveCall {
    std::string dxcall;
    std::string dxgrid;
    int rx_snr = 0;     // received report from DX (field3)
    int rpt_snr = 0;    // report we send to DX (frozen per QSO)
    int offset_hz = 0;  // latest RX offset
    int slot_id = 0;    // DX TX slot (0 even / 1 odd)
    int sequence = 0;   // 0..5 corresponds to tx6-1-2-3-4-5
    bool signoff = false;
    int age = 0;        // slots since last update
};

void active_calls_init();
void active_calls_tick(); // increment ages and evict stale
ActiveCall* active_calls_touch(const std::string& dxcall,
                               const std::string& dxgrid,
                               int rx_snr,
                               int rpt_snr,
                               int offset_hz,
                               int slot_id,
                               int sequence,
                               bool signoff);
ActiveCall* active_calls_find(const std::string& dxcall);
const std::vector<ActiveCall>& active_calls_list();
