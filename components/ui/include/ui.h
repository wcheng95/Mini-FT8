#pragma once
#include <vector>
#include <string>
#include <stdint.h>

// A lightweight RX line format you can fill from your decoder
struct UiRxLine {
    std::string text;  // already formatted for display
    int snr = 0;
    bool is_cq = false;
    bool is_to_me = false;
};

void ui_init();
void ui_set_waterfall_row(int row, const uint8_t* bins, int len);
// Push a new row into the waterfall ring buffer (advances head) and redraws it.
void ui_push_waterfall_row(const uint8_t* bins, int len);
void ui_draw_waterfall();
void ui_draw_countdown(float fraction, bool even_slot);  // 0.0-1.0 fill of the countdown bar
void ui_set_rx_list(const std::vector<UiRxLine>& lines);
void ui_draw_rx(int flash_index = -1);
void ui_force_redraw_rx();
void ui_draw_tx(const std::string& next, const std::vector<std::string>& queue, int page, int selected, const std::vector<bool>& mark_delete);
// Returns selected absolute index or -1 if none
int ui_handle_rx_key(char c);
// Generic list draw (6 lines per page)
void ui_draw_list(const std::vector<std::string>& lines, int page, int highlight_abs = -1);
void ui_draw_debug(const std::vector<std::string>& lines, int page);
