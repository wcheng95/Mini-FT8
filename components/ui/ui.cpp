#include "ui.h"
#include <M5Unified.h>
#include <M5Cardputer.h>
#include <cstring>
#include "freertos/semphr.h"

static constexpr int SCREEN_W = 240;
static constexpr int SCREEN_H = 135;
// Layout: 18px waterfall, 3px countdown bar, 6 lines: each 16px text + 3px gap
static constexpr int WATERFALL_H = 18;
static constexpr int COUNTDOWN_H = 3;
static constexpr int RX_LINES = 6;

static uint8_t waterfall[WATERFALL_H][SCREEN_W];
static int waterfall_head = 0;

static std::vector<UiRxLine> rx_lines;
static int rx_page = 0;
static int rx_selected = -1;  // global index into rx_lines
static std::vector<UiRxLine> last_drawn_lines;
static int last_page = -1;

static SemaphoreHandle_t g_disp_mutex = nullptr;

static void disp_lock() {
    if (g_disp_mutex) {
        xSemaphoreTake(g_disp_mutex, portMAX_DELAY);
    }
}

static void disp_unlock() {
    if (g_disp_mutex) {
        xSemaphoreGive(g_disp_mutex);
    }
}

struct DispGuard {
    DispGuard() { disp_lock(); }
    ~DispGuard() { disp_unlock(); }
};

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Draw TX view: line 1 = next, lines 2-6 from queue/page
void ui_draw_tx(const std::string& next, const std::vector<std::string>& queue, int page, int selected, const std::vector<bool>& mark_delete) {
    const int line_h = 19; // 16 text + 3 gap
    const int start_y = WATERFALL_H + COUNTDOWN_H + 3;

    DispGuard guard;
    M5.Display.startWrite();
    M5.Display.setTextSize(2);
    // Line 1: next (always)
    M5.Display.fillRect(0, start_y, SCREEN_W, line_h, TFT_BLACK);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setCursor(0, start_y);
    M5.Display.printf("1 %s", next.c_str());

    // Lines 2-6: queue items based on page
    int start_idx = page * 5; // show up to 5 items after next
    for (int i = 0; i < 5; ++i) {
        int idx = start_idx + i;
        int y = start_y + (i + 1) * line_h;
        M5.Display.fillRect(0, y, SCREEN_W, line_h, TFT_BLACK);
        if (idx < (int)queue.size()) {
            bool del = (idx < (int)mark_delete.size() && mark_delete[idx]);
            bool sel = (idx == selected);
            uint16_t bg = sel ? rgb565(30, 30, 60) : (del ? rgb565(60, 30, 30) : TFT_BLACK);
            M5.Display.fillRect(0, y, SCREEN_W, line_h, bg);
            M5.Display.setTextColor(TFT_WHITE, bg);
            M5.Display.setCursor(0, y);
            M5.Display.printf("%d %s", i + 2, queue[idx].c_str());
        }
    }
    M5.Display.endWrite();
}

void ui_init() {
    g_disp_mutex = xSemaphoreCreateMutex();
    auto cfg = M5.config();
    cfg.output_power = true;
    cfg.external_rtc = false;
    M5Cardputer.begin(cfg, true);
    M5.Display.setRotation(1);
    M5.Display.fillScreen(TFT_BLACK);
    ui_draw_countdown(0.0f, true);
}

void ui_set_waterfall_row(int row, const uint8_t* bins, int len) {
    if (len > SCREEN_W) len = SCREEN_W;
    if (row < 0 || row >= WATERFALL_H) return;
    memcpy(waterfall[row], bins, len);
}

void ui_push_waterfall_row(const uint8_t* bins, int len) {
    if (len > SCREEN_W) len = SCREEN_W;
    memcpy(waterfall[waterfall_head], bins, len);
    if (len < SCREEN_W) {
        memset(waterfall[waterfall_head] + len, 0, SCREEN_W - len);
    }
    waterfall_head = (waterfall_head + 1) % WATERFALL_H;
    ui_draw_waterfall();
}

void ui_draw_waterfall() {
    DispGuard guard;
    int dst_y = 0;
    for (int i = 0; i < WATERFALL_H; ++i) {
        int src = (waterfall_head + i) % WATERFALL_H;
        for (int x = 0; x < SCREEN_W; ++x) {
            uint8_t v = waterfall[src][x];
            // Yellow gradient on black background
            uint8_t r = v;
            uint8_t g = v;
            uint8_t b = 0;
            uint16_t c = rgb565(r, g, b);
            M5.Display.drawPixel(x, dst_y + i, c);
        }
    }
}

void ui_draw_countdown(float fraction, bool even_slot) {
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;
    int filled = (int)(fraction * SCREEN_W);
    int y = WATERFALL_H;
    // Draw a faint background to make the bar visible even at 0%
    DispGuard guard;
    M5.Display.fillRect(0, y, SCREEN_W, COUNTDOWN_H, rgb565(20, 20, 40));
    if (filled > 0) {
        uint16_t color = even_slot ? rgb565(0, 180, 0) : rgb565(180, 0, 0);
        M5.Display.fillRect(0, y, filled, COUNTDOWN_H, color);
    }
}

void ui_set_rx_list(const std::vector<UiRxLine>& lines) {
    rx_lines = lines;
    rx_page = 0;       // reset to first page
    rx_selected = -1;  // clear selection
    last_drawn_lines.clear();
    last_page = -1;
}

void ui_force_redraw_rx() {
    last_drawn_lines.clear();
    last_page = -1;
}

static void draw_rx_line(int y, const UiRxLine& l, int line_no, bool selected) {
    uint16_t color = TFT_WHITE;
    if (l.is_to_me) color = rgb565(255, 0, 0);
    else if (l.is_cq) color = rgb565(0, 220, 0);
    // Sticky line number in first column
    uint16_t bg = selected ? rgb565(30, 30, 60) : TFT_BLACK;
    M5.Display.fillRect(0, y, SCREEN_W, 16, bg);  // clear text band; gap handled by line_h
    M5.Display.setTextColor(TFT_WHITE, bg);
    M5.Display.setCursor(0, y);
    M5.Display.printf("%d ", line_no);
    M5.Display.setTextColor(color, bg);
    M5.Display.printf("%s", l.text.c_str());
}

void ui_draw_rx(int flash_index) {
    const int line_h = 19; // 16 text + 3 gap
    // Add a 3px gap below the countdown before the first line
    const int start_y = WATERFALL_H + COUNTDOWN_H + 3;
    // Only redraw when page changes or content changes, but always draw if list is empty
    if (!(rx_lines.empty()) && flash_index < 0) {
        if (rx_page == last_page && last_drawn_lines.size() == rx_lines.size()) {
            bool same = true;
            for (size_t i = 0; i < rx_lines.size(); ++i) {
                if (rx_lines[i].text != last_drawn_lines[i].text ||
                    rx_lines[i].is_cq != last_drawn_lines[i].is_cq ||
                    rx_lines[i].is_to_me != last_drawn_lines[i].is_to_me) {
                    same = false;
                    break;
                }
            }
            if (same) return;
        }
    }

    DispGuard guard;
    M5.Display.startWrite();
    M5.Display.setTextSize(2);
    int start = rx_page * RX_LINES;
    for (int i = 0; i < RX_LINES; ++i) {
        int idx = start + i;
        int y = start_y + i * line_h;
        M5.Display.fillRect(0, y, SCREEN_W, line_h, TFT_BLACK);
        if (idx < (int)rx_lines.size()) {
            bool selected = (idx == flash_index);
            draw_rx_line(y, rx_lines[idx], i + 1, selected);
        }
    }
    M5.Display.endWrite();

    // cache drawn content
    if (flash_index < 0) {
        last_page = rx_page;
        last_drawn_lines = rx_lines;
    } else {
        last_page = -1;
        last_drawn_lines.clear();
    }
}

// Simple keyboard: dot/‘.’ scroll forward page, comma/‘,’ scroll back.
int ui_handle_rx_key(char c) {
    int selected_idx = -1;
    if (c == 0) return selected_idx;
    if (c == ';') {
        if (rx_page > 0) {
            rx_page--;
            ui_draw_rx();
        }
    } else if (c == '.') {
        if ((rx_page + 1) * RX_LINES < (int)rx_lines.size()) {
            rx_page++;
            ui_draw_rx();
        }
    } else if (c >= '1' && c <= '6') {
        int line = c - '1';
        int idx = rx_page * RX_LINES + line;
        if (idx >= 0 && idx < (int)rx_lines.size()) {
            rx_selected = idx;
            ui_draw_rx();
            selected_idx = idx;
        }
    }
    return selected_idx;
}

// Simple numbered list drawing helper (6 lines/page), optional highlight by absolute index
void ui_draw_list(const std::vector<std::string>& lines, int page, int highlight_abs) {
    const int line_h = 19; // 16 text + 3 gap
    const int start_y = WATERFALL_H + COUNTDOWN_H + 3;
    DispGuard guard;
    M5.Display.startWrite();
    M5.Display.setTextSize(2);
    for (int i = 0; i < RX_LINES; ++i) {
        int idx = page * RX_LINES + i;
        int y = start_y + i * line_h;
        uint16_t bg = (idx == highlight_abs) ? rgb565(30, 30, 60) : TFT_BLACK;
        M5.Display.fillRect(0, y, SCREEN_W, line_h, bg);
        if (idx < (int)lines.size()) {
            M5.Display.setTextColor(TFT_WHITE, bg);
            M5.Display.setCursor(0, y);
            M5.Display.printf("%d %s", i + 1, lines[idx].c_str());
        }
    }
    M5.Display.endWrite();
}

void ui_draw_debug(const std::vector<std::string>& lines, int page) {
    const int line_h = 19;
    const int start_y = WATERFALL_H + COUNTDOWN_H + 3;
    DispGuard guard;
    M5.Display.startWrite();
    M5.Display.setTextSize(2);
    for (int i = 0; i < RX_LINES; ++i) {
        int idx = page * RX_LINES + i;
        int y = start_y + i * line_h;
        M5.Display.fillRect(0, y, SCREEN_W, line_h, TFT_BLACK);
        if (idx < (int)lines.size()) {
            M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
            M5.Display.setCursor(0, y);
            M5.Display.printf("%s", lines[idx].c_str());
        }
    }
    M5.Display.endWrite();
}
