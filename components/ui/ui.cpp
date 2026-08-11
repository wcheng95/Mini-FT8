#include "ui.h"
#include <M5Unified.h>
#include <M5Cardputer.h>
#include <cstring>
#include "freertos/semphr.h"
#include "esp_heap_caps.h"

static bool ui_paused = false;      // waterfall updates paused

static uint8_t waterfall[WATERFALL_H][SCREEN_W];
static int waterfall_head = 0;
static bool waterfall_dirty = false;

// Off-screen RGB565 frame buffer for bulk-blit waterfall rendering.
// Replaces 4,320 individual drawPixel() SPI calls (~43 ms) with a single
// pushImage() burst (~1 ms), keeping the main loop under its 2 ms TX budget.
// 240 x 18 x 2 = 8,640 bytes allocated from heap in ui_init().
static uint16_t* wf_fb = nullptr;

// Static RX list — zero-heap display pipeline
static RxDecodeEntry rx_lines[RX_MAX_DECODES];
static int rx_lines_count = 0;
static int rx_page = 0;
static int rx_selected = -1;  // global index into rx_lines

static std::string fit_text_width(std::string text, int max_width) {
    if (M5.Display.textWidth(text.c_str()) <= max_width) return text;
    while (!text.empty() && M5.Display.textWidth((text + ">").c_str()) > max_width) {
        text.pop_back();
    }
    if (!text.empty()) text.push_back('>');
    return text;
}

struct RxDrawCacheEntry {
    char text[RX_TEXT_MAX];
    bool is_cq;
    bool is_to_me;
    bool valid;
};
static RxDrawCacheEntry last_drawn_cache[RX_MAX_DECODES];
static int last_drawn_count = 0;
static int last_page = -1;
static std::string g_visible_rows[RX_LINES];

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

void ui_set_paused(bool paused) { ui_paused = paused; }
bool ui_is_paused() { return ui_paused; }

// When true, ui_push_waterfall_row() from the RX audio pipeline is silenced.
// ui_push_tx_waterfall_row() bypasses this flag so TX tone markers always appear.
static bool s_rx_waterfall_muted = false;
void ui_set_rx_waterfall_muted(bool muted) { s_rx_waterfall_muted = muted; }

bool ui_waterfall_dirty() { return waterfall_dirty; }
void ui_draw_waterfall_if_dirty() { if (waterfall_dirty) ui_draw_waterfall(); }

// Draw TX view: line 1 = next, lines 2-6 from queue/page
// slot_colors: optional per-line slot parity (0 even, 1 odd) for coloring text
void ui_draw_tx(const std::string& next, const std::vector<std::string>& queue, int page, int selected, const std::vector<bool>& mark_delete, const std::vector<int>& slot_colors) {
    const int line_h = 19; // 16 text + 3 gap
    const int start_y = UI_START_Y;

    DispGuard guard;
    M5.Display.startWrite();
    M5.Display.setTextSize(2);
    // Line 1: next (always)
    M5.Display.fillRect(0, start_y, SCREEN_W, line_h, TFT_BLACK);
    uint16_t next_color = TFT_WHITE;
    if (slot_colors.size() >= 1) {
        next_color = (slot_colors[0] & 1) ? TFT_RED : TFT_GREEN;
    }
    M5.Display.setTextColor(next_color, TFT_BLACK);
    M5.Display.setCursor(0, start_y);
    M5.Display.printf("1 %s", next.c_str());
    g_visible_rows[0] = std::string("1 ") + next;

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
            uint16_t fg = TFT_WHITE;
            if (slot_colors.size() > (size_t)(idx + 1)) {
                fg = (slot_colors[idx + 1] & 1) ? TFT_RED : TFT_GREEN;
            }
            M5.Display.setTextColor(fg, bg);
            M5.Display.setCursor(0, y);
            M5.Display.printf("%d %s", i + 2, queue[idx].c_str());
            g_visible_rows[i + 1] = std::to_string(i + 2) + " " + queue[idx];
        } else {
            g_visible_rows[i + 1].clear();
        }
    }
    M5.Display.endWrite();
}

void ui_init(bool display_only) {
    g_disp_mutex = xSemaphoreCreateMutex();

    // Allocate the waterfall frame buffer before later runtime allocations so
    // the request is served while DRAM is unfragmented.
    if (!wf_fb) {
        wf_fb = static_cast<uint16_t*>(
            heap_caps_malloc(WATERFALL_H * SCREEN_W * sizeof(uint16_t), MALLOC_CAP_DEFAULT));
        if (wf_fb) {
            memset(wf_fb, 0, WATERFALL_H * SCREEN_W * sizeof(uint16_t));
        }
    }

    if (display_only) {
        // KH1-MIC needs display-only board init: full M5Unified startup can
        // claim ES8311/I2S audio resources before the native mic path opens them.
        M5Cardputer.beginDisplayOnly(true);
    } else {
        auto cfg = M5.config();
        cfg.output_power = true;
        cfg.external_rtc = false;
        cfg.internal_mic = false;
        cfg.internal_spk = false;
        cfg.external_speaker_value = 0;
        M5Cardputer.begin(cfg, true);
    }
    M5.Display.setRotation(1);
    M5.Display.fillScreen(TFT_BLACK);
    ui_draw_countdown(0.0f, true, 1500);
}

void ui_set_waterfall_row(int row, const uint8_t* bins, int len) {
    if (len > SCREEN_W) len = SCREEN_W;
    if (row < 0 || row >= WATERFALL_H) return;
    memcpy(waterfall[row], bins, len);
}

// Internal: push one row unconditionally (respects ui_paused, ignores mute).
static void push_waterfall_row_impl(const uint8_t* bins, int len) {
    if (ui_paused) return;
    if (len > SCREEN_W) len = SCREEN_W;
    memcpy(waterfall[waterfall_head], bins, len);
    if (len < SCREEN_W) {
        memset(waterfall[waterfall_head] + len, 0, SCREEN_W - len);
    }
    waterfall_head = (waterfall_head + 1) % WATERFALL_H;
    waterfall_dirty = true;
}

// Called by audio RX pipeline — suppressed during TX so received-spectrum
// pixels don't mix with TX tone markers.
void ui_push_waterfall_row(const uint8_t* bins, int len) {
    if (s_rx_waterfall_muted) return;
    push_waterfall_row_impl(bins, len);
}

// Called by the TX tone synthesiser — always pushes regardless of mute so
// TX tone markers appear even when the RX audio path is silenced.
void ui_push_tx_waterfall_row(const uint8_t* bins, int len) {
    push_waterfall_row_impl(bins, len);
}

void ui_clear_waterfall() {
    for (int r = 0; r < WATERFALL_H; ++r) {
        memset(waterfall[r], 0, SCREEN_W);
    }
    waterfall_head = 0;
    waterfall_dirty = true;
    ui_draw_waterfall();
}

void ui_draw_waterfall() {
    waterfall_dirty = false;
    if (ui_paused) return;

    // Phase 1: convert intensity → RGB565 into the off-screen frame buffer.
    // Pure CPU work with no SPI traffic; no lock needed (wf_fb is only written here).
    // Guard against unlikely allocation failure (device keeps running, waterfall blank).
    if (!wf_fb) return;
    //
    // LovyanGFX pushImage(uint16_t*) sends bytes straight from memory without
    // swapping — it expects data already in big-endian (wire) order.  Our
    // rgb565() returns little-endian uint16_t, so we must bswap16 each value
    // when storing.  Example: yellow = rgb565(v,v,0) = 0xFFE0; stored LE as
    // [0xE0,0xFF]; without swap SPI sends 0xE0FF → purple.  After bswap16,
    // stored as [0xFF,0xE0]; SPI sends 0xFFE0 → yellow.
    for (int i = 0; i < WATERFALL_H; ++i) {
        int src = (waterfall_head + i) % WATERFALL_H;
        for (int x = 0; x < SCREEN_W; ++x) {
            uint8_t v = waterfall[src][x];
            wf_fb[i * SCREEN_W + x] = __builtin_bswap16(rgb565(v, v, 0));  // yellow, wire order
        }
    }

    // Phase 2: single pushImage() burst transfers all 4,320 pixels in one SPI
    // transaction (~1 ms) instead of 4,320 individual drawPixel() calls (~43 ms).
    DispGuard guard;
    M5.Display.pushImage(0, 0, SCREEN_W, WATERFALL_H, wf_fb);
}

static inline int hz_to_x(int hz) {
    // clamp to waterfall range
    //if (hz < 200)  hz = 200;
    //if (hz > 3000) hz = 3000;

    // map [200..3000] -> [0..SCREEN_W-1]
    const int in_min = 200, in_max = 3000;
    int x = (int)((int64_t)(hz - in_min) * (SCREEN_W - 1) / (in_max - in_min));
    if (x < 0) x = 0;
    if (x > SCREEN_W - 1) x = SCREEN_W - 1;
    return x;
}

static void ui_draw_offset_cursor_dot(int offset_hz) {
    const int y = WATERFALL_H;                 // countdown bar top
    const int cy = y + (COUNTDOWN_H / 2);      // vertically centered in bar
    int cx = hz_to_x(offset_hz);

    // 3x3 dot centered at (cx, cy)
    int x0 = cx - 1;
    int y0 = cy - 1;

    // clamp so we don't draw outside
    if (x0 < 0) x0 = 0;
    if (y0 < y) y0 = y;
    if (x0 + 3 > SCREEN_W) x0 = SCREEN_W - 3;
    if (y0 + 3 > y + COUNTDOWN_H) y0 = y + COUNTDOWN_H - 3;

    if(offset_hz>=200 && offset_hz <=3000)
      M5.Display.fillRect(x0, y0, 5, 3, rgb565(0, 80, 160));   // blue cursor dot
}

void ui_draw_countdown(float fraction, bool even_slot, int offset_hz) {
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
    // draw cursor last so countdown never overwrites it
    ui_draw_offset_cursor_dot(offset_hz);
}

// Helper: copy a UiRxLine into a RxDecodeEntry with bounded string copies
static void ui_copy_uirxline_to_entry(const UiRxLine& src, RxDecodeEntry* dst) {
    strncpy(dst->text,   src.text.c_str(),   RX_TEXT_MAX  - 1); dst->text[RX_TEXT_MAX - 1] = '\0';
    strncpy(dst->field1, src.field1.c_str(), RX_FIELD_MAX - 1); dst->field1[RX_FIELD_MAX - 1] = '\0';
    strncpy(dst->field2, src.field2.c_str(), RX_FIELD_MAX - 1); dst->field2[RX_FIELD_MAX - 1] = '\0';
    strncpy(dst->field3, src.field3.c_str(), RX_FIELD_MAX - 1); dst->field3[RX_FIELD_MAX - 1] = '\0';
    dst->snr       = src.snr;
    dst->offset_hz = src.offset_hz;
    dst->slot_id   = src.slot_id;
    dst->time_s    = 0.0f;
    dst->is_cq     = src.is_cq;
    dst->is_to_me  = src.is_to_me;
}

void ui_set_rx_list(const std::vector<UiRxLine>& lines) {
    int n = (int)lines.size();
    if (n > RX_MAX_DECODES) n = RX_MAX_DECODES;
    for (int i = 0; i < n; ++i) ui_copy_uirxline_to_entry(lines[i], &rx_lines[i]);
    rx_lines_count = n;
    rx_page = 0;
    rx_selected = -1;
    last_drawn_count = 0;
    last_page = -1;
}

void ui_set_rx_list_static(const RxDecodeEntry* entries, int count) {
    DispGuard guard;
    int n = count;
    if (n > RX_MAX_DECODES) n = RX_MAX_DECODES;
    if (n < 0) n = 0;
    for (int i = 0; i < n; ++i) rx_lines[i] = entries[i];  // POD copy, no heap
    rx_lines_count = n;
    rx_page = 0;
    rx_selected = -1;
    last_drawn_count = 0;
    last_page = -1;
}

bool ui_get_rx_entry(int idx, RxDecodeEntry* out) {
    if (!out) return false;
    DispGuard guard;
    if (idx < 0 || idx >= rx_lines_count) return false;
    *out = rx_lines[idx];  // POD copy
    return true;
}

int ui_get_rx_count() {
    DispGuard guard;
    return rx_lines_count;
}

void ui_force_redraw_rx() {
    last_drawn_count = 0;
    last_page = -1;
}

static void draw_rx_line(int y, const RxDecodeEntry& l, int line_no, bool selected, bool cyan_index_marker) {
    uint16_t color = TFT_WHITE;
    if (l.is_to_me) {
        color = rgb565(255, 0, 0);
    } else if (l.is_cq) {
        color = rgb565(0, 220, 0);
    }
    // Sticky line number in first column
    uint16_t bg = selected ? rgb565(30, 30, 60) : TFT_BLACK;
    const uint16_t index_color = cyan_index_marker ? rgb565(0, 255, 255) : TFT_WHITE;
    M5.Display.fillRect(0, y, SCREEN_W, 16, bg);  // clear text band; gap handled by line_h
    M5.Display.setTextColor(index_color, bg);
    M5.Display.setCursor(0, y);
    M5.Display.printf("%d ", line_no);
    M5.Display.setTextColor(color, bg);
    M5.Display.printf("%s", l.text);
    int row_idx = line_no - 1;
    if (row_idx >= 0 && row_idx < RX_LINES) {
        char buf[RX_TEXT_MAX + 8];
        snprintf(buf, sizeof(buf), "%d %s", line_no, l.text);
        g_visible_rows[row_idx] = buf;
    }
}

void ui_draw_rx(int flash_index) {
    const int line_h = 19; // 16 text + 3 gap
    // Add a 3px gap below the countdown before the first line
    const int start_y = UI_START_Y;
    // Only redraw when page changes or content changes, but always draw if list is empty
    if (rx_lines_count > 0 && flash_index < 0) {
        if (rx_page == last_page && last_drawn_count == rx_lines_count) {
            bool same = true;
            for (int i = 0; i < rx_lines_count; ++i) {
                if (strcmp(rx_lines[i].text, last_drawn_cache[i].text) != 0 ||
                    rx_lines[i].is_cq != last_drawn_cache[i].is_cq ||
                    rx_lines[i].is_to_me != last_drawn_cache[i].is_to_me) {
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
    const bool can_page_up = (rx_page > 0);
    const bool can_page_down = ((rx_page + 1) * RX_LINES < rx_lines_count);
    int start = rx_page * RX_LINES;
    for (int i = 0; i < RX_LINES; ++i) {
        int idx = start + i;
        int y = start_y + i * line_h;
        M5.Display.fillRect(0, y, SCREEN_W, line_h, TFT_BLACK);
        if (idx < rx_lines_count) {
            bool selected = (idx == flash_index);
            bool cyan_marker = ((i == 0) && can_page_up) || ((i == RX_LINES - 1) && can_page_down);
            draw_rx_line(y, rx_lines[idx], i + 1, selected, cyan_marker);
        } else {
            g_visible_rows[i].clear();
        }
    }
    M5.Display.endWrite();

    // cache drawn content
    if (flash_index < 0) {
        last_page = rx_page;
        last_drawn_count = rx_lines_count;
        for (int i = 0; i < rx_lines_count; ++i) {
            strncpy(last_drawn_cache[i].text, rx_lines[i].text, RX_TEXT_MAX - 1);
            last_drawn_cache[i].text[RX_TEXT_MAX - 1] = '\0';
            last_drawn_cache[i].is_cq = rx_lines[i].is_cq;
            last_drawn_cache[i].is_to_me = rx_lines[i].is_to_me;
            last_drawn_cache[i].valid = true;
        }
    } else {
        last_page = -1;
        last_drawn_count = 0;
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
        if ((rx_page + 1) * RX_LINES < rx_lines_count) {
            rx_page++;
            ui_draw_rx();
        }
    } else if (c >= '1' && c <= '6') {
        int line = c - '1';
        int idx = rx_page * RX_LINES + line;
        if (idx >= 0 && idx < rx_lines_count) {
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
    const int start_y = UI_START_Y;
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
            std::string row = fit_text_width(std::to_string(i + 1) + " " + lines[idx], SCREEN_W);
            M5.Display.print(row.c_str());
            g_visible_rows[i] = row;
        } else {
            g_visible_rows[i].clear();
        }
    }
    M5.Display.endWrite();
}

void ui_draw_debug(const std::vector<std::string>& lines, int page) {
    const int line_h = 19;
    const int start_y = UI_START_Y;
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
            g_visible_rows[i] = lines[idx];
        } else {
            g_visible_rows[i].clear();
        }
    }
    M5.Display.endWrite();
}

void ui_get_visible_text_lines(std::vector<std::string>& out) {
    out.clear();
    out.reserve(RX_LINES);
    for (int i = 0; i < RX_LINES; ++i) {
        out.push_back(g_visible_rows[i]);
    }
}

void ui_set_visible_text_line(int row_idx, const std::string& text) {
    if (row_idx < 0 || row_idx >= RX_LINES) return;
    g_visible_rows[row_idx] = text;
}

void ui_get_rx_page_info(int& current_page, int& total_pages) {
    total_pages = (rx_lines_count <= 0) ? 1 : ((rx_lines_count + RX_LINES - 1) / RX_LINES);
    if (total_pages < 1) total_pages = 1;
    current_page = rx_page + 1;
    if (current_page < 1) current_page = 1;
    if (current_page > total_pages) current_page = total_pages;
}

