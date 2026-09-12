#define DEBUG_LOG 1

#include <cstdio>
#include <cmath>
#include "esp_log.h"
extern "C" {
  #include "ft8/decode.h"
  #include "ft8/constants.h"
  #include "ft8/message.h"
  #include "ft8/encode.h"
  #include "ft8/debug.h"
  #include "common/monitor.h"
  }

#include "board_power.h"
#include "ui.h"
#include <vector>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_freertos_hooks.h"
#include "autoseq.h"
#include "core_api.h"
#include "core_api_internal.h"
#include <M5Cardputer.h>
#include <sstream>
#include <iterator>
#include <cstdio>
#include <string>
#include <cstdint>
#include <vector>
#include <array>
#include <cstring>
#include <algorithm>
#include <memory>
#include "driver/usb_serial_jtag.h"
#include "hal/uart_ll.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_random.h"
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <sys/time.h>
#include "esp_timer.h"
#include "esp_sleep.h"
#include "audio_source.h"
#include "stream_uac.h"
#include "dds_q15.h"
#include "radio_control.h"
#include "radio_control_backend.h"
#include "gps.h"
#include "external_rtc.h"

#include "storage_service.h"

static const char* STATION_FILE = "Station.txt";

#include "feature_flags.h"
#include "protocol.h"

// Active protocol for this boot session — set once by load_station_data() from
// Station.txt (protocol_mode=FT4), defaults to FT8.  Never changed mid-session;
// reboot to apply a mode change.
const ProtocolConfig* g_protocol = &kProtocolFT8;

#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 6000
#endif

int64_t rtc_now_ms();
using CopyLogsResult = StorageCopyResult;

static void debug_log_line(const std::string& msg);
//exported symbol (linkable from other .cpp)
void debug_log_line_public(const std::string& msg) {
  debug_log_line(msg);
}

static void build_rxtx_log_path(char* path, size_t path_sz) {
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);

  // RT[YYMMDD].txt
  snprintf(path, path_sz, "RT%02d%02d%02d.txt",
           (t.tm_year + 1900) % 100,
           (t.tm_mon + 1) % 100,
           t.tm_mday % 100);
}

static std::string today_qso_file_name() {
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);
  char name[20];
  snprintf(name, sizeof(name), "%04d%02d%02d.txt",
           (t.tm_year + 1900) % 10000, (t.tm_mon + 1) % 100, t.tm_mday % 100);
  return name;
}

static std::string storage_basename(const std::string& name_or_path) {
  size_t slash = name_or_path.find_last_of('/');
  if (slash == std::string::npos) return name_or_path;
  return name_or_path.substr(slash + 1);
}

static std::string today_rt_file_name() {
  char rt_path[64];
  build_rxtx_log_path(rt_path, sizeof(rt_path));
  return storage_basename(rt_path);
}

static bool storage_is_active_log_name(const std::string& name_or_path) {
  const std::string name = storage_basename(name_or_path);
  char rt_path[64];
  build_rxtx_log_path(rt_path, sizeof(rt_path));
  return name == storage_basename(rt_path) ||
         name == today_qso_file_name() ||
         name == "fieldday.txt";
}

static CopyLogsResult copy_logs_to_sd_overwrite();

// 128 entries × 16 bytes = 2 KB of BSS. 256 was the original size but
// well over typical working set (FT8 rarely sees >50 unique hashed
// callsigns in an active period; the aging + eviction logic keeps it
// fresh). Reducing by 2 KB gives the USB DMA buffer (4608 bytes) a
// Keep this compact so USB host setup retains contiguous heap.
// fragmentation.
#define CALLSIGN_HASHTABLE_SIZE 128

static struct
{
    char callsign[12]; /// Up to 11 symbols of callsign + trailing zero
    uint32_t hash;     /// 8 MSBs = age, 22 LSBs = hash value
} callsign_hashtable[CALLSIGN_HASHTABLE_SIZE];

static int callsign_hashtable_size;

void hashtable_init(void)
{
    callsign_hashtable_size = 0;
    memset(callsign_hashtable, 0, sizeof(callsign_hashtable));
}

// Increment age for all existing entries (saturate at 255). Call once per slot.
static void hashtable_age_all(void)
{
    for (int i = 0; i < CALLSIGN_HASHTABLE_SIZE; ++i)
    {
        if (callsign_hashtable[i].callsign[0] != '\0')
        {
            uint8_t age = (uint8_t)(callsign_hashtable[i].hash >> 24);
            if (age < 255)
            {
                age++;
                callsign_hashtable[i].hash =
                    ((uint32_t)age << 24) | (callsign_hashtable[i].hash & 0x003FFFFFu);
            }
        }
    }
}

// Trim the hash table if it grows too large by evicting the oldest entries
void hashtable_trim_size(int max_size)
{
    while (callsign_hashtable_size > max_size)
    {
        int oldest_idx = -1;
        uint8_t oldest_age = 0;

        for (int i = 0; i < CALLSIGN_HASHTABLE_SIZE; ++i)
        {
            if (callsign_hashtable[i].callsign[0] == '\0')
                continue;

            uint8_t age = (uint8_t)(callsign_hashtable[i].hash >> 24);
            if (oldest_idx < 0 || age > oldest_age)
            {
                oldest_idx = i;
                oldest_age = age;
            }
        }

        if (oldest_idx < 0)
            break;

        LOG(LOG_INFO, "Hashtable trim: removing oldest [%s], age=%u\n",
            callsign_hashtable[oldest_idx].callsign, (unsigned)oldest_age);

        callsign_hashtable[oldest_idx].callsign[0] = '\0';
        callsign_hashtable[oldest_idx].hash = 0;
        callsign_hashtable_size--;
    }
}

void hashtable_add(const char* callsign, uint32_t hash)
{
    if (!callsign || !callsign[0])
        return;

    uint32_t hash_payload = hash & 0x003FFFFFu;   // 22-bit value
    uint16_t hash10 = (hash_payload >> 12) & 0x03FFu;
    int idx = (hash10 * 23) % CALLSIGN_HASHTABLE_SIZE;
    int start_idx = idx;

    while (callsign_hashtable_size >= CALLSIGN_HASHTABLE_SIZE)
    {
        hashtable_trim_size(CALLSIGN_HASHTABLE_SIZE - 50);
        if (callsign_hashtable_size >= CALLSIGN_HASHTABLE_SIZE)
        {
            LOG(LOG_INFO, "Hash table full; ignoring new callsign [%s]\n", callsign);
            return;
        }
    }

    // Linear probing: must match lookup logic
    while (callsign_hashtable[idx].callsign[0] != '\0')
    {
        uint32_t existing_hash = callsign_hashtable[idx].hash & 0x003FFFFFu;

        if ((existing_hash == hash_payload) &&
            (strcmp(callsign_hashtable[idx].callsign, callsign) == 0))
        {
            // Refresh age to 0, keep same callsign/hash
            callsign_hashtable[idx].hash = hash_payload;
            LOG(LOG_DEBUG, "Found duplicate [%s], refreshed age\n", callsign);
            return;
        }

        if (existing_hash == hash_payload)
        {
            // Same 22-bit hash but different callsign: replace old one
            LOG(LOG_INFO, "Replacing [%s] with [%s] on same hash\n",
                callsign_hashtable[idx].callsign, callsign);

            strncpy(callsign_hashtable[idx].callsign, callsign, 11);
            callsign_hashtable[idx].callsign[11] = '\0';
            callsign_hashtable[idx].hash = hash_payload;
            return;
        }

        idx = (idx + 1) % CALLSIGN_HASHTABLE_SIZE;
        if (idx == start_idx)
        {
            LOG(LOG_INFO, "Hash table probe wrapped; abort insert for [%s]\n", callsign);
            return;
        }
    }

    strncpy(callsign_hashtable[idx].callsign, callsign, 11);
    callsign_hashtable[idx].callsign[11] = '\0';
    callsign_hashtable[idx].hash = hash_payload;  // age=0
    callsign_hashtable_size++;
}

bool hashtable_lookup(ftx_callsign_hash_type_t hash_type, uint32_t hash, char* callsign)
{
    if (!callsign)
        return false;

    uint8_t hash_shift =
        (hash_type == FTX_CALLSIGN_HASH_10_BITS) ? 12 :
        (hash_type == FTX_CALLSIGN_HASH_12_BITS) ? 10 : 0;

    // Derive the same start bucket from the top 10 bits of the 22-bit hash.
    // For 10-bit lookup: hash is already the top 10 bits.
    // For 12-bit lookup: top 10 bits are hash >> 2.
    // For 22-bit lookup: top 10 bits are hash >> 12.
    uint16_t hash10 =
        (hash_type == FTX_CALLSIGN_HASH_10_BITS) ? (hash & 0x03FFu) :
        (hash_type == FTX_CALLSIGN_HASH_12_BITS) ? ((hash >> 2) & 0x03FFu) :
                                                   ((hash >> 12) & 0x03FFu);

    int idx = (hash10 * 23) % CALLSIGN_HASHTABLE_SIZE;
    // Important: entries can be deleted by hashtable_trim_size(), which creates
    // empty holes in probe chains. Stopping at the first empty slot can miss
    // valid entries that were inserted later in that chain. Scan the full table.
    for (int probe = 0; probe < CALLSIGN_HASHTABLE_SIZE; ++probe)
    {
        int scan_idx = (idx + probe) % CALLSIGN_HASHTABLE_SIZE;
        if (callsign_hashtable[scan_idx].callsign[0] == '\0')
            continue;

        uint32_t existing_hash = callsign_hashtable[scan_idx].hash & 0x003FFFFFu;

        if ((existing_hash >> hash_shift) == hash)
        {
            strcpy(callsign, callsign_hashtable[scan_idx].callsign);

            // Reset age to 0 on successful hit, preserve 22-bit payload.
            callsign_hashtable[scan_idx].hash = existing_hash;
            return true;
        }
    }

    callsign[0] = '\0';
    return false;
}

ftx_callsign_hash_interface_t hash_if = {
    .lookup_hash = hashtable_lookup,
    .save_hash = hashtable_add
};

static std::string normalize_call_token(std::string s) {
  // trim <> wrappers used for hashed nonstd calls
  if (!s.empty() && s.front() == '<') s.erase(s.begin());
  if (!s.empty() && s.back()  == '>') s.pop_back();

  for (auto& ch : s) ch = (char)toupper((unsigned char)ch);
  return s;
}

static bool rewrite_dxpedition_for_mycall(const std::string& raw_text,
                                          const std::string& mycall_up,
                                          std::string& rewritten_text) {
  std::istringstream iss(raw_text);
  std::string call1, rr73_tok, call2, foxcall, rpt;
  if (!(iss >> call1 >> rr73_tok >> call2 >> foxcall >> rpt)) return false;

  std::string trailing;
  if (iss >> trailing) return false;
  if (rr73_tok != "RR73;") return false;

  std::string call1_up = normalize_call_token(call1);
  std::string call2_up = normalize_call_token(call2);
  if (call1_up.empty() || call2_up.empty() || mycall_up.empty()) return false;

  if (call1_up == mycall_up) {
    rewritten_text = call1 + " " + foxcall + " RR73";
    return true;
  }
  if (call2_up == mycall_up) {
    rewritten_text = call2 + " " + foxcall + " " + rpt;
    return true;
  }
  return false;
}

static const char* TAG = "FT8";
static const char* STORAGE_COPY_TAG = "STORAGE_COPY";
enum class UIMode { RX, TX, BAND, MENU, MSC, DEBUG, STATUS, QSO, GPS, PERF };
enum class RtcTimeSource : uint8_t {
  SAVED = 0,
  ESP_RTC,
  DS3231,
  GPS,
  MANUAL,
};
static UIMode ui_mode = UIMode::RX;
static int tx_page = 0;
// NOTE: previous `std::vector<UiRxLine> g_rx_lines` was removed to eliminate
// the last heap allocation in the decode/display path. The RX list now lives
// as a static RxDecodeEntry array inside ui.cpp, populated via
// ui_set_rx_list_static() and read back via ui_get_rx_entry()/ui_get_rx_count().
static volatile bool g_tx_view_dirty = false;  // Set when autoseq state changes
int64_t g_decode_slot_idx = -1; // set at decode trigger to tag RX lines with slot parity
// Monotonic index of the most recent slot whose decode has been fully applied to
// autoseq state (or whose audio was never decoded, e.g. paused/skipped). Enforces
// the sequential invariant "TX in slot N is blocked until decode for slot N-1 is
// applied." Written by stream_uac_task on core 1, read by check_slot_boundary on
// core 0. Initialized to -1 so the first TX after boot isn't blocked.
volatile int64_t g_decode_applied_slot_idx = -1;

// Set after CDC-ACM opens during an explicit user connection. The main loop
// consumes it once to synchronize the selected band and mode.
volatile bool g_cdc_initial_sync_pending = false;

// Deferred-save flag. main.cpp owns storage; core_api commands only request
// a deferred save.
volatile bool g_config_save_pending = false;

// State machine variables (matching reference project architecture)
// TX is scheduled by setting these flags; actual TX starts at slot boundary
// Global TX-arming state: read by tx_tick on the next slot boundary.
// Non-static so core_api.cpp can arm it from any UI consumer.
volatile bool g_qso_xmit = false;        // TX is pending
volatile int g_target_slot_parity = 0;   // 0=even, 1=odd - parity of slot to TX on
static volatile bool g_was_txing = false;       // We were transmitting (for tick timing)
volatile bool g_decode_in_progress = false; // Block TX trigger while decoding
static int g_last_slot_parity = -1;             // For slot boundary detection (just parity, like reference)

static volatile uint32_t g_perf_idle_count[2] = {0, 0};
static uint32_t g_perf_prev_idle_count[2] = {0, 0};
static TickType_t g_perf_prev_sample_tick = 0;
static uint8_t g_perf_cpu_busy_pct[2] = {0, 0};
static bool g_perf_cpu_hook_ok[2] = {false, false};
static bool g_perf_cpu_sample_valid = false;

// BeaconMode and BandItem now defined in station_types.h
#include "station_types.h"
std::vector<BandItem> g_bands = {   // visible to core_api.cpp
    {"160m", 1840},   {"80m", 3573},   {"60m", 5357},   {"40m", 7074},
    {"30m", 10136},   {"20m", 14074},  {"17m", 18100},  {"15m", 21074},
    {"12m", 24915},   {"10m", 28074},  {"6m", 50313},   {"2m", 144174},
};
static std::string g_active_band_text = "80 40 20 17 15 12 10";
static std::vector<int> g_active_band_indices;
static int band_page = 0;
static int band_edit_idx = -1;       // absolute index into g_bands
static std::string band_edit_buffer; // text while editing
void update_autoseq_cq_type();  // visible to core_api.cpp
BeaconMode g_beacon = BeaconMode::OFF;   // visible to core_api.cpp
int g_offset_hz = 1500;                  // visible to core_api.cpp
int g_band_sel = 1; // default 80m       // visible to core_api.cpp
static bool g_tune = false;
static BeaconMode g_status_beacon_temp = BeaconMode::OFF;
[[maybe_unused]] static bool g_cat_toggle_high = false;
std::string g_date = "2025-12-11";      // visible to core_api.cpp
std::string g_time = "10:10:00";        // visible to core_api.cpp
static int status_edit_idx = -1;     // 0-5
static std::string status_edit_buffer;
static int status_cursor_pos = -1;
static std::vector<std::string> g_debug_lines;
static int debug_page = 0;
static const size_t DEBUG_MAX_LINES = 18; // 3 pages
static const size_t DEBUG_HUD_LINES = 2;  // slots 0-1 reserved for live HUD
static constexpr uint32_t APP_CORE0_STACK_BYTES = 12288; // Tune to 16384/18432 if Amin < 1536B
static TickType_t g_app_core0_stack_last_sample_tick = 0;
static uint32_t g_app_core0_stack_cur_free_bytes = 0;
static uint32_t g_app_core0_stack_min_free_bytes = 0;

static void enter_msc_mode(const char* reason);
static void exit_msc_mode();
static void host_handle_line(const std::string& line);
void save_station_data();  // visible to core_api.cpp

// Core commands request a save; the main task performs storage I/O.
extern volatile bool g_config_save_pending;
// TX entry for display and scheduling (populated by autoseq)
// Non-static for the same reason as g_qso_xmit / g_target_slot_parity
// above — core_api.cpp's tap_rx RPC arms these on user-pick events.
AutoseqTxEntry g_pending_tx;
bool g_pending_tx_valid = false;

// Forward declarations — definitions live near check_slot_boundary, where
// g_offset_src has been declared.
void arm_pending_tx(const AutoseqTxEntry& pending);
volatile bool g_tx_cancel_requested = false;   // visible to core_api.cpp
static void host_process_bytes(const uint8_t* buf, size_t len);
[[maybe_unused]] static void poll_host_uart();
static void enter_mode(UIMode new_mode);
static std::string menu_sleep_batt_line();
static int normalize_gps_baud_value(int value);
static gps_pins_t gps_pins_for_current_source();
static const char* gps_source_name();
static void apply_debug_uart_pin_policy();
static void restore_debug_uart_pins_after_sd();
static bool rtc_set_from_strings_source(RtcTimeSource source);
static esp_err_t rtc_write_external_from_soft(const char* reason);
static const char* rtc_time_source_suffix();
bool rtc_set_from_strings();
bool rtc_apply_manual_time_from_strings();   // visible to core_api.cpp
void rtc_sync_to_esp_rtc();                  // visible to core_api.cpp
static bool g_rx_dirty = false;



static std::vector<std::string> g_msc_lines = {
    "Mini-FT8 USB drive",
    "now mounted on PC.",
    "",
    "Safely eject on PC,",
    "then press C to",
    "return to radio."
};

static std::vector<std::string> g_startup_lines = {
    "** Mini-FT8 V2.1d **",
    " S/R/T: Operate",
    " M/N/O: Menu",
    " Q/F/D: File",
    "      * * * * *     ",
    "  By N6HAN & AG6AQ "
};

// Runtime latch: when true, we're still showing the startup screen. Either
// a keypress or the 1 s auto-dismiss timer (g_startup_start_ms) takes us
// out.
static bool    g_startup_active  = true;
static int64_t g_startup_start_ms = 0;    // set on the first tick we see in the splash branch
static constexpr int64_t kStartupAutoDismissMs = 1000;

static bool is_startup_direct_mode_key(char c) {
  const char k = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  switch (k) {
    case 'S':
    case 'R':
    case 'T':
    case 'G':
    case 'Q':
    case 'M':
    case 'N':
    case 'O':
    case 'B':
    case 'F':
    case 'P':
    case 'C':
    case 'D':
      return true;
    default:
      return false;
  }
}

static std::vector<std::string> g_q_lines;
static std::vector<std::string> g_q_files;
enum class QPageView { Default, Alternate };
struct QsoLogEntry {
  std::string time_on;
  std::string band;
  std::string call;
  bool has_rst_rcvd = false;
  int rst_rcvd = 0;
  bool has_rst_sent = false;
  int rst_sent = 0;
};
static QPageView g_q_page_view = QPageView::Default;
static std::vector<QsoLogEntry> g_q_entries;
static bool g_q_entries_have_next_page = false;
static bool g_q_show_entries = false;
static int q_page = 0;
static std::string g_q_current_file;
static std::vector<std::string> g_d_lines;
static std::vector<std::string> g_d_files;
static int d_page = 0;
static std::string host_input;
static const char* HOST_PROMPT = "MINIFT8> ";
static bool usb_ready = false;
static QueueHandle_t s_key_inject_queue = nullptr;
static bool host_bin_active = false;
static size_t host_bin_remaining = 0;
static StorageStream* host_bin_stream = nullptr;
static uint32_t host_bin_crc = 0;
static uint32_t host_bin_expected_crc = 0;
static size_t host_bin_received = 0;
static std::vector<uint8_t> host_bin_buf;
static const size_t HOST_BIN_CHUNK = 512;
static size_t host_bin_chunk_expect = 0; // payload bytes this chunk (excludes CRC trailer)
static uint8_t host_bin_first8[8] = {0};
static uint8_t host_bin_last8[8] = {0};
static size_t host_bin_first_filled = 0;
static std::string host_bin_path;

// Software RTC
static time_t rtc_epoch_base = 0;
static int64_t rtc_ms_start = 0;
static int64_t rtc_last_update = 0;
static bool rtc_valid = false;
static RtcTimeSource g_rtc_time_source = RtcTimeSource::SAVED;

// RTC deep sleep compensation
// rtc_sleep_epoch: epoch time when entering deep sleep (for calculating elapsed time)
// rtc_comp is seconds per 10000 seconds. It remains load/save/core-API
// compatible, but the local O-page editor is no longer exposed.
static constexpr int kRtcCompFixed = 120;
static time_t g_rtc_sleep_epoch = 0;
int g_rtc_comp = kRtcCompFixed;        // visible to core_api.cpp
static int clamp_rtc_comp_value(int value) {
  if (value < -9000) return -9000;
  if (value > 9000) return 9000;
  return value;
}

// CqType, OffsetSrc, RadioType now defined in station_types.h
struct RadioProfileBinding {
  audio_source_backend_t audio_backend;
  radio_control_backend_t radio_backend;
};
CqType g_cq_type = CqType::CQ;                // visible to core_api.cpp
std::string g_cq_freetext = "FreeText";       // visible to core_api.cpp
bool g_skip_tx1 = false;                      // visible to core_api.cpp
int g_autoseq_max_retry = AUTOSEQ_MAX_RETRY;  // visible to core_api.cpp
static std::string g_free_text = "TNX 73";
std::string g_call = "YOURCALL";   // visible to core_api.cpp
std::string g_grid = "CM97";       // visible to core_api.cpp
static std::string g_grid_saved_manual = "CM97";
static bool g_grid_from_gps = false;
static bool g_time_synced_from_gps = false;
static std::string g_grid_gps_display8;
bool g_decode_enabled = true;
int g_time_osr = 2;
int g_freq_osr = 1;
OffsetSrc g_offset_src = OffsetSrc::RANDOM;  // visible to core_api.cpp
RadioType g_radio = RadioType::QMX;          // visible to core_api.cpp
static bool g_kh1_connected = false;
static int g_gps_baud = 115200;
static bool g_gnss_lora_enabled = false;
static constexpr size_t kIgnorePrefixTextMaxLen = 64;
std::string g_comment1 = "MiniFT8 /Radio";      // visible to core_api.cpp
static std::string g_ignore_prefix_text;
std::vector<std::string> g_ignore_prefixes;     // visible to core_api.cpp
static bool g_rxtx_log = true;
static RadioType canonical_radio_type(RadioType r);
static RadioType parse_radio_config_value(const char* raw);
static bool is_kh1_radio(RadioType r);
static bool radio_type_uses_display_only(RadioType r);
static RadioProfileBinding get_radio_profile_binding(RadioType r);
void apply_radio_profile_binding();   // visible to core_api.cpp
static void gps_runtime_tick();
static std::string expand_comment_macros(const std::string& src);
static std::string normalize_grid_maidenhead(const std::string& src);
// Non-static so core_api.cpp's set_call / set_grid RPCs can refresh the
// autoseq station info exactly like the on-device MENU/STATUS edits do.
std::string grid_ft8_4(const std::string& grid);
// Single-threaded TX state machine (replaces separate tx_send_task)
// TX runs in main loop via tx_tick(), one tone at a time
static bool g_tx_active = false;           // TX state machine is running
static int g_tx_tone_idx = 0;              // Current tone index (0..total_symbols-1)
static int64_t g_tx_next_tone_time = 0;    // When to send next tone (ms)
static int64_t g_tx_slot_start_ms = 0;     // Slot boundary time for tone alignment
static uint8_t g_tx_tones[FT4_NN];         // Encoded tones — sized for FT4 (105 > FT8's 79)
static int g_tx_base_hz = 0;               // Base frequency for TA commands
static int64_t g_tx_slot_idx = 0;          // Slot index for autoseq_mark_sent
static bool g_tx_cat_ok = false;           // CAT available for this TX
static int g_tx_last_ta_int = -1;          // For TA command deduplication
static int g_tx_last_ta_frac = -1;

static bool storage_should_guard_active_logs() {
  return g_tx_active || g_decode_in_progress || audio_source_is_streaming() || host_bin_active;
}

static bool storage_reject_active_log_user_mutation(const std::string& name_or_path) {
  return storage_should_guard_active_logs() && storage_is_active_log_name(name_or_path);
}

static int menu_page = 0;
static int menu_edit_idx = -1;
// Tracks the protocol mode that has been saved to Station.txt and will take
// effect on next reboot.  Initialised from g_protocol after load_station_data().
// Differs from g_protocol when the user has toggled Mode but not yet rebooted.
#if ENABLE_FT4
static bool g_protocol_pending_ft4 = false;
#endif
static std::string menu_edit_buf;
static int menu_cursor_edit_original = 0;
static bool menu_long_edit = false;
static enum { LONG_NONE, LONG_FT, LONG_COMMENT, LONG_ACTIVE, LONG_IGNORE } menu_long_kind = LONG_NONE;
static std::string menu_long_buf;
static std::string menu_long_backup;
static int menu_flash_idx = -1;          // absolute index to flash highlight
static int64_t menu_flash_deadline = 0;  // ms timestamp when flash ends
static std::string menu_copy_feedback_text;
static int64_t menu_copy_feedback_deadline = 0;
static constexpr int64_t kMenuCopyFeedbackMs = 1800;
static int rx_flash_idx = -1;
static int64_t rx_flash_deadline = 0;
bool g_streaming = false;
static void draw_menu_view();
static void draw_battery_icon(int x, int y, int w, int h, int level, bool charging);
static void draw_status_view();
static void draw_status_line(int idx, const std::string& text, bool highlight);
void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui);
static void update_countdown();
static void consume_cdc_initial_sync();
// Non-static so core_api.cpp can push band changes to the radio immediately.
bool sync_radio_to_current_band(const char* reason);
static void menu_flash_tick();
static void rx_flash_tick();
static bool looks_like_grid(const std::string& s);
static bool looks_like_report(const std::string& s, int& out);
static std::string g_last_reply_text;
void rebuild_active_bands();   // visible to core_api.cpp
static void schedule_tx_if_idle();
static int64_t s_last_tx_slot_idx = -1000;  // Track last TX slot for retry scheduling
[[maybe_unused]] static bool g_sync_pending = false;
[[maybe_unused]] static int g_sync_delta_ms = 0;
static void enqueue_beacon_cq();
static void load_storage_regular_files(std::vector<std::string>& files);
static void qso_load_file_list();
static void qso_load_fetch_file_list();
static void delete_load_file_list();
static void qso_load_entries(const std::string& path);
static void qso_draw_page();

static void log_rxtx_line(char dir, int snr, int offset_hz, const std::string& text, int repeat_counter = -1);
static bool log_adif_entry(const std::string& dxcall, const std::string& dxgrid, int rst_sent, int rst_rcvd);
static bool storage_append_text_locked_path(const std::string& path,
                                            const std::string& line,
                                            const std::string& header_if_new,
                                            bool sync_to_flash);
static bool storage_write_cabrillo_fd_entry(const std::string& mycall,
                                             const std::string& location,
                                             const std::string& qso_line);
#if !MIC_PROBE_APP
void log_heap(const char* tag) {
  size_t free_sz = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  ESP_LOGI(tag, "HEAP: free=%u min=%u largest=%u", (unsigned)free_sz, (unsigned)min_free, (unsigned)largest);
}
static void log_mem_caps(const char* tag) {
  size_t free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t largest_8bit = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  size_t largest_internal = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  size_t free_dma = heap_caps_get_free_size(MALLOC_CAP_DMA);
  size_t largest_dma = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
  size_t min_8bit = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  ESP_LOGI(tag,
           "MEM: 8bit_free=%u 8bit_largest=%u internal_free=%u internal_largest=%u dma_free=%u dma_largest=%u 8bit_min=%u",
           (unsigned)free_8bit,
           (unsigned)largest_8bit,
           (unsigned)free_internal,
           (unsigned)largest_internal,
           (unsigned)free_dma,
           (unsigned)largest_dma,
           (unsigned)min_8bit);
}
static std::string fd_trim(const std::string& s) {
  size_t a = 0, b = s.size();
  while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) ++a;
  while (b > a && (s[b-1] == ' ' || s[b-1] == '\t' || s[b-1] == '\r' || s[b-1] == '\n')) --b;
  return s.substr(a, b - a);
}

static std::string fd_strip_R(const std::string& s) {
  std::string t = fd_trim(s);
  if (t.size() >= 2 && t[0] == 'R' && t[1] == ' ') return fd_trim(t.substr(2));
  return t;
}

static std::string fd_get_section_from_exchange(const std::string& ex) {
  // ex: "1B SCV" (or "R 1B SCV")
  std::string t = fd_strip_R(ex);
  size_t sp = t.find(' ');
  if (sp == std::string::npos) return "DX";
  return fd_trim(t.substr(sp + 1));
}

// Called by autoseq when an FD QSO completes. We derive freq/time from current radio state
// and use FreeText as our FD exchange (e.g. "1B SCV").
static bool log_cabrillo_fd_entry(const std::string& dxcall, const std::string& their_fd_exchange) {
  if (g_cq_type != CqType::CQFD) return true;

  const std::string my_fd = fd_strip_R(g_free_text);
  const std::string their_fd = fd_strip_R(their_fd_exchange);

  if (my_fd.empty() || their_fd.empty() || dxcall.empty()) return false;

  // Time (UTC assumed as RTC timebase, same as ADIF writer)
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);

  char date_ymd[16];
  snprintf(date_ymd, sizeof(date_ymd), "%04d-%02d-%02d",
           (t.tm_year + 1900) % 10000, (t.tm_mon + 1) % 100, t.tm_mday % 100);

  char time_hhmm[8];
  snprintf(time_hhmm, sizeof(time_hhmm), "%02d%02d", t.tm_hour % 100, t.tm_min % 100);

  // Frequency: use selected band dial frequency (kHz); round float to nearest integer.
  int freq_khz = (int)(g_bands[g_band_sel].freq + 0.5f);

  std::string location = fd_get_section_from_exchange(my_fd);

  char qso_line[128];
  snprintf(qso_line, sizeof(qso_line), "QSO: %d DG %s %s %s %s %s %s",
           freq_khz,
           date_ymd,
           time_hhmm,
           g_call.c_str(),
           my_fd.c_str(),
           dxcall.c_str(),
           their_fd.c_str());

  return storage_write_cabrillo_fd_entry(g_call, location, qso_line);
}

#else
static inline void log_heap(const char*) {}
static inline void log_mem_caps(const char*) {}
static bool log_cabrillo_fd_entry(const std::string&, const std::string&) { return true; }
#endif

static bool storage_append_text_locked_path(const std::string& path,
                                             const std::string& line,
                                             const std::string& header_if_new,
                                             bool sync_to_flash) {
  return storage_file_append(path, line, header_if_new, sync_to_flash);
}

static bool storage_write_cabrillo_fd_entry(const std::string& mycall,
                                            const std::string& location,
                                            const std::string& qso_line) {
#if !MIC_PROBE_APP
  return storage_file_append_cabrillo(mycall, location, qso_line);
#else
  (void)mycall;
  (void)location;
  (void)qso_line;
  return true;
#endif
}

static void log_rxtx_line(char dir, int snr, int offset_hz, const std::string& text, int repeat_counter) {
  if (!g_rxtx_log) return;

  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);
  char ts[32];
  snprintf(ts, sizeof(ts), "%04d%02d%02d %02d%02d%02d",
           t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
           t.tm_hour, t.tm_min, t.tm_sec);
  double freq_mhz = 0.001 * (double)g_bands[g_band_sel].freq;

  char log_path[64];
  build_rxtx_log_path(log_path, sizeof(log_path));

  char line[256];
  if (dir == 'T') {
    snprintf(line, sizeof(line), "%c [%s][%.3f] %s %d\n",
             dir, ts, freq_mhz, text.c_str(), offset_hz);
  } else {
    snprintf(line, sizeof(line), "%c [%s][%.3f] %s %d %d\n",
             dir, ts, freq_mhz, text.c_str(), snr, offset_hz);
  }
  (void)repeat_counter;
  (void)storage_append_text_locked_path(log_path, line, "", false);
}

static bool log_gps_grid_line(const std::string& grid8) {
  if (!g_rxtx_log) return false;
  if (grid8.size() != 8) return false;

  // GPS grid breadcrumbs use RT files but not log_rxtx_line(), which appends
  // RX SNR/offset fields that do not apply to this record type.
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);
  char ts[32];
  snprintf(ts, sizeof(ts), "%04d%02d%02d %02d%02d%02d",
           t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
           t.tm_hour, t.tm_min, t.tm_sec);
  double freq_mhz = 0.001 * (double)g_bands[g_band_sel].freq;

  char log_path[64];
  build_rxtx_log_path(log_path, sizeof(log_path));

  char line[128];
  snprintf(line, sizeof(line), "G [%s][%.3f] %s\n", ts, freq_mhz, grid8.c_str());
  bool ok = storage_append_text_locked_path(log_path, line, "", false);
  if (ok) ESP_LOGI(TAG, "GPS grid logged: %s", grid8.c_str());
  return ok;
}

static bool is_daily_qso_txt_file(const char* name) {
  if (!name) return false;
  if (strlen(name) != 12) return false;  // YYYYMMDD.txt
  for (int i = 0; i < 8; ++i) {
    if (!std::isdigit(static_cast<unsigned char>(name[i]))) return false;
  }
  return name[8] == '.' &&
         std::tolower(static_cast<unsigned char>(name[9])) == 't' &&
         std::tolower(static_cast<unsigned char>(name[10])) == 'x' &&
         std::tolower(static_cast<unsigned char>(name[11])) == 't';
}

static const char* storage_owner_label(StorageOwner owner) {
  switch (owner) {
    case StorageOwner::UNAVAILABLE: return "unavailable";
    case StorageOwner::FIRMWARE:    return "firmware";
    case StorageOwner::USB_HOST:    return "usb_host";
    case StorageOwner::TRANSITION:  return "transition";
  }
  return "unknown";
}

static const char* qso_storage_list_failure_text(StorageOwner owner) {
  switch (owner) {
    case StorageOwner::USB_HOST:
    case StorageOwner::TRANSITION:
      return "Storage busy";
    case StorageOwner::UNAVAILABLE:
      return "Storage unavailable";
    case StorageOwner::FIRMWARE:
      return "List failed";
  }
  return "List failed";
}

static CopyLogsResult copy_logs_busy_result() {
  CopyLogsResult result{};
  result.err = ESP_ERR_INVALID_STATE;
  result.status = StorageCopyStatus::STORAGE_BUSY;
  result.missed_count = 1;
  result.missed_files.push_back("<storage busy>");
  return result;
}

static CopyLogsResult copy_logs_to_sd_overwrite() {
  const std::string qso_name = today_qso_file_name();
  const std::string rt_name = today_rt_file_name();
  const StorageOwner owner = storage_service_owner();
  const size_t open_streams = storage_service_open_stream_count();
  const bool usb_drive = storage_service_usb_drive_enabled();
  const bool tx_active = g_tx_active;
  const bool decode_active = g_decode_in_progress;
  const bool audio_streaming = audio_source_is_streaming();
  const bool host_binary_active = host_bin_active;
  const bool ui_msc = ui_mode == UIMode::MSC;

  ESP_LOGI(STORAGE_COPY_TAG,
           "request owner=%s open_streams=%u tx=%d decode=%d audio=%d host_bin=%d ui_msc=%d usb_drive=%d qso=%s rt=%s",
           storage_owner_label(owner),
           static_cast<unsigned>(open_streams),
           tx_active,
           decode_active,
           audio_streaming,
           host_binary_active,
           ui_msc,
           usb_drive,
           qso_name.c_str(),
           rt_name.c_str());

  if (tx_active || decode_active || audio_streaming || host_binary_active ||
      ui_msc || usb_drive || owner != StorageOwner::FIRMWARE || open_streams != 0) {
    ESP_LOGW(STORAGE_COPY_TAG,
             "copy blocked: owner=%s open_streams=%u tx=%d decode=%d audio=%d host_bin=%d ui_msc=%d usb_drive=%d",
             storage_owner_label(owner),
             static_cast<unsigned>(open_streams),
             tx_active,
             decode_active,
             audio_streaming,
             host_binary_active,
             ui_msc,
             usb_drive);
    return copy_logs_busy_result();
  }

  const CopyLogsResult result = storage_copy_all_to_sd(qso_name, rt_name);
  // SD access temporarily claims GPIO5 (LoRa-1262 CS); restore prior pin policy.
  restore_debug_uart_pins_after_sd();
  return result;
}

static void qso_load_file_list() {
  g_q_files.clear();
  g_q_entries.clear();
  g_q_lines.clear();
  g_q_entries_have_next_page = false;
  std::vector<std::string> files;
  if (!storage_file_list(files)) {
    const StorageOwner owner = storage_service_owner();
    const char* reason = qso_storage_list_failure_text(owner);
    ESP_LOGW(TAG, "QSO file list failed: owner=%s reason=%s",
             storage_owner_label(owner), reason);
    g_q_lines.push_back(reason);
    return;
  }
  for (const auto& name : files) {
    if (is_daily_qso_txt_file(name.c_str())) {
      g_q_files.push_back(name);
    }
  }
  std::sort(g_q_files.begin(), g_q_files.end(), std::greater<std::string>());
  const StorageOwner owner = storage_service_owner();
  ESP_LOGI(TAG, "QSO file list: owner=%s files=%u daily=%u",
           storage_owner_label(owner),
           static_cast<unsigned>(files.size()),
           static_cast<unsigned>(g_q_files.size()));
  if (g_q_files.empty()) {
    g_q_lines.push_back("No YYYYMMDD.txt");
    return;
  }
  for (size_t i = 0; i < g_q_files.size(); ++i) {
    g_q_lines.push_back(g_q_files[i]);
  }
}

static void load_storage_regular_files(std::vector<std::string>& files) {
  if (!storage_file_list(files)) files.clear();
  std::sort(files.begin(), files.end(), std::greater<std::string>());
}

static void delete_load_file_list() {
  g_d_files.clear();
  g_d_lines.clear();
  load_storage_regular_files(g_d_files);
  g_d_files.erase(std::remove(g_d_files.begin(), g_d_files.end(), "Station.txt"), g_d_files.end());
  if (g_d_files.empty()) {
    g_d_lines.push_back("No storage files");
    return;
  }
  for (size_t i = 0; i < g_d_files.size(); ++i) {
    g_d_lines.push_back(std::string("DEL ") + g_d_files[i]);
  }
}

static void qso_load_fetch_file_list() {
  g_q_files.clear();
  g_q_entries.clear();
  g_q_lines.clear();
  g_q_entries_have_next_page = false;
  load_storage_regular_files(g_q_files);
  if (g_q_files.empty()) {
    g_q_lines.push_back("No storage files");
    return;
  }
  for (size_t i = 0; i < g_q_files.size(); ++i) {
    g_q_lines.push_back(g_q_files[i]);
  }
}

static std::string qso_trim_head(const std::string& in, size_t max_len) {
  if (in.size() <= max_len) return in;
  if (max_len == 0) return "";
  if (max_len == 1) return ">";
  return in.substr(0, max_len - 1) + ">";
}

static bool qso_parse_rst(const std::string& raw, int& out) {
  if (raw.empty()) return false;
  char* end = nullptr;
  long v = std::strtol(raw.c_str(), &end, 10);
  if (end == raw.c_str() || !end || *end != '\0') return false;
  if (v < -99) v = -99;
  if (v > 99) v = 99;
  out = static_cast<int>(v);
  return true;
}

static std::string qso_format_signed3(bool has_value, int value) {
  if (!has_value) return "-??";
  char out[4];
  std::snprintf(out, sizeof(out), "%+03d", value);
  return out;
}

static std::string qso_format_sent4(bool has_value, int value) {
  if (!has_value) return "S-??";
  char out[5];
  std::snprintf(out, sizeof(out), "S%+03d", value);
  return out;
}

static void qso_rebuild_entry_lines() {
  g_q_lines.clear();
  for (const auto& e : g_q_entries) {
    std::string call_field = qso_trim_head(e.call, 11);
    if (call_field.size() < 11) {
      call_field.append(11 - call_field.size(), ' ');
    }

    if (g_q_page_view == QPageView::Alternate) {
      const std::string rcvd = qso_format_signed3(e.has_rst_rcvd, e.rst_rcvd);
      const std::string sent = qso_format_sent4(e.has_rst_sent, e.rst_sent);
      g_q_lines.push_back(call_field + rcvd + " " + sent);
    } else {
      const std::string band_disp = qso_trim_head(e.band, 6);
      g_q_lines.push_back(e.time_on + " " + band_disp + " " + call_field);
    }
  }

  if (g_q_lines.empty()) {
    g_q_lines.push_back("No QSOs");
  }
}

static void qso_load_entries(const std::string& path) {
  g_q_entries.clear();
  g_q_lines.clear();
  g_q_entries_have_next_page = false;
  StorageStream* stream = storage_stream_open(path, StorageOpenMode::READ);
  if (!stream) {
    g_q_lines.push_back("Open fail");
    return;
  }
  const int first_qso = std::max(0, q_page) * 6;
  int qso_index = 0;
  int page_count = 0;
  char line[512];
  while (storage_stream_read_line(stream, line, sizeof(line))) {
    std::string s(line);
    std::string s_lower = s;
    std::transform(s_lower.begin(), s_lower.end(), s_lower.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (s_lower.find("<call:") == std::string::npos) continue;
    if (qso_index++ < first_qso) continue;
    if (page_count >= 6) {
      g_q_entries_have_next_page = true;
      break;
    }
    auto get_field = [&](const std::string& tag)->std::string {
      size_t p = s_lower.find("<" + tag);
      if (p == std::string::npos) return "";
      size_t gt = s.find('>', p);
      if (gt == std::string::npos) return "";
      size_t end_space = s.find(' ', gt + 1);
      size_t end_tag = s.find('<', gt + 1);
      size_t end = s.size();
      if (end_space != std::string::npos && end_space < end) end = end_space;
      if (end_tag != std::string::npos && end_tag < end) end = end_tag;
      return s.substr(gt + 1, end - gt - 1);
    };
    std::string call = get_field("call:");
    std::string time_on = get_field("time_on:");
    std::string freq = get_field("freq:");
    std::string rst_rcvd_raw = get_field("rst_rcvd:");
    std::string rst_sent_raw = get_field("rst_sent:");
    std::string band = freq;
    if (!freq.empty()) {
      // crude map: take MHz and map to band name from our band list
      double mhz = atof(freq.c_str());
      for (const auto& b : g_bands) {
        double bm = b.freq * 0.001;
        if (fabs(bm - mhz) < 0.1) { band = b.name; break; }
      }
    }
    if (time_on.size() >= 4) {
      time_on = time_on.substr(0,4);
      time_on.insert(2, ":");
    }
    if (time_on.size() != 5) time_on = "??:??";
    if (call.empty()) call = "?";
    if (band.empty()) band = freq.empty() ? "?" : freq;

    QsoLogEntry e;
    e.time_on = time_on;
    e.band = band;
    e.call = call;
    e.has_rst_rcvd = qso_parse_rst(rst_rcvd_raw, e.rst_rcvd);
    e.has_rst_sent = qso_parse_rst(rst_sent_raw, e.rst_sent);
    g_q_entries.push_back(e);
    page_count++;
  }
  storage_stream_close(stream);
  qso_rebuild_entry_lines();
}

static void qso_draw_page() {
  if (g_q_show_entries) {
    // Entry view: render raw QSO lines without "1..6 " prefixes.
    ui_draw_debug(g_q_lines, 0);
  } else {
    // File list view: keep numbered selection rows.
    ui_draw_list(g_q_lines, q_page, -1);
  }
}

static bool log_adif_entry(const std::string& dxcall, const std::string& dxgrid, int rst_sent, int rst_rcvd) {
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);
  char date[16];
  int year = t.tm_year + 1900;
  int month = t.tm_mon + 1;
  int day = t.tm_mday;
  snprintf(date, sizeof(date), "%04d%02d%02d", year % 10000, month % 100, day % 100);
  char path[64];
  snprintf(path, sizeof(path), "%s.txt", date);

  char time_on[16];
  int hour = t.tm_hour;
  int min = t.tm_min;
  int sec = t.tm_sec;
  snprintf(time_on, sizeof(time_on), "%02d%02d%02d", hour % 100, min % 100, sec % 100);
  double freq_mhz = 0.001 * (double)g_bands[g_band_sel].freq;
  char freq_str[16];
  snprintf(freq_str, sizeof(freq_str), "%.3f", freq_mhz);

  std::string comment_expanded = expand_comment_macros(g_comment1);
  const std::string my_grid4 = grid_ft8_4(g_grid);
  // Build rst_sent/rst_rcvd fragments — omit when -99 (no data),
  // matching DXFT8 reference behavior (ADIF.c omits when value is 0).
  char rst_sent_buf[32] = "";
  char rst_rcvd_buf[32] = "";
  if (rst_sent != -99) {
    snprintf(rst_sent_buf, sizeof(rst_sent_buf), "<rst_sent:%d>%d ",
             (int)snprintf(nullptr, 0, "%d", rst_sent), rst_sent);
  }
  if (rst_rcvd != -99) {
    snprintf(rst_rcvd_buf, sizeof(rst_rcvd_buf), "<rst_rcvd:%d>%d ",
             (int)snprintf(nullptr, 0, "%d", rst_rcvd), rst_rcvd);
  }
  const char* mode_name = g_protocol->name;
  char line[512];
  snprintf(line, sizeof(line),
           "<call:%zu>%s <gridsquare:%zu>%s <mode:%zu>%s<qso_date:8>%s <time_on:6>%s <freq:%zu>%s <station_callsign:%zu>%s <my_gridsquare:%zu>%s %s%s<comment:%zu>%s <eor>\n",
           dxcall.size(), dxcall.c_str(),
           dxgrid.size(), dxgrid.c_str(),
           strlen(mode_name), mode_name,
           date, time_on,
           strlen(freq_str), freq_str,
           g_call.size(), g_call.c_str(),
           my_grid4.size(), my_grid4.c_str(),
           rst_sent_buf, rst_rcvd_buf,
           comment_expanded.size(), comment_expanded.c_str());
  bool ok = storage_append_text_locked_path(path, line, "ADIF EXPORT\n<eoh>\n", true);
  if (!ok) ESP_LOGW(TAG, "ADIF write failed: %s", path);
  return ok;
}


static void ensure_usb() {
  if (usb_ready) return;
  usb_serial_jtag_driver_config_t cfg = {
    .tx_buffer_size = 1024,
    .rx_buffer_size = 4096,
  };
  if (usb_serial_jtag_driver_install(&cfg) == ESP_OK) {
    usb_ready = true;
  }
}

static bool uart_inject_last_was_cr = false;
static bool g_debug_uart_pins_enabled = false;
static int64_t s_uart_inject_mute_until_ms = 0;

static void drain_console_uart_rx_fifo() {
  uart_dev_t *hw = UART_LL_GET_HW(0);
  uint8_t scratch[64];
  while (uart_ll_get_rxfifo_len(hw) > 0) {
    uint32_t n = uart_ll_get_rxfifo_len(hw);
    if (n > sizeof(scratch)) n = sizeof(scratch);
    uart_ll_read_rxfifo(hw, scratch, n);
  }
  if (s_key_inject_queue) xQueueReset(s_key_inject_queue);
  uart_inject_last_was_cr = false;
}

static void poll_uart_inject_keys() {
  if (!s_key_inject_queue || !g_debug_uart_pins_enabled) return;
  if ((esp_timer_get_time() / 1000) < s_uart_inject_mute_until_ms) {
    drain_console_uart_rx_fifo();
    return;
  }
  // Read directly from the console UART FIFO — no driver needed.
  // sdkconfig configures ESP console on UART0 peripheral with custom
  // pins TX=G4, RX=G5 (see CONFIG_ESP_CONSOLE_UART_CUSTOM_NUM_0
  // and CONFIG_ESP_CONSOLE_UART_TX_GPIO / _RX_GPIO). KH1 CAT uses
  // UART1 peripheral on GPIO1 — no conflict.
  uart_dev_t *hw = UART_LL_GET_HW(0);
  while (true) {
    uint32_t avail = uart_ll_get_rxfifo_len(hw);
    if (avail == 0) break;
    if (avail > 64) avail = 64;
    uint8_t buf[64];
    uart_ll_read_rxfifo(hw, buf, avail);
    for (uint32_t i = 0; i < avail; i++) {
      char ch = (char)buf[i];
      // CR/LF handling: \r -> Enter, \n after \r -> skip (avoid double Enter)
      if (ch == '\r') {
        char enter = '\n';
        xQueueSend(s_key_inject_queue, &enter, 0);
        uart_inject_last_was_cr = true;
      } else if (ch == '\n' && uart_inject_last_was_cr) {
        uart_inject_last_was_cr = false;  // skip LF after CR
      } else {
        uart_inject_last_was_cr = false;
        xQueueSend(s_key_inject_queue, &ch, 0);
      }
    }
  }
}

static void host_write_str(const std::string& s) {
  ensure_usb();
  if (usb_ready) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(s.data());
    size_t remaining = s.size();
    while (remaining > 0) {
      size_t chunk = remaining;
      if (chunk > 256) chunk = 256;
      int written = usb_serial_jtag_write_bytes(p, chunk, portMAX_DELAY);
      if (written <= 0) break;
      p += written;
      remaining -= written;
    }
  }
}

// ================================================================
// UART screen mirror
//
// Debug aid for headless boards (e.g. StampS3Bat): every time a
// keystroke arrives over the console UART, dump the text that would
// have been displayed on the Cardputer LCD to the same UART TX, so
// a terminal shows the current page contents.
//
// To disable: comment out the `#define UART_SCREEN_MIRROR 1` below.
// ================================================================
#define UART_SCREEN_MIRROR 1

#if UART_SCREEN_MIRROR
static volatile bool g_uart_mirror_pending = false;

static const char* uart_mirror_mode_label(UIMode mode) {
  switch (mode) {
    case UIMode::RX:      return "RX";
    case UIMode::TX:      return "TX";
    case UIMode::BAND:    return "BAND";
    case UIMode::MENU:    return "MENU";
    case UIMode::MSC:     return "MSC";
    case UIMode::DEBUG:   return "DEBUG";
    case UIMode::STATUS:  return "STATUS";
    case UIMode::QSO:     return "QSO";
    case UIMode::GPS:     return "GPS";
    case UIMode::PERF:    return "PERF";
  }
  return "?";
}

static void uart_mirror_dump_screen() {
  std::vector<std::string> lines;
  ui_get_visible_text_lines(lines);

  // RX mode has proper paging info; other modes fall back to "page 1/1".
  int cur = 1, total = 1;
  if (ui_mode == UIMode::RX) {
    ui_get_rx_page_info(cur, total);
  }

  const char* label = uart_mirror_mode_label(ui_mode);
  printf("\n---- [%s %d/%d] ----\n", label, cur, total);
  for (size_t i = 0; i < lines.size(); ++i) {
    printf("%s\n", lines[i].c_str());
  }
  printf("--------------------\n");
  fflush(stdout);
}
#endif  // UART_SCREEN_MIRROR

static void set_gpio_floating_input(gpio_num_t pin) {
  gpio_reset_pin(pin);
  gpio_set_direction(pin, GPIO_MODE_INPUT);
  gpio_set_pull_mode(pin, GPIO_FLOATING);
  gpio_intr_disable(pin);
}

static void apply_debug_uart_pin_policy() {
  const bool enable = !g_gnss_lora_enabled;
  if (enable && g_debug_uart_pins_enabled) return;

  const gpio_num_t tx = (gpio_num_t)CONFIG_ESP_CONSOLE_UART_TX_GPIO;
  const gpio_num_t rx = (gpio_num_t)CONFIG_ESP_CONSOLE_UART_RX_GPIO;
  if (enable) {
    uart_set_pin(UART_NUM_0, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    gpio_set_pull_mode(rx, GPIO_PULLUP_ONLY);
    drain_console_uart_rx_fifo();
    s_uart_inject_mute_until_ms = esp_timer_get_time() / 1000 + 300;
    g_debug_uart_pins_enabled = true;
    ESP_LOGI(TAG, "G4/G5 debug UART enabled");
  } else {
    drain_console_uart_rx_fifo();
    s_uart_inject_mute_until_ms = esp_timer_get_time() / 1000 + 300;
#if UART_SCREEN_MIRROR
    g_uart_mirror_pending = false;
#endif
    set_gpio_floating_input(tx);
    set_gpio_floating_input(rx);
    const bool changed = g_debug_uart_pins_enabled;
    g_debug_uart_pins_enabled = false;
    if (changed) ESP_LOGI(TAG, "G4/G5 debug UART disabled for GNSS LoRa");
  }
}

static void restore_debug_uart_pins_after_sd() {
  // Force re-apply: SD mount briefly drives GPIO5 HIGH as LoRa-1262 CS.
  g_debug_uart_pins_enabled = false;
  apply_debug_uart_pin_policy();
}

struct WAVHeader {
  char riff[4];
  uint32_t file_size;
  char wave[4];
  char fmt[4];
  uint32_t fmt_size;
  uint16_t audio_format;
  uint16_t num_channels;
  uint32_t sample_rate;
  uint32_t byte_rate;
  uint16_t block_align;
  uint16_t bits_per_sample;
  char data[4];
  uint32_t data_size;
};

[[maybe_unused]] static esp_err_t decode_wav(const char* path) {
  ESP_LOGI(TAG, "Decoding %s", path);
  StorageStream* stream = storage_stream_open(path, StorageOpenMode::READ);
  if (!stream) {
    ESP_LOGE(TAG, "Failed to open %s", path);
    return ESP_FAIL;
  }

  WAVHeader hdr;
  if (storage_stream_read(stream, &hdr, sizeof(hdr)) != sizeof(hdr)) {
    ESP_LOGE(TAG, "Failed to read WAV header");
    storage_stream_close(stream);
    return ESP_FAIL;
  }
  if (memcmp(hdr.riff, "RIFF", 4) != 0 || memcmp(hdr.wave, "WAVE", 4) != 0) {
    ESP_LOGE(TAG, "Invalid WAV header");
    storage_stream_close(stream);
    return ESP_FAIL;
  }
  if (hdr.sample_rate != FT8_SAMPLE_RATE || hdr.num_channels != 1) {
    ESP_LOGE(TAG, "WAV must be mono %d Hz (got %u Hz, %u ch)", FT8_SAMPLE_RATE, hdr.sample_rate, hdr.num_channels);
    storage_stream_close(stream);
    return ESP_FAIL;
  }

  const int bytes_per_sample = hdr.bits_per_sample / 8;

  monitor_config_t mon_cfg;
  mon_cfg.f_min = 200.0f;
  mon_cfg.f_max = 2900.0f;
  mon_cfg.sample_rate = FT8_SAMPLE_RATE;
  mon_cfg.time_osr = g_time_osr;
  mon_cfg.freq_osr = g_freq_osr;
  mon_cfg.protocol = g_protocol->protocol_id;

  monitor_t mon;
  monitor_init(&mon, &mon_cfg);
  monitor_reset(&mon);

  float* chunk = (float*)malloc(sizeof(float) * mon.block_size);
  if (!chunk) {
    ESP_LOGE(TAG, "Chunk alloc failed");
    storage_stream_close(stream);
    monitor_free(&mon);
    return ESP_ERR_NO_MEM;
  }

  bool eof = false;
  while (!eof) {
    int read_samples = 0;
    while (read_samples < mon.block_size && !eof) {
      float sample_value = 0.0f;
      if (bytes_per_sample == 1) {
        uint8_t sample = 0;
        if (storage_stream_read(stream, &sample, 1) != 1) {
          eof = true;
          break;
        }
        int s = sample;
        sample_value = ((float)s - 128.0f) / 128.0f;
      } else if (bytes_per_sample == 2) {
        uint8_t sample[2] = {};
        if (storage_stream_read(stream, sample, sizeof(sample)) != sizeof(sample)) {
          eof = true;
          break;
        }
        int low = sample[0];
        int high = sample[1];
        int16_t s = (int16_t)((high << 8) | low);
        sample_value = (float)s / 32768.0f;
      } else {
        eof = true;
        break;
      }
      chunk[read_samples++] = sample_value;
    }
    if (read_samples == 0) break;
    for (int i = read_samples; i < mon.block_size; ++i) {
      chunk[i] = 0.0f;
    }

    // Simple per-block AGC to ~0.1 target level
    double acc = 0.0;
    for (int i = 0; i < mon.block_size; ++i) acc += fabsf(chunk[i]);
    float level = (float)(acc / mon.block_size);
    float gain = (level > 1e-6f) ? 0.1f / level : 1.0f;
    if (gain < 0.1f) gain = 0.1f;
    if (gain > 10.0f) gain = 10.0f;
    for (int i = 0; i < mon.block_size; ++i) {
      chunk[i] *= gain;
    }

    monitor_process(&mon, chunk);
  }

  free(chunk);
  storage_stream_close(stream);

  if (mon.wf.num_blocks == 0) {
    ESP_LOGW(TAG, "No audio blocks processed");
    monitor_free(&mon);
    return ESP_FAIL;
  }
  decode_monitor_results(&mon, &mon_cfg, false); // defer UI to main loop on core1
  monitor_free(&mon);

  return ESP_OK;
}

static void redraw_tx_view() {
  // Get QSO states from autoseq for display
  std::vector<std::string> qtext;
  autoseq_get_qso_states(qtext);

  std::vector<bool> marks(qtext.size(), false);  // No delete marks with autoseq
  std::vector<int> slots;

  // Slot color for pending TX
  slots.push_back(g_pending_tx_valid ? (g_pending_tx.slot_id & 1) : 0);
  // All QSO entries use their context's slot
  for (size_t i = 0; i < qtext.size(); ++i) {
    slots.push_back(0);  // Default to even; autoseq manages internally
  }

  std::string next_line;
  if (g_pending_tx_valid && !g_pending_tx.text.empty()) {
    // Use scheduled TX text if available
    next_line = g_pending_tx.text;
  } else {
    // Fall back to autoseq's next TX (for display when TX not yet scheduled)
    autoseq_get_next_tx(next_line);
  }

  ui_draw_tx(next_line, qtext, tx_page, -1, marks, slots);
}

static void draw_band_view() {
  std::vector<std::string> lines;
  lines.reserve(g_bands.size());
  for (size_t i = 0; i < g_bands.size(); ++i) {
    std::string freq_str;
    if ((int)i == band_edit_idx && !band_edit_buffer.empty()) {
      freq_str = band_edit_buffer;
    } else {
      char fbuf[16];
      float f = g_bands[i].freq;
      if (f == (int)f) snprintf(fbuf, sizeof(fbuf), "%d", (int)f);
      else             snprintf(fbuf, sizeof(fbuf), "%.1f", f);
      freq_str = fbuf;
    }
    lines.push_back(std::string(g_bands[i].name) + ": " + freq_str);
  }
  ui_draw_list(lines, band_page, band_edit_idx);
}

static const char* beacon_name(BeaconMode m) {
  switch (m) {
    case BeaconMode::OFF: return "OFF";
    case BeaconMode::EVEN: return "EVEN";
    //case BeaconMode::EVEN2: return "EVEN2";
    case BeaconMode::ODD: return "ODD";
    //case BeaconMode::ODD2: return "ODD2";
  }
  return "OFF";
}

static const char* cq_type_name(CqType t) {
  switch (t) {
    case CqType::CQ: return "CQ";
    case CqType::CQSOTA: return "CQ SOTA";
    case CqType::CQPOTA: return "CQ POTA";
    case CqType::CQQRP: return "CQ QRP";
    case CqType::CQFD: return "CQ FD";
    case CqType::CQFREETEXT: return "FreeText";
  }
  return "CQ";
}

static const char* offset_name(OffsetSrc o) {
  switch (o) {
    case OffsetSrc::RANDOM: return "Random";
    case OffsetSrc::CURSOR: return "Fixed";
    case OffsetSrc::RX: return "RX";
  }
  return "Random";
}

static RadioType canonical_radio_type(RadioType r) {
  if (r == RadioType::QDX ||
      r == RadioType::KH1_USBC || r == RadioType::KH1_MIC) return r;
  return RadioType::QMX;
}

static bool is_kh1_radio(RadioType r) {
  r = canonical_radio_type(r);
  return r == RadioType::KH1_USBC || r == RadioType::KH1_MIC;
}

static bool radio_type_uses_display_only(RadioType r) {
  // Always use display-only board init (upstream design): audio input is owned
  // exclusively by the selected backend (UAC for QMX/KH1-USBC, native I2S mic
  // for KH1-MIC), so general M5Unified startup must not claim speaker/mic/audio
  // resources. The keyboard still works because beginDisplayOnly() initializes
  // it via Keyboard.begin() (auto-detects board type) — see
  // components/M5Cardputer/src/M5Cardputer.cpp.
  (void)r;
  return true;
}

static RadioType radio_type_from_saved_int(int value) {
  switch (value) {
    case (int)RadioType::KH1_USBC:
      return RadioType::KH1_USBC;
    case (int)RadioType::KH1_MIC:
      return RadioType::KH1_MIC;
    case (int)RadioType::QDX:
      return RadioType::QDX;
    case (int)RadioType::QMX:
    default:
      return RadioType::QMX;
  }
}

static RadioType parse_radio_config_value(const char* raw) {
  if (!raw) return RadioType::QMX;

  char* end = nullptr;
  long as_int = strtol(raw, &end, 10);
  if (end != raw) {
    return radio_type_from_saved_int((int)as_int);
  }

  std::string token;
  for (const char* p = raw; *p; ++p) {
    unsigned char ch = (unsigned char)*p;
    if (ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t') continue;
    token.push_back((char)std::toupper(ch));
  }

  if (token == "KH1" || token == "KH1-USBC" || token == "KH1_USBC" || token == "KH1USB") {
    return RadioType::KH1_USBC;
  }
  if (token == "KH1-MIC" || token == "KH1_MIC" || token == "KH1MIC") {
    return RadioType::KH1_MIC;
  }
  if (token == "QDX") {
    return RadioType::QDX;
  }
  return RadioType::QMX;
}

static RadioProfileBinding get_radio_profile_binding(RadioType r) {
  switch (canonical_radio_type(r)) {
    case RadioType::KH1_USBC:
      return {AUDIO_SOURCE_USB_UAC_GENERIC, RADIO_CONTROL_KH1_CAT};
    case RadioType::KH1_MIC:
      return {AUDIO_SOURCE_KH1_MIC, RADIO_CONTROL_KH1_CAT};
    case RadioType::QDX:
      return {AUDIO_SOURCE_QMX_UAC, RADIO_CONTROL_QDX};
    case RadioType::QMX:
    default:
      return {AUDIO_SOURCE_QMX_UAC, RADIO_CONTROL_QMX};
  }
}

static const char* radio_name(RadioType r) {
  switch (canonical_radio_type(r)) {
    case RadioType::QMX: return "QMX";
    case RadioType::QDX: return "QDX";
    case RadioType::KH1_USBC: return "KH1-USBC";
    case RadioType::KH1_MIC: return "KH1-MIC";
    default: break;
  }
  return "None";
}

void apply_radio_profile_binding() {
  audio_source_backend_t prev_audio = audio_source_get_backend();
  g_radio = canonical_radio_type(g_radio);
  g_gps_baud = normalize_gps_baud_value(g_gps_baud);
  auto start_gps = [&]() {
    gps_start(gps_pins_for_current_source());
  };
  if (is_kh1_radio(g_radio)) {
    // KH1 CAT and PORTA GPS both use UART1 on G1/G2. GNSS LoRa uses UART2
    // on G15/G13, so it can keep running while KH1 is connected.
    if (g_kh1_connected) {
      if (g_gnss_lora_enabled) {
        start_gps();
      } else {
        gps_stop();
      }
      radio_control_kh1_set_enabled(true);
    } else {
      radio_control_kh1_set_enabled(false);
      start_gps();
    }
  } else {
    // Leaving KH1 releases UART1 back to PORTA GPS.
    g_kh1_connected = false;
    radio_control_kh1_set_enabled(false);
    start_gps();
  }
  RadioProfileBinding binding = get_radio_profile_binding(g_radio);
  audio_source_set_backend(binding.audio_backend);
  radio_control_set_backend(binding.radio_backend);
  if (audio_source_is_streaming() && prev_audio != binding.audio_backend) {
    ESP_LOGW(TAG, "Audio backend changed while streaming; stop/start audio to apply (%s -> %s)",
             audio_source_backend_name(prev_audio),
             audio_source_backend_name(binding.audio_backend));
  }
  ESP_LOGI(TAG, "Profile bind radio=%s audio=%s control=%s",
           radio_name(g_radio),
           audio_source_backend_name(binding.audio_backend),
           radio_control_backend_name(binding.radio_backend));
}

static bool notify_radio_control_audio_start_if_allowed(const char* reason) {
  if (is_kh1_radio(g_radio) && !g_kh1_connected) {
    ESP_LOGI(TAG, "Skip CAT audio start for %s: KH1 CAT/TX not connected",
             radio_name(g_radio));
    return false;
  }

  esp_err_t rc = radio_control_on_audio_start();
  const bool ok = (rc == ESP_OK);
  ESP_LOGI(TAG, "CAT audio start %s radio=%s reason=%s rc=%d",
           ok ? "ok" : "failed",
           radio_name(g_radio),
           reason ? reason : "",
           (int)rc);
  debug_log_line(ok ? "CAT audio ok" : "CAT audio fail");
  return ok;
}

static bool start_rx_audio_for_current_radio(const char* reason, bool notify_cat_if_allowed) {
  apply_radio_profile_binding();

  if (audio_source_is_streaming()) {
    ESP_LOGI(TAG, "RX audio already streaming radio=%s reason=%s",
             radio_name(g_radio),
             reason ? reason : "");
    if (notify_cat_if_allowed) {
      notify_radio_control_audio_start_if_allowed(reason);
    }
    return true;
  }

  const char* mode = radio_name(g_radio);
  const char* backend = audio_source_backend_name(audio_source_get_backend());
  ESP_LOGI(TAG, "RX audio start radio=%s backend=%s reason=%s",
           mode,
           backend,
           reason ? reason : "");
  debug_log_line(std::string("Audio start ") + mode);
  debug_log_line(std::string("Audio bind ") + backend);

  const bool is_uac_backend = (std::strstr(backend, "uac") != nullptr);
  if (is_uac_backend) log_mem_caps("UAC_BEFORE_START");
  if (!audio_source_start()) {
    if (is_uac_backend) log_mem_caps("UAC_AFTER_START");
    ESP_LOGW(TAG, "RX audio start failed radio=%s backend=%s reason=%s",
             mode,
             backend,
             reason ? reason : "");
    debug_log_line("Audio start fail");
    return false;
  }
  if (is_uac_backend) log_mem_caps("UAC_AFTER_START");

  debug_log_line("Audio start ok");
  g_decode_enabled = true;
  ui_set_paused(false);
  ui_clear_waterfall();

  if (notify_cat_if_allowed) {
    notify_radio_control_audio_start_if_allowed(reason);
  }
  return true;
}

static bool handle_kh1_diag_key(char c) {
  char key = (char)std::tolower((unsigned char)c);
  if (key != 'u' && key != 'i' && key != 'j' && key != 'k' && key != 'l') {
    return false;
  }

  if (!is_kh1_radio(g_radio) || !radio_control_kh1_is_enabled() || !radio_control_ready()) {
    ESP_LOGW(TAG, "KH1 CAT diagnostic %c skipped: not ready", key);
    debug_log_line("KH1 CAT not ready");
    return true;
  }

  int freq_hz = (int)(g_bands[g_band_sel].freq * 1000.0f);
  int rx_fa = (freq_hz + 5) / 10;
  int tx_fa = rx_fa + ((g_offset_hz + 5) / 10);

  char seq[128];
  switch (key) {
    case 'u':
      snprintf(seq, sizeof(seq), "u FA%07d if changed; wait; repeat;", tx_fa);
      break;
    case 'i':
      snprintf(seq, sizeof(seq), "i HK1; wait; HK0;");
      break;
    case 'j':
      snprintf(seq, sizeof(seq), "j FA%07d if changed; HK1; wait; HK0; FA%07d if changed;", tx_fa, rx_fa);
      break;
    case 'k':
      snprintf(seq, sizeof(seq), "k FA%07d if changed; HK1; FO00; wait; HK0; FA%07d if changed; FO99;", tx_fa, rx_fa);
      break;
    case 'l':
      snprintf(seq, sizeof(seq), "l FA%07d if changed; HK1; 79xFO; HK0; FA%07d if changed; FO99;", tx_fa, rx_fa);
      break;
    default:
      return true;
  }

  ESP_LOGI(TAG, "KH1 diag %s", seq);
  debug_log_line(std::string("KH1 diag ") + seq);
  bool fa_sent = false;
  esp_err_t err = radio_control_kh1_diag_test(key, freq_hz, g_offset_hz, &fa_sent);
  if (err == ESP_OK) {
    if (key == 'u') {
      debug_log_line(fa_sent ? "KH1 diag FA sent" : "KH1 diag FA skipped");
    }
    debug_log_line("KH1 diag OK");
  } else {
    ESP_LOGW(TAG, "KH1 diagnostic %c failed: %s", key, esp_err_to_name(err));
    debug_log_line(std::string("KH1 diag fail ") + esp_err_to_name(err));
  }
  return true;
}

static std::string lat_lon_to_maidenhead8(double lat, double lon) {
  if (lon < -180.0 || lon > 180.0 || lat < -90.0 || lat > 90.0) return "";
  // Clamp exact upper edge so index math stays in range.
  if (lon >= 180.0) lon = 179.999999;
  if (lat >= 90.0) lat = 89.999999;

  lon += 180.0;
  lat += 90.0;

  int field_lon = (int)(lon / 20.0);
  int field_lat = (int)(lat / 10.0);
  lon -= field_lon * 20.0;
  lat -= field_lat * 10.0;

  int square_lon = (int)(lon / 2.0);
  int square_lat = (int)(lat / 1.0);
  lon -= square_lon * 2.0;
  lat -= square_lat * 1.0;

  const double sub_lon_w = 2.0 / 24.0;
  const double sub_lat_h = 1.0 / 24.0;
  int sub_lon = (int)(lon / sub_lon_w);
  int sub_lat = (int)(lat / sub_lat_h);
  lon -= sub_lon * sub_lon_w;
  lat -= sub_lat * sub_lat_h;

  const double ext_lon_w = sub_lon_w / 10.0;
  const double ext_lat_h = sub_lat_h / 10.0;
  int ext_lon = (int)(lon / ext_lon_w);
  int ext_lat = (int)(lat / ext_lat_h);

  field_lon = std::clamp(field_lon, 0, 17);
  field_lat = std::clamp(field_lat, 0, 17);
  square_lon = std::clamp(square_lon, 0, 9);
  square_lat = std::clamp(square_lat, 0, 9);
  sub_lon = std::clamp(sub_lon, 0, 23);
  sub_lat = std::clamp(sub_lat, 0, 23);
  ext_lon = std::clamp(ext_lon, 0, 9);
  ext_lat = std::clamp(ext_lat, 0, 9);

  std::string out = "AA00aa00";
  out[0] = (char)('A' + field_lon);
  out[1] = (char)('A' + field_lat);
  out[2] = (char)('0' + square_lon);
  out[3] = (char)('0' + square_lat);
  out[4] = (char)('a' + sub_lon);
  out[5] = (char)('a' + sub_lat);
  out[6] = (char)('0' + ext_lon);
  out[7] = (char)('0' + ext_lat);
  return out;
}

static void draw_gps_view(bool force_redraw = false);

static void gps_runtime_tick() {
  static int64_t s_last_apply_ms = 0;
  static bool s_time_synced_once = false;
  static bool s_gps_grid_logged = false;
  static int s_last_time_sync_hour_key = -1;

  if (is_kh1_radio(g_radio) && g_kh1_connected && !g_gnss_lora_enabled) return;

  gps_tick();

  int detected_baud = 0;
  if (gps_take_baud_update(&detected_baud)) {
    detected_baud = normalize_gps_baud_value(detected_baud);
    if (detected_baud != g_gps_baud) {
      g_gps_baud = detected_baud;
      save_station_data();
      ESP_LOGI(TAG, "GPS baud persisted: %d", g_gps_baud);
    }
  }

  const int64_t now = rtc_now_ms();
  if ((now - s_last_apply_ms) < 1000) return;
  s_last_apply_ms = now;

  if (ui_mode == UIMode::GPS) {
    draw_gps_view();
  }

  gps_state_t st = gps_get_state();
  if (!st.valid_fix) return;

  bool changed = false;
  if (!st.grid_square.empty() && st.grid_square != "    ") {
    const std::string gps_grid = normalize_grid_maidenhead(st.grid_square);
    if (!gps_grid.empty()) {
      const std::string grid8 = lat_lon_to_maidenhead8(st.latitude, st.longitude);
      if (!grid8.empty()) {
        g_grid_gps_display8 = grid8;
      }
      g_grid_from_gps = true;
      if (gps_grid != g_grid) {
        g_grid = gps_grid;
        autoseq_set_station(g_call, grid_ft8_4(g_grid));
        changed = true;
        ESP_LOGI(TAG, "GPS grid synced: %s", g_grid.c_str());
      }
    }
  }

  if (!st.date_utc.empty() && !st.time_utc.empty()) {
    int y = 0, M = 0, d = 0;
    int h = 0, m = 0, s = 0;
    const bool parsed_date = (sscanf(st.date_utc.c_str(), "%d-%d-%d", &y, &M, &d) == 3);
    const bool parsed_time = (sscanf(st.time_utc.c_str(), "%d:%d:%d", &h, &m, &s) == 3);
    int hour_key = -1;
    if (parsed_date && parsed_time) {
      hour_key = (((y * 100) + M) * 100 + d) * 100 + h;
    }

    bool do_time_sync = !s_time_synced_once;
    if (!do_time_sync && parsed_time && !g_tx_active && !g_decode_in_progress) {
      if (m == 0 && s <= 5 && hour_key >= 0 && hour_key != s_last_time_sync_hour_key) {
        do_time_sync = true;
      }
    }

    if (do_time_sync) {
      const std::string old_date = g_date;
      const std::string old_time = g_time;
      g_date = st.date_utc;
      g_time = st.time_utc;
      if (rtc_set_from_strings_source(RtcTimeSource::GPS)) {
        rtc_sync_to_esp_rtc();
        (void)rtc_write_external_from_soft("GPS");
        s_time_synced_once = true;
        g_time_synced_from_gps = true;
        if (hour_key >= 0) s_last_time_sync_hour_key = hour_key;
        changed = true;
        ESP_LOGI(TAG, "GPS time synced: %s %s", g_date.c_str(), g_time.c_str());
        radio_control_set_time(h, m, s);
      } else {
        g_date = old_date;
        g_time = old_time;
      }
    }
  }

  // One session breadcrumb is enough to preserve the GPS grid even if no QSO
  // completes; retry later if logging is disabled or the file write fails.
  if (!s_gps_grid_logged &&
      g_time_synced_from_gps &&
      g_grid_from_gps &&
      g_grid_gps_display8.size() == 8) {
    s_gps_grid_logged = log_gps_grid_line(g_grid_gps_display8);
  }

  if (changed) {
    save_station_data();
  }
}

static std::string expand_comment_macros(const std::string& src) {
  std::string out = src;
  auto repl = [](std::string& s, const std::string& from, const std::string& to) {
    size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
      s.replace(pos, from.size(), to);
      pos += to.size();
    }
  };
  repl(out, "/Radio", radio_name(g_radio));

  const std::string grid_macro =
      (g_time_synced_from_gps && g_grid_from_gps && g_grid_gps_display8.size() == 8)
          ? g_grid_gps_display8
          : g_grid;
  repl(out, "/Grid", grid_macro);
  return out;
}

static std::string expand_comment1() {
  return expand_comment_macros(g_comment1);
}

void rebuild_ignore_prefixes() {
  g_ignore_prefixes.clear();
  std::istringstream iss(g_ignore_prefix_text);
  std::string tok;
  while (iss >> tok) {
    std::string norm = normalize_call_token(tok);
    if (norm.empty()) continue;
    bool duplicate = false;
    for (const auto& existing : g_ignore_prefixes) {
      if (existing == norm) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) g_ignore_prefixes.push_back(norm);
  }
}

void sync_ignore_prefix_text_from_list() {
  std::vector<std::string> normalized;
  for (const auto& prefix : g_ignore_prefixes) {
    std::string norm = normalize_call_token(prefix);
    if (norm.empty()) continue;
    if (std::find(normalized.begin(), normalized.end(), norm) == normalized.end()) {
      normalized.push_back(norm);
    }
  }

  std::string text;
  for (const auto& prefix : normalized) {
    const size_t needed = text.size() + (text.empty() ? 0 : 1) + prefix.size();
    if (needed > kIgnorePrefixTextMaxLen) break;
    if (!text.empty()) text.push_back(' ');
    text += prefix;
  }
  g_ignore_prefix_text = text;
  rebuild_ignore_prefixes();
}

static bool ignorelist_matches_normalized_dxcall(const std::string& dxcall_norm) {
  if (dxcall_norm.empty()) return false;
  for (const auto& prefix : g_ignore_prefixes) {
    if (!prefix.empty() && dxcall_norm.rfind(prefix, 0) == 0) return true;
  }
  return false;
}

static std::string clamp_ignore_prefix_text(const std::string& s) {
  if (s.size() <= kIgnorePrefixTextMaxLen) return s;
  return s.substr(0, kIgnorePrefixTextMaxLen);
}

static std::string normalize_time_hms(const std::string& src) {
  int h = 0, m = 0, s = 0;
  if (sscanf(src.c_str(), "%d:%d:%d", &h, &m, &s) == 3) {
    if (h >= 0 && h <= 23 && m >= 0 && m <= 59 && s >= 0 && s <= 59) {
      char out[16];
      snprintf(out, sizeof(out), "%02d:%02d:%02d", h, m, s);
      return out;
    }
  }

  std::string digits;
  digits.reserve(src.size());
  for (unsigned char ch : src) {
    if (std::isdigit(ch)) digits.push_back((char)ch);
  }
  if (digits.size() >= 6) {
    h = (digits[0] - '0') * 10 + (digits[1] - '0');
    m = (digits[2] - '0') * 10 + (digits[3] - '0');
    s = (digits[4] - '0') * 10 + (digits[5] - '0');
    if (h >= 0 && h <= 23 && m >= 0 && m <= 59 && s >= 0 && s <= 59) {
      char out[16];
      snprintf(out, sizeof(out), "%02d:%02d:%02d", h, m, s);
      return out;
    }
  }
  return src;
}

static int normalize_gps_baud_value(int value) {
  return (value == 9600 || value == 115200) ? value : 115200;
}

static gps_pins_t gps_pins_for_current_source() {
  gps_pins_t pins = {};
  if (g_gnss_lora_enabled) {
    pins.uart = UART_NUM_2;
    pins.rx = GPIO_NUM_15;
    pins.tx = GPIO_NUM_13;
    pins.default_baud = 115200;
    pins.auto_baud = false;
  } else {
    pins.uart = UART_NUM_1;
    pins.rx = GPIO_NUM_1;
    pins.tx = GPIO_NUM_2;
    pins.default_baud = normalize_gps_baud_value(g_gps_baud);
    pins.auto_baud = true;
  }
  return pins;
}

static const char* gps_source_name() {
  return g_gnss_lora_enabled ? "GNSS_LoRa" : "PORTA";
}

static std::string normalize_date_ymd(const std::string& src) {
  auto date_in_range = [](int y, int M, int d) -> bool {
    return (y >= 2024 && y <= 2099 && M >= 1 && M <= 12 && d >= 1 && d <= 31);
  };

  int y = 0, M = 0, d = 0;
  if (sscanf(src.c_str(), "%d-%d-%d", &y, &M, &d) == 3 && date_in_range(y, M, d)) {
    char out[16];
    snprintf(out, sizeof(out), "%04d-%02d-%02d", y, M, d);
    return out;
  }

  std::string digits;
  digits.reserve(src.size());
  for (unsigned char ch : src) {
    if (std::isdigit(ch)) digits.push_back((char)ch);
  }
  if (digits.size() >= 8) {
    y = (digits[0] - '0') * 1000 + (digits[1] - '0') * 100 +
        (digits[2] - '0') * 10 + (digits[3] - '0');
    M = (digits[4] - '0') * 10 + (digits[5] - '0');
    d = (digits[6] - '0') * 10 + (digits[7] - '0');
    if (date_in_range(y, M, d)) {
      char out[16];
      snprintf(out, sizeof(out), "%04d-%02d-%02d", y, M, d);
      return out;
    }
  }

  return "";
}

static std::string normalize_grid_maidenhead(const std::string& src) {
  size_t b = 0;
  size_t e = src.size();
  while (b < e && std::isspace(static_cast<unsigned char>(src[b]))) ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(src[e - 1]))) --e;

  const size_t n = e - b;
  if (n != 4 && n != 6 && n != 8) return "";

  std::string out = src.substr(b, n);
  auto is_digit_char = [](char ch) { return ch >= '0' && ch <= '9'; };
  auto to_upper = [](char ch) { return static_cast<char>(std::toupper(static_cast<unsigned char>(ch))); };
  auto to_lower = [](char ch) { return static_cast<char>(std::tolower(static_cast<unsigned char>(ch))); };

  char c0 = to_upper(out[0]);
  char c1 = to_upper(out[1]);
  if (c0 < 'A' || c0 > 'R' || c1 < 'A' || c1 > 'R') return "";
  if (!is_digit_char(out[2]) || !is_digit_char(out[3])) return "";
  out[0] = c0;
  out[1] = c1;

  if (n >= 6) {
    char c4 = to_upper(out[4]);
    char c5 = to_upper(out[5]);
    if (c4 < 'A' || c4 > 'X' || c5 < 'A' || c5 > 'X') return "";
    out[4] = to_lower(c4);
    out[5] = to_lower(c5);
  }

  if (n == 8) {
    if (!is_digit_char(out[6]) || !is_digit_char(out[7])) return "";
  }

  return out;
}

std::string grid_ft8_4(const std::string& grid) {
  const std::string norm = normalize_grid_maidenhead(grid);
  if (norm.size() >= 4) return norm.substr(0, 4);
  return "CM97";
}

bool set_manual_grid_config(const std::string& grid) {
  const std::string norm_grid = normalize_grid_maidenhead(grid);
  if (norm_grid.empty()) return false;
  g_grid = norm_grid;
  g_grid_saved_manual = g_grid;
  g_grid_from_gps = false;
  g_grid_gps_display8.clear();
  return true;
}

static std::string menu_sleep_batt_line() {
  board_power_status_t ps = {};
  char buf[48];

  if (board_power_read(&ps) == ESP_OK && ps.valid) {
    snprintf(buf, sizeof(buf), "Sleep/Batt %d%%", ps.percent);
  } else {
    snprintf(buf, sizeof(buf), "Sleep/Batt --");
  }

  return std::string(buf);
}

static std::string elide_right(const std::string& s, size_t max_len = 22) {
  if (s.size() <= max_len) return s;
  if (max_len <= 3) return s.substr(s.size() - max_len);
  return std::string("...") + s.substr(s.size() - (max_len - 3));
}

static std::string head_trim(const std::string& s, size_t max_len = 16) {
  if (s.size() <= max_len) return s;
  if (max_len == 0) return "";
  if (max_len == 1) return ">";
  return s.substr(0, max_len - 1) + ">";
}

static std::string highlight_pos(const std::string& s, int pos) {
  if (pos < 0 || pos >= (int)s.size()) return s;
  std::string out;
  out.reserve(s.size() + 2);
  out.append(s, 0, pos);
  out.push_back('[');
  out.push_back(s[pos]);
  out.push_back(']');
  out.append(s, pos + 1, std::string::npos);
  return out;
}

static void draw_status_view();

static const char* rtc_time_source_suffix() {
  switch (g_rtc_time_source) {
    case RtcTimeSource::DS3231: return " R";
    case RtcTimeSource::GPS: return " G";
    case RtcTimeSource::SAVED:
    case RtcTimeSource::ESP_RTC:
    case RtcTimeSource::MANUAL:
    default:
      return "";
  }
}

static void rtc_update_strings_from_epoch(time_t now) {
  struct tm t;
  localtime_r(&now, &t);
  char buf_date[32];
  snprintf(buf_date, sizeof(buf_date), "%04d-%02d-%02d",
           t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
  g_date = buf_date;
  char buf_time[16];
  snprintf(buf_time, sizeof(buf_time), "%02d:%02d:%02d",
           t.tm_hour, t.tm_min, t.tm_sec);
  g_time = buf_time;
}

static time_t rtc_current_epoch_seconds() {
  if (!rtc_valid) {
    return (time_t)(esp_timer_get_time() / 1000000);
  }
  return rtc_epoch_base + (esp_timer_get_time() / 1000 - rtc_ms_start) / 1000;
}

static void rtc_seed_epoch(time_t epoch, int64_t ms_start, RtcTimeSource source) {
  rtc_epoch_base = epoch;
  rtc_ms_start = ms_start;
  rtc_last_update = ms_start;
  rtc_valid = true;
  g_rtc_time_source = source;
  rtc_update_strings_from_epoch(epoch);
}

static bool rtc_set_external_datetime_strings(const external_rtc_datetime_t& datetime) {
  char buf_date[32];
  char buf_time[16];
  snprintf(buf_date, sizeof(buf_date), "%04u-%02u-%02u",
           (unsigned)datetime.year,
           (unsigned)datetime.month,
           (unsigned)datetime.day);
  snprintf(buf_time, sizeof(buf_time), "%02u:%02u:%02u",
           (unsigned)datetime.hour,
           (unsigned)datetime.minute,
           (unsigned)datetime.second);
  g_date = buf_date;
  g_time = buf_time;
  return true;
}

static external_rtc_datetime_t rtc_external_datetime_from_soft() {
  time_t now = rtc_current_epoch_seconds();
  struct tm t;
  localtime_r(&now, &t);

  external_rtc_datetime_t datetime = {};
  datetime.year = (uint16_t)(t.tm_year + 1900);
  datetime.month = (uint8_t)(t.tm_mon + 1);
  datetime.day = (uint8_t)t.tm_mday;
  datetime.hour = (uint8_t)t.tm_hour;
  datetime.minute = (uint8_t)t.tm_min;
  datetime.second = (uint8_t)t.tm_sec;
  return datetime;
}

static bool rtc_set_from_strings_source(RtcTimeSource source) {
  int y, M, d, h, m, s;
  if (sscanf(g_date.c_str(), "%d-%d-%d", &y, &M, &d) != 3) return false;
  if (sscanf(g_time.c_str(), "%d:%d:%d", &h, &m, &s) != 3) return false;
  struct tm t = {};
  t.tm_year = y - 1900;
  t.tm_mon = M - 1;
  t.tm_mday = d;
  t.tm_hour = h;
  t.tm_min = m;
  t.tm_sec = s;
  time_t epoch = mktime(&t);
  if (epoch == (time_t)-1) return false;
  rtc_seed_epoch(epoch, esp_timer_get_time() / 1000, source);
  return true;
}

bool rtc_set_from_strings() {
  return rtc_set_from_strings_source(RtcTimeSource::SAVED);
}

void rtc_sync_to_esp_rtc() {
  if (!rtc_valid) return;

  time_t now = rtc_current_epoch_seconds();
  struct timeval tv = { .tv_sec = now, .tv_usec = 0 };
  settimeofday(&tv, NULL);
  ESP_LOGI(TAG, "ESP RTC synced from soft RTC");
}

static esp_err_t rtc_write_external_from_soft(const char* reason) {
  if (!rtc_valid || !external_rtc_available()) {
    return ESP_ERR_INVALID_STATE;
  }

  external_rtc_datetime_t datetime = rtc_external_datetime_from_soft();
  esp_err_t err = external_rtc_write_datetime(&datetime);
  if (err == ESP_OK) {
    ESP_LOGI(TAG,
             "DS3231 time updated from %s: %04u-%02u-%02u %02u:%02u:%02u",
             reason ? reason : "soft RTC",
             (unsigned)datetime.year,
             (unsigned)datetime.month,
             (unsigned)datetime.day,
             (unsigned)datetime.hour,
             (unsigned)datetime.minute,
             (unsigned)datetime.second);
  } else {
    ESP_LOGW(TAG,
             "DS3231 time update from %s failed: %s",
             reason ? reason : "soft RTC",
             esp_err_to_name(err));
  }
  return err;
}

bool rtc_apply_manual_time_from_strings() {
  if (!rtc_set_from_strings_source(RtcTimeSource::MANUAL)) {
    return false;
  }

  g_time_synced_from_gps = false;
  rtc_sync_to_esp_rtc();
  if (rtc_write_external_from_soft("manual time") == ESP_OK) {
    g_rtc_time_source = RtcTimeSource::DS3231;
  }
  return true;
}

static bool rtc_init_from_ds3231() {
  esp_err_t err = external_rtc_init();
  if (err != ESP_OK) {
    return false;
  }

  external_rtc_datetime_t datetime = {};
  err = external_rtc_read_datetime(&datetime);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "DS3231 time not loaded; using ESP RTC or saved time: %s",
             esp_err_to_name(err));
    return false;
  }

  rtc_set_external_datetime_strings(datetime);
  if (!rtc_set_from_strings_source(RtcTimeSource::DS3231)) {
    ESP_LOGW(TAG, "DS3231 time parse failed; using ESP RTC or saved time");
    return false;
  }

  g_time_synced_from_gps = false;
  g_rtc_sleep_epoch = 0;
  rtc_sync_to_esp_rtc();
  ESP_LOGI(TAG,
           "DS3231 time loaded: %04u-%02u-%02u %02u:%02u:%02u",
           (unsigned)datetime.year,
           (unsigned)datetime.month,
           (unsigned)datetime.day,
           (unsigned)datetime.hour,
           (unsigned)datetime.minute,
           (unsigned)datetime.second);
  return true;
}

// Initialize soft RTC from ESP RTC (persists through deep sleep)
// Applies compensation if we have valid sleep epoch data
static bool rtc_init_from_esp_rtc() {
  struct timeval tv;
  if (gettimeofday(&tv, NULL) != 0) return false;

  // Check if ESP RTC has valid time (year > 2020)
  struct tm t;
  localtime_r(&tv.tv_sec, &t);
  if (t.tm_year + 1900 < 2020) return false;

  time_t compensated_now = tv.tv_sec;

  // Apply compensation if we have valid sleep data
  if (g_rtc_sleep_epoch > 0 && tv.tv_sec > g_rtc_sleep_epoch) {
    int64_t raw_elapsed = tv.tv_sec - g_rtc_sleep_epoch;
    int64_t actual_elapsed = raw_elapsed;

    // Apply compensation: actual = raw * 10000 / (10000 + comp)
    if (g_rtc_comp != 0) {
      actual_elapsed = raw_elapsed * 10000 / (10000 + g_rtc_comp);
    }

    // Fixed 1s boot delay: deep sleep entry → wake → gettimeofday
    static constexpr int64_t BOOT_DELAY_SEC = 1;
    compensated_now = g_rtc_sleep_epoch + actual_elapsed + BOOT_DELAY_SEC;

    ESP_LOGI(TAG, "RTC wake: raw_elapsed=%lld, actual_elapsed=%lld, comp=%d, boot_adj=%lld",
             (long long)raw_elapsed, (long long)actual_elapsed, g_rtc_comp,
             (long long)BOOT_DELAY_SEC);

    // Clear sleep epoch after use (one-time compensation)
    g_rtc_sleep_epoch = 0;
  }

  // Account for sub-second offset: tv.tv_usec tells us how far past the
  // whole second we are, so rewind rtc_ms_start by that amount.
  rtc_seed_epoch(compensated_now,
                 esp_timer_get_time() / 1000 - tv.tv_usec / 1000,
                 RtcTimeSource::ESP_RTC);

  g_time_synced_from_gps = false;
  ESP_LOGI(TAG, "ESP RTC initialized: %s %s (compensated=%s)",
           g_date.c_str(), g_time.c_str(),
           (g_rtc_comp != 0) ? "yes" : "no");
  return true;
}

static void rtc_update_strings() {
  if (!rtc_valid) return;
  rtc_update_strings_from_epoch(rtc_current_epoch_seconds());
}

int64_t rtc_now_ms() {
  if (!rtc_valid) {
    return esp_timer_get_time() / 1000;
  }
  return (int64_t)rtc_epoch_base * 1000 + (esp_timer_get_time() / 1000 - rtc_ms_start);
}

static void rtc_tick() {
  if (!rtc_valid) {
    rtc_set_from_strings();
    if (!rtc_valid) return;
  }
  int64_t now_ms = esp_timer_get_time() / 1000;
  if (now_ms - rtc_last_update >= 1000) {
    rtc_last_update += 1000; // Increment by interval to prevent drift accumulation
    if (status_edit_idx != 5) { // keep time ticking unless editing time
      std::string old_date = g_date;
      std::string old_time = g_time;
      rtc_update_strings();
      if (ui_mode == UIMode::STATUS && status_edit_idx == -1) {
        if (old_date != g_date) {
          draw_status_line(4, std::string("Date: ") + g_date, false);
        }
        if (old_time != g_time) {
          draw_status_line(5, std::string("Time: ") + g_time + rtc_time_source_suffix(), false);
        }
      }
    }
  }
}

// Push current in-memory band to the radio, ensuring it's in RX mode.
// Called from: QMX first-connect path (consume_cdc_initial_sync), STATUS
// exit (enter_mode), and S->3 band-change key handler. Guards:
//   - radio_control_ready(): CAT link must be up
//   - !g_tx_active: never interrupt an ongoing transmission
// Returns true on success. Callers can use the return value to decide
// whether to clear a deferred-sync flag.
// The `reason` string is logged for debugging.
bool sync_radio_to_current_band(const char* reason) {
  if (!radio_control_ready()) return false;
  if (g_tx_active) return false;
  int freq_hz = (int)(g_bands[g_band_sel].freq * 1000.0f);
  radio_control_end_tx();  // ensure RX mode (idempotent)
  esp_err_t rc = radio_control_sync_frequency_mode(freq_hz);
  if (rc == ESP_OK) {
    ESP_LOGI(TAG, "CAT sync ok (%s) freq=%d", reason ? reason : "", freq_hz);
    std::string msg = std::string("CAT sync: ") + (reason ? reason : "");
    debug_log_line(msg);
    return true;
  }
  ESP_LOGW(TAG, "CAT sync failed (%s) rc=%d", reason ? reason : "", (int)rc);
  return false;
}

// Consume the "CDC initial sync pending" flag set by stream_uac_task after
// a successful QMX CDC-ACM open. Runs the same sync sequence as the manual
// STATUS->2 button (put radio in RX + push current band to VFO), so users
// don't have to press anything after plugging in QMX. Called from the main
// loop every iteration (before early-exit branches). Fires at most once
// per CDC open — cleared on successful sync, retries on later iterations
// until CAT becomes ready and we're not TXing. For KH1 (UART CAT, no USB
// enumeration event), this flag is never set; the STATUS-exit auto-sync
// and S->3 handler cover KH1.
static void consume_cdc_initial_sync() {
  if (!g_cdc_initial_sync_pending) return;
  if (sync_radio_to_current_band("initial QMX connect")) {
    g_cdc_initial_sync_pending = false;
  }
}

static void update_countdown() {
  int64_t now_ms = rtc_now_ms();
  const int slot_period = g_protocol->slot_time_ms;
  int64_t slot_idx = now_ms / slot_period;
  int64_t slot_ms = now_ms % slot_period;
  static int64_t last_slot_idx = -1;
  static int last_sec = -1;
  int sec = (int)(slot_ms / 1000);
  if (slot_idx != last_slot_idx || sec != last_sec) {
    float frac = (float)slot_ms / (float)slot_period;
    bool even = (slot_idx % 2) == 0;
    ui_draw_countdown(frac, even, g_offset_hz);
    last_slot_idx = slot_idx;
    last_sec = sec;
  }
}

static void redraw_countdown_now() {
  int64_t now_ms = rtc_now_ms();
  const int slot_period = g_protocol->slot_time_ms;
  int64_t slot_idx = now_ms / slot_period;
  int64_t slot_ms = now_ms % slot_period;
  float frac = (float)slot_ms / (float)slot_period;
  bool even = (slot_idx % 2) == 0;
  ui_draw_countdown(frac, even, g_offset_hz);
}

// Forward declarations for single-threaded TX state machine
static void tx_start(int skip_tones);
static void tx_tick();

// Slot boundary check - called from main loop
// Matches reference project: tick after TX slot ends, TX trigger at slot start
// Compute the actual audio offset the next TX will use, given the
// configured g_offset_src and the autoseq pending entry. Storing the
// resolved value at scheduling time (rather than at the slot boundary,
// as the firmware used to do) means UI consumers reading core_get_qso see
// the same number that will actually go on air — important for the
// waterfall offset marker, especially in RANDOM / beacon-CQ modes where
// the random was previously rolled inside check_slot_boundary.
static int resolve_tx_offset(const AutoseqTxEntry& e) {
  if (g_offset_src == OffsetSrc::CURSOR) {
    return g_offset_hz;
  }
  if (g_offset_src == OffsetSrc::RX &&
      e.offset_hz > 0 &&
      e.text.rfind("CQ ", 0) != 0) {
    return e.offset_hz;
  }
  // RANDOM, or RX mode + CQ: roll a fresh offset in [500, 2500] Hz.
  return 500 + (int)(esp_random() % 2001);
}

// Single point of truth for arming the next TX. Replaces the 4-line
// "g_qso_xmit / g_target_slot_parity / g_pending_tx / g_pending_tx_valid"
// block that used to be repeated at every scheduling site (autoseq tick,
// beacon-on, free-text queue, and RX selection).
void arm_pending_tx(const AutoseqTxEntry& pending) {
  g_qso_xmit           = true;
  g_target_slot_parity = pending.slot_id & 1;
  g_pending_tx         = pending;
  g_pending_tx.offset_hz = resolve_tx_offset(g_pending_tx);
  g_pending_tx_valid   = true;
}

static void check_slot_boundary() {
  int64_t now_ms = rtc_now_ms();
  const int slot_period = g_protocol->slot_time_ms;
  int64_t slot_idx = now_ms / slot_period;
  int slot_ms = (int)(now_ms % slot_period);
  int slot_parity = (int)(slot_idx & 1);

  // Detect slot boundary (parity change)
  if (slot_parity != g_last_slot_parity) {
    g_last_slot_parity = slot_parity;
  }

  // Call tick AFTER TX has completed (not while TX is still active)
  // This ensures autoseq_tick() operates on the correct completed TX entry
  if (g_was_txing && !g_tx_active) {
    ESP_LOGI(TAG, "TX completed, calling tick (slot %lld, parity %d)",
             (long long)slot_idx, slot_parity);
    autoseq_tick(slot_idx, slot_parity, 0);
    g_was_txing = false;
    core_fire_qso_changed();  // propagates to all registered consumers
  }

  // TX trigger: check if we should start TX in this slot
  // Conditions: qso_xmit flag set, correct parity, early enough in slot, not already TXing,
  // and decode must be complete (TX is always triggered by decode results).
  // Additional guard (g_decode_applied_slot_idx): enforces that decode for the
  // previous RX slot (slot_idx - 1) has been fully applied to autoseq state before
  // we fire TX. Without this, a slot boundary that arrives before audio capture
  // has completed (FT8: ~12.6s of 15s slot; FT4: ~5.0s of 7.5s slot) could fire
  // TX based on a prior cycle's state. See AUTOSEQ_INACTIVE_QUEUE.md.
  // Window = 4/15 of slot_time_ms (~26.7%): FT8=4000ms, FT4=2000ms.
  if (g_qso_xmit &&
      g_target_slot_parity == slot_parity &&
      slot_ms < (g_protocol->slot_time_ms * 4 / 15) &&
      !g_tx_active &&
      !g_decode_in_progress &&
      g_decode_applied_slot_idx >= slot_idx - 1) {

    ESP_LOGI(TAG, "TX trigger: starting TX in slot %lld (parity %d)",
             (long long)slot_idx, slot_parity);

    // Calculate skip_tones for partial slot
    const int sym_ms = (int)roundf(g_protocol->symbol_period * 1000.0f);
    int skip_tones = slot_ms / sym_ms;
    if (skip_tones < g_protocol->total_symbols) {
      // Only proceed if we have a valid pending TX
      // NOTE: Don't clear g_qso_xmit until we're sure g_pending_tx is valid.
      // This avoids a race condition where decode_monitor_results is still
      // writing g_pending_tx on core 1 while we read it on core 0.
      if (g_pending_tx_valid && !g_pending_tx.text.empty()) {
        g_qso_xmit = false;  // Clear flag only AFTER validation succeeds
        g_was_txing = true;  // Set IMMEDIATELY when TX starts (prevents decode_monitor_results from re-setting flags)

        // Offset was resolved at scheduling time by arm_pending_tx, so
        // g_pending_tx.offset_hz is already what's going on air.
        log_rxtx_line('T', 0, g_pending_tx.offset_hz, g_pending_tx.text,
                      g_pending_tx.repeat_counter);
        tx_start(skip_tones);
      }
    }
  }
}

  static void menu_flash_tick() {
    if (menu_flash_idx < 0) return;
    int64_t now = rtc_now_ms();
    if (now >= menu_flash_deadline) {
      menu_flash_idx = -1;
      if (ui_mode == UIMode::MENU && !menu_long_edit && menu_edit_idx < 0) {
        draw_menu_view();
      }
  }
}

static void rx_flash_tick() {
  if (rx_flash_idx < 0) return;
  int64_t now = rtc_now_ms();
  if (now >= rx_flash_deadline) {
    rx_flash_idx = -1;
    rx_flash_deadline = 0;
    if (ui_mode == UIMode::RX) {
      ui_draw_rx();
    }
  }
}

static void apply_pending_sync() {}

static int band_number_from_name(const std::string& name) {
  int num = 0;
  for (char c : name) {
    if (c >= '0' && c <= '9') {
      num = num * 10 + (c - '0');
    } else {
      break;
    }
  }
  return num;
}

void rebuild_active_bands() {
  std::string cleaned = g_active_band_text;
  for (char& c : cleaned) {
    if (c == ',' || c == '/' || c == '\\' || c == ';') c = ' ';
    if (c == 'm' || c == 'M') c = ' ';
  }
  std::istringstream iss(cleaned);
  std::vector<int> bands;
  int v;
  while (iss >> v) {
    if (v <= 0) continue;
    // match to g_bands by number prefix
    for (size_t i = 0; i < g_bands.size(); ++i) {
      if (band_number_from_name(g_bands[i].name) == v) {
        if (std::find(bands.begin(), bands.end(), (int)i) == bands.end()) {
          bands.push_back((int)i);
        }
        break;
      }
    }
  }
  if (bands.empty()) {
    bands.resize(g_bands.size());
    for (size_t i = 0; i < g_bands.size(); ++i) bands[i] = (int)i;
  }
  g_active_band_indices = bands;
  if (std::find(g_active_band_indices.begin(), g_active_band_indices.end(), g_band_sel) == g_active_band_indices.end()) {
    g_band_sel = g_active_band_indices[0];
  }
  // normalize text
  std::ostringstream oss;
  for (size_t i = 0; i < g_active_band_indices.size(); ++i) {
    if (i) oss << ' ';
    oss << band_number_from_name(g_bands[g_active_band_indices[i]].name);
  }
  g_active_band_text = oss.str();
}

void update_autoseq_cq_type() {
  AutoseqCqType t = AutoseqCqType::CQ;
  switch (g_cq_type) {
    case CqType::CQSOTA: t = AutoseqCqType::SOTA; break;
    case CqType::CQPOTA: t = AutoseqCqType::POTA; break;
    case CqType::CQQRP:  t = AutoseqCqType::QRP;  break;
    case CqType::CQFD:   t = AutoseqCqType::FD;   break;
    case CqType::CQFREETEXT: t = AutoseqCqType::FREETEXT; break;
    default: t = AutoseqCqType::CQ; break;
  }
  const std::string& ft =
    (g_cq_type == CqType::CQFREETEXT || g_cq_type == CqType::CQFD) ? g_free_text : g_cq_freetext;
  autoseq_set_cq_type(t, ft);
}

static void advance_active_band(int delta) {
  if (g_active_band_indices.empty()) rebuild_active_bands();
  if (g_active_band_indices.empty()) return;
  int pos = 0;
  for (size_t i = 0; i < g_active_band_indices.size(); ++i) {
    if (g_active_band_indices[i] == g_band_sel) { pos = (int)i; break; }
  }
  int n = (int)g_active_band_indices.size();
  pos = (pos + delta + n) % n;
  g_band_sel = g_active_band_indices[pos];
}

static int tx_waterfall_hz_to_x(float tone_hz) {
  constexpr int kScreenW = 240;
  constexpr float kMinHz = 200.0f;
  constexpr float kMaxHz = 3000.0f;
  int x = (int)lrintf((tone_hz - kMinHz) * (float)(kScreenW - 1) / (kMaxHz - kMinHz));
  if (x < 0) x = 0;
  if (x >= kScreenW) x = kScreenW - 1;
  return x;
}

static void tx_waterfall_set_max(std::array<uint8_t, 240>& row, int x, uint8_t value) {
  if (x < 0 || x >= (int)row.size()) return;
  if (row[(size_t)x] < value) row[(size_t)x] = value;
}

static void fft_waterfall_tx_tone(float tone_hz) {
  std::array<uint8_t, 240> row{};
  static uint8_t noise_phase = 0;
  for (size_t i = 0; i < row.size(); ++i) {
    row[i] = (uint8_t)(2 + ((i * 17 + noise_phase) & 0x03));
  }
  noise_phase += 29;

  const int pos = tx_waterfall_hz_to_x(tone_hz);
  tx_waterfall_set_max(row, pos - 2, 50);
  tx_waterfall_set_max(row, pos - 1, 120);
  tx_waterfall_set_max(row, pos, 230);
  tx_waterfall_set_max(row, pos + 1, 120);
  tx_waterfall_set_max(row, pos + 2, 50);
  ui_push_tx_waterfall_row(row.data(), (int)row.size());
}

[[maybe_unused]] static bool is_grid4(const std::string& s) {
  if (s.size() != 4) return false;
  auto is_letter = [](char c){ return c >= 'A' && c <= 'R'; };
  auto is_digitc = [](char c){ return c >= '0' && c <= '9'; };
  return is_letter(toupper((unsigned char)s[0])) &&
         is_letter(toupper((unsigned char)s[1])) &&
         is_digitc(s[2]) &&
         is_digitc(s[3]);
}

[[maybe_unused]] static int parse_report_snr(const std::string& f3) {
  if (f3.empty()) return -99;
  std::string s = f3;
  if (!s.empty() && (s[0] == 'R' || s[0] == 'r')) {
    s = s.substr(1);
  }
  if (s.empty()) return -99;
  bool neg = false;
  size_t idx = 0;
  if (s[0] == '+' || s[0] == '-') {
    neg = (s[0] == '-');
    idx = 1;
  }
  int val = 0;
  bool found = false;
  for (; idx < s.size(); ++idx) {
    char c = s[idx];
    if (c < '0' || c > '9') break;
    val = val * 10 + (c - '0');
    found = true;
    if (val > 99) break;
  }
  if (!found) return -99;
  if (neg) val = -val;
  return val;
}

// ---- Static decode workspace (zero heap allocation) ----
// Use the shared RxDecodeEntry type from ui.h so we can hand it directly
// to ui_set_rx_list_static without any conversion.
#define DEC_MAX       RX_MAX_DECODES       // 32
#define DEC_TEXT_MAX  RX_TEXT_MAX          // 64
#define DEC_FIELD_MAX RX_FIELD_MAX         // 20
typedef RxDecodeEntry DecodeMsg;

static DecodeMsg s_dec[DEC_MAX];
static int       s_dec_count;

// Plain-C field parser: tokenize text into field1/field2/field3.
// Equivalent to the old fill_fields_from_text lambda but uses no heap.
static void dec_fill_fields(DecodeMsg* d) {
  d->field1[0] = d->field2[0] = d->field3[0] = '\0';
  char tmp[DEC_TEXT_MAX];
  strncpy(tmp, d->text, sizeof(tmp));
  tmp[sizeof(tmp) - 1] = '\0';

  char* saveptr = nullptr;
  char* toks[8];
  int ntoks = 0;
  for (char* p = strtok_r(tmp, " ", &saveptr); p && ntoks < 8; p = strtok_r(nullptr, " ", &saveptr)) {
    toks[ntoks++] = p;
  }
  if (ntoks == 0) return;

  // Helpers
  auto all_digits = [](const char* s, int len) {
    for (int i = 0; i < len; ++i) if (s[i] < '0' || s[i] > '9') return false;
    return true;
  };
  auto all_alpha = [](const char* s, int len) {
    for (int i = 0; i < len; ++i) {
      char c = s[i];
      if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))) return false;
    }
    return true;
  };

  // CQ <short_token> CALL GRID pattern
  if (strcmp(toks[0], "CQ") == 0 && ntoks >= 2) {
    int len1 = (int)strlen(toks[1]);
    bool short_tok = (len1 <= 3 && all_digits(toks[1], len1)) ||
                     (len1 <= 4 && all_alpha(toks[1], len1));
    if (short_tok) {
      strncpy(d->field1, toks[1], DEC_FIELD_MAX - 1); d->field1[DEC_FIELD_MAX - 1] = '\0';
      if (ntoks > 2) { strncpy(d->field2, toks[2], DEC_FIELD_MAX - 1); d->field2[DEC_FIELD_MAX - 1] = '\0'; }
      if (ntoks > 3) {
        d->field3[0] = '\0';
        for (int i = 3; i < ntoks; ++i) {
          if (i > 3) strncat(d->field3, " ", DEC_FIELD_MAX - strlen(d->field3) - 1);
          strncat(d->field3, toks[i], DEC_FIELD_MAX - strlen(d->field3) - 1);
        }
      }
      return;
    }
  }

  // Default: first 2 tokens + remainder
  strncpy(d->field1, toks[0], DEC_FIELD_MAX - 1); d->field1[DEC_FIELD_MAX - 1] = '\0';
  if (ntoks > 1) { strncpy(d->field2, toks[1], DEC_FIELD_MAX - 1); d->field2[DEC_FIELD_MAX - 1] = '\0'; }
  if (ntoks > 2) {
    d->field3[0] = '\0';
    for (int i = 2; i < ntoks; ++i) {
      if (i > 2) strncat(d->field3, " ", DEC_FIELD_MAX - strlen(d->field3) - 1);
      strncat(d->field3, toks[i], DEC_FIELD_MAX - strlen(d->field3) - 1);
    }
  }
}

// Plain-C normalize: strip <>, uppercase, write into out[out_sz].
static void dec_normalize_call(const char* src, char* out, int out_sz) {
  const char* p = src;
  if (*p == '<') ++p;
  int len = (int)strlen(p);
  if (len > 0 && p[len - 1] == '>') --len;
  if (len >= out_sz) len = out_sz - 1;
  for (int i = 0; i < len; ++i) out[i] = (char)toupper((unsigned char)p[i]);
  out[len] = '\0';
}

// Sort comparator: to_me first (0), then CQ (1), then others (2)
static int dec_sort_cmp(const void* a, const void* b) {
  const DecodeMsg* da = (const DecodeMsg*)a;
  const DecodeMsg* db = (const DecodeMsg*)b;
  int ga = da->is_to_me ? 0 : (da->is_cq ? 1 : 2);
  int gb = db->is_to_me ? 0 : (db->is_cq ? 1 : 2);
  if (ga != gb) return ga - gb;
  // CQ block only: strongest first.
  if (ga == 1 && da->snr != db->snr) {
    return db->snr - da->snr;
  }
  return 0;
}

void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui) {
  // ---- heap instrumentation ----
  size_t heap_entry = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t heap_entry_largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  UBaseType_t stack_hw_entry = uxTaskGetStackHighWaterMark(NULL);
  ESP_LOGW(TAG, "DECODE_HEAP ENTER: free=%u largest=%u alltime_min=%u stack_hw=%u",
           (unsigned)heap_entry, (unsigned)heap_entry_largest,
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
           (unsigned)stack_hw_entry);

  s_dec_count = 0;

  const int max_cand = 50;
  static ftx_candidate_t candidates[max_cand];
  int num_candidates = ftx_find_candidates(&mon->wf, max_cand, candidates, 5);
  ESP_LOGI(TAG, "Candidates found: %d", num_candidates);

  // ---- slot index + once-per-slot hashtable maintenance ----
  int64_t slot_idx = (g_decode_slot_idx >= 0) ? g_decode_slot_idx : rtc_now_ms() / g_protocol->slot_time_ms;
  int slot_id = (int)(slot_idx & 1);

  static int64_t s_last_aged_slot = -1;
  if (slot_idx != s_last_aged_slot) {
    s_last_aged_slot = slot_idx;
    hashtable_age_all();
  }

  // ---- estimate noise floor ----
  float noise_db = -120.0f;
  if (mon->wf.mag && mon->wf.num_blocks > 0) {
    const size_t total = (size_t)mon->wf.num_blocks * (size_t)mon->wf.block_stride;
    static uint32_t hist[256];
    memset(hist, 0, sizeof(hist));
    for (size_t i = 0; i < total; ++i) hist[mon->wf.mag[i]]++;
    uint64_t target = total * 25 / 100;
    uint64_t accum = 0;
    int noise_scaled = 0;
    for (int v = 0; v < 256; ++v) {
      accum += hist[v];
      if (accum >= target) { noise_scaled = v; break; }
    }
    noise_db = 0.5f * ((float)noise_scaled - 240.0f);
  }

  // ---- mycall uppercase (stack, not heap) ----
  char mycall_up[DEC_FIELD_MAX];
  {
    const char* src = g_call.c_str();
    int len = (int)g_call.size();
    if (len >= DEC_FIELD_MAX) len = DEC_FIELD_MAX - 1;
    for (int i = 0; i < len; ++i) mycall_up[i] = (char)toupper((unsigned char)src[i]);
    mycall_up[len] = '\0';
  }

  // ---- decode candidates into static s_dec[] ----
  const int kMaxDecoded = 50;
  static ftx_message_t decoded[kMaxDecoded];
  static ftx_message_t* decoded_hashtable[kMaxDecoded];
  for (int i = 0; i < kMaxDecoded; ++i) decoded_hashtable[i] = nullptr;
  int num_decoded = 0;

  if (num_candidates <= 0) {
    ESP_LOGW(TAG, "No candidates found");
    ui_set_rx_list_static(nullptr, 0);
    if (update_ui) { ui_draw_rx(); }
    else core_fire_rx_changed();
    // No candidates means we processed the slot's audio but found nothing —
    // still counts as "applied" for the TX-trigger guard.
    if (g_decode_slot_idx > g_decode_applied_slot_idx) {
      g_decode_applied_slot_idx = g_decode_slot_idx;
    }
    g_decode_in_progress = false;
    return;
  }

  for (int i = 0; i < num_candidates && s_dec_count < DEC_MAX; ++i) {
    ftx_message_t message;
    ftx_decode_status_t status;
    memset(&message, 0, sizeof(message));
    memset(&status, 0, sizeof(status));

    if (!ftx_decode_candidate(&mon->wf, &candidates[i], 25, &message, &status))
      continue;

    // payload/hash dedupe (open addressing)
    int idx_hash = (int)(message.hash % kMaxDecoded);
    bool found_empty = false, found_dup = false;
    for (int probe = 0; probe < kMaxDecoded; ++probe) {
      ftx_message_t* p = decoded_hashtable[idx_hash];
      if (!p) { found_empty = true; break; }
      if (p->hash == message.hash &&
          0 == memcmp(p->payload, message.payload, sizeof(message.payload))) {
        found_dup = true; break;
      }
      idx_hash = (idx_hash + 1) % kMaxDecoded;
    }
    if (found_dup || !found_empty) continue;

    memcpy(&decoded[idx_hash], &message, sizeof(message));
    decoded_hashtable[idx_hash] = &decoded[idx_hash];
    ++num_decoded;

    // decode to human text
    char text[FTX_MAX_MESSAGE_LENGTH] = {0};
    ftx_message_offsets_t offsets;
    ftx_message_rc_t urc = ftx_message_decode(&message, &hash_if, text, &offsets);
    if (urc != FTX_MESSAGE_RC_OK || text[0] == '\0') continue;

    // freq / time / SNR
    float freq_hz = (mon->min_bin + candidates[i].freq_offset +
                    candidates[i].freq_sub / (float)cfg->freq_osr) / mon->symbol_period;
    float time_s = (candidates[i].time_offset +
                   candidates[i].time_sub / (float)cfg->time_osr) * mon->symbol_period;

    float cand_db = noise_db;
    {
      int t_index = candidates[i].time_offset * mon->wf.time_osr + candidates[i].time_sub;
      const int t_count = mon->wf.num_blocks * mon->wf.time_osr;
      if (t_count > 0) { if (t_index < 0) t_index = 0; if (t_index >= t_count) t_index = t_count - 1; }
      else t_index = 0;

      int f_index = candidates[i].freq_sub * mon->wf.num_bins + candidates[i].freq_offset;
      const int f_count = mon->wf.freq_osr * mon->wf.num_bins;
      if (f_count > 0) { if (f_index < 0) f_index = 0; if (f_index >= f_count) f_index = f_count - 1; }
      else f_index = 0;

      size_t offset2 = (size_t)t_index * (size_t)f_count + (size_t)f_index;
      size_t total2 = (size_t)mon->wf.num_blocks * (size_t)mon->wf.block_stride;
      if (mon->wf.mag && offset2 < total2) cand_db = 0.5f * ((float)mon->wf.mag[offset2] - 240.0f);
    }

    int snr_q = (int)lrintf(cand_db - noise_db);
    if (snr_q < -30) snr_q = -30;
    if (snr_q >  99) snr_q = 99;

    // DXpedition rewrite (uses heap briefly via std::string — bounded, rare path)
    char final_text[DEC_TEXT_MAX];
    {
      std::string raw(text);
      std::string rewritten(text);
      if (rewrite_dxpedition_for_mycall(raw, mycall_up, rewritten)) {
        ESP_LOGI(TAG, "DXpedition raw match: %s", text);
      }
      strncpy(final_text, rewritten.c_str(), DEC_TEXT_MAX - 1);
      final_text[DEC_TEXT_MAX - 1] = '\0';
    }

    // UI text dedup (linear scan — 32 entries max, no hash map needed)
    int dup_idx = -1;
    for (int j = 0; j < s_dec_count; ++j) {
      if (strcmp(s_dec[j].text, final_text) == 0) { dup_idx = j; break; }
    }
    if (dup_idx >= 0) {
      if (snr_q > s_dec[dup_idx].snr) {
        s_dec[dup_idx].snr = snr_q;
        s_dec[dup_idx].offset_hz = (int)lrintf(freq_hz);
        s_dec[dup_idx].slot_id = slot_id;
      }
      continue;
    }

    ESP_LOGI(TAG, "Decoded[%d] t=%.2fs f=%.1fHz snr=%d : %s",
             s_dec_count, time_s, freq_hz, snr_q, final_text);

    // Fill static entry
    DecodeMsg* d = &s_dec[s_dec_count];
    strncpy(d->text, final_text, DEC_TEXT_MAX - 1); d->text[DEC_TEXT_MAX - 1] = '\0';
    d->snr = snr_q;
    d->offset_hz = (int)lrintf(freq_hz);
    d->slot_id = slot_id;
    d->time_s = time_s;

    dec_fill_fields(d);

    d->is_cq = (strncmp(d->text, "CQ ", 3) == 0 || strcmp(d->text, "CQ") == 0);

    char f1_norm[DEC_FIELD_MAX];
    dec_normalize_call(d->field1, f1_norm, DEC_FIELD_MAX);
    d->is_to_me = (mycall_up[0] != '\0' && strcmp(f1_norm, mycall_up) == 0);

    log_rxtx_line('R', snr_q, (int)lrintf(freq_hz), std::string(final_text), -1);

    s_dec_count++;
  }

  ESP_LOGI(TAG, "Decoded %d unique messages", s_dec_count);

  // ---- Auto sync soft RTC from decode timing ----
  if (s_dec_count > 3) {
    // Simple insertion sort to find median of time_s values
    float sorted_t[DEC_MAX];
    int nt = 0;
    for (int i = 0; i < s_dec_count; ++i) sorted_t[nt++] = s_dec[i].time_s;
    for (int i = 1; i < nt; ++i) {
      float key = sorted_t[i];
      int j = i - 1;
      while (j >= 0 && sorted_t[j] > key) { sorted_t[j + 1] = sorted_t[j]; --j; }
      sorted_t[j + 1] = key;
    }
    float median = sorted_t[nt / 2];
    if (fabsf(median) > 0.3f) {
      int delta_ms = (int)lrintf(-median * 1000.0f);
      if (delta_ms > 320) delta_ms = 320;
      if (delta_ms < -320) delta_ms = -320;
      rtc_ms_start -= delta_ms;
      rtc_last_update -= delta_ms;
      rtc_update_strings();
      rtc_sync_to_esp_rtc();
      ESP_LOGI("SYNC", "Applied RTC sync: median=%.2fs delta=%dms", median, delta_ms);
    }
  }

  // ---- Sort in-place: to_me first, CQ second, others last ----
  qsort(s_dec, s_dec_count, sizeof(DecodeMsg), dec_sort_cmp);

  // ---- Autoseq: build small to_me vector at boundary (only to_me entries) ----
  if (!g_was_txing) {
    std::vector<UiRxLine> to_me_auto;
    for (int i = 0; i < s_dec_count; ++i) {
      if (!s_dec[i].is_to_me) break;  // sorted, so once we pass to_me we're done
      char dxnorm[DEC_FIELD_MAX];
      dec_normalize_call(s_dec[i].field2, dxnorm, DEC_FIELD_MAX);
      if (ignorelist_matches_normalized_dxcall(std::string(dxnorm))) {
        ESP_LOGI(TAG, "IgnoreList: skip auto reply to %s", dxnorm);
        continue;
      }
      UiRxLine rx;
      rx.text = s_dec[i].text;
      rx.field1 = s_dec[i].field1;
      rx.field2 = s_dec[i].field2;
      rx.field3 = s_dec[i].field3;
      rx.snr = s_dec[i].snr;
      rx.offset_hz = s_dec[i].offset_hz;
      rx.slot_id = s_dec[i].slot_id;
      rx.is_cq = s_dec[i].is_cq;
      rx.is_to_me = true;
      to_me_auto.push_back(std::move(rx));
    }

    if (!to_me_auto.empty()) {
      autoseq_on_decodes(to_me_auto);
      core_fire_qso_changed();  // propagates to all registered consumers
      g_last_reply_text = to_me_auto.front().text;
    }

    AutoseqTxEntry pending;
    if (autoseq_fetch_pending_tx(pending)) {
      arm_pending_tx(pending);
      ESP_LOGI(TAG, "TX ready: %s parity=%d", pending.text.c_str(), g_target_slot_parity);
    } else if (g_beacon != BeaconMode::OFF) {
      enqueue_beacon_cq();
      if (autoseq_fetch_pending_tx(pending)) {
        arm_pending_tx(pending);
        ESP_LOGI(TAG, "Beacon CQ ready: %s parity=%d", pending.text.c_str(), g_target_slot_parity);
      }
    }
  }

  // ---- Zero-heap handoff: static s_dec[] → ui.cpp's static rx_lines[] ----
  ui_set_rx_list_static(s_dec, s_dec_count);

  if (update_ui) {
    ui_draw_rx();
    char buf[64];
    snprintf(buf, sizeof(buf), "Heap %u", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    debug_log_line(buf);
  } else {
    core_fire_rx_changed();
  }

  // ---- heap instrumentation (exit) ----
  {
    size_t heap_exit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t heap_exit_largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    UBaseType_t stack_hw_exit = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGW(TAG, "DECODE_HEAP EXIT: free=%u largest=%u alltime_min=%u stack_hw=%u (delta_free=%d)",
             (unsigned)heap_exit, (unsigned)heap_exit_largest,
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
             (unsigned)stack_hw_exit,
             (int)heap_exit - (int)heap_entry);
  }

  // Mark this slot's decode as fully applied BEFORE clearing the in-progress
  // flag. Readers (TX trigger on core 0) must see the applied marker as soon
  // as in_progress drops, not later.
  if (g_decode_slot_idx > g_decode_applied_slot_idx) {
    g_decode_applied_slot_idx = g_decode_slot_idx;
  }
  g_decode_in_progress = false;
}

static void draw_menu_long_edit() {
  std::vector<std::string> lines(6, "");
  std::string text = menu_long_buf;
  size_t idx = 0;
  int line = 0;
  while (idx < text.size() && line < 6) {
    size_t chunk = std::min<size_t>(18, text.size() - idx);
    lines[line] = text.substr(idx, chunk);
    idx += chunk;
    line++;
  }
  // cursor indicator on the last line
  if (line == 0) {
    lines[0] = "_";
  } else {
    if (lines[line - 1].size() < 20) lines[line - 1].push_back('_');
    else if (line < 6) lines[line] = "_";
  }
  ui_draw_debug(lines, 0);
}

static void log_tones(const uint8_t* tones, size_t n) {
  std::string line;
  for (size_t i = 0; i < n; ++i) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%u", (unsigned)tones[i]);
    line += buf;
    if ((i + 1) % 20 == 0 || i + 1 == n) {
      debug_log_line(line);
      line.clear();
    }
  }
}

static void encode_and_log_pending_tx() {
  if (!g_pending_tx_valid || g_pending_tx.text.empty()) {
    debug_log_line("No pending TX to encode");
    return;
  }
  ftx_message_t msg;
  ftx_message_rc_t rc = ftx_message_encode(&msg, &hash_if, g_pending_tx.text.c_str());
  if (rc != FTX_MESSAGE_RC_OK) {
    debug_log_line("Encode failed");
    return;
  }
  uint8_t tones[FT4_NN] = {0};
  if (g_protocol->protocol_id == FTX_PROTOCOL_FT4) {
    ft4_encode(msg.payload, tones);
  } else {
    ft8_encode(msg.payload, tones);
  }
  debug_log_line(std::string("Tones for '") + g_pending_tx.text + "'");
  log_tones(tones, g_protocol->total_symbols);
}

[[maybe_unused]] static bool looks_like_grid(const std::string& s) {
  if (s.size() != 4) return false;
  return std::isalpha((unsigned char)s[0]) && std::isalpha((unsigned char)s[1]) &&
         std::isdigit((unsigned char)s[2]) && std::isdigit((unsigned char)s[3]);
}

[[maybe_unused]] static bool looks_like_report(const std::string& s, int& out) {
  if (s.empty()) return false;
  int sign = 1;
  size_t idx = 0;
  if (s[0] == '-') { sign = -1; idx = 1; }
  else if (s[0] == '+') { idx = 1; }
  if (idx >= s.size()) return false;
  int val = 0;
  for (; idx < s.size(); ++idx) {
    if (!std::isdigit((unsigned char)s[idx])) return false;
    val = val * 10 + (s[idx] - '0');
  }
  out = sign * val;
  return true;
}

// Enqueue a beacon CQ. Parity is determined by beacon mode.
// Duplicate prevention is handled by autoseq_start_cq().
// TX trigger happens at slot boundary via check_slot_boundary().
static void enqueue_beacon_cq() {
  int target_parity = (g_beacon == BeaconMode::EVEN) ? 0 : 1;
  autoseq_start_cq(target_parity);
  core_fire_qso_changed();  // propagates to all registered consumers
}

static bool autoseq_has_pending_tx() {
  AutoseqTxEntry tmp;
  return autoseq_fetch_pending_tx(tmp);
}

// Schedule a one-off pending TX (e.g., manual FreeText) without touching autoseq state.
// Returns false if TX is already active or if scheduling failed.
// Uses the single-threaded state machine - TX will trigger at next matching slot boundary.
static bool schedule_manual_pending_tx(const AutoseqTxEntry& pending) {
  // Already transmitting or TX pending?
  if (g_tx_active || g_qso_xmit) {
    return false;
  }

  arm_pending_tx(pending);
  ESP_LOGI(TAG, "schedule_manual_pending_tx: queued TX=%s for parity=%d",
           pending.text.c_str(), g_target_slot_parity);
  return true;
}

// NOTE: This function is now mostly superseded by the state machine approach.
// TX scheduling is done via g_qso_xmit and g_target_slot_parity flags,
// and check_slot_boundary() triggers TX at the right time.
// Keeping this as a no-op for now in case any code still calls it.
[[maybe_unused]] static void schedule_tx_if_idle() {
  // No-op: TX scheduling is now handled by decode_monitor_results setting
  // g_qso_xmit and check_slot_boundary triggering TX at slot start.
}

// Helper to send TA command (deduplicated)
static void tx_send_ta(float tone_hz) {
  int ta_int = (int)lrintf(tone_hz);
  float frac = tone_hz - (float)ta_int;
  int ta_frac = (int)lrintf(frac * 100.0f);
  if (ta_int == g_tx_last_ta_int && ta_frac == g_tx_last_ta_frac) return;
  if (radio_control_set_tone_hz(tone_hz) == ESP_OK) {
    g_tx_last_ta_int = ta_int;
    g_tx_last_ta_frac = ta_frac;
  }
}

// Start TX (single-threaded state machine initialization)
// Called from check_slot_boundary at the right time
// Uses g_pending_tx which was prepared by check_slot_boundary with correct offset
static void tx_start(int skip_tones) {
  // Already transmitting?
  if (g_tx_active) {
    return;
  }

  // Use g_pending_tx which was prepared by check_slot_boundary
  if (!g_pending_tx_valid || g_pending_tx.text.empty()) {
    ESP_LOGW(TAG, "tx_start: no pending TX");
    return;
  }

  // Get current slot info
  int64_t now_ms = rtc_now_ms();
  const int slot_period = g_protocol->slot_time_ms;
  g_tx_slot_idx = now_ms / slot_period;

  ESP_LOGI(TAG, "tx_start: TX=%s offset=%d skip=%d slot=%lld proto=%s",
           g_pending_tx.text.c_str(), g_pending_tx.offset_hz, skip_tones, (long long)g_tx_slot_idx,
           g_protocol->name);

  // Notify autoseq that TX emission is starting. This is the single canonical
  // logging trigger — if we're about to emit TX4 (RR73) or TX5 (73), autoseq
  // writes the ADIF entry now.
  autoseq_on_tx_starting();

  // Encode message to tones
  ftx_message_t msg;
  ftx_message_rc_t rc = ftx_message_encode(&msg, &hash_if, g_pending_tx.text.c_str());
  if (rc != FTX_MESSAGE_RC_OK) {
    ESP_LOGE(TAG, "Encode failed for TX");
    return;
  }
  if (g_protocol->protocol_id == FTX_PROTOCOL_FT4) {
    ft4_encode(msg.payload, g_tx_tones);
  } else {
    ft8_encode(msg.payload, g_tx_tones);
  }

  // Set up TX state machine
  // IMPORTANT: Tone timing must be based on slot boundary, not TX start time.
  // This ensures TX ends at the correct time even if TX started late,
  // allowing RX to start cleanly at the next slot boundary.
  g_tx_base_hz = g_pending_tx.offset_hz;
  g_tx_slot_start_ms = (now_ms / slot_period) * slot_period;  // Slot boundary time
  g_tx_tone_idx = (skip_tones >= g_protocol->total_symbols) ? g_protocol->total_symbols : skip_tones;
  // Next tone time = slot_start + tone_idx * symbol_period_ms
  // Aligns all tones to the slot boundary, not to when TX started
  const int sym_ms = (int)roundf(g_protocol->symbol_period * 1000.0f);
  g_tx_next_tone_time = g_tx_slot_start_ms + g_tx_tone_idx * sym_ms;
  g_tx_last_ta_int = -1;
  g_tx_last_ta_frac = -1;

  ESP_LOGI(TAG, "TX base_hz=%d (from pre-computed offset, text=%s)", g_tx_base_hz, g_pending_tx.text.c_str());

  // Send CAT setup commands
  g_tx_cat_ok = radio_control_ready();
  if (g_tx_cat_ok) {
    int freq_hz = (int)(g_bands[g_band_sel].freq * 1000.0f);
    esp_err_t err = radio_control_begin_tx(freq_hz, g_tx_base_hz);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "tx_start: radio TX begin failed: %s", esp_err_to_name(err));
      g_tx_cat_ok = false;
    }
  }

  // QDX uses sample-counted UAC OUT. QMX and KH1 retain their existing
  // per-symbol CAT paths.
  if (g_tx_cat_ok && canonical_radio_type(g_radio) == RadioType::QDX) {
    const int remaining_tones = g_protocol->total_symbols - g_tx_tone_idx;
    if (remaining_tones <= 0 ||
        !uac_tx_begin_cpfsk(static_cast<float>(g_tx_base_hz),
                            g_tx_tones + g_tx_tone_idx,
                            static_cast<size_t>(remaining_tones),
                            g_protocol->tone_spacing,
                            g_protocol->samples_per_symbol)) {
      ESP_LOGW(TAG, "tx_start: QDX UAC OUT start failed");
      radio_control_end_tx();
      g_tx_cat_ok = false;
      return;
    }
  }

  if (skip_tones > 0) {
    ESP_LOGI("TXTONE", "Skipping first %d tones due to late start", skip_tones);
  }

  // Send first tone TA if CAT is ready
  if (g_tx_cat_ok && g_tx_tone_idx < g_protocol->total_symbols) {
    float tone_hz = g_tx_base_hz + g_protocol->tone_spacing * g_tx_tones[g_tx_tone_idx];
    tx_send_ta(tone_hz);
  }

  // Mark TX as active
  ui_set_rx_waterfall_muted(true);
  g_tx_active = true;
}

// TX state machine tick - called from main loop
// Sends one tone at a time, non-blocking
static void tx_tick() {
  if (!g_tx_active) {
    return;
  }

  int64_t now_ms = rtc_now_ms();

  // Check for cancel request
  if (g_tx_cancel_requested) {
    ESP_LOGI(TAG, "tx_tick: TX cancelled at tone %d", g_tx_tone_idx);
    if (g_tx_cat_ok) {
      radio_control_end_tx();
    }
    ui_set_rx_waterfall_muted(false);
    g_tx_active = false;
    g_pending_tx_valid = false;
    g_tx_cancel_requested = false;
    g_was_txing = false;  // TX was cancelled - don't call tick at slot boundary
    core_fire_qso_changed();  // propagates to all registered consumers
    return;
  }

  // Time to send next tone?
  if (now_ms < g_tx_next_tone_time) {
    return;  // Not yet
  }

  // All tones sent?
  if (g_tx_tone_idx >= g_protocol->total_symbols) {
    ESP_LOGI(TAG, "tx_tick: TX complete, all %d tones sent", g_protocol->total_symbols);
    if (g_tx_cat_ok) {
      radio_control_end_tx();
    }
    ui_set_rx_waterfall_muted(false);
    // Record slot index for spacing and notify autoseq
    s_last_tx_slot_idx = g_tx_slot_idx;
    autoseq_mark_sent(g_tx_slot_idx);
    // g_was_txing stays true - tick will be called at slot boundary

    g_tx_active = false;
    g_pending_tx_valid = false;
    g_tx_cancel_requested = false;
    core_fire_qso_changed();  // propagates to all registered consumers
    return;
  }

  // Send current tone to the local visualizer and selected radio backend.
  ESP_LOGD("TXTONE", "%02d %u", g_tx_tone_idx, (unsigned)g_tx_tones[g_tx_tone_idx]);
  float tone_hz = g_tx_base_hz + g_protocol->tone_spacing * g_tx_tones[g_tx_tone_idx];
  fft_waterfall_tx_tone(tone_hz);
  if (g_tx_cat_ok) {
    tx_send_ta(tone_hz);
  }

  // Advance to next tone
  g_tx_tone_idx++;
  // Calculate next tone time from slot boundary to ensure TX ends at correct time
  // This guarantees RX can start cleanly at the next slot boundary
  g_tx_next_tone_time = g_tx_slot_start_ms + g_tx_tone_idx * (int)roundf(g_protocol->symbol_period * 1000.0f);
}

static void draw_menu_view() {
    if (menu_long_edit) {
      draw_menu_long_edit();
      return;
    }
  int64_t now = rtc_now_ms();
  if (menu_copy_feedback_deadline > 0 && now >= menu_copy_feedback_deadline) {
    menu_copy_feedback_deadline = 0;
    menu_copy_feedback_text.clear();
  }

  std::vector<std::string> lines;
  lines.reserve(12);

  std::string cq_line = std::string("CQ Type:");
  if (g_cq_type == CqType::CQFREETEXT) cq_line += g_cq_freetext;
  else cq_line += cq_type_name(g_cq_type);
  lines.push_back(cq_line);
  lines.push_back("Send FreeText");
  lines.push_back(std::string("F:") + head_trim(g_free_text, 16));
  lines.push_back(std::string("Call:") + elide_right(menu_edit_idx == 3 ? menu_edit_buf : g_call));
  std::string display_grid = g_grid;
  if (menu_edit_idx == 4) {
    display_grid = menu_edit_buf;
  } else if (g_time_synced_from_gps && g_grid_from_gps && g_grid_gps_display8.size() == 8) {
    display_grid = g_grid_gps_display8;
  }
  lines.push_back(std::string("Grid:") + elide_right(display_grid));
  lines.push_back(menu_sleep_batt_line());

  lines.push_back(std::string("Offset:") + offset_name(g_offset_src));
  if (menu_edit_idx == 7) {
    lines.push_back(std::string("Fixed:") + menu_edit_buf);
  } else {
    lines.push_back(std::string("Fixed:") + std::to_string(g_offset_hz));
  }
  lines.push_back(std::string("Radio:") + radio_name(g_radio));
  lines.push_back(std::string("IgnoreList:") + head_trim(g_ignore_prefix_text, 10));
  lines.push_back(std::string("C:") + head_trim(expand_comment1(), 16));
#if ENABLE_FT4
  {
    // Show the saved (pending) mode.  Add '*' if it differs from the running
    // boot mode so the user knows a reboot is needed to apply the change.
    const char* pending_name = g_protocol_pending_ft4 ? "FT4" : "FT8";
    bool needs_reboot = g_protocol_pending_ft4 != (g_protocol == &kProtocolFT4);
    lines.push_back(std::string("Mode: ") + pending_name + (needs_reboot ? "*" : ""));
  }
#else
  lines.push_back("USB:Manual S->2");
#endif

  // Page 2 content (index 12+)
  lines.push_back(std::string("RxTxLog:") + (g_rxtx_log ? "ON" : "OFF"));
  lines.push_back(std::string("SkipTX1:") + (g_skip_tx1 ? "ON" : "OFF"));
  lines.push_back(std::string("Bands:") + head_trim(g_active_band_text, 11));
  lines.push_back(std::string("LoRa GPS:") + (g_gnss_lora_enabled ? "ON" : "OFF"));
  if (menu_copy_feedback_deadline > 0 && !menu_copy_feedback_text.empty()) {
    lines.push_back(menu_copy_feedback_text);
  } else {
    lines.push_back("Copy Files to SD");
  }
  if (menu_edit_idx == 17) {
    lines.push_back(std::string("Max Retry:") + menu_edit_buf);
  } else {
    lines.push_back(std::string("Max Retry:") + std::to_string(g_autoseq_max_retry));
  }

  int highlight_abs = -1;
  if (menu_edit_idx >= 0) {
    highlight_abs = menu_edit_idx;
  } else if (menu_flash_idx >= 0 && now < menu_flash_deadline) {
    highlight_abs = menu_flash_idx;
  } else {
    menu_flash_idx = -1;
  }
  // Auto-clear flash after timeout
  if (menu_flash_idx >= 0 && now >= menu_flash_deadline) {
    menu_flash_idx = -1;
  }
  ui_draw_list(lines, menu_page, highlight_abs);
  // Draw battery icon on visible battery line
  int battery_abs_idx = 5;
  if (menu_page == (battery_abs_idx / 6)) {
    int line_on_page = battery_abs_idx % 6;
    const int line_h = 19;
    const int start_y = UI_START_Y;
    (void)line_on_page;
    (void)line_h;
    (void)start_y;
    //int y = start_y + line_on_page * line_h + 3;
    //int level = (int)M5.Power.getBatteryLevel();
    //bool charging = M5.Power.isCharging();
    //draw_battery_icon(190, y, 24, 12, level, charging);
  }
}

static std::string status_sync_line() {
  const bool streaming = audio_source_is_streaming();
  const RadioType radio = canonical_radio_type(g_radio);
  if (is_kh1_radio(radio)) {
    const char* name = radio_name(radio);
    if (!g_kh1_connected) {
      return std::string("Connect ") + name;
    }
    const bool cat_ready = radio_control_ready();
    if (radio == RadioType::KH1_USBC && !streaming) {
      const std::string rx_status = audio_source_get_status_string();
      if (rx_status == "No 48k UAC mic format") {
        return head_trim(rx_status, 19);
      }
    }
    if (cat_ready && streaming) return std::string("Sync ") + name + "(RX+TX)";
    if (cat_ready && !streaming) return std::string("Sync ") + name + "(TX)";
    return std::string("Connect ") + name;
  }
  if (streaming) return std::string("Sync to ") + radio_name(radio);
  return std::string("Connect to ") + radio_name(radio);
}

static std::string s_last_gps_lines[6];

static void draw_gps_view(bool force_redraw) {
  std::vector<std::string> lines;
  lines.reserve(6);
  gps_state_t state = gps_get_state();
  lines.push_back(std::string("Src:") + gps_source_name());
  if (state.valid_fix) {
    lines.push_back(std::string("Fix: 3D (") + std::to_string(state.satellites) + " Sats)");
  } else {
    lines.push_back(std::string("Fix: NO FIX (") + std::to_string(state.satellites) + " Sats)");
  }
  lines.push_back(std::string("Time: ") + (state.time_utc.empty() ? "Wait..." : state.time_utc));
  lines.push_back(std::string("Grid: ") + (state.grid_square.empty() ? "----" : state.grid_square));
  char loc[64];
  snprintf(loc, sizeof(loc), "L: %.3f, %.3f", state.latitude, state.longitude);
  lines.push_back(loc);
  if (state.last_rx_ms > 0) {
    uint32_t diff = (xTaskGetTickCount() * portTICK_PERIOD_MS - state.last_rx_ms) / 1000;
    lines.push_back(std::string("Sync: Good (") + std::to_string(diff) + "s ago)");
  } else {
    lines.push_back("Sync: Pending...");
  }
  
  const int line_h = 19;
  const int start_y = UI_START_Y;

  M5.Display.startWrite();
  M5.Display.setTextSize(2);
  for (size_t i = 0; i < 6; ++i) {
    std::string text = (i < lines.size()) ? lines[i] : "";
    // Keep the UI text snapshot in sync regardless of LCD redraw.
    ui_set_visible_text_line((int)i, text);
    if (force_redraw || text != s_last_gps_lines[i]) {
      s_last_gps_lines[i] = text;
      int y = start_y + i * line_h;
      M5.Display.fillRect(0, y, 240, line_h, TFT_BLACK);
      if (!text.empty()) {
        M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5.Display.setCursor(0, y);
        M5.Display.printf("%s", text.c_str());
      }
    }
  }
  M5.Display.endWrite();
}

static void draw_status_view() {
  std::string lines[6];
  BeaconMode disp_beacon = (ui_mode == UIMode::STATUS) ? g_status_beacon_temp : g_beacon;
  lines[0] = std::string("Beacon: ") + beacon_name(disp_beacon);
  lines[1] = status_sync_line();
  {
    char fbuf[16];
    float f = g_bands[g_band_sel].freq;
    if (f == (int)f) snprintf(fbuf, sizeof(fbuf), "%d", (int)f);
    else             snprintf(fbuf, sizeof(fbuf), "%.1f", f);
    lines[2] = std::string("Band: ") + std::string(g_bands[g_band_sel].name) + " " + fbuf;
  }
  lines[3] = std::string("Tune: ") + (g_tune ? "ON" : "OFF");
  if (status_edit_idx == 4 && !status_edit_buffer.empty()) {
    lines[4] = std::string("Date: ") + highlight_pos(status_edit_buffer, status_cursor_pos);
  } else {
    lines[4] = std::string("Date: ") + g_date;
  }
  if (status_edit_idx == 5 && !status_edit_buffer.empty()) {
    lines[5] = std::string("Time: ") + highlight_pos(status_edit_buffer, status_cursor_pos);
  } else {
    lines[5] = std::string("Time: ") + g_time + rtc_time_source_suffix();
  }
  for (int i = 0; i < 6; ++i) {
    bool hl = (status_edit_idx == i);
    draw_status_line(i, lines[i], hl);
  }
}

static bool perf_idle_hook_cpu0() {
  g_perf_idle_count[0] = g_perf_idle_count[0] + 1u;
  return true;
}

static bool perf_idle_hook_cpu1() {
  g_perf_idle_count[1] = g_perf_idle_count[1] + 1u;
  return true;
}

static uint8_t perf_busy_percent(uint32_t idle_delta, TickType_t tick_delta) {
  if (tick_delta == 0) return 0;
  uint32_t idle_pct = ((idle_delta * 100u) + ((uint32_t)tick_delta / 2u)) / (uint32_t)tick_delta;
  if (idle_pct > 100u) idle_pct = 100u;
  return (uint8_t)(100u - idle_pct);
}

static void perf_monitor_sample(TickType_t now_ticks) {
  if (g_perf_prev_sample_tick == 0) {
    g_perf_prev_sample_tick = now_ticks;
    g_perf_prev_idle_count[0] = g_perf_idle_count[0];
    g_perf_prev_idle_count[1] = g_perf_idle_count[1];
    return;
  }

  TickType_t tick_delta = now_ticks - g_perf_prev_sample_tick;
  if (tick_delta == 0) return;

  for (int core = 0; core < 2; ++core) {
    uint32_t idle_now = g_perf_idle_count[core];
    uint32_t idle_delta = idle_now - g_perf_prev_idle_count[core];
    g_perf_prev_idle_count[core] = idle_now;
    if (g_perf_cpu_hook_ok[core]) {
      g_perf_cpu_busy_pct[core] = perf_busy_percent(idle_delta, tick_delta);
    }
  }

  g_perf_prev_sample_tick = now_ticks;
  g_perf_cpu_sample_valid = g_perf_cpu_hook_ok[0] || g_perf_cpu_hook_ok[1];
}

static void perf_monitor_init() {
  static bool initialized = false;
  if (initialized) return;

  esp_err_t err0 = esp_register_freertos_idle_hook_for_cpu(perf_idle_hook_cpu0, 0);
  if (err0 == ESP_OK) {
    g_perf_cpu_hook_ok[0] = true;
  } else {
    ESP_LOGW(TAG, "CPU0 perf idle hook failed: %s", esp_err_to_name(err0));
  }

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
  esp_err_t err1 = esp_register_freertos_idle_hook_for_cpu(perf_idle_hook_cpu1, 1);
  if (err1 == ESP_OK) {
    g_perf_cpu_hook_ok[1] = true;
  } else {
    ESP_LOGW(TAG, "CPU1 perf idle hook failed: %s", esp_err_to_name(err1));
  }
#endif

  g_perf_prev_sample_tick = xTaskGetTickCount();
  g_perf_prev_idle_count[0] = g_perf_idle_count[0];
  g_perf_prev_idle_count[1] = g_perf_idle_count[1];
  initialized = true;
}

static uint16_t perf_color_for_pct(uint8_t pct) {
  if (pct >= 85) return TFT_RED;
  if (pct >= 65) return TFT_YELLOW;
  return TFT_GREEN;
}

static unsigned perf_kib_rounded(size_t bytes) {
  return (unsigned)((bytes + 512u) / 1024u);
}

static uint8_t perf_heap_used_pct(uint32_t caps, size_t free_bytes) {
  size_t total = heap_caps_get_total_size(caps);
  if (total == 0 || free_bytes >= total) return 0;
  return (uint8_t)(((total - free_bytes) * 100u + (total / 2u)) / total);
}

static void perf_make_cpu_line(char* out, size_t out_len, int core) {
  char bar[9];
  uint8_t pct = g_perf_cpu_busy_pct[core];
  int filled = g_perf_cpu_sample_valid && g_perf_cpu_hook_ok[core] ? (pct * 8 + 50) / 100 : 0;
  if (filled < 0) filled = 0;
  if (filled > 8) filled = 8;
  for (int i = 0; i < 8; ++i) bar[i] = (i < filled) ? '#' : '-';
  bar[8] = '\0';

  if (g_perf_cpu_sample_valid && g_perf_cpu_hook_ok[core]) {
    snprintf(out, out_len, "C%d %3u%% [%s]", core, (unsigned)pct, bar);
  } else {
    snprintf(out, out_len, "C%d --%% [%s]", core, bar);
  }
}

static void perf_make_heap_line(char* out, size_t out_len, const char* label, uint32_t caps) {
  size_t free_bytes = heap_caps_get_free_size(caps);
  size_t largest = heap_caps_get_largest_free_block(caps);
  uint8_t used_pct = perf_heap_used_pct(caps, free_bytes);
  snprintf(out, out_len, "%s %3u%% F%uK L%uK",
           label,
           (unsigned)used_pct,
           perf_kib_rounded(free_bytes),
           perf_kib_rounded(largest));
}

static void draw_perf_view(bool force_redraw) {
  static char last_lines[6][32] = {{0}};
  char lines[6][32];
  uint16_t colors[6] = {
      perf_color_for_pct(g_perf_cpu_busy_pct[0]),
      perf_color_for_pct(g_perf_cpu_busy_pct[1]),
      TFT_WHITE,
      TFT_WHITE,
      TFT_WHITE,
      TFT_WHITE,
  };

  perf_make_cpu_line(lines[0], sizeof(lines[0]), 0);
  perf_make_cpu_line(lines[1], sizeof(lines[1]), 1);
  perf_make_heap_line(lines[2], sizeof(lines[2]), "8B", MALLOC_CAP_8BIT);
  perf_make_heap_line(lines[3], sizeof(lines[3]), "IN", MALLOC_CAP_INTERNAL);
  perf_make_heap_line(lines[4], sizeof(lines[4]), "DM", MALLOC_CAP_DMA);
  snprintf(lines[5], sizeof(lines[5]), "ST C%uK M%uK",
           perf_kib_rounded(g_app_core0_stack_cur_free_bytes),
           perf_kib_rounded(g_app_core0_stack_min_free_bytes));

  const int line_h = 19;
  const int start_y = UI_START_Y;
  M5.Display.startWrite();
  M5.Display.setTextSize(2);
  for (int i = 0; i < 6; ++i) {
    ui_set_visible_text_line(i, lines[i]);
    if (force_redraw || strcmp(last_lines[i], lines[i]) != 0) {
      snprintf(last_lines[i], sizeof(last_lines[i]), "%s", lines[i]);
      int y = start_y + i * line_h;
      M5.Display.fillRect(0, y, 240, line_h, TFT_BLACK);
      M5.Display.setTextColor(colors[i], TFT_BLACK);
      M5.Display.setCursor(0, y);
      M5.Display.printf("%s", lines[i]);
    }
  }
  M5.Display.endWrite();
}

static void debug_ensure_hud_lines() {
  while (g_debug_lines.size() < DEBUG_HUD_LINES) {
    g_debug_lines.emplace_back();
  }
}

static void debug_update_app_core0_stack_hud(bool redraw_now) {
  debug_ensure_hud_lines();
  char cur_line[20];
  char min_line[20];
  std::snprintf(cur_line, sizeof(cur_line), "Acur %luB",
                (unsigned long)g_app_core0_stack_cur_free_bytes);
  std::snprintf(min_line, sizeof(min_line), "Amin %luB",
                (unsigned long)g_app_core0_stack_min_free_bytes);
  g_debug_lines[0] = cur_line;
  g_debug_lines[1] = min_line;
  (void)redraw_now;
}

static void debug_log_line(const std::string& msg) {
  debug_ensure_hud_lines();
  if (g_debug_lines.size() >= DEBUG_MAX_LINES) {
    if (g_debug_lines.size() > DEBUG_HUD_LINES) {
      g_debug_lines.erase(g_debug_lines.begin() + DEBUG_HUD_LINES);
    } else {
      return;
    }
  }
  g_debug_lines.push_back(msg);
  debug_page = (int)((g_debug_lines.size() - 1) / 6);
}

static std::string trim_copy(const std::string& s) {
  size_t b = 0, e = s.size();
  while (b < e && isspace((unsigned char)s[b])) ++b;
  while (e > b && isspace((unsigned char)s[e - 1])) --e;
  return s.substr(b, e - b);
}

static void ascii_upper_inplace(std::string& s) {
  for (auto& ch : s) {
    ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
  }
}

static std::string trim_upper_copy(const std::string& s) {
  std::string out = trim_copy(s);
  ascii_upper_inplace(out);
  return out;
}

static uint32_t parse_crc_hex(const std::string& hex) {
  if (hex.empty()) return 0;
  char* end = nullptr;
  unsigned long v = strtoul(hex.c_str(), &end, 16);
  if (end == hex.c_str() || *end != '\0') return 0;
  return (uint32_t)v;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = crc ^ 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

static void host_debug_hex8(const char* prefix, const uint8_t* b) {
  char buf[64];
  int n = snprintf(buf, sizeof(buf), "%s ", prefix);
  for (int i = 0; i < 8 && n + 3 < (int)sizeof(buf); ++i) {
    n += snprintf(buf + n, sizeof(buf) - n, "%02X ", b[i]);
  }
  if (n > 0 && buf[n - 1] == ' ') buf[n - 1] = 0;
  host_write_str(std::string(buf) + "\r\n");
}

static void host_handle_line(const std::string& line_in) {
  bool send_prompt = true;
  std::string line = trim_copy(line_in);
  if (line.empty()) { /* host_write_str(HOST_PROMPT);*/ return; }
  debug_log_line(std::string("[HOST RX] ") + line);
  //std::string echo = std::string("ECHO: ") + line + "\r\n";
  //host_write_str(echo);

  auto to_upper = [](std::string s) {
    for (auto& c : s) c = toupper((unsigned char)c);
    return s;
  };
  std::istringstream iss(line);
  std::string cmd;
  iss >> cmd;
  std::string cmd_up = to_upper(cmd);
  std::string rest;
  std::getline(iss, rest);
  rest = trim_copy(rest);

  auto send = [](const std::string& msg) { host_write_str(msg + "\r\n"); };

  if (cmd_up == "WRITE" || cmd_up == "APPEND") {
    std::istringstream rs(rest);
    std::string fname;
    rs >> fname;
    std::string content;
    std::getline(rs, content);
    content = trim_copy(content);
    if (fname.empty()) {
      send("ERROR: filename required");
    } else if (cmd_up == "WRITE" && storage_reject_active_log_user_mutation(fname)) {
      send("ERROR: active log protected");
    } else {
      if (cmd_up == "WRITE") {
        send(storage_file_write_atomic(fname, content) ? "OK" : "ERROR: write failed");
      } else {
        send(storage_file_append(fname, content, "", true) ? "OK" : "ERROR: write failed");
      }
    }
  } else if (cmd_up == "READ") {
    if (rest.empty()) send("ERROR: filename required");
    else {
      StorageStream* stream = storage_stream_open(rest, StorageOpenMode::READ);
      if (!stream) send("ERROR: open failed");
      else {
        char buf[128];
        while (storage_stream_read_line(stream, buf, sizeof(buf))) {
          host_write_str(std::string(buf));
        }
        storage_stream_close(stream);
        send_prompt = false;
      }
    }
  } else if (cmd_up == "DELETE") {
    if (rest.empty()) send("ERROR: filename required");
    else if (storage_reject_active_log_user_mutation(rest)) send("ERROR: active log protected");
    else {
      if (storage_file_remove(rest)) send("OK"); else send("ERROR: delete failed");
    }
  } else if (cmd_up == "LIST") {
    std::vector<std::string> files;
    if (!storage_file_list(files)) send("ERROR: storage unavailable");
    else {
      for (const auto& file : files) send(file);
      send("OK");
    }
  } else if (cmd_up == "WRITEBIN") {
    std::istringstream rs(rest);
    std::string fname;
    size_t size = 0;
    std::string crc_hex;
    rs >> fname >> size >> crc_hex;
    uint32_t crc_exp = parse_crc_hex(crc_hex);
    if (fname.empty() || size == 0 || crc_hex.empty()) {
      send("ERROR: filename, size, crc32_hex required");
    } else if (host_bin_active) {
      send("ERROR: binary upload in progress");
    } else if (storage_reject_active_log_user_mutation(fname)) {
      send("ERROR: active log protected");
    } else {
      StorageStream* stream = storage_stream_open(fname, StorageOpenMode::WRITE_TRUNCATE);
      if (!stream) {
        send("ERROR: open failed");
      } else {
          host_bin_path = fname;
          host_bin_active = true;
          host_bin_remaining = size;
          host_bin_stream = stream;
          host_bin_crc = 0;
          host_bin_expected_crc = crc_exp;
          host_bin_received = 0;
          host_bin_buf.clear();
          host_bin_buf.reserve(HOST_BIN_CHUNK);
          host_bin_chunk_expect = (host_bin_remaining < HOST_BIN_CHUNK) ? host_bin_remaining : HOST_BIN_CHUNK;
          host_bin_first_filled = 0;
          memset(host_bin_first8, 0, sizeof(host_bin_first8));
          memset(host_bin_last8, 0, sizeof(host_bin_last8));
          host_write_str("OK: send " + std::to_string(size) + " bytes, chunk " + std::to_string(HOST_BIN_CHUNK) + " +4crc\r\n");
          send_prompt = false; // prompt after binary upload completes
      }
    }
  } else if (cmd_up == "DATE") {
    if (rest.empty()) {
      send("DATE " + g_date);
    } else {
      int y, M, d;
      if (sscanf(rest.c_str(), "%d-%d-%d", &y, &M, &d) != 3 ||
          y < 2024 || y > 2099 || M < 1 || M > 12 || d < 1 || d > 31) {
        send("ERROR: use DATE YYYY-MM-DD");
      } else {
        char buf[16];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d", y, M, d);
        g_date = buf;
        if (rtc_apply_manual_time_from_strings()) { save_station_data(); send("OK"); }
        else send("ERROR: invalid date");
      }
    }
  } else if (cmd_up == "TIME") {
    if (rest.empty()) {
      send("TIME " + g_time);
    } else {
      int h, m, s;
      if (sscanf(rest.c_str(), "%d:%d:%d", &h, &m, &s) != 3 ||
          h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) {
        send("ERROR: use TIME HH:MM:SS");
      } else {
        char buf[16];
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d", h, m, s);
        g_time = buf;
        if (rtc_apply_manual_time_from_strings()) { save_station_data(); send("OK"); }
        else send("ERROR: invalid time");
      }
    }
  } else if (cmd_up == "SLEEP") {
    if (rtc_valid) {
      // Compute current time in milliseconds, round up to next second boundary
      int64_t elapsed_ms = esp_timer_get_time() / 1000 - rtc_ms_start;
      int64_t now_ms = (time_t)rtc_epoch_base * 1000LL + elapsed_ms;
      int64_t frac = now_ms % 1000;
      int64_t wait_ms = (frac > 0) ? (1000 - frac) : 0;
      time_t sleep_epoch = (time_t)((now_ms + 999) / 1000);  // ceil to next second

      g_rtc_sleep_epoch = sleep_epoch;
      save_station_data();

      // Wait until the second boundary, then set ESP RTC and sleep
      if (wait_ms > 0) vTaskDelay(pdMS_TO_TICKS(wait_ms));
      struct timeval tv = { .tv_sec = sleep_epoch, .tv_usec = 0 };
      settimeofday(&tv, NULL);
    }
    send("OK: entering deep sleep");
    M5.Display.sleep();
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
    esp_deep_sleep_start();
  } else if (cmd_up == "INFO") {
    send("Heap: " + std::to_string(heap_caps_get_free_size(MALLOC_CAP_DEFAULT)));
    send("OK");
  } else if (cmd_up == "HELP") {
    for (auto& l : g_msc_lines) send(l);
  } else if (cmd_up == "EXIT") {
    send("OK: exit host");
    enter_mode(UIMode::RX);
    return;
  } else {
    send("ERROR: Unknown command. Type HELP.");
  }

  if (send_prompt) host_write_str(std::string(HOST_PROMPT));
}

static void host_bin_close_release() {
  if (host_bin_stream) {
    storage_stream_sync(host_bin_stream);
    storage_stream_close(host_bin_stream);
    host_bin_stream = nullptr;
  }
  host_bin_active = false;
  host_bin_remaining = 0;
  host_bin_buf.clear();
}

static void host_process_bytes(const uint8_t* buf, size_t len) {
  ESP_LOGD(TAG, "host_process_bytes len=%u", (unsigned)len);
  for (size_t i = 0; i < len; ) {
    if (host_bin_active) {
      // Skip any stray CR/LF before first payload byte
      if (host_bin_received == 0 && host_bin_buf.empty() && (buf[i] == '\r' || buf[i] == '\n')) {
        ++i;
        continue;
      }
      size_t payload_need = host_bin_chunk_expect;
      size_t total_need = payload_need + 4; // payload + crc32 trailer
      size_t avail = len - i;
      size_t copy = total_need - host_bin_buf.size();
      if (copy > avail) copy = avail;
      host_bin_buf.insert(host_bin_buf.end(), buf + i, buf + i + copy);
      i += copy;

      if (host_bin_buf.size() >= total_need) {
        size_t payload_len = payload_need;
        uint32_t recv_crc = (uint32_t(host_bin_buf[payload_len])) |
                            (uint32_t(host_bin_buf[payload_len + 1]) << 8) |
                            (uint32_t(host_bin_buf[payload_len + 2]) << 16) |
                            (uint32_t(host_bin_buf[payload_len + 3]) << 24);
        uint32_t calc_crc = crc32_update(0, host_bin_buf.data(), payload_len);
        if (calc_crc != recv_crc) {
          char dbg[128];
          snprintf(dbg, sizeof(dbg), "ERROR: chunk crc off=%u len=%u calc=%08X recv=%08X\r\n",
                   (unsigned)(host_bin_received + payload_len), (unsigned)payload_len,
                   (unsigned)calc_crc, (unsigned)recv_crc);
          host_write_str(std::string(dbg));
          // Send first/last bytes of the chunk to compare
          if (payload_len >= 8) host_debug_hex8("DBG CHUNK FIRST8", host_bin_buf.data());
          if (payload_len >= 8) host_debug_hex8("DBG CHUNK LAST8", host_bin_buf.data() + payload_len - 8);
          if (payload_len < 8) host_debug_hex8("DBG CHUNK PART", host_bin_buf.data());
          // Also report the CRC trailer bytes as seen
          uint8_t crc_bytes[4] = {
            host_bin_buf[payload_len],
            host_bin_buf[payload_len + 1],
            host_bin_buf[payload_len + 2],
            host_bin_buf[payload_len + 3]
          };
          host_debug_hex8("DBG CRC BYTES", crc_bytes);
          host_bin_close_release();
          host_write_str(std::string(HOST_PROMPT));
          continue;
        }

        // Capture first/last bytes for debugging
        if (host_bin_first_filled < 8) {
          size_t need = 8 - host_bin_first_filled;
          if (need > payload_len) need = payload_len;
          memcpy(host_bin_first8 + host_bin_first_filled, host_bin_buf.data(), need);
          host_bin_first_filled += need;
        }
        // update last8 buffer
        if (payload_len >= 8) {
          memcpy(host_bin_last8, host_bin_buf.data() + payload_len - 8, 8);
        } else {
          // shift existing and append
          size_t shift = (payload_len + 8 > 8) ? (payload_len) : payload_len;
          if (shift > 0) {
            memmove(host_bin_last8, host_bin_last8 + shift, 8 - shift);
            memcpy(host_bin_last8 + (8 - payload_len), host_bin_buf.data(), payload_len);
          }
        }

        size_t written = storage_stream_write(host_bin_stream, host_bin_buf.data(), payload_len);
        if (written != payload_len) {
          host_write_str("ERROR: write failed\r\n");
          host_bin_close_release();
          host_write_str(std::string(HOST_PROMPT));
          continue;
        }
        host_bin_crc = crc32_update(host_bin_crc, host_bin_buf.data(), payload_len);
        host_bin_remaining -= payload_len;
        host_bin_received += payload_len;
        host_bin_buf.clear();
        host_write_str("ACK " + std::to_string(host_bin_received) + "\r\n");

        if (host_bin_remaining == 0) {
          uint32_t crc_final = host_bin_crc;
          host_bin_close_release();
          // Reopen file to send first/last 8 bytes for debugging
          host_debug_hex8("DBG FIRST8", host_bin_first8);
          host_debug_hex8("DBG LAST8", host_bin_last8);
          char crc_line[64];
          snprintf(crc_line, sizeof(crc_line), "DBG CRC %08X EXPECT %08X\r\n",
                   (unsigned)crc_final, (unsigned)host_bin_expected_crc);
          host_write_str(std::string(crc_line));
          if (crc_final != host_bin_expected_crc) {
            host_write_str("ERROR: crc mismatch\r\n");
          } else {
            host_write_str("OK crc " + std::to_string(crc_final) + "\r\n");
          }
          host_write_str(std::string(HOST_PROMPT));
        } else {
          host_bin_chunk_expect = (host_bin_remaining < HOST_BIN_CHUNK) ? host_bin_remaining : HOST_BIN_CHUNK;
        }
      }
      continue;
    }
    char ch = (char)buf[i++];
    if (ch == '\r' || ch == '\n') {
      if (!host_input.empty()) {
    //ESP_LOGI(TAG, "HOST line: %s", host_input.c_str());
        host_handle_line(host_input);
        host_input.clear();
      } else {
        //host_write_str(std::string(HOST_PROMPT));
      }
    } else if (ch == 0x08 || ch == 0x7f) {
      if (!host_input.empty()) host_input.pop_back();
    } else if (ch >= 32 && ch < 127) {
      host_input.push_back(ch);
    }
  }
}

[[maybe_unused]] static void poll_host_uart() {
  ensure_usb();
  if (!usb_ready) return;
  uint8_t buf[512];
  while (true) {
    int r = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
    if (r <= 0) break;
    host_process_bytes(buf, (size_t)r);
  }
}

static RadioType load_station_radio_type_only() {
  StorageStream* stream = storage_stream_open(STATION_FILE, StorageOpenMode::READ);
  if (!stream) return canonical_radio_type(g_radio);

  char line[128];
  RadioType radio = canonical_radio_type(g_radio);
  while (storage_stream_read_line(stream, line, sizeof(line))) {
    if (strncmp(line, "radio=", 6) == 0) {
      radio = parse_radio_config_value(line + 6);
      break;
    }
  }
  storage_stream_close(stream);
  return canonical_radio_type(radio);
}

static void load_station_data() {
  storage_sync_station_from_sd();

  // Load-time defaults for runtime settings.
  g_rtc_comp = kRtcCompFixed;
  g_autoseq_max_retry = AUTOSEQ_MAX_RETRY;
  g_gps_baud = 115200;
  g_gnss_lora_enabled = false;
  g_grid_saved_manual = g_grid;
  g_grid_from_gps = false;
  g_grid_gps_display8.clear();

  {
    StorageStream* stream = storage_stream_open(STATION_FILE, StorageOpenMode::READ);
    if (!stream) {
      autoseq_set_max_retry(g_autoseq_max_retry);
      return;
    }

#if ENABLE_FT4
    // Pass 1: detect protocol_mode so we can set correct band defaults before
    // the full parse overwrites them.  Band entries are written before
    // protocol_mode in Station.txt, so a single-pass parse would load FT8
    // frequencies and never correct them when switching to FT4.
    {
      char line1[128];
      while (storage_stream_read_line(stream, line1, sizeof(line1))) {
        if (strncmp(line1, "protocol_mode=", 14) == 0) {
          char mode[8] = {};
          sscanf(line1 + 14, "%7s", mode);
          if (strcmp(mode, "FT4") == 0) {
            g_protocol = &kProtocolFT4;
            // Reset band frequencies to FT4 defaults.  The full parse below
            // will overwrite individual entries if the user has saved custom
            // FT4 frequencies (band0=…, band1=…, …).
            g_bands = {
              {"160m", 1843.0f}, {"80m",  3575.0f}, {"60m",  5357.0f}, {"40m",  7047.5f},
              {"30m", 10140.0f}, {"20m", 14080.0f}, {"17m", 18104.0f}, {"15m", 21140.0f},
              {"12m", 24919.0f}, {"10m", 28180.0f}, {"6m",  50318.0f}, {"2m", 144170.0f},
            };
            ESP_LOGI(TAG, "Station.txt: protocol_mode=FT4 — reset bands to FT4 defaults");
          }
          break;
        }
      }
      storage_stream_seek(stream, 0, SEEK_SET);
    }
#endif  // ENABLE_FT4

    // Pass 2 (or only pass when ENABLE_FT4=0): full field parse.
    char line[128];
    while (storage_stream_read_line(stream, line, sizeof(line))) {
      int idx = -1;
      int val = 0;
      float fval = 0.0f;
#if ENABLE_FT4
      // Per-protocol band keys: FT4 uses "ft4_band%d=", FT8 uses "band%d=".
      // This prevents FT4 frequencies (e.g. 14080) from overwriting FT8
      // defaults (e.g. 14074) when switching protocol and rebooting.
      const bool is_ft4_boot = (g_protocol == &kProtocolFT4);
      const int band_parse_ok = is_ft4_boot
          ? sscanf(line, "ft4_band%d=%f", &idx, &fval)
          : sscanf(line, "band%d=%f",     &idx, &fval);
      if (band_parse_ok == 2) {
#else
      if (sscanf(line, "band%d=%f", &idx, &fval) == 2) {
#endif
      if (idx >= 0 && idx < (int)g_bands.size()) {
        g_bands[idx].freq = fval;
      }
    } else if (sscanf(line, "beacon=%d", &val) == 1) {
      // beacon persists OFF only; ignore saved value
    } else if (sscanf(line, "offset=%d", &val) == 1) {
      g_offset_hz = val;
    } else if (sscanf(line, "band_sel=%d", &val) == 1) {
      if (val >= 0 && val < (int)g_bands.size()) g_band_sel = val;
    } else if (sscanf(line, "date=%63s", line) == 1) {
      g_date = line;
    } else if (sscanf(line, "time=%63s", line) == 1) {
      g_time = normalize_time_hms(line);
    } else if (sscanf(line, "cq_type=%d", &val) == 1) {
      if (val >= 0 && val <= 5) g_cq_type = (CqType)val;
    } else if (sscanf(line, "offset_src=%d", &val) == 1) {
      if (val >= 0 && val <= 2) g_offset_src = (OffsetSrc)val;
    } else if (strncmp(line, "radio=", 6) == 0) {
      g_radio = parse_radio_config_value(line + 6);
    } else if (sscanf(line, "gps_baud=%d", &val) == 1) {
      g_gps_baud = normalize_gps_baud_value(val);
    } else if (sscanf(line, "gnss_lora=%d", &val) == 1) {
      g_gnss_lora_enabled = (val != 0);
    } else if (sscanf(line, "gps_source=%d", &val) == 1) {
      if (val == 2) g_gnss_lora_enabled = true;
    } else if (strncmp(line, "cq_ft=", 6) == 0) {
      g_cq_freetext = trim_upper_copy(line + 6);
    } else if (strncmp(line, "free_text=", 10) == 0) {
      g_free_text = trim_upper_copy(line + 10);
    } else if (strncmp(line, "call=", 5) == 0) {
      g_call = trim_upper_copy(line + 5);
    } else if (strncmp(line, "grid=", 5) == 0) {
      const std::string norm_grid = normalize_grid_maidenhead(line + 5);
      if (!norm_grid.empty()) {
        g_grid = norm_grid;
        g_grid_saved_manual = g_grid;
        g_grid_from_gps = false;
        g_grid_gps_display8.clear();
      }
    } else if (strncmp(line, "comment1=", 9) == 0) {
      g_comment1 = trim_copy(line + 9);
    } else if (strncmp(line, "ignore_prefixes=", 16) == 0) {
      g_ignore_prefix_text = clamp_ignore_prefix_text(trim_upper_copy(line + 16));
    } else if (sscanf(line, "rxtx_log=%d", &val) == 1) {
      g_rxtx_log = (val != 0);
    } else if (sscanf(line, "skiptx1=%d", &val) == 1) {
      g_skip_tx1 = (val != 0); autoseq_set_skip_tx1(g_skip_tx1);
    } else if (sscanf(line, "active_band=%d", &val) == 1) { // legacy single value
      g_active_band_text = std::to_string(val);
    } else if (strncmp(line, "active_bands=", 13) == 0) {
      g_active_band_text = trim_upper_copy(line + 13);
    } else if (sscanf(line, "autoseq_max_retry=%d", &val) == 1) {
      if (val >= 0) g_autoseq_max_retry = val;
    } else if (strncmp(line, "protocol_mode=", 14) == 0) {
      // Already handled in pass 1 above (g_protocol + band defaults set there).
      // Nothing to do here in pass 2.
      (void)0;
    } else if (sscanf(line, "rtc_comp=%d", &val) == 1) {
      g_rtc_comp = clamp_rtc_comp_value(val);
    } else {
      long long epoch_tmp = 0;
      if (sscanf(line, "rtc_sleep_epoch=%lld", &epoch_tmp) == 1) {
        g_rtc_sleep_epoch = (time_t)epoch_tmp;
      }
    }
    }
    storage_stream_close(stream);
  }
  autoseq_set_max_retry(g_autoseq_max_retry);
  // Prefer an external DS3231 when present, then ESP RTC/deep-sleep
  // compensation, then the saved Station.txt strings.
  if (!rtc_init_from_ds3231() && !rtc_init_from_esp_rtc()) {
    ESP_LOGI(TAG, "No valid DS3231 or ESP RTC time; using saved time strings");
    rtc_set_from_strings_source(RtcTimeSource::SAVED);
  }
  rebuild_active_bands();
  rebuild_ignore_prefixes();
  g_beacon = BeaconMode::OFF; // force off on load
#if ENABLE_FT4
  g_protocol_pending_ft4 = (g_protocol == &kProtocolFT4);
#endif
}

void save_station_data() {
  if (!storage_service_firmware_available()) {
    static bool warned_once = false;
    if (!warned_once) {
      warned_once = true;
      ESP_LOGW(TAG, "Firmware does not own storage; station data save skipped");
    }
    return;
  }
  std::ostringstream out;
#if ENABLE_FT4
  // Per-protocol band keys keep FT8 and FT4 frequencies independent so that
  // switching protocol (reboot-to-apply) doesn't cross-contaminate band lists.
  const char* band_prefix = (g_protocol == &kProtocolFT4) ? "ft4_band" : "band";
#else
  const char* band_prefix = "band";
#endif
  for (size_t i = 0; i < g_bands.size(); ++i) {
    char fbuf[16];
    float f = g_bands[i].freq;
    if (f == (int)f) snprintf(fbuf, sizeof(fbuf), "%d", (int)f);
    else             snprintf(fbuf, sizeof(fbuf), "%.1f", f);
    out << band_prefix << (unsigned)i << "=" << fbuf << "\n";
  }
  // Beacon is not persisted (stays OFF on reload)
  out << "offset=" << g_offset_hz << "\n";
  out << "band_sel=" << g_band_sel << "\n";
  out << "date=" << g_date << "\n";
  out << "time=" << g_time << "\n";
  out << "cq_type=" << (int)g_cq_type << "\n";
  out << "cq_ft=" << g_cq_freetext << "\n";
  out << "skiptx1=" << (g_skip_tx1 ? 1 : 0) << "\n";
  out << "free_text=" << g_free_text << "\n";
  out << "call=" << g_call << "\n";
  out << "grid=" << g_grid_saved_manual << "\n";
  out << "offset_src=" << (int)g_offset_src << "\n";
  out << "radio=" << (int)canonical_radio_type(g_radio) << "\n";
  out << "gps_baud=" << normalize_gps_baud_value(g_gps_baud) << "\n";
  out << "gnss_lora=" << (g_gnss_lora_enabled ? 1 : 0) << "\n";
  out << "comment1=" << g_comment1 << "\n";
  out << "ignore_prefixes=" << g_ignore_prefix_text << "\n";
  out << "rxtx_log=" << (g_rxtx_log ? 1 : 0) << "\n";
  out << "active_bands=" << g_active_band_text << "\n";
  out << "rtc_sleep_epoch=" << (long long)g_rtc_sleep_epoch << "\n";
  out << "rtc_comp=" << g_rtc_comp << "\n";
  out << "autoseq_max_retry=" << g_autoseq_max_retry << "\n";
#if ENABLE_FT4
  // Save the pending protocol mode (may differ from g_protocol if user toggled
  // Mode in the menu without rebooting yet).
  if (g_protocol_pending_ft4) {
    out << "protocol_mode=FT4\n";
  }
#endif
  if (!storage_file_write_atomic(STATION_FILE, out.str())) {
    ESP_LOGE(TAG, "Failed to write %s", STATION_FILE);
    return;
  }
  // Every config mutation in the Cardputer UI funnels through here, so this
  // is the canonical place to notify core_api consumers.
  core_fire_config_changed();
}

static void enter_mode(UIMode new_mode) {
  // No special handling needed when leaving TX mode - autoseq manages queue internally
  if (ui_mode == UIMode::STATUS && new_mode != UIMode::STATUS) {
    if (g_beacon != g_status_beacon_temp) {
      bool was_off = (g_beacon == BeaconMode::OFF);
      g_beacon = g_status_beacon_temp;
      save_station_data();
      // No need to clear autoseq when beacon is turned off.
      // Any CQ in queue will transmit once, then tick moves CALLING→IDLE.
      core_fire_qso_changed();  // propagates to all registered consumers

      // If beacon was just enabled, enqueue CQ and set TX flag
      // TX will trigger at next slot boundary via check_slot_boundary()
      if (was_off && g_beacon != BeaconMode::OFF) {
        enqueue_beacon_cq();
        AutoseqTxEntry pending;
        if (autoseq_fetch_pending_tx(pending)) {
          arm_pending_tx(pending);
        }
      }
    }
    status_edit_idx = -1;
    status_edit_buffer.clear();

    // Auto-sync VFO + RX mode on STATUS exit. Picks up any in-STATUS
    // changes (band advance via S->3, etc.) without needing a manual
    // "Sync to QMX" button press. Idempotent — safe even if the same
    // sync already fired (e.g. from S->3 in-menu push, or from the
    // initial-connect path for QMX). For KH1 this is the primary sync
    // path (UART CAT has no discrete "first connect" event).
    sync_radio_to_current_band("STATUS exit");
  }
  ui_mode = new_mode;
  rx_flash_idx = -1;
  switch (ui_mode) {
    case UIMode::RX:
      // Force RX list redraw
      ui_force_redraw_rx();
      ui_draw_rx();
      break;
    case UIMode::TX:
      tx_page = 0;
      redraw_tx_view();
      break;
    case UIMode::BAND:
      band_page = 0;
      band_edit_idx = -1;
      draw_band_view();
      break;
    case UIMode::MENU:
      menu_page = 0;
      menu_edit_idx = -1;
      menu_edit_buf.clear();
      draw_menu_view();
      break;
    case UIMode::DEBUG:
      d_page = 0;
      delete_load_file_list();
      ui_draw_list(g_d_lines, d_page, -1);
      break;
    case UIMode::MSC:
      ui_draw_debug(g_msc_lines, 0);
      break;
    case UIMode::QSO:
      g_q_show_entries = false;
      q_page = 0;
      qso_load_file_list();
      qso_draw_page();
      break;
    case UIMode::STATUS:
      g_status_beacon_temp = g_beacon;
      status_edit_idx = -1;
      status_cursor_pos = -1;
      draw_status_view();
      break;
    case UIMode::GPS:
      draw_gps_view(true);
      break;
    case UIMode::PERF:
      draw_perf_view(true);
      break;
  }
}


static void enter_msc_mode(const char* reason) {
  ESP_LOGI(TAG, "Entering MSC mode: %s", reason);
  debug_log_line("Entering MSC mode");

  if (g_config_save_pending) {
    g_config_save_pending = false;
    save_station_data();
  }

  core_cmd_cancel_tx();
  if (g_tx_active) {
    tx_tick();
  }
  g_tx_cancel_requested = false;
  g_qso_xmit = false;
  g_pending_tx_valid = false;
  g_decode_enabled = false;
  ui_set_paused(true);
  audio_source_stop();
  vTaskDelay(pdMS_TO_TICKS(200));

  const esp_err_t err = storage_service_set_usb_drive_enabled(true);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "USB Drive enable failed: %s", esp_err_to_name(err));
    debug_log_line(std::string("USB Drive fail: ") + esp_err_to_name(err));
    return;
  }

  ESP_LOGI(TAG, "MSC ready: PC will see Mini-FT8 as a USB drive");
  enter_mode(UIMode::MSC);
}

static void exit_msc_mode() {
  ESP_LOGI(TAG, "Leaving MSC mode");
  const esp_err_t err = storage_service_set_usb_drive_enabled(false);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "USB Drive disable failed: %s", esp_err_to_name(err));
    debug_log_line(std::string("USB Drive off fail: ") + esp_err_to_name(err));
    if (storage_service_owner() == StorageOwner::UNAVAILABLE) {
      enter_mode(UIMode::RX);
      ui_force_redraw_rx();
      ui_draw_rx();
    }
    return;
  }
  enter_mode(UIMode::RX);
  ui_force_redraw_rx();
  ui_draw_rx();
}

// Perform the STATUS -> '2' action: start the UAC audio source that feeds
// the QMX (or KH1) into the decoder, and sync CAT to the currently selected
// band.
static void begin_usb_host_mode() {
  // The status-view feedback only makes sense on the STATUS page.
  const bool on_status_page = (ui_mode == UIMode::STATUS);
  if (on_status_page) {
    status_edit_idx = 1;
    draw_status_view();
  }
  if (is_kh1_radio(g_radio) && !g_kh1_connected) {
    g_kh1_connected = true;
    apply_radio_profile_binding();
  }
  const bool kh1_usbc = (canonical_radio_type(g_radio) == RadioType::KH1_USBC);
  if (!audio_source_is_streaming()) {
    start_rx_audio_for_current_radio("status key 2", !kh1_usbc);
  } else if (!kh1_usbc) {
    notify_radio_control_audio_start_if_allowed("status key 2");
  }
  int freq_hz = (int)(g_bands[g_band_sel].freq * 1000.0f);
  if (radio_control_ready()) {
    bool ok = (radio_control_sync_frequency_mode(freq_hz) == ESP_OK);
    debug_log_line(ok ? "CAT sync sent" : "CAT sync failed");
  } else {
    debug_log_line("CAT not ready");
  }
  if (on_status_page) {
    status_edit_idx = -1;
    draw_status_view();
  }
}

static void app_task_core0(void* /*param*/) {
  esp_err_t storage_err = storage_service_init();
  if (storage_err != ESP_OK) {
    ESP_LOGE(TAG, "Storage service init failed: %s; continuing without log/config storage",
             esp_err_to_name(storage_err));
    debug_log_line("Storage init fail");
  }

  if (storage_service_firmware_available()) {
    storage_sync_station_from_sd();
  }
  board_power_init();
  g_radio = load_station_radio_type_only();
  ui_init(radio_type_uses_display_only(g_radio));
  hashtable_init();

  // Q15 NCO LUT for UAC OUT FT8 audio synthesis. One-time table fill,
  // ~514 B in BSS. Must run before the speaker pump task starts.
  dds_init();

  // Initialize autoseq engine
  autoseq_init();

  // Initialize the functional-core API (creates internal sync primitives).
  // After this, UI consumers can call core_get_*, core_cmd_*, and register
  // callbacks.
  core_init();

  // Register the Cardputer UI as a core_api consumer. The callbacks just set
  // the existing dirty flags — the UI main loop drains them on each tick.
  // Trivial handlers only (spec in docs/NATIVE_CLIENT_ARCHITECTURE.md).
  core_on_rx_changed    ([]{ g_rx_dirty = true; });
  core_on_qso_changed   ([]{ g_tx_view_dirty = true; });
  // config changes redraw whatever view is showing them (MENU/STATUS);
  // set both dirty flags so the next UI tick re-evaluates.
  core_on_config_changed([]{ g_rx_dirty = true; g_tx_view_dirty = true; });
  
autoseq_set_adif_callback(log_adif_entry);
autoseq_set_cabrillo_fd_callback(log_cabrillo_fd_entry);


  ui_mode = UIMode::RX;
  load_station_data();
  apply_debug_uart_pin_policy();
  apply_radio_profile_binding();
  update_autoseq_cq_type();

  // Update autoseq with station info after loading
  autoseq_set_station(g_call, grid_ft8_4(g_grid));

  // Prepare RX list (but don't draw yet - startup screen may be shown)
  std::vector<UiRxLine> empty;
  ui_set_rx_list(empty);

  if (g_startup_active) {
    ui_draw_debug(g_startup_lines, 0);
  } else {
    ui_force_redraw_rx();
    ui_draw_rx();
  }

  ESP_LOGI(TAG, "Free heap: %u, internal: %u, 8bit: %u",
           heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           heap_caps_get_free_size(MALLOC_CAP_8BIT));
  {
    char buf[64];
    snprintf(buf, sizeof(buf), "Heap %u", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    debug_log_line(buf);
  }
  log_heap("BOOT");

  g_app_core0_stack_last_sample_tick = xTaskGetTickCount();
  {
    UBaseType_t free_words = uxTaskGetStackHighWaterMark2(NULL);
    uint32_t free_bytes = (uint32_t)free_words * (uint32_t)sizeof(StackType_t);
    g_app_core0_stack_cur_free_bytes = free_bytes;
    g_app_core0_stack_min_free_bytes = free_bytes;
    debug_update_app_core0_stack_hud(false);
  }
  perf_monitor_init();

  // Key injection queue for console UART RX
  s_key_inject_queue = xQueueCreate(32, sizeof(char));

  // sdkconfig puts the ESP console on UART0 peripheral with a custom TX pin,
  // but IDF's custom-console init only guarantees the TX pin routing —
  // it doesn't always hook up RX. Explicitly route the configured RX GPIO
  // to UART0 RXD. This is a no-op if already set, and doesn't install a
  // driver. Using CONFIG_ESP_CONSOLE_UART_RX_GPIO keeps this in sync with
  // sdkconfig automatically.
  uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE,
               (gpio_num_t)CONFIG_ESP_CONSOLE_UART_RX_GPIO,
               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  // Drain any stale bytes left in the FIFO from ROM-bootloader time
  // (when UART0 RX was still on its IO_MUX default pin, likely floating).
  drain_console_uart_rx_fifo();
  apply_debug_uart_pin_policy();

  // UI loop
  char last_key = 0;
  while (true) {
    M5Cardputer.update();
    M5Cardputer.Keyboard.updateKeysState();
    auto &state = M5Cardputer.Keyboard.keysState();
    char c = 0;
    if (!state.word.empty()) {
      c = state.word.back();
      state.word.clear();  // consume key
    } else if (state.del) {
      c = 0x7f;  // treat delete/backspace
    } else if (state.enter) {
      c = '\n';  // enter/return
    }
    // Merge injected keys from console UART RX (G4/G5 per sdkconfig)
    poll_uart_inject_keys();
    if (c == 0 && s_key_inject_queue && g_debug_uart_pins_enabled) {
      char injected = 0;
      if (xQueueReceive(s_key_inject_queue, &injected, 0) == pdTRUE) {
        c = injected;
        last_key = 0;  // Reset debounce so same-key injection works
#if UART_SCREEN_MIRROR
        g_uart_mirror_pending = true;  // dump screen at top of next iteration
#endif
      }
    }

#if UART_SCREEN_MIRROR
    // Dump screen on the iteration AFTER a UART keypress was consumed,
    // once the UI has had a chance to process the key and redraw.
    static bool s_uart_mirror_fire = false;
    if (!g_debug_uart_pins_enabled) {
      g_uart_mirror_pending = false;
      s_uart_mirror_fire = false;
    } else if (s_uart_mirror_fire) {
      uart_mirror_dump_screen();
      s_uart_mirror_fire = false;
    }
    if (g_uart_mirror_pending) {
      g_uart_mirror_pending = false;
      s_uart_mirror_fire = true;  // fire on the next iteration
    }
#endif
    gps_runtime_tick();
    TickType_t now_ticks = xTaskGetTickCount();
    if ((now_ticks - g_app_core0_stack_last_sample_tick) >= pdMS_TO_TICKS(1000)) {
      g_app_core0_stack_last_sample_tick = now_ticks;
      UBaseType_t free_words = uxTaskGetStackHighWaterMark2(NULL);
      uint32_t free_bytes = (uint32_t)free_words * (uint32_t)sizeof(StackType_t);
      g_app_core0_stack_cur_free_bytes = free_bytes;
      if (g_app_core0_stack_min_free_bytes == 0 || free_bytes < g_app_core0_stack_min_free_bytes) {
        g_app_core0_stack_min_free_bytes = free_bytes;
      }
      debug_update_app_core0_stack_hud(true);
      perf_monitor_sample(now_ticks);
      if (ui_mode == UIMode::PERF) {
        draw_perf_view(false);
      }
    }
    // Startup splash: show briefly, then remain in RX. Radio connection is
    // explicit through STATUS -> 2; direct-mode keys still work immediately.
    if (g_startup_active) {
      if (g_startup_start_ms == 0) {
        g_startup_start_ms = esp_timer_get_time() / 1000;
      }

      if (c != 0 && c != last_key) {
        const bool direct_mode_entry = is_startup_direct_mode_key(c);
        g_startup_active = false;
        save_station_data();
        if (!direct_mode_entry) {
          // Non-mode key: dismiss, show RX, consume the key.
          last_key = c;
          ui_force_redraw_rx();
          ui_draw_rx();
          vTaskDelay(pdMS_TO_TICKS(10));
          continue;
        }
        // Direct-mode key: fall through so the main dispatcher handles it.
        last_key = 0;
      } else {
        const int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - g_startup_start_ms >= kStartupAutoDismissMs) {
          // No key within the window: dismiss the splash and remain in RX.
          g_startup_active = false;
          save_station_data();
          enter_mode(UIMode::RX);
          ui_force_redraw_rx();
          ui_draw_rx();
          last_key = 0;
          vTaskDelay(pdMS_TO_TICKS(10));
          continue;
        }
        last_key = c;  // 0 or the same key still held
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
    }

    if (ui_mode == UIMode::MSC) {
      if ((c == 'c' || c == 'C') && c != last_key) {
        exit_msc_mode();
      }
      last_key = c;
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    rtc_tick();
    update_countdown();
    consume_cdc_initial_sync();  // auto-sync VFO on first QMX connect (every iter)
    check_slot_boundary();  // TX trigger at slot boundary (matching reference architecture)
    tx_tick();              // Process TX state machine (single-threaded, non-blocking)

    // Drain deferred config saves requested by core commands.
    if (g_config_save_pending && storage_service_firmware_available()) {
      g_config_save_pending = false;
      save_station_data();
    }

    // Global TX cancel (Esc/` in RX/TX/Status when not editing)
    if (c == '`' &&
        (ui_mode == UIMode::RX || ui_mode == UIMode::TX || ui_mode == UIMode::STATUS) &&
        status_edit_idx == -1) {
      core_cmd_cancel_tx();
      debug_log_line("TX cancel requested");
      last_key = c;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (c == 0) {
      if (g_rx_dirty && ui_mode == UIMode::RX) {
        // decode_monitor_results already called ui_set_rx_list_static(),
        // so UI's internal list is current. Just redraw.
        ui_draw_rx(rx_flash_idx);
        g_rx_dirty = false;
      }
      if (ui_mode == UIMode::TX && g_tx_view_dirty) {
        g_tx_view_dirty = false;
        redraw_tx_view();
      }
      // NOTE: Beacon scheduling moved to decode_monitor_results() to match
      // reference architecture - beacon CQ is only added after decodes processed
      ui_draw_waterfall_if_dirty();
      menu_flash_tick();
      rx_flash_tick();
      last_key = 0;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
  if (c == last_key) {
    // No new keypress - still need to refresh dirty views
    if (ui_mode == UIMode::TX && g_tx_view_dirty) {
      g_tx_view_dirty = false;
      redraw_tx_view();
    }
    // NOTE: Beacon scheduling moved to decode_monitor_results()
    ui_draw_waterfall_if_dirty();
    vTaskDelay(pdMS_TO_TICKS(10));
    continue;
  }
  last_key = c;

  rtc_tick();
  update_countdown();
  // consume_cdc_initial_sync() already called above, before the early-exit
  // branches; no need to repeat here.
  check_slot_boundary();  // TX trigger at slot boundary (matching reference architecture)
  tx_tick();              // Process TX state machine (single-threaded, non-blocking)
  menu_flash_tick();
  rx_flash_tick();
  apply_pending_sync();

  // NOTE: TX scheduling now follows reference architecture:
  // 1. decode_monitor_results() sets g_qso_xmit flag after processing
  // 2. check_slot_boundary() triggers TX at slot boundary when parity matches
  // 3. autoseq_tick() is called at slot boundary AFTER TX slot ends

  // Refresh TX view if autoseq state changed
  if (ui_mode == UIMode::TX && g_tx_view_dirty) {
    g_tx_view_dirty = false;
    redraw_tx_view();
  }

  static int last_status_sync_sig = -1; // -1 forces a redraw on first entry
  static std::string last_status_sync_text;
  int cur_status_sync_sig = audio_source_is_streaming() ? 1 : 0;
  cur_status_sync_sig |= ((int)canonical_radio_type(g_radio) << 4);
  if (is_kh1_radio(g_radio)) {
    cur_status_sync_sig |= 2;
    if (g_kh1_connected) cur_status_sync_sig |= 8;
    if (g_kh1_connected && radio_control_ready()) cur_status_sync_sig |= 4;
  }
  std::string cur_status_sync_text;
  if (ui_mode == UIMode::STATUS) {
    cur_status_sync_text = status_sync_line();
  }
  if (ui_mode == UIMode::STATUS &&
      (cur_status_sync_sig != last_status_sync_sig ||
       cur_status_sync_text != last_status_sync_text)) {
    draw_status_view();
  }
  if (ui_mode != UIMode::STATUS) {
    last_status_sync_sig = -1;
    last_status_sync_text.clear();
  } else {
    last_status_sync_sig = cur_status_sync_sig;
    last_status_sync_text = cur_status_sync_text;
  }

  // Ensure decode is enabled whenever streaming becomes active.
  if (audio_source_is_streaming() && !g_decode_enabled) {
    g_decode_enabled = true;
    ui_set_paused(false);
  }

  if (g_rx_dirty && ui_mode == UIMode::RX) {
      // decode already populated ui.cpp's internal list via ui_set_rx_list_static
      ui_draw_rx(rx_flash_idx);
      g_rx_dirty = false;
  }
  ui_draw_waterfall_if_dirty();

  bool switched = false;
  auto cancel_status_edit = []() {
    if (status_edit_idx != -1) {
      status_edit_idx = -1;
      status_edit_buffer.clear();
      status_cursor_pos = -1;
    }
  };
  if (!(ui_mode == UIMode::MENU && (menu_edit_idx >= 0 || menu_long_edit))) {
      // Mode switch keys (disabled while editing in MENU)
      if (c == 'r' || c == 'R') { cancel_status_edit(); enter_mode(UIMode::RX); ui_force_redraw_rx(); ui_draw_rx(); switched = true; }
      else if (c == 't' || c == 'T') { cancel_status_edit(); enter_mode(ui_mode == UIMode::TX ? UIMode::RX : UIMode::TX); switched = true; }
      else if (c == 'b' || c == 'B') { cancel_status_edit(); enter_mode(ui_mode == UIMode::BAND ? UIMode::RX : UIMode::BAND); switched = true; }
      else if (c == 'm' || c == 'M') {
        cancel_status_edit();
        if (ui_mode == UIMode::MENU) {
          if (menu_page == 0) {
            enter_mode(UIMode::RX);
          } else {
            menu_page = 0;
            draw_menu_view();
          }
        } else {
          enter_mode(UIMode::MENU);
        }
        switched = true;
      }
      else if (c == 'n' || c == 'N') {
        cancel_status_edit();
        if (ui_mode == UIMode::MENU) {
          if (menu_page == 1) {
            enter_mode(UIMode::RX);
          } else {
            menu_page = 1;
            draw_menu_view();
          }
        } else {
          menu_page = 0;
          enter_mode(UIMode::MENU);
          if (menu_page < 2) menu_page++;  // one "." press
          draw_menu_view();
        }
        switched = true;
      }
      else if (c == 'o' || c == 'O') {
        cancel_status_edit();
        if (ui_mode == UIMode::MENU) {
          if (menu_page == 2) {
            enter_mode(UIMode::RX);
          } else {
            menu_page = 2;
            draw_menu_view();
          }
        } else {
          menu_page = 0;
          enter_mode(UIMode::MENU);
          if (menu_page < 2) menu_page++;  // first "."
          if (menu_page < 2) menu_page++;  // second "."
          draw_menu_view();
        }
        switched = true;
      }
      else if (c == 'q' || c == 'Q') { cancel_status_edit(); enter_mode(ui_mode == UIMode::QSO ? UIMode::RX : UIMode::QSO); switched = true; }
      else if (c == 'c' || c == 'C') {
        {
          cancel_status_edit();
          if (ui_mode != UIMode::MSC) {
            enter_msc_mode("user pressed C");
          }
          switched = true;
        }
      }
      else if (c == 'd' || c == 'D') { cancel_status_edit(); enter_mode(ui_mode == UIMode::DEBUG ? UIMode::RX : UIMode::DEBUG); switched = true; }
      else if (c == 's' || c == 'S') { cancel_status_edit(); enter_mode(ui_mode == UIMode::STATUS ? UIMode::RX : UIMode::STATUS); switched = true; }
      else if (c == 'g' || c == 'G') { cancel_status_edit(); enter_mode(ui_mode == UIMode::GPS ? UIMode::RX : UIMode::GPS); switched = true; }
      else if (c == 'p' || c == 'P') { cancel_status_edit(); enter_mode(ui_mode == UIMode::PERF ? UIMode::RX : UIMode::PERF); switched = true; }
    }

  if (!switched && c) {
    // Mode-specific handling
    switch (ui_mode) {
      case UIMode::GPS: break;
      case UIMode::PERF: break;
      case UIMode::RX: {
        int sel = ui_handle_rx_key(c);
        if (sel >= 0 && core_cmd_tap_rx(sel)) {
          // TX-state arming lives inside core_cmd_tap_rx for every UI path.
          rx_flash_idx = sel;
          rx_flash_deadline = rtc_now_ms() + 500;
          ui_draw_rx(rx_flash_idx);
        }
        break;
      }
      case UIMode::TX: {
        // TX view shows QSO states from autoseq
        // Pagination through QSO list (max 9 QSOs)
        int qso_count = autoseq_queue_size();
        int start_idx = tx_page * 5;
        if (c == ';') {
          if (tx_page > 0) { tx_page--; redraw_tx_view(); }
        } else if (c == '.') {
          if (start_idx + 5 < qso_count) { tx_page++; redraw_tx_view(); }
        } else if (c >= '2' && c <= '6') {
          int idx = start_idx + (c - '2');
          if (core_cmd_drop_qso(idx)) {  // routes through core_api (fires qso_changed)
            g_pending_tx_valid = false;
            redraw_tx_view();
            // Re-evaluate TX after queue change
            AutoseqTxEntry pending;
            if (autoseq_fetch_pending_tx(pending)) {
              arm_pending_tx(pending);
            }
          }
        } else if (c == '1') {
          if (autoseq_rotate_same_parity()) {
            g_pending_tx_valid = false;
            redraw_tx_view();
            // Re-evaluate TX after queue change
            AutoseqTxEntry pending;
            if (autoseq_fetch_pending_tx(pending)) {
              arm_pending_tx(pending);
            }
          }
        } else if (c == 'e' || c == 'E') {
          encode_and_log_pending_tx();
        }
        break;
      }
        case UIMode::BAND: {
          if (band_edit_idx >= 0) {
            if ((c >= '0' && c <= '9') || c == '.') { band_edit_buffer.push_back(c); draw_band_view(); }
            else if (c == 0x08 || c == 0x7f) {
              if (!band_edit_buffer.empty()) { band_edit_buffer.pop_back(); draw_band_view(); }
            } else if (c == '\r' || c == '\n') {
              if (!band_edit_buffer.empty()) {
                float val = std::stof(band_edit_buffer);
                g_bands[band_edit_idx].freq = val;
                save_station_data();
              }
              band_edit_idx = -1;
              band_edit_buffer.clear();
              draw_band_view();
            }
          } else {
            if (c == ';') {
              if (band_page > 0) { band_page--; draw_band_view(); }
            } else if (c == '.') {
              if ((band_page + 1) * 6 < (int)g_bands.size()) { band_page++; draw_band_view(); }
            } else if (c >= '1' && c <= '6') {
              int idx = band_page * 6 + (c - '1');
              if (idx >= 0 && idx < (int)g_bands.size()) {
                band_edit_idx = idx;
                {
                  char fbuf[16];
                  float f = g_bands[idx].freq;
                  if (f == (int)f) snprintf(fbuf, sizeof(fbuf), "%d", (int)f);
                  else             snprintf(fbuf, sizeof(fbuf), "%.1f", f);
                  band_edit_buffer = fbuf;
                }
                draw_band_view();
              }
            }
          }
          break;
        }
        case UIMode::STATUS: {
        if (status_edit_idx == -1) {
          if (handle_kh1_diag_key(c)) { draw_status_view(); }
          else if (c == '1') { g_status_beacon_temp = (BeaconMode)(((int)g_status_beacon_temp + 1) % 3); draw_status_view(); }
          else if (c == '2') {
            begin_usb_host_mode();
          }
          else if (c == '3') {
            advance_active_band(1);
            save_station_data();
            draw_status_view();
            debug_log_line("Band changed");
            // In-memory only. CAT push is deferred to:
            //   - STATUS exit (enter_mode), or
            //   - QMX initial-connect (consume_cdc_initial_sync reads
            //     current g_band_sel at sync time, so band edits made
            //     while QMX was still enumerating get picked up).
            // Why deferred: KH1 band change engages a physical antenna
            // relay, and we don't want to click it on every S->3 press.
          }
              else if (c == '4') {
                g_tune = !g_tune;
                if (radio_control_ready()) {
                  int freq_hz = (int)(g_bands[g_band_sel].freq * 1000.0f);
                  int tune_hz = (g_offset_src == OffsetSrc::CURSOR) ? g_offset_hz : 1500;
                  if (radio_control_set_tune(g_tune, freq_hz, tune_hz) == ESP_OK) {
                    debug_log_line(g_tune ? "CAT tune: TX" : "CAT tune: RX");
                  } else {
                    ESP_LOGW(TAG, "CAT tune command failed");
                    debug_log_line("CAT tune failed");
                  }
                } else {
                  ESP_LOGW(TAG, "CAT not ready; tune skipped");
                }
                draw_status_view();
              }
              else if (c == '5') {
                status_edit_idx = 4; status_edit_buffer = g_date; status_cursor_pos = 0; while (status_cursor_pos < (int)status_edit_buffer.size() && (status_edit_buffer[status_cursor_pos] == '-')) status_cursor_pos++; draw_status_view();
              }
              else if (c == '6') {
                status_edit_idx = 5; status_edit_buffer = g_time; status_cursor_pos = 0; while (status_cursor_pos < (int)status_edit_buffer.size() && (status_edit_buffer[status_cursor_pos] == ':')) status_cursor_pos++; draw_status_view();
              }
            } else {
              if (status_edit_idx == 1) {
                if (c == '`') { status_edit_idx = -1; status_edit_buffer.clear(); draw_status_view(); }
                if (c == ';') { g_offset_hz += 100; draw_status_view(); }
                else if (c == '.') { g_offset_hz -= 100; draw_status_view(); }
                else if (c == ',') { g_offset_hz -= 10; draw_status_view(); }
                else if (c == '/') { g_offset_hz += 10; draw_status_view(); }
                else if (c == '\n') { save_station_data(); status_edit_idx = -1; draw_status_view(); }
              } else if (status_edit_idx == 4 || status_edit_idx == 5) {
                if (c == '`') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
                else if (c == ',') { // left
                  int pos = status_cursor_pos - 1;
                  while (pos >= 0 && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos--;
                  if (pos >= 0) status_cursor_pos = pos;
                  draw_status_view();
                } else if (c == '/') { // right
                  int pos = status_cursor_pos + 1;
                  while (pos < (int)status_edit_buffer.size() && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos++;
                  if (pos < (int)status_edit_buffer.size()) status_cursor_pos = pos;
                  draw_status_view();
                } else if (c >= '0' && c <= '9') {
                  if (status_cursor_pos >= 0 && status_cursor_pos < (int)status_edit_buffer.size()) {
                    status_edit_buffer[status_cursor_pos] = c;
                    int pos = status_cursor_pos + 1;
                    while (pos < (int)status_edit_buffer.size() && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos++;
                    if (pos < (int)status_edit_buffer.size()) status_cursor_pos = pos;
                  }
                  draw_status_view();
                } else if (c == '\n') {
                  if (status_edit_idx == 4) g_date = status_edit_buffer;
                  else g_time = normalize_time_hms(status_edit_buffer);
                  if (rtc_apply_manual_time_from_strings()) {
                    save_station_data();
                  } else {
                    debug_log_line("Invalid date/time");
                  }
                  status_edit_idx = -1;
                  status_cursor_pos = -1;
                  status_edit_buffer.clear();
                  draw_status_view();
                }
              } else {
                if (c == '`') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
                else if (c == '\n') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
              }
            }
            break;
          }
        case UIMode::DEBUG: {
          if (c == ';') {
            if (d_page > 0) { d_page--; ui_draw_list(g_d_lines, d_page, -1); }
          } else if (c == '.') {
            if ((d_page + 1) * 6 < (int)g_d_lines.size()) { d_page++; ui_draw_list(g_d_lines, d_page, -1); }
          } else if (c >= '1' && c <= '6') {
            int idx = d_page * 6 + (c - '1');
            if (idx >= 0 && idx < (int)g_d_files.size()) {
              std::string deleted = g_d_files[idx];
              bool reload_list = true;
              if (storage_reject_active_log_user_mutation(deleted)) {
                debug_log_line(std::string("Active log protected: ") + deleted);
                if (idx < (int)g_d_lines.size()) g_d_lines[idx] = std::string("LOCK ") + deleted;
                reload_list = false;
              } else if (storage_file_remove(deleted)) {
                debug_log_line(std::string("Deleted: ") + deleted);
              } else {
                debug_log_line(std::string("Delete failed: ") + deleted);
                if (idx < (int)g_d_lines.size()) g_d_lines[idx] = std::string("FAIL ") + deleted;
                reload_list = false;
              }
              if (reload_list) {
                delete_load_file_list();
                int max_page = 0;
                if (!g_d_lines.empty()) {
                  max_page = ((int)g_d_lines.size() - 1) / 6;
                }
                if (d_page > max_page) d_page = max_page;
              }
              ui_draw_list(g_d_lines, d_page, -1);
            }
          }
          break;
        }
        case UIMode::QSO: {
          if (!g_q_show_entries) {
            if (c == ';') {
              if (q_page > 0) { q_page--; qso_draw_page(); }
            } else if (c == '.') {
              if ((q_page + 1) * 6 < (int)g_q_lines.size()) { q_page++; qso_draw_page(); }
            } else if (c >= '1' && c <= '6') {
              int idx = q_page * 6 + (c - '1');
              if (idx >= 0 && idx < (int)g_q_files.size()) {
                const std::string selected_file = g_q_files[idx];
                if (selected_file != g_q_current_file) {
                  g_q_page_view = QPageView::Default;
                }
                g_q_current_file = selected_file;
                g_q_show_entries = true;
                q_page = 0;
                qso_load_entries(g_q_current_file);
                qso_draw_page();
              }
            }
          } else {
            if (c == ',') {  // left: default view (time / band / call)
              if (g_q_page_view != QPageView::Default) {
                g_q_page_view = QPageView::Default;
                qso_rebuild_entry_lines();
                qso_draw_page();
              }
            } else if (c == '/') {  // right: alternate view (call / R-SNR / S-SNR)
              if (g_q_page_view != QPageView::Alternate) {
                g_q_page_view = QPageView::Alternate;
                qso_rebuild_entry_lines();
                qso_draw_page();
              }
            } else if (c == ';') {
              if (q_page > 0) { q_page--; qso_load_entries(g_q_current_file); qso_draw_page(); }
            } else if (c == '.') {
              if (g_q_entries_have_next_page) { q_page++; qso_load_entries(g_q_current_file); qso_draw_page(); }
            } else if (c == '`') {
              // back to file list
              g_q_show_entries = false;
              q_page = 0;
              qso_load_file_list();
              qso_draw_page();
            }
          }
          break;
        }
        case UIMode::MSC:
          break;
        case UIMode::MENU: {
          if (ui_mode == UIMode::MENU) {
            if (menu_long_edit) {
              if (c == '\n' || c == '\r') {
                if (menu_long_kind == LONG_FT) {
                  g_free_text = menu_long_buf;
                  if (g_cq_type == CqType::CQFREETEXT) g_cq_freetext = g_free_text;
                  update_autoseq_cq_type();
                } else if (menu_long_kind == LONG_COMMENT) {
                  g_comment1 = menu_long_buf;
                } else if (menu_long_kind == LONG_ACTIVE) {
                  g_active_band_text = menu_long_buf;
                  rebuild_active_bands();
                } else if (menu_long_kind == LONG_IGNORE) {
                  g_ignore_prefix_text = clamp_ignore_prefix_text(menu_long_buf);
                  rebuild_ignore_prefixes();
                }
                save_station_data();
                menu_long_edit = false;
                menu_long_kind = LONG_NONE;
                menu_long_buf.clear();
                menu_long_backup.clear();
                draw_menu_view();
              } else if (c == '`') {
                menu_long_edit = false;
                menu_long_kind = LONG_NONE;
                menu_long_buf.clear();
                menu_long_backup.clear();
                draw_menu_view();
              } else if (c == 0x08 || c == 0x7f) {
                if (!menu_long_buf.empty()) menu_long_buf.pop_back();
                draw_menu_view();
              } else if (c >= 32 && c < 127) {
                char ch = c;
                if (menu_long_kind == LONG_FT || menu_long_kind == LONG_IGNORE) {
                  ch = toupper((unsigned char)ch);
                }
                if (!(menu_long_kind == LONG_IGNORE &&
                      menu_long_buf.size() >= kIgnorePrefixTextMaxLen)) {
                  menu_long_buf.push_back(ch);
                }
                draw_menu_view();
              }
              break;
            } else if (menu_edit_idx >= 0) {
              if (c == '\n' || c == '\r') {
                bool should_save = true;
                // Absolute indices across pages
                if (menu_edit_idx == 3) { g_call = menu_edit_buf; autoseq_set_station(g_call, grid_ft8_4(g_grid)); }
                else if (menu_edit_idx == 4) {
                  const std::string norm_grid = normalize_grid_maidenhead(menu_edit_buf);
                  if (!norm_grid.empty()) {
                    g_grid = norm_grid;
                    g_grid_saved_manual = g_grid;
                    g_grid_from_gps = false;
                    autoseq_set_station(g_call, grid_ft8_4(g_grid));
                  } else {
                    should_save = false;
                    debug_log_line("Grid format: AA00/AA00aa/AA00aa00");
                  }
                }
                else if (menu_edit_idx == 7) { g_offset_hz = atoi(menu_edit_buf.c_str()); redraw_countdown_now(); }
                else if (menu_edit_idx == 10) { g_comment1 = menu_edit_buf; }
                else if (menu_edit_idx == 15) {
                  char* end = nullptr;
                  long v = std::strtol(menu_edit_buf.c_str(), &end, 10);
                  if (end != menu_edit_buf.c_str() && end && *end == '\0') {
                    g_rtc_comp = clamp_rtc_comp_value((int)v);
                  }
                } else if (menu_edit_idx == 17) {
                  char* end = nullptr;
                  long v = std::strtol(menu_edit_buf.c_str(), &end, 10);
                  if (end != menu_edit_buf.c_str() && end && *end == '\0') {
                    if (v < 0) v = 0;
                    g_autoseq_max_retry = (int)v;
                    autoseq_set_max_retry(g_autoseq_max_retry);
                  }
                }
                if (should_save) {
                  save_station_data();
                }
                menu_edit_idx = -1;
                menu_edit_buf.clear();
                draw_menu_view();
              } else if (c == 0x08 || c == 0x7f) {
                if (!menu_edit_buf.empty()) menu_edit_buf.pop_back();
                draw_menu_view();
                if (menu_edit_idx == 7) {
                  g_offset_hz = atoi(menu_edit_buf.c_str());
                  redraw_countdown_now();
                }
              } else if (c == '`') {
                if (menu_edit_idx == 7) {
                  g_offset_hz = menu_cursor_edit_original;
                  redraw_countdown_now();
                }
                menu_edit_idx = -1;
                menu_edit_buf.clear();
                draw_menu_view();
              } else if (menu_edit_idx == 7 && (c == ';' || c == '.' || c == ',' || c == '/')) {
                // Arrow mode starts from the currently shown edit value.
                int cursor_val = g_offset_hz;
                if (!menu_edit_buf.empty()) {
                  cursor_val = atoi(menu_edit_buf.c_str());
                }
                if (c == ';') cursor_val += 100;
                else if (c == '.') cursor_val -= 100;
                else if (c == ',') cursor_val -= 10;
                else cursor_val += 10; // '/'
                // Clamp applies only to arrow mode.
                if (cursor_val < 200) cursor_val = 200;
                if (cursor_val > 3000) cursor_val = 3000;
                g_offset_hz = cursor_val;
                menu_edit_buf = std::to_string(cursor_val);
                draw_menu_view();
                redraw_countdown_now();
              } else if (c >= 32 && c < 127) {
                char ch = c;
                if (menu_edit_idx == 15) {
                  const bool is_sign = (ch == '+' || ch == '-');
                  const bool is_digit = (ch >= '0' && ch <= '9');
                  if (is_sign) {
                    if (!menu_edit_buf.empty()) break;
                  } else if (!is_digit) {
                    break;
                  }
                  if (menu_edit_buf.size() >= 11) break;
                } else if (menu_edit_idx == 17) {
                  if (ch < '0' || ch > '9') break;
                  if (menu_edit_buf.size() >= 10) break;
                }
                if (menu_edit_idx % 6 == 3 || menu_edit_idx % 6 == 4 || menu_edit_idx % 6 == 5) {
                  ch = toupper((unsigned char)ch);
                }
                menu_edit_buf.push_back(ch);
                draw_menu_view();
                if (menu_edit_idx == 7) {
                  g_offset_hz = atoi(menu_edit_buf.c_str());
                  redraw_countdown_now();
                }
              }
              break;
            }

        if (c == ';') {
          if (menu_page > 0) { menu_page--; draw_menu_view(); }
        } else if (c == '.') {
          if (menu_page < 2) { menu_page++; draw_menu_view(); }
        } else if (menu_page == 0) {
              if (c == '1') {
                g_cq_type = (CqType)(((int)g_cq_type + 1) % 6);
                if (g_cq_type == CqType::CQFREETEXT) g_cq_freetext = g_free_text;
                save_station_data();
                update_autoseq_cq_type();
                draw_menu_view();
              } else if (c == '2') {
                // Send Free Text via autoseq queue. FT is a one-shot entry
                // that sorts to the FRONT of the active queue — guarantees
                // the next TX is the FT, preempting any active QSO. The QSO
                // ctx is preserved (FT is one-shot, popped after TX) and
                // resumes on the slot after FT fires.
                // Slot parity: inherits from queue[0] if non-empty (joins
                // the current activation period); uses next-slot fallback
                // if empty.
                int64_t now_slot = rtc_now_ms() / g_protocol->slot_time_ms;
                int fallback_parity = (int)((now_slot + 1) & 1);
                if (autoseq_schedule_freetext(g_free_text, fallback_parity)) {
                  // Re-fetch and update g_pending_tx so the FT replaces any
                  // previously-scheduled QSO TX. Without this, a QSO TX
                  // that was already armed by a prior decode cycle would
                  // still fire instead of the FT.
                  AutoseqTxEntry pending;
                  if (autoseq_fetch_pending_tx(pending)) {
                    arm_pending_tx(pending);
                  }
                  menu_flash_idx = 1; // absolute index of "Send FreeText"
                  menu_flash_deadline = rtc_now_ms() + 500;
                  draw_menu_view();
                  debug_log_line(std::string("Queued: ") + g_free_text);
                }
              } else if (c == '3') {
                menu_long_edit = true;
                menu_long_kind = LONG_FT;
                menu_long_buf = g_free_text;
                menu_long_backup = g_free_text;
                draw_menu_view();
              } else if (c == '4') {
                menu_edit_idx = 3; // Call (line index 3)
                menu_edit_buf = g_call;
                draw_menu_view();
              } else if (c == '5') {
                menu_edit_idx = 4; // Grid (line index 4)
                menu_edit_buf = g_grid;
                draw_menu_view();
              } else if (c == '6') {
                ESP_LOGI(TAG, "Entering deep sleep (GPIO0 wake)");
                // Save current accurate time for compensation after wake-up
                if (rtc_valid) {
                  g_rtc_sleep_epoch = rtc_epoch_base +
                      (esp_timer_get_time() / 1000 - rtc_ms_start) / 1000;
                  rtc_sync_to_esp_rtc();
                  save_station_data();
                  ESP_LOGI(TAG, "Saved sleep epoch: %ld, comp=%d",
                           (long)g_rtc_sleep_epoch, g_rtc_comp);
                }
                M5.Display.sleep();
                vTaskDelay(pdMS_TO_TICKS(100));
                // Configure GPIO0 as wake-up source (active low)
                esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
                esp_deep_sleep_start();
              }
            } else if (menu_page == 1) {
                if (c == '1') {
                  g_offset_src = (OffsetSrc)(((int)g_offset_src + 1) % 3);
                  save_station_data();
                  draw_menu_view();
                } else if (c == '2') {
                  menu_edit_idx = 7; // Cursor line
                  menu_cursor_edit_original = g_offset_hz;
                  menu_edit_buf = std::to_string(g_offset_hz);
                  draw_menu_view();
                } else if (c == '3') {
                  RadioType old_radio = canonical_radio_type(g_radio);
                  audio_source_backend_t old_audio = get_radio_profile_binding(old_radio).audio_backend;
                  bool was_streaming = audio_source_is_streaming();
                  switch (canonical_radio_type(g_radio)) {
                    case RadioType::QMX:
                      g_radio = RadioType::QDX;
                      break;
                    case RadioType::QDX:
                      g_radio = RadioType::KH1_USBC;
                      break;
                    case RadioType::KH1_USBC:
                      g_radio = RadioType::KH1_MIC;
                      break;
                    case RadioType::KH1_MIC:
                    default:
                      g_radio = RadioType::QMX;
                      break;
                  }
                  RadioType new_radio = canonical_radio_type(g_radio);
                  audio_source_backend_t new_audio = get_radio_profile_binding(new_radio).audio_backend;
                  if (was_streaming && old_audio != new_audio) {
                    ESP_LOGI(TAG, "Stopping audio for radio change %s/%s -> %s/%s",
                             radio_name(old_radio),
                             audio_source_backend_name(old_audio),
                             radio_name(new_radio),
                             audio_source_backend_name(new_audio));
                    debug_log_line(std::string("Audio stop ") + radio_name(old_radio));
                    audio_source_stop();
                  }
                  apply_radio_profile_binding();
                  save_station_data();
                  draw_menu_view();
                } else if (c == '4') {
                  menu_long_edit = true;
                  menu_long_kind = LONG_IGNORE;
                  menu_long_buf = g_ignore_prefix_text;
                  menu_long_backup = g_ignore_prefix_text;
                  draw_menu_view();
                } else if (c == '5') {
                  menu_long_edit = true;
                  menu_long_kind = LONG_COMMENT;
                  menu_long_buf = g_comment1;
                  menu_long_backup = g_comment1;
                  draw_menu_view();
                } else if (c == '6') {
#if ENABLE_FT4
                  // Toggle the pending protocol mode (FT8 <-> FT4).
                  // g_protocol stays as-is for this boot session; the change
                  // takes effect on next reboot.
                  g_protocol_pending_ft4 = !g_protocol_pending_ft4;
                  save_station_data();
                  draw_menu_view();
#endif
                }
            } else if (menu_page == 2) {
              if (c == '1') {
                g_rxtx_log = !g_rxtx_log;
                save_station_data();
                draw_menu_view();
              } else if (c == '2') {
                g_skip_tx1 = !g_skip_tx1;
                autoseq_set_skip_tx1(g_skip_tx1);
                save_station_data();
                draw_menu_view();
              } else if (c == '3') {
                menu_long_edit = true;
                menu_long_kind = LONG_ACTIVE;
                menu_long_buf = g_active_band_text;
                menu_long_backup = g_active_band_text;
                draw_menu_view();
              } else if (c == '4') {
                gps_stop();
                g_gnss_lora_enabled = !g_gnss_lora_enabled;
                apply_debug_uart_pin_policy();
                save_station_data();
                apply_radio_profile_binding();
                draw_menu_view();
              } else if (c == '5') {
                CopyLogsResult copy_res = copy_logs_to_sd_overwrite();
                menu_flash_idx = 16; // abs index of page 2 line 5
                menu_flash_deadline = rtc_now_ms() + 500;
                if (copy_res.status == StorageCopyStatus::SD_MOUNT_FAILED) {
                  menu_copy_feedback_text = "SD mount fail";
                } else if (copy_res.status == StorageCopyStatus::STORAGE_BUSY) {
                  menu_copy_feedback_text = "Storage busy";
                } else if (copy_res.status == StorageCopyStatus::LIST_FAILED) {
                  menu_copy_feedback_text = "List failed";
                } else if (copy_res.missed_count <= 0) {
                  menu_copy_feedback_text = "Copied OK";
                } else if (copy_res.missed_count == 1 && !copy_res.missed_files.empty()) {
                  menu_copy_feedback_text =
                      std::string("Fail ") + head_trim(storage_basename(copy_res.missed_files.front()), 14);
                } else {
                  menu_copy_feedback_text =
                      std::string("Missed ") + std::to_string(copy_res.missed_count) + ", see log";
                }
                if (menu_copy_feedback_text.size() > 19) {
                  menu_copy_feedback_text.resize(19);
                }
                menu_copy_feedback_deadline = rtc_now_ms() + kMenuCopyFeedbackMs;

                char log_msg[64];
                snprintf(log_msg, sizeof(log_msg), "Copy SD C%d M%d",
                         copy_res.copied_count, copy_res.missed_count);
                debug_log_line(log_msg);
                if (copy_res.err == ESP_OK) {
                  debug_log_line("Copied storage files to SD");
                } else if (!copy_res.missed_files.empty()) {
                  debug_log_line(std::string("Copy fail: ") +
                                 head_trim(storage_basename(copy_res.missed_files.front()), 16));
                }

                draw_menu_view();
              } else if (c == '6') {
                menu_edit_idx = 17; // Max Retry line
                menu_edit_buf = std::to_string(g_autoseq_max_retry);
                draw_menu_view();
              }
            }
          }
          break;
        }
      }
    }


    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

extern "C" void app_main(void) {
  // Run the main application loop on core0.
  xTaskCreatePinnedToCore(app_task_core0, "app_core0", APP_CORE0_STACK_BYTES, nullptr, 5, nullptr, 0);
}
static void draw_status_line(int idx, const std::string& text, bool highlight) {
  const int line_h = 19;
  const int start_y = UI_START_Y;
  int y = start_y + idx * line_h;
  uint16_t bg = highlight ? M5.Display.color565(30, 30, 60) : TFT_BLACK;
  M5.Display.setTextSize(2);
  M5.Display.fillRect(0, y, 240, line_h, bg);
  M5.Display.setTextColor(TFT_WHITE, bg);
  M5.Display.setCursor(0, y);
  char buf[160];
  std::snprintf(buf, sizeof(buf), "%d %s", idx + 1, text.c_str());
  ui_set_visible_text_line(idx, buf);
  M5.Display.printf("%s", buf);
}
[[maybe_unused]] static void draw_battery_icon(int x, int y, int w, int h, int level, bool charging) {
  if (level < 0) level = 0;
  if (level > 100) level = 100;
  // Outline
  M5.Display.startWrite();
  M5.Display.fillRect(x, y, w, h, TFT_BLACK);
  M5.Display.drawRect(x, y, w - 3, h, TFT_WHITE);
  M5.Display.fillRect(x + w - 3, y + h / 4, 3, h / 2, TFT_WHITE); // tab
  // Fill
  int inner_w = w - 5;
  int inner_h = h - 4;
  int fill_w = (inner_w * level) / 100;
  uint16_t fill_color = (level > 30) ? M5.Display.color565(0, 200, 0)
                        : (level > 15) ? M5.Display.color565(200, 180, 0)
                                        : M5.Display.color565(200, 0, 0);
  M5.Display.fillRect(x + 2, y + 2, fill_w, inner_h, fill_color);
  // Charging bolt
  if (charging) {
    int bx = x + w / 2 - 2;
    int by = y + 2;
    M5.Display.fillTriangle(bx, by, bx + 4, by + h / 2, bx + 2, by, M5.Display.color565(255, 255, 0));
    M5.Display.fillTriangle(bx + 2, by + h / 2, bx + 6, by + h - 2, bx + 4, by + h - 2, M5.Display.color565(255, 255, 0));
  }
  M5.Display.endWrite();
}
