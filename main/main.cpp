#define DEBUG_LOG 1

#include <cstdio>
#include <cmath>
#include "esp_log.h"
#include "wear_levelling.h"
extern "C" {
  #include "ft8/decode.h"
  #include "ft8/constants.h"
  #include "ft8/message.h"
  #include "ft8/encode.h"
  #include "ft8/debug.h"
  #include "common/monitor.h"
  }

#include "ui.h"
#include <vector>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
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
#include <cstring>
#include <unordered_map>
#include <algorithm>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <memory>
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
#include "radio_control.h"
#include "radio_control_backend.h"
#include "gps.h"

#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_partition.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"

static const char* STATION_FILE = "/storage/Station.txt";
static sdmmc_card_t* g_sd_card = NULL;
static bool g_sd_mounted = false;
static bool g_ble_enabled = true;

#define ENABLE_BLE 1

#if ENABLE_BLE
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"
#include "esp_bt.h"
#include "esp_mac.h"
#include "ble_native.h"

#endif
#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 6000
#endif

#define BLE_UI_SVC_UUID   0xFFE0
#define BLE_UI_RX_UUID    0xFFE1
#define BLE_UI_TX_UUID    0xFFE2

#if ENABLE_BLE
static const ble_uuid16_t ble_ui_svc_uuid = BLE_UUID16_INIT(BLE_UI_SVC_UUID);
static const ble_uuid16_t ble_ui_rx_uuid = BLE_UUID16_INIT(BLE_UI_RX_UUID);
static const ble_uuid16_t ble_ui_tx_uuid = BLE_UUID16_INIT(BLE_UI_TX_UUID);
#endif

static constexpr size_t BLE_UI_INPUT_MAX = 160;
struct BleUiInput {
    uint16_t len = 0;
    char data[BLE_UI_INPUT_MAX] = {};
};


#if ENABLE_BLE

static QueueHandle_t ble_cmd_queue = nullptr;
static uint16_t gatt_tx_handle = 0;
static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool g_ble_synced = false;
static bool g_ble_force_send = false;
static std::string g_ble_adv_name;
static std::string g_ble_last_payload;
static std::string g_ble_last_screen_lines[6];
static std::string g_ble_last_line7;
static bool g_ble_last_screen_valid = false;
static int64_t g_ble_status_clock_slot_sent = -1;
static int64_t g_ble_gps_slot_sent = -1;
static int64_t g_ble_last_tick_slot = -1;
static int g_ble_last_tick_sec = -1;
static std::string g_ble_waterfall_header = "|                           |";
static int64_t g_ble_waterfall_slot_idx = -1;
static bool g_ble_text_mode = false;
static volatile uint32_t g_ble_decode_event_seq = 0;
static volatile int g_ble_decode_event_count = 0;
static uint32_t g_ble_decode_event_seq_seen = 0;
static void ble_publish_decode_event(int decoded_count) {
  g_ble_decode_event_count = decoded_count;
  g_ble_decode_event_seq = g_ble_decode_event_seq + 1;
}
static const char* BT_TAG = "BLE_INIT";

enum class BleDumpTxMode : uint8_t {
    Notify = 0,
    Indicate = 1,
};

struct BleDumpTransferState {
    bool active = false;
    BleDumpTxMode mode = BleDumpTxMode::Notify;
    int file_lines = 0;
    int retries = 0;
    int failed_lines = 0;
    int notify_pace_ms = 8;
    uint16_t mtu = 23;
};

static BleDumpTransferState g_ble_dump_xfer{};
static volatile bool g_ble_tx_notify_enabled = false;
static volatile bool g_ble_tx_indicate_enabled = false;
static volatile bool g_ble_indicate_waiting = false;
static volatile int g_ble_indicate_status = 0;
static volatile uint16_t g_ble_att_mtu = 23;

static constexpr int kBleDumpIndicateAckTimeoutMs = 1500;
static constexpr int kBleDumpIndicateMaxRetries = 3;
static constexpr int kBleDumpNotifyMaxRetries = 4;
static constexpr int kBleDumpNotifyBackoffMs[kBleDumpNotifyMaxRetries] = {5, 10, 20, 40};
static constexpr int kBleDumpNotifyPaceMinMs = 8;
static constexpr int kBleDumpNotifyPaceMaxMs = 20;

static int gap_cb(struct ble_gap_event *event, void *arg);
static void nimble_host_task(void *param);
static void ble_on_sync(void);
static void ble_app_advertise(void);
static void ble_update_name_from_station(bool restart_adv);
static void ble_countdown_tick();
static std::string ble_blank_waterfall_header();
static void ble_update_waterfall_header_if_due(int64_t slot_idx, int sec);
static int ble_send_payload_raw(const std::string& payload, bool indicate);
static bool ble_wait_for_indicate_ack(int timeout_ms);
static void ble_dump_reset_transfer_state(bool use_indicate);
static bool ble_dump_send_line(const std::string& raw);

static std::string ble_trim_trailing_crlf(const char* data, uint16_t len)
{
    if (!data || len == 0) return std::string();
    size_t used = len;
    while (used > 0 && (data[used - 1] == '\r' || data[used - 1] == '\n')) used--;
    return std::string(data, data + used);
}

static char ble_parse_ui_command(const char* data, uint16_t len)
{
    const std::string payload = ble_trim_trailing_crlf(data, len);
    if (payload.size() != 1) return 0;  // command mode is single-character only
    char c = payload[0];
    if (c >= 'a' && c <= 'z') c = static_cast<char>(c - ('a' - 'A'));

    if (c >= '1' && c <= '6') return c;
    switch (c) {
      case 'S':
      case 'R':
      case 'T':
      case 'G':
      case 'M':
      case 'Q':
      case 'B':
      case 'F':
      case 'D':
      case 'N':
      case 'O':
        return c;
      case 'U':  // page up
        return ';';
      case 'V':  // page down
        return '.';
      case 'Z':  // left (Zuo)
        return ',';
      case 'X':  // right (You)
        return '/';
      case 'E':  // ESC / TX cancel
        return '`';
      default:
        return 0;
    }
}

static int ble_ui_rx_cb(uint16_t conn_handle,
                        uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt,
                        void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (!ble_cmd_queue || !ctxt || !ctxt->om) return 0;
    BleUiInput input{};
    size_t copy_len = ctxt->om->om_len;
    if (copy_len > BLE_UI_INPUT_MAX) copy_len = BLE_UI_INPUT_MAX;
    input.len = static_cast<uint16_t>(copy_len);
    if (copy_len > 0) {
      std::memcpy(input.data, ctxt->om->om_data, copy_len);
    }
    xQueueSend(ble_cmd_queue, &input, 0);
    return 0;  // ignore unsupported input silently
}

static int ble_ui_tx_cb(uint16_t conn_handle,
                        uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt,
                        void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)ctxt;
    (void)arg;
    return 0;
}

#include "esp_nimble_hci.h"

// C++-safe static characteristics table (fully initialized for -Werror).
static const struct ble_gatt_chr_def gatt_uart_chrs[] = {
    {
        &ble_ui_rx_uuid.u,                       // uuid
        ble_ui_rx_cb,                            // access_cb
        nullptr,                                 // arg
        nullptr,                                 // descriptors
        BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP, // flags
        0,                                       // min_key_size
        nullptr,                                 // val_handle
        nullptr,                                 // cpfd
    },
    {
        &ble_ui_tx_uuid.u,                       // uuid
        ble_ui_tx_cb,                            // access_cb
        nullptr,                                 // arg
        nullptr,                                 // descriptors
        BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE, // flags
        0,                                       // min_key_size
        &gatt_tx_handle,                         // val_handle
        nullptr,                                 // cpfd
    },
    {
        nullptr,                                 // uuid terminator
        nullptr,                                 // access_cb
        nullptr,                                 // arg
        nullptr,                                 // descriptors
        0,                                       // flags
        0,                                       // min_key_size
        nullptr,                                 // val_handle
        nullptr,                                 // cpfd
    },
};

// C++-safe service table (fully initialized for -Werror).
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        BLE_GATT_SVC_TYPE_PRIMARY,               // type
        &ble_ui_svc_uuid.u,                      // uuid
        nullptr,                                 // includes
        gatt_uart_chrs,                          // characteristics
    },
    {
        0,                                       // type terminator
        nullptr,                                 // uuid
        nullptr,                                 // includes
        nullptr,                                 // characteristics
    }
};


static void init_bluetooth(void)
{
    static bool inited = false;
    if (inited) return;
    inited = true;
    ESP_LOGI(BT_TAG, "init_bluetooth start");

    esp_err_t nvrc = nvs_flash_init();
    if (nvrc == ESP_ERR_NVS_NO_FREE_PAGES || nvrc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvrc = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvrc);

    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "nimble_port_init failed: %d", rc);
        return;
    }
    ESP_LOGI(BT_TAG, "nimble_port_init OK");

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_LOGI(BT_TAG, "GAP/GATT init done");

    ble_update_name_from_station(false);

    // Register the native client GATT service (handles its own count_cfg + add_svcs).
    // The former text-terminal service was removed; the mobile app is the
    // only BLE client now.
    if (!ble_native_init()) {
        ESP_LOGE(BT_TAG, "ble_native_init failed");
        return;
    }
    ESP_LOGI(BT_TAG, "Native BLE service registered");

    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(nimble_host_task);
    ESP_LOGI(BT_TAG, "Host task started");
}

// One-way teardown of the BLE stack. Releases the BT controller pool
// (~60 KB) back to the heap via esp_bt_controller_mem_release. Once
// called, BLE cannot be re-enabled until the chip resets — used only
// from enter_msc_mode where MSC owns the system until reboot.
static void nimble_teardown(void)
{
    ESP_LOGI(BT_TAG, "nimble_teardown start");
    uint32_t before = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);

    // 1. Drop active connection (if any) and stop advertising.
    if (g_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(g_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    }
    if (g_ble_synced) {
        ble_gap_adv_stop();
        g_ble_synced = false;
    }

    // 2. Stop the ble_native TX task BEFORE deinit so it can't call
    // ble_gatts_* against torn-down state.
    ble_native_shutdown();

    // 3. Stop the host event loop, wait for nimble_host_task to exit,
    // then deinit (which internally also disables + deinits the BT
    // controller via the IDF NimBLE port).
    nimble_port_stop();
    nimble_port_freertos_deinit();
    nimble_port_deinit();

    // 4. Release the BT controller's static BSS+data back to the heap.
    // This is the big win — the controller pool is the bulk of the
    // ~60 KB we want back for TinyUSB MSC.
    esp_err_t err = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (err != ESP_OK) {
        ESP_LOGW(BT_TAG, "esp_bt_controller_mem_release: %s",
                 esp_err_to_name(err));
    }

    uint32_t after = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    ESP_LOGI(BT_TAG, "nimble_teardown done: heap %u -> %u (+%d)",
             (unsigned)before, (unsigned)after, (int)(after - before));
}

static int gap_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            g_conn_handle = event->connect.conn_handle;
            if (!g_ble_enabled) {
              ble_gap_terminate(g_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
              g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
              return 0;
            }
            g_ble_force_send = true;
            g_ble_last_screen_valid = false;
            g_ble_status_clock_slot_sent = -1;
            g_ble_gps_slot_sent = -1;
            g_ble_last_tick_slot = -1;
            g_ble_last_tick_sec = -1;
            g_ble_waterfall_slot_idx = -1;
            g_ble_waterfall_header = ble_blank_waterfall_header();
            g_ble_text_mode = false;
            g_ble_tx_notify_enabled = false;
            g_ble_tx_indicate_enabled = false;
            g_ble_indicate_waiting = false;
            g_ble_indicate_status = 0;
            g_ble_att_mtu = 23;
            ble_native_on_connect(g_conn_handle);
            ESP_LOGI(BT_TAG, "Connected, handle=%u", g_conn_handle);
        } else {
            g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGW(BT_TAG, "Connect failed; restarting adv");
            ble_app_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        g_ble_last_payload.clear();
        g_ble_last_screen_valid = false;
        g_ble_last_line7.clear();
        g_ble_status_clock_slot_sent = -1;
        g_ble_gps_slot_sent = -1;
        g_ble_last_tick_slot = -1;
        g_ble_last_tick_sec = -1;
        g_ble_waterfall_slot_idx = -1;
        g_ble_waterfall_header = ble_blank_waterfall_header();
        g_ble_text_mode = false;
        g_ble_tx_notify_enabled = false;
        g_ble_tx_indicate_enabled = false;
        g_ble_indicate_waiting = false;
        g_ble_indicate_status = 0;
        g_ble_att_mtu = 23;
        if (ble_cmd_queue) xQueueReset(ble_cmd_queue);
        ble_native_on_disconnect();
        ESP_LOGW(BT_TAG, "Disconnected; restarting adv");
        ble_app_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.conn_handle == g_conn_handle &&
            event->subscribe.attr_handle == gatt_tx_handle) {
            g_ble_tx_notify_enabled = event->subscribe.cur_notify != 0;
            g_ble_tx_indicate_enabled = event->subscribe.cur_indicate != 0;
            ESP_LOGI(BT_TAG, "TX subscribe: notify=%d indicate=%d",
                     g_ble_tx_notify_enabled ? 1 : 0,
                     g_ble_tx_indicate_enabled ? 1 : 0);
        }
        // Forward all subscribe events to the native server — it tracks
        // its own characteristic handles and ignores unrelated events.
        ble_native_on_subscribe(event->subscribe.attr_handle,
                                event->subscribe.cur_notify != 0,
                                event->subscribe.cur_indicate != 0);
        break;

    case BLE_GAP_EVENT_NOTIFY_TX:
        // Indication confirmations pace the native log stream.
        ble_native_on_notify_tx(event->notify_tx.attr_handle,
                                event->notify_tx.status,
                                event->notify_tx.indication);
        if (event->notify_tx.conn_handle == g_conn_handle &&
            event->notify_tx.attr_handle == gatt_tx_handle &&
            event->notify_tx.indication &&
            g_ble_indicate_waiting) {
            const int st = event->notify_tx.status;
            if (st != 0) {
                g_ble_indicate_status = st;
                g_ble_indicate_waiting = false;
            }
        }
        break;

    case BLE_GAP_EVENT_MTU:
        if (event->mtu.conn_handle == g_conn_handle && event->mtu.value > 0) {
            g_ble_att_mtu = event->mtu.value;
            ble_native_on_mtu(event->mtu.value);
            ESP_LOGI(BT_TAG, "ATT MTU=%u", (unsigned)g_ble_att_mtu);
        }
        break;

    default:
        break;
    }
    return 0;
}


static bool ble_pop_input(BleUiInput& out) {
    if (!ble_cmd_queue) return false;
    return xQueueReceive(ble_cmd_queue, &out, 0) == pdTRUE;
}
#endif // ENABLE_BLE

int64_t rtc_now_ms();
struct CopyLogsResult {
  esp_err_t err = ESP_OK;
  int copied_count = 0;
  int missed_count = 0;
};
static esp_err_t copy_file_overwrite(const char* src_path, const char* dst_path);

static void debug_log_line(const std::string& msg);
//exported symbol (linkable from other .cpp)
void debug_log_line_public(const std::string& msg) {
  debug_log_line(msg);
}

//static const char *TAG = "sdtest";

#define PIN_NUM_MISO GPIO_NUM_39
#define PIN_NUM_MOSI GPIO_NUM_14
#define PIN_NUM_CLK  GPIO_NUM_40
#define PIN_NUM_CS   GPIO_NUM_12

void mount_sd_spi(void)
{
    esp_err_t ret;
    const char mount_point[] = "/sdcard";

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = PIN_NUM_MOSI;
    bus_cfg.miso_io_num = PIN_NUM_MISO;
    bus_cfg.sclk_io_num = PIN_NUM_CLK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4000;

    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = SPI2_HOST;

    esp_vfs_fat_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 5000;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &g_sd_card);
    if (ret != ESP_OK) {
        spi_bus_free(SPI2_HOST);
        g_sd_card = NULL;
        g_sd_mounted = false;
        return;
    }

    g_sd_mounted = true;
}

void unmount_sd_spi(const char *mount_point)
{
    if (g_sd_mounted && g_sd_card) {
        esp_vfs_fat_sdcard_unmount(mount_point, g_sd_card);
        g_sd_card = NULL;
        g_sd_mounted = false;
    }
    spi_bus_free(SPI2_HOST);
}

// ---------- Log copy/delete helpers ----------
static bool sdcard_is_mounted() {
  struct stat st;
  return (stat("/sdcard", &st) == 0) && S_ISDIR(st.st_mode);
}

static esp_err_t ensure_sdcard_mounted() {
  if (sdcard_is_mounted()) return ESP_OK;
  mount_sd_spi();
  if (sdcard_is_mounted()) return ESP_OK;
  return ESP_FAIL;
}

static void build_rxtx_log_path(char* path, size_t path_sz) {
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);

  // RT[YYMMDD].txt
  snprintf(path, path_sz, "/storage/RT%02d%02d%02d.txt",
           (t.tm_year + 1900) % 100,
           (t.tm_mon + 1) % 100,
           t.tm_mday % 100);
}

static bool file_exists(const char* path) {
  struct stat st;
  return (stat(path, &st) == 0) && S_ISREG(st.st_mode);
}

static void sync_station_txt_from_sd_to_spiffs() {
  static const char* TAG = "FT8";

  if (ensure_sdcard_mounted() != ESP_OK) {
    ESP_LOGI(TAG, "SD not mounted, using SPIFFS Station.txt");
    return;
  }

  const char* sd_path = "/sdcard/Station.txt";
  const char* spiffs_path = "/storage/Station.txt";

  if (!file_exists(sd_path)) {
    ESP_LOGI(TAG, "No Station.txt on SD, using SPIFFS Station.txt");
    unmount_sd_spi("/sdcard");
    return;
  }

  if (copy_file_overwrite(sd_path, spiffs_path) == ESP_OK) {
    ESP_LOGI(TAG, "Copied Station.txt from SD to SPIFFS");
  } else {
    ESP_LOGW(TAG, "Failed to copy Station.txt from SD, using SPIFFS Station.txt");
  }

  unmount_sd_spi("/sdcard");
}


static esp_err_t copy_file_overwrite(const char* src_path, const char* dst_path) {
  FILE* fs = fopen(src_path, "rb");
  if (!fs) return ESP_FAIL;

  FILE* fd = fopen(dst_path, "wb");  // overwrite
  if (!fd) { fclose(fs); return ESP_FAIL; }

  uint8_t buf[4096];
  size_t r = 0;

  while ((r = fread(buf, 1, sizeof(buf), fs)) > 0) {
    if (fwrite(buf, 1, r, fd) != r) {
      fclose(fd);
      fclose(fs);
      return ESP_FAIL;
    }
  }

  // Detect read error (not just EOF)
  if (ferror(fs)) {
    fclose(fd);
    fclose(fs);
    return ESP_FAIL;
  }

  // Ensure SD gets the bytes
  fflush(fd);
  fsync(fileno(fd));

  fclose(fd);
  fclose(fs);
  return ESP_OK;
}

static esp_err_t copy_file_overwrite_retry(const char* src_path, const char* dst_path,
                                           int max_attempts = 5, int retry_delay_ms = 80) {
  if (max_attempts < 1) max_attempts = 1;
  esp_err_t last_err = ESP_FAIL;
  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    last_err = copy_file_overwrite(src_path, dst_path);
    if (last_err == ESP_OK) return ESP_OK;
    if (attempt + 1 < max_attempts) {
      vTaskDelay(pdMS_TO_TICKS(retry_delay_ms));
    }
  }
  return last_err;
}

static bool collect_spiffs_regular_files(std::vector<std::string>& out_files) {
  out_files.clear();
  DIR* d = opendir("/storage");
  if (!d) return false;

  struct dirent* ent;
  while ((ent = readdir(d)) != nullptr) {
    const char* name = ent->d_name;
    if (!name || name[0] == '.') continue;
    std::string src = std::string("/storage/") + name;
    struct stat st;
    if (stat(src.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) continue;
    out_files.emplace_back(name);
  }
  closedir(d);
  std::sort(out_files.begin(), out_files.end());
  return true;
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

// Copy all regular files from SPIFFS -> SD card, overwriting destination.
static CopyLogsResult copy_logs_spiffs_to_sd_overwrite() {
  CopyLogsResult result{};
  esp_err_t mret = ensure_sdcard_mounted();
  if (mret != ESP_OK) {
    std::vector<std::string> files;
    int candidate_count = 0;
    if (collect_spiffs_regular_files(files)) {
      candidate_count = (int)files.size();
    }
    result.err = mret;
    result.missed_count = std::max(1, candidate_count);
    return result;
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  std::unordered_map<std::string, bool> copied_ok;
  bool scan_failed = false;

  auto set_copy_status = [&](const std::string& name, esp_err_t err) {
    auto it = copied_ok.find(name);
    if (err == ESP_OK) {
      copied_ok[name] = true;
      return;
    }
    if (it == copied_ok.end()) copied_ok[name] = false;
  };

  auto copy_pass = [&]() {
    std::vector<std::string> files;
    if (!collect_spiffs_regular_files(files)) {
      scan_failed = true;
      return;
    }
    for (const auto& name : files) {
      auto it = copied_ok.find(name);
      if (it != copied_ok.end() && it->second) continue;
      const std::string src = std::string("/storage/") + name;
      const std::string dst = std::string("/sdcard/") + name;
      esp_err_t err = copy_file_overwrite_retry(src.c_str(), dst.c_str());
      set_copy_status(name, err);
    }
  };

  copy_pass();
  vTaskDelay(pdMS_TO_TICKS(120));
  copy_pass();

  // Explicit post-pass for current-day QSO file.
  const std::string qso_name = today_qso_file_name();
  const std::string qso_src = std::string("/storage/") + qso_name;
  struct stat qso_st;
  if (stat(qso_src.c_str(), &qso_st) == 0 && S_ISREG(qso_st.st_mode)) {
    const std::string qso_dst = std::string("/sdcard/") + qso_name;
    esp_err_t err = copy_file_overwrite_retry(qso_src.c_str(), qso_dst.c_str(), 6, 100);
    set_copy_status(qso_name, err);
  }

  for (const auto& kv : copied_ok) {
    if (kv.second) result.copied_count++;
    else result.missed_count++;
  }
  if (scan_failed && copied_ok.empty()) {
    result.missed_count = std::max(1, result.missed_count);
  }
  result.err = (result.missed_count == 0) ? ESP_OK : ESP_FAIL;

  unmount_sd_spi("/sdcard");
  return result;
}

// 128 entries × 16 bytes = 2 KB of BSS. 256 was the original size but
// well over typical working set (FT8 rarely sees >50 unique hashed
// callsigns in an active period; the aging + eviction logic keeps it
// fresh). Reducing by 2 KB gives the USB DMA buffer (4608 bytes) a
// better chance at finding a contiguous block after BLE+USB host
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
enum class UIMode { RX, TX, BAND, MENU, MSC, DEBUG, STATUS, QSO, GPS };
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

// Set by stream_uac_task after a successful CDC-ACM open (QMX CAT USB-CDC
// enumeration). Consumed exactly once by the main loop to auto-sync VFO /
// mode to the radio — covers the "first connect" case so users don't have
// to press S->2 manually. QMX only enumerates once per power cycle and
// mini-ft8 restarts on QMX disconnect, so this flag effectively fires at
// most once per mini-ft8 lifetime. For KH1 (UART CAT, persistent), the
// STATUS-exit auto-sync in enter_mode handles the analogous case.
volatile bool g_cdc_initial_sync_pending = false;

// Deferred-save flag — see extern declaration above. Storage lives on
// main.cpp side; ble_native / core_api.cpp's RPC dispatch only flips it.
volatile bool g_config_save_pending = false;

// State machine variables (matching reference project architecture)
// TX is scheduled by setting these flags; actual TX starts at slot boundary
// Global TX-arming state: read by tx_tick on the next slot boundary.
// Non-static so core_api.cpp can arm it from the BLE tap_rx RPC, matching
// what the Cardputer's RX-key handler does inline.
volatile bool g_qso_xmit = false;        // TX is pending
volatile int g_target_slot_parity = 0;   // 0=even, 1=odd - parity of slot to TX on
static volatile bool g_was_txing = false;       // We were transmitting (for tick timing)
volatile bool g_decode_in_progress = false; // Block TX trigger while decoding
static int g_last_slot_parity = -1;             // For slot boundary detection (just parity, like reference)

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
static bool g_ble_qso_pick_mode = false;
static bool g_ble_dump_in_progress = false;
static UIMode g_ble_qso_return_mode = UIMode::RX;

static void enter_msc_mode(const char* reason);
void save_station_data();  // visible to core_api.cpp

// Storage helpers — defined alongside the FATFS mount code further
// down. enter_msc_mode (which appears earlier in the file) needs to
// unmount before handing the WL partition off to TinyUSB MSC.
static bool storage_is_mounted();
static void unmount_storage();

// Deferred-save flag. core_api.cpp's RPC handlers (running on the
// shallow ble_native task) flip this to ask the main task to flush
// station data — the 22-fprintf chain is too deep for that 4 KB
// stack. The main app_task_core0 has 12 KB and processes the flag
// once per loop tick (~10 ms latency).
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
static bool ble_pop_input(BleUiInput& out);
static void ble_update_name_from_station(bool restart_adv);
static void ble_mirror_tick();
static void ble_countdown_tick();
static void enter_mode(UIMode new_mode);
static void apply_ble_enabled_policy(bool runtime_apply);
static std::string menu_sleep_batt_line();
static int normalize_gps_baud_value(int value);
bool rtc_set_from_strings();   // visible to core_api.cpp
void rtc_sync_to_hw();         // visible to core_api.cpp
static bool g_rx_dirty = false;
#if ENABLE_BLE
static void ble_enter_text_mode();
static void ble_exit_text_mode();
static bool ble_text_target_active();
static void ble_commit_text_input(const BleUiInput& input);
static void ble_start_qso_pick_mode();
static void ble_cancel_qso_pick_mode();
static void ble_try_dump_qso_file_by_key(char key);
#endif



static std::vector<std::string> g_msc_lines = {
    "Mini-FT8 USB drive",
    "now mounted on PC.",
    "",
    "Copy ADIF logs,",
    "then press any key",
    "to reboot."
};

static std::vector<std::string> g_startup_lines = {
    "** Mini-FT8 V2.0.1 *",
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
// Log entries come straight from core_api's reader — the one ADIF parser
// this firmware has. The viewer used to parse the files itself, which is how
// it ended up with a length-unaware field scanner that truncated any value
// containing a space (the default comment, "MiniFT8 /Radio", is one).
static QPageView g_q_page_view = QPageView::Default;
static std::vector<CoreLogEntry> g_q_entries;
static bool g_q_show_entries = false;
static int q_page = 0;
static std::string g_q_current_file;
static std::vector<std::string> g_d_lines;
static std::vector<std::string> g_d_files;
static int d_page = 0;
static QueueHandle_t s_key_inject_queue = nullptr;

// Software RTC
static time_t rtc_epoch_base = 0;
static int64_t rtc_ms_start = 0;
static int64_t rtc_last_update = 0;
static bool rtc_valid = false;

// RTC deep sleep compensation
// rtc_sleep_epoch: epoch time when entering deep sleep (for calculating elapsed time)
// rtc_comp is seconds per 10000 seconds and can be adjusted via MENU O page.
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
static constexpr size_t kIgnorePrefixTextMaxLen = 64;
std::string g_comment1 = "MiniFT8 /Radio";      // visible to core_api.cpp
static std::string g_ignore_prefix_text;
std::vector<std::string> g_ignore_prefixes;     // visible to core_api.cpp
static bool g_rxtx_log = true;
static RadioType canonical_radio_type(RadioType r);
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
static int g_tx_tone_idx = 0;              // Current tone index (0-78)
static int64_t g_tx_next_tone_time = 0;    // When to send next tone (ms)
static int64_t g_tx_slot_start_ms = 0;     // Slot boundary time for tone alignment
static uint8_t g_tx_tones[79];             // Encoded tones
static int g_tx_base_hz = 0;               // Base frequency for TA commands
static int64_t g_tx_slot_idx = 0;          // Slot index for autoseq_mark_sent
static bool g_tx_cat_ok = false;           // CAT available for this TX
static int g_tx_last_ta_int = -1;          // For TA command deduplication
static int g_tx_last_ta_frac = -1;

static SemaphoreHandle_t log_mutex = NULL;                       // Protects log_rxtx_line file access
static int menu_page = 0;
static int menu_edit_idx = -1;
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
// Non-static so core_api.cpp's BLE set_band RPC can push the CAT change
// immediately, matching what the Cardputer's STATUS-exit path does.
bool sync_radio_to_current_band(const char* reason);
static void menu_flash_tick();
static void rx_flash_tick();
#if ENABLE_BLE
static uint8_t g_own_addr_type;
#endif
static bool looks_like_grid(const std::string& s);
static bool looks_like_report(const std::string& s, int& out);
static std::string g_last_reply_text;
void rebuild_active_bands();   // visible to core_api.cpp
static void schedule_tx_if_idle();
static int64_t s_last_tx_slot_idx = -1000;  // Track last TX slot for retry scheduling
[[maybe_unused]] static bool g_sync_pending = false;
[[maybe_unused]] static int g_sync_delta_ms = 0;
static void enqueue_beacon_cq();
static void load_spiffs_regular_files(std::vector<std::string>& files);
static void qso_load_file_list();
static void qso_load_fetch_file_list();
static void delete_load_file_list();
static void qso_load_entries(const std::string& path);
static void qso_draw_page();

static void log_rxtx_line(char dir, int snr, int offset_hz, const std::string& text, int repeat_counter = -1);
static void log_adif_entry(const std::string& dxcall, const std::string& dxgrid, int rst_sent, int rst_rcvd);
#if !MIC_PROBE_APP
void log_heap(const char* tag) {
  size_t free_sz = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  ESP_LOGI(tag, "HEAP: free=%u min=%u largest=%u", (unsigned)free_sz, (unsigned)min_free, (unsigned)largest);
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

static void cabrillo_fd_ensure_header(const char* path, const std::string& mycall, const std::string& location) {
  struct stat st;
  if (stat(path, &st) == 0) return;

  FILE* f = fopen(path, "w");
  if (!f) return;

  fprintf(f, "START-OF-LOG: 3.0\n");
  fprintf(f, "CREATED-BY: Mini-FT8\n");
  fprintf(f, "CONTEST: ARRL-FIELD-DAY\n");
  fprintf(f, "CALLSIGN: %s\n", mycall.c_str());
  fprintf(f, "CATEGORY-OPERATOR: SINGLE-OP\n");
  fprintf(f, "CATEGORY-TRANSMITTER: ONE\n");
  fprintf(f, "CATEGORY-ASSISTED: NON-ASSISTED\n");
  fprintf(f, "CATEGORY-BAND: ALL\n");
  fprintf(f, "CATEGORY-MODE: MIXED\n");
  fprintf(f, "CATEGORY-POWER: LOW\n");
  fprintf(f, "CATEGORY-STATION: PORTABLE\n");
  fprintf(f, "LOCATION: %s\n", location.c_str());
  fprintf(f, "OPERATORS: %s\n", mycall.c_str());
  fprintf(f, "END-OF-LOG:\n");
  fclose(f);
}

static bool cabrillo_fd_truncate_end_marker(FILE* f) {
  if (!f) return false;

  if (fseek(f, 0, SEEK_END) != 0) return false;
  long file_end = ftell(f);
  if (file_end <= 0) return false;

  const long kMaxTail = 256;
  long tail_start = (file_end > kMaxTail) ? (file_end - kMaxTail) : 0;

  if (fseek(f, tail_start, SEEK_SET) != 0) return false;

  std::string tail;
  tail.resize((size_t)(file_end - tail_start));
  size_t n = fread(tail.data(), 1, tail.size(), f);
  tail.resize(n);

  // Find end of last non-empty line
  size_t line_end = tail.size();
  while (line_end > 0 && (tail[line_end - 1] == '\n' || tail[line_end - 1] == '\r')) {
    line_end--;
  }
  if (line_end == 0) return false;

  size_t line_start = tail.rfind('\n', line_end - 1);
  line_start = (line_start == std::string::npos) ? 0 : (line_start + 1);

  std::string last = tail.substr(line_start, line_end - line_start);
  if (last != "END-OF-LOG:") return false;

  long truncate_at = tail_start + (long)line_start;
  int fd = fileno(f);
  if (fd < 0) return false;
  if (ftruncate(fd, truncate_at) != 0) return false;

  // Seek to new end
  fseek(f, 0, SEEK_END);
  return true;
}

static void cabrillo_fd_append_qso_with_end(const char* path, const std::string& qso_line) {
  FILE* f = fopen(path, "r+");
  if (!f) {
    f = fopen(path, "a+");
    if (!f) return;
  }

  // Remove trailing END-OF-LOG if present
  cabrillo_fd_truncate_end_marker(f);

  // Append QSO and END-OF-LOG
  fseek(f, 0, SEEK_END);

  // Ensure newline separation
  long end = ftell(f);
  if (end > 0) {
    if (fseek(f, -1, SEEK_END) == 0) {
      int c = fgetc(f);
      fseek(f, 0, SEEK_END);
      if (c != '\n') fputc('\n', f);
    } else {
      fseek(f, 0, SEEK_END);
    }
  }

  fprintf(f, "%s\n", qso_line.c_str());
  fprintf(f, "END-OF-LOG:\n");
  fclose(f);
}

// Called by autoseq when an FD QSO completes. We derive freq/time from current radio state
// and use FreeText as our FD exchange (e.g. "1B SCV").
static void log_cabrillo_fd_entry(const std::string& dxcall, const std::string& their_fd_exchange) {
  if (g_cq_type != CqType::CQFD) return;

  const std::string my_fd = fd_strip_R(g_free_text);
  const std::string their_fd = fd_strip_R(their_fd_exchange);

  if (my_fd.empty() || their_fd.empty() || dxcall.empty()) return;

  // Time (UTC assumed as RTC timebase, same as ADIF writer)
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);

  char date_ymd[16];
  snprintf(date_ymd, sizeof(date_ymd), "%04d-%02d-%02d",
           (t.tm_year + 1900) % 10000, (t.tm_mon + 1) % 100, t.tm_mday % 100);

  char time_hhmm[8];
  snprintf(time_hhmm, sizeof(time_hhmm), "%02d%02d", t.tm_hour % 100, t.tm_min % 100);

  // Frequency: use selected band dial frequency (kHz)
  int freq_khz = (int)g_bands[g_band_sel].freq;

  const char* path = "/storage/fieldday.txt";

  std::string location = fd_get_section_from_exchange(my_fd);
  cabrillo_fd_ensure_header(path, g_call, location);

  char qso_line[128];
  snprintf(qso_line, sizeof(qso_line), "QSO: %d DG %s %s %s %s %s %s",
           freq_khz,
           date_ymd,
           time_hhmm,
           g_call.c_str(),
           my_fd.c_str(),
           dxcall.c_str(),
           their_fd.c_str());

  cabrillo_fd_append_qso_with_end(path, qso_line);
}

#else
static inline void log_heap(const char*) {}
#endif

static void log_rxtx_line(char dir, int snr, int offset_hz, const std::string& text, int repeat_counter) {
  if (!g_rxtx_log) return;
  if (!log_mutex) return;  // Not initialized yet

  // Prepare log line outside mutex
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

  // Take mutex for file access
  if (xSemaphoreTake(log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    ESP_LOGW(TAG, "RxTxLog mutex timeout");
    return;
  }

  FILE* f = fopen(log_path, "a");
  if (!f) {
    ESP_LOGW(TAG, "RxTxLog open failed: %s", log_path);
    xSemaphoreGive(log_mutex);
    return;
  }

  // For TX, omit SNR and repeat; for RX keep SNR.
  if (dir == 'T') {
    fprintf(f, "%c [%s][%.3f] %s %d\n",
            dir, ts, freq_mhz, text.c_str(), offset_hz);
  } else {
    fprintf(f, "%c [%s][%.3f] %s %d %d\n",
            dir, ts, freq_mhz, text.c_str(), snr, offset_hz);
  }

  fclose(f);
  xSemaphoreGive(log_mutex);
}

static void qso_load_file_list() {
  g_q_files.clear();
  g_q_entries.clear();
  g_q_lines.clear();

  CoreLogDay days[32];
  const int total = core_log_list_days(days, 32);
  const int shown = total < 32 ? total : 32;
  for (int i = 0; i < shown; ++i) {
    g_q_files.emplace_back(std::string(days[i].date) + ".txt");
    g_q_lines.push_back(g_q_files.back());
  }

  if (g_q_files.empty()) {
    g_q_lines.push_back("No QSO logs");
  }
}

static void load_spiffs_regular_files(std::vector<std::string>& files) {
  files.clear();
  DIR* dir = opendir("/storage");
  if (!dir) return;
  struct dirent* ent;
  while ((ent = readdir(dir)) != nullptr) {
    const char* name = ent->d_name;
    if (!name || name[0] == '.') continue;
    std::string path = std::string("/storage/") + name;
    struct stat st;
    if (stat(path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) continue;
    files.emplace_back(name);
  }
  closedir(dir);
  std::sort(files.begin(), files.end(), std::greater<std::string>());
}

static void delete_load_file_list() {
  g_d_files.clear();
  g_d_lines.clear();
  load_spiffs_regular_files(g_d_files);
  g_d_files.erase(std::remove(g_d_files.begin(), g_d_files.end(), "Station.txt"), g_d_files.end());
  if (g_d_files.empty()) {
    g_d_lines.push_back("No SPIFFS files");
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
  load_spiffs_regular_files(g_q_files);
  if (g_q_files.empty()) {
    g_q_lines.push_back("No SPIFFS files");
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

// core stores time_on as logged ("HHMMSS"); this view has always shown HH:MM.
static std::string qso_hhmm(const char* hhmmss) {
  if (!hhmmss || strlen(hhmmss) < 4) return "??:??";
  return std::string(hhmmss, 2) + ":" + std::string(hhmmss + 2, 2);
}

static void qso_rebuild_entry_lines() {
  g_q_lines.clear();
  for (const auto& e : g_q_entries) {
    std::string call_field = qso_trim_head(e.call[0] ? e.call : "?", 11);
    if (call_field.size() < 11) {
      call_field.append(11 - call_field.size(), ' ');
    }

    if (g_q_page_view == QPageView::Alternate) {
      const std::string rcvd = qso_format_signed3(e.has_rst_rcvd, e.rst_rcvd);
      const std::string sent = qso_format_sent4(e.has_rst_sent, e.rst_sent);
      g_q_lines.push_back(call_field + rcvd + " " + sent);
    } else {
      const std::string band_disp =
          qso_trim_head(e.band[0] ? e.band : (e.freq[0] ? e.freq : "?"), 6);
      g_q_lines.push_back(qso_hhmm(e.time_on) + " " + band_disp + " " + call_field);
    }
  }

  if (g_q_lines.empty()) {
    g_q_lines.push_back("No QSOs");
  }
}

static void qso_load_entries(const std::string& path) {
  g_q_entries.clear();
  g_q_lines.clear();

  // Filenames are "YYYYMMDD.txt"; core addresses days by the date alone.
  if (path.size() < 8) {
    g_q_lines.push_back("Open fail");
    return;
  }
  const std::string date = path.substr(0, 8);

  // Pull in batches so a long day never needs one huge contiguous buffer.
  constexpr int kBatch = 16;
  CoreLogEntry batch[kBatch];
  int from = 0;
  while (true) {
    const int n = core_log_read(date.c_str(), from, kBatch, batch);
    if (n < 0) {
      g_q_lines.push_back("Open fail");
      return;
    }
    for (int i = 0; i < n; ++i) g_q_entries.push_back(batch[i]);
    if (n < kBatch) break;
    from += n;
  }

  qso_rebuild_entry_lines();
}

static void qso_draw_page() {
  if (g_q_show_entries) {
    // Entry view: render raw QSO lines without "1..6 " prefixes.
    ui_draw_debug(g_q_lines, q_page);
  } else {
    // File list view: keep numbered selection rows.
    ui_draw_list(g_q_lines, q_page, -1);
  }
}

static void log_adif_entry(const std::string& dxcall, const std::string& dxgrid, int rst_sent, int rst_rcvd) {
  // Protect ADIF file access with the same mutex used for RxTxLog.
  // log_qso_if_needed can be called from the UAC streaming task (core 1)
  // via generate_response, so concurrent writes must be serialized.
  if (!log_mutex) return;
  if (xSemaphoreTake(log_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    ESP_LOGW(TAG, "ADIF mutex timeout");
    return;
  }

  // Build file name based on current date
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);
  char date[16];
  int year = t.tm_year + 1900;
  int month = t.tm_mon + 1;
  int day = t.tm_mday;
  snprintf(date, sizeof(date), "%04d%02d%02d", year % 10000, month % 100, day % 100);
  char path[64];
  snprintf(path, sizeof(path), "/storage/%s.txt", date);

  bool need_header = false;
  struct stat st;
  if (stat(path, &st) != 0) need_header = true;

  FILE* f = fopen(path, "a");
  if (!f) {
    ESP_LOGW(TAG, "ADIF open failed");
    xSemaphoreGive(log_mutex);
    return;
  }
  if (need_header) {
    fprintf(f, "ADIF EXPORT\n<eoh>\n");
  }

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
  fprintf(f, "<call:%zu>%s <gridsquare:%zu>%s <mode:3>FT8<qso_date:8>%s <time_on:6>%s <freq:%zu>%s <station_callsign:%zu>%s <my_gridsquare:%zu>%s %s%s<comment:%zu>%s <eor>\n",
          dxcall.size(), dxcall.c_str(),
          dxgrid.size(), dxgrid.c_str(),
          date, time_on,
          strlen(freq_str), freq_str,
          g_call.size(), g_call.c_str(),
          my_grid4.size(), my_grid4.c_str(),
          rst_sent_buf, rst_rcvd_buf,
          comment_expanded.size(), comment_expanded.c_str());
  fclose(f);
  xSemaphoreGive(log_mutex);
}


static bool uart_inject_last_was_cr = false;

static void poll_uart_inject_keys() {
  if (!s_key_inject_queue) return;
  // Read directly from the console UART FIFO — no driver needed.
  // sdkconfig configures ESP console on UART0 peripheral with custom
  // pins TX=GPIO13, RX=GPIO15 (see CONFIG_ESP_CONSOLE_UART_CUSTOM_NUM_0
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
    case UIMode::MSC: return "MSC";
    case UIMode::DEBUG:   return "DEBUG";
    case UIMode::STATUS:  return "STATUS";
    case UIMode::QSO:     return "QSO";
    case UIMode::GPS:     return "GPS";
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
  FILE* f = fopen(path, "rb");
  if (!f) {
    ESP_LOGE(TAG, "Failed to open %s", path);
    return ESP_FAIL;
  }

  WAVHeader hdr;
  if (fread(&hdr, 1, sizeof(hdr), f) != sizeof(hdr)) {
    ESP_LOGE(TAG, "Failed to read WAV header");
    fclose(f);
    return ESP_FAIL;
  }
  if (memcmp(hdr.riff, "RIFF", 4) != 0 || memcmp(hdr.wave, "WAVE", 4) != 0) {
    ESP_LOGE(TAG, "Invalid WAV header");
    fclose(f);
    return ESP_FAIL;
  }
  if (hdr.sample_rate != FT8_SAMPLE_RATE || hdr.num_channels != 1) {
    ESP_LOGE(TAG, "WAV must be mono %d Hz (got %u Hz, %u ch)", FT8_SAMPLE_RATE, hdr.sample_rate, hdr.num_channels);
    fclose(f);
    return ESP_FAIL;
  }

  const int bytes_per_sample = hdr.bits_per_sample / 8;

  monitor_config_t mon_cfg;
  mon_cfg.f_min = 200.0f;
  mon_cfg.f_max = 2900.0f;
  mon_cfg.sample_rate = FT8_SAMPLE_RATE;
  mon_cfg.time_osr = g_time_osr;
  mon_cfg.freq_osr = g_freq_osr;
  mon_cfg.protocol = FTX_PROTOCOL_FT8;

  monitor_t mon;
  monitor_init(&mon, &mon_cfg);
  monitor_reset(&mon);

  float* chunk = (float*)malloc(sizeof(float) * mon.block_size);
  if (!chunk) {
    ESP_LOGE(TAG, "Chunk alloc failed");
    fclose(f);
    monitor_free(&mon);
    return ESP_ERR_NO_MEM;
  }

  while (!feof(f)) {
    int read_samples = 0;
    while (read_samples < mon.block_size && !feof(f)) {
      float sample_value = 0.0f;
      if (bytes_per_sample == 1) {
        int s = fgetc(f);
        if (s == EOF) break;
        sample_value = ((float)s - 128.0f) / 128.0f;
      } else if (bytes_per_sample == 2) {
        int low = fgetc(f);
        int high = fgetc(f);
        if (low == EOF || high == EOF) break;
        int16_t s = (int16_t)((high << 8) | low);
        sample_value = (float)s / 32768.0f;
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
  fclose(f);

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
      freq_str = std::to_string(g_bands[i].freq);
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
  return (r == RadioType::KH1) ? RadioType::KH1 : RadioType::QMX;
}

static RadioProfileBinding get_radio_profile_binding(RadioType r) {
  switch (canonical_radio_type(r)) {
    case RadioType::KH1:
      return {AUDIO_SOURCE_USB_UAC_GENERIC, RADIO_CONTROL_KH1_CAT};
    case RadioType::QMX:
    default:
      return {AUDIO_SOURCE_QMX_UAC, RADIO_CONTROL_QMX};
  }
}

static const char* radio_name(RadioType r) {
  switch (canonical_radio_type(r)) {
    case RadioType::QMX: return "QMX";
    case RadioType::KH1: return "KH1";
    default: break;
  }
  return "None";
}

void apply_radio_profile_binding() {
  audio_source_backend_t prev_audio = audio_source_get_backend();
  g_radio = canonical_radio_type(g_radio);
  g_gps_baud = normalize_gps_baud_value(g_gps_baud);
  if (g_radio == RadioType::KH1) {
    // KH1 mode is explicit-connect: GPS keeps UART1 until user presses Connect to KH1.
    if (g_kh1_connected) {
      gps_stop();
      radio_control_kh1_set_enabled(true);
    } else {
      radio_control_kh1_set_enabled(false);
      gps_start(g_gps_baud);
    }
  } else {
    // Leaving KH1 releases UART1 back to GPS.
    g_kh1_connected = false;
    radio_control_kh1_set_enabled(false);
    gps_start(g_gps_baud);
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
  static int s_last_time_sync_hour_key = -1;

  if (canonical_radio_type(g_radio) == RadioType::KH1 && g_kh1_connected) return;

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
      if (rtc_set_from_strings()) {
        rtc_sync_to_hw();
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

static std::string menu_sleep_batt_line() {
  int level = (int)M5.Power.getBatteryLevel();
  if (level < 0 || level > 100) level = 0;
  char buf[32];
  snprintf(buf, sizeof(buf), "Sleep/Batt %d%%", level);
  return buf;
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

bool rtc_set_from_strings() {
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
  rtc_epoch_base = epoch;
  rtc_ms_start = esp_timer_get_time() / 1000;
  rtc_last_update = rtc_ms_start;
  rtc_valid = true;
  return true;
}

// Initialize soft RTC from hardware RTC (persists through deep sleep)
// Applies compensation if we have valid sleep epoch data
static bool rtc_init_from_hw() {
  struct timeval tv;
  if (gettimeofday(&tv, NULL) != 0) return false;

  // Check if hardware RTC has valid time (year > 2020)
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

  rtc_epoch_base = compensated_now;
  // Account for sub-second offset: tv.tv_usec tells us how far past the
  // whole second we are, so rewind rtc_ms_start by that amount.
  rtc_ms_start = esp_timer_get_time() / 1000 - tv.tv_usec / 1000;
  rtc_last_update = rtc_ms_start;
  rtc_valid = true;

  // Update g_date/g_time strings from compensated time
  localtime_r(&compensated_now, &t);
  char buf_date[32];
  snprintf(buf_date, sizeof(buf_date), "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
  g_date = buf_date;
  char buf_time[16];
  snprintf(buf_time, sizeof(buf_time), "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
  g_time = buf_time;

  ESP_LOGI(TAG, "RTC initialized: %s %s (compensated=%s)",
           g_date.c_str(), g_time.c_str(),
           (g_rtc_comp != 0) ? "yes" : "no");
  return true;
}

// Sync hardware RTC from soft RTC (call after FT8 time sync)
void rtc_sync_to_hw() {
  if (!rtc_valid) return;

  time_t now = rtc_epoch_base + (esp_timer_get_time() / 1000 - rtc_ms_start) / 1000;
  struct timeval tv = { .tv_sec = now, .tv_usec = 0 };
  settimeofday(&tv, NULL);
  ESP_LOGI(TAG, "Hardware RTC synced from soft RTC");
}

static void rtc_update_strings() {
  if (!rtc_valid) return;
  struct tm t;
  time_t now = rtc_epoch_base + (esp_timer_get_time() / 1000 - rtc_ms_start) / 1000;
  localtime_r(&now, &t);
  char buf_date[32];
  snprintf(buf_date, sizeof(buf_date), "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
  g_date = buf_date;
  char buf_time[16];
  snprintf(buf_time, sizeof(buf_time), "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
  g_time = buf_time;
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
          draw_status_line(5, std::string("Time: ") + g_time, false);
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
  int freq_hz = g_bands[g_band_sel].freq * 1000;
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
  int64_t slot_idx = now_ms / 15000;
  int64_t slot_ms = now_ms % 15000;
  static int64_t last_slot_idx = -1;
  static int last_sec = -1;
  int sec = (int)(slot_ms / 1000);
  if (slot_idx != last_slot_idx || sec != last_sec) {
    float frac = (float)slot_ms / 15000.0f;
    bool even = (slot_idx % 2) == 0;
    ui_draw_countdown(frac, even, g_offset_hz);
    last_slot_idx = slot_idx;
    last_sec = sec;
  }
}

static void redraw_countdown_now() {
  int64_t now_ms = rtc_now_ms();
  int64_t slot_idx = now_ms / 15000;
  int64_t slot_ms = now_ms % 15000;
  float frac = (float)slot_ms / 15000.0f;
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
// as the firmware used to do) means BLE clients reading core_get_qso
// see the same number that will actually go on air — important for the
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
// beacon-on, freetext queue, BLE tap_rx, …).
void arm_pending_tx(const AutoseqTxEntry& pending) {
  g_qso_xmit           = true;
  g_target_slot_parity = pending.slot_id & 1;
  g_pending_tx         = pending;
  g_pending_tx.offset_hz = resolve_tx_offset(g_pending_tx);
  g_pending_tx_valid   = true;
}

static void check_slot_boundary() {
  int64_t now_ms = rtc_now_ms();
  int64_t slot_idx = now_ms / 15000;
  int slot_ms = (int)(now_ms % 15000);
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
  // has completed (audio is 12.64s, slot is 15s — tight window) could fire TX
  // based on a prior cycle's state. See AUTOSEQ_INACTIVE_QUEUE.md.
  if (g_qso_xmit &&
      g_target_slot_parity == slot_parity &&
      slot_ms < 4000 &&
      !g_tx_active &&
      !g_decode_in_progress &&
      g_decode_applied_slot_idx >= slot_idx - 1) {

    ESP_LOGI(TAG, "TX trigger: starting TX in slot %lld (parity %d)",
             (long long)slot_idx, slot_parity);

    // Calculate skip_tones for partial slot
    int skip_tones = slot_ms / 160;
    if (skip_tones < 79) {
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

static void fft_waterfall_tx_tone(uint8_t tone) {
  // Map tone 0-7 to screen width and push a bright bin
  std::array<uint8_t, 240> row{};
  int pos = (int)((tone * row.size()) / 8);
  if (pos < 0) pos = 0;
  if (pos >= (int)row.size()) pos = (int)row.size() - 1;
  row[pos] = 200;
  ui_push_waterfall_row(row.data(), (int)row.size());
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
  int64_t slot_idx = (g_decode_slot_idx >= 0) ? g_decode_slot_idx : rtc_now_ms() / 15000LL;
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
    else core_fire_rx_changed();  // propagates to all registered consumers (Cardputer, future BLE)
    ble_publish_decode_event(0);
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

  // ---- Auto sync RTC ----
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
      rtc_sync_to_hw();
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
    core_fire_rx_changed();  // propagates to all registered consumers (Cardputer, future BLE)
  }
  ble_publish_decode_event(s_dec_count);

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
  uint8_t tones[79] = {0};
  ft8_encode(msg.payload, tones);
  debug_log_line(std::string("Tones for '") + g_pending_tx.text + "'");
  log_tones(tones, 79);
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
  g_tx_slot_idx = now_ms / 15000;

  ESP_LOGI(TAG, "tx_start: TX=%s offset=%d skip=%d slot=%lld",
           g_pending_tx.text.c_str(), g_pending_tx.offset_hz, skip_tones, (long long)g_tx_slot_idx);

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
  ft8_encode(msg.payload, g_tx_tones);

  // Set up TX state machine
  // IMPORTANT: Tone timing must be based on slot boundary, not TX start time.
  // This ensures TX ends at the correct time even if TX started late,
  // allowing RX to start cleanly at the next slot boundary.
  g_tx_base_hz = g_pending_tx.offset_hz;
  g_tx_slot_start_ms = (now_ms / 15000) * 15000;  // Slot boundary time
  g_tx_tone_idx = (skip_tones >= 79) ? 79 : skip_tones;
  // Next tone time = slot_start + tone_idx * 160ms
  // This aligns all tones to the slot boundary, not to when TX started
  g_tx_next_tone_time = g_tx_slot_start_ms + g_tx_tone_idx * 160;
  g_tx_last_ta_int = -1;
  g_tx_last_ta_frac = -1;

  ESP_LOGI(TAG, "TX base_hz=%d (from pre-computed offset, text=%s)", g_tx_base_hz, g_pending_tx.text.c_str());

  // Send CAT setup commands
  g_tx_cat_ok = radio_control_ready();
  if (g_tx_cat_ok) {
    int freq_hz = g_bands[g_band_sel].freq * 1000;
    esp_err_t err = radio_control_begin_tx(freq_hz, g_tx_base_hz);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "tx_start: radio TX begin failed: %s", esp_err_to_name(err));
      g_tx_cat_ok = false;
    }
  }

  if (skip_tones > 0) {
    ESP_LOGI("TXTONE", "Skipping first %d tones due to late start", skip_tones);
  }

  // Send first tone TA if CAT is ready
  if (g_tx_cat_ok && g_tx_tone_idx < 79) {
    float tone_hz = g_tx_base_hz + 6.25f * g_tx_tones[g_tx_tone_idx];
    tx_send_ta(tone_hz);
  }

  // Mark TX as active
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
  if (g_tx_tone_idx >= 79) {
    ESP_LOGI(TAG, "tx_tick: TX complete, all 79 tones sent");
    if (g_tx_cat_ok) {
      radio_control_end_tx();
    }
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

  // Send current tone
  ESP_LOGD("TXTONE", "%02d %u", g_tx_tone_idx, (unsigned)g_tx_tones[g_tx_tone_idx]);
  fft_waterfall_tx_tone(g_tx_tones[g_tx_tone_idx]);
  if (g_tx_cat_ok) {
    float tone_hz = g_tx_base_hz + 6.25f * g_tx_tones[g_tx_tone_idx];
    tx_send_ta(tone_hz);
  }

  // Advance to next tone
  g_tx_tone_idx++;
  // Calculate next tone time from slot boundary to ensure TX ends at correct time
  // This guarantees RX can start cleanly at the next slot boundary
  g_tx_next_tone_time = g_tx_slot_start_ms + g_tx_tone_idx * 160;
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
  lines.push_back(std::string("BLE ") + (g_ble_enabled ? "ON" : "OFF"));

  // Page 2 content (index 12+)
  lines.push_back(std::string("RxTxLog:") + (g_rxtx_log ? "ON" : "OFF"));
  lines.push_back(std::string("SkipTX1:") + (g_skip_tx1 ? "ON" : "OFF"));
  lines.push_back(std::string("ActiveBand:") + head_trim(g_active_band_text, 16));
  if (menu_edit_idx == 15) {
    lines.push_back(std::string("RTC Comp:") + menu_edit_buf);
  } else {
    lines.push_back(std::string("RTC Comp:") + std::to_string(g_rtc_comp));
  }
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
    const int start_y = 18 + 3 + 3; // WATERFALL_H + COUNTDOWN_H + gap
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
  if (radio == RadioType::KH1) {
    if (!g_kh1_connected) return "Connect to KH1";
    const bool cat_ready = radio_control_ready();
    if (cat_ready && streaming) return "Sync to KH1(RX+TX)";
    if (cat_ready && !streaming) return "Sync to KH1(TX)";
    return "Connect to KH1";
  }
  if (streaming) return std::string("Sync to ") + radio_name(radio);
  return std::string("Connect to ") + radio_name(radio);
}

static std::string s_last_gps_lines[6];

static void draw_gps_view(bool force_redraw) {
  std::vector<std::string> lines;
  lines.reserve(6);
  gps_state_t state = gps_get_state();
  lines.push_back("--- GPS TELEMETRY ---");
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
  const int start_y = 18 + 3 + 3;

  M5.Display.startWrite();
  M5.Display.setTextSize(2);
  for (size_t i = 0; i < 6; ++i) {
    std::string text = (i < lines.size()) ? lines[i] : "";
    // Keep BLE mirror source in sync with GPS mode text regardless of LCD redraw.
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
  lines[2] = std::string("Band: ") +
             std::string(g_bands[g_band_sel].name) + " " +
             std::to_string(g_bands[g_band_sel].freq);
  lines[3] = std::string("Tune: ") + (g_tune ? "ON" : "OFF");
  if (status_edit_idx == 4 && !status_edit_buffer.empty()) {
    lines[4] = std::string("Date: ") + highlight_pos(status_edit_buffer, status_cursor_pos);
  } else {
    lines[4] = std::string("Date: ") + g_date + (g_time_synced_from_gps ? "G" : "");
  }
  if (status_edit_idx == 5 && !status_edit_buffer.empty()) {
    lines[5] = std::string("Time: ") + highlight_pos(status_edit_buffer, status_cursor_pos);
  } else {
    lines[5] = std::string("Time: ") + g_time;
  }
  for (int i = 0; i < 6; ++i) {
    bool hl = (status_edit_idx == i);
    draw_status_line(i, lines[i], hl);
  }
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

#if ENABLE_BLE

static int page_count(int items, int page_size) {
  if (page_size <= 0) return 1;
  if (items <= 0) return 1;
  return (items + page_size - 1) / page_size;
}

static int ble_send_payload_raw(const std::string& payload, bool indicate) {
  if (!g_ble_enabled) return BLE_HS_EINVAL;
  if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) return BLE_HS_ENOTCONN;
  if (!gatt_tx_handle) return BLE_HS_EINVAL;
  struct os_mbuf* om = ble_hs_mbuf_from_flat(payload.data(), payload.size());
  if (!om) return BLE_HS_ENOMEM;
  int rc = indicate
             ? ble_gatts_indicate_custom(g_conn_handle, gatt_tx_handle, om)
             : ble_gatts_notify_custom(g_conn_handle, gatt_tx_handle, om);
  if (rc != 0) {
    ESP_LOGD(BT_TAG, "%s failed rc=%d", indicate ? "indicate" : "notify", rc);
  }
  return rc;
}

static void ble_notify_payload(const std::string& payload) {
  (void)ble_send_payload_raw(payload, false);
}

static bool ble_wait_for_indicate_ack(int timeout_ms) {
  const int step_ms = 10;
  int waited = 0;
  while (g_ble_indicate_waiting && waited < timeout_ms) {
    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
      g_ble_indicate_status = BLE_HS_ENOTCONN;
      g_ble_indicate_waiting = false;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(step_ms));
    waited += step_ms;
  }
  if (g_ble_indicate_waiting) {
    g_ble_indicate_status = BLE_HS_ETIMEOUT;
    g_ble_indicate_waiting = false;
  }
  return g_ble_indicate_status == BLE_HS_EDONE;
}

static void ble_dump_reset_transfer_state(bool use_indicate) {
  g_ble_dump_xfer = BleDumpTransferState{};
  g_ble_dump_xfer.active = true;
  g_ble_dump_xfer.mode = use_indicate ? BleDumpTxMode::Indicate : BleDumpTxMode::Notify;
  g_ble_dump_xfer.notify_pace_ms = kBleDumpNotifyPaceMinMs;
  g_ble_dump_xfer.mtu = g_ble_att_mtu;
  g_ble_indicate_waiting = false;
  g_ble_indicate_status = 0;
}

static size_t ble_dump_max_chunk_len() {
  // BLE notification/indication usable payload is ATT_MTU - 3.
  // Use g_ble_att_mtu if available; otherwise default to 23-byte MTU => 20-byte payload.
  uint16_t mtu = g_ble_att_mtu;
  if (mtu < 23) mtu = 23;

  size_t n = (size_t)mtu - 3;

  // Conservative cap. Helps avoid phone-side / NimBLE queue issues.
  if (n > 180) n = 180;
  if (n < 20) n = 20;

  return n;
}

static bool ble_dump_send_chunk(const std::string& chunk) {
  if (chunk.empty()) {
    return true;
  }

  if (g_ble_dump_xfer.mode == BleDumpTxMode::Indicate) {
    for (int attempt = 0; attempt <= kBleDumpIndicateMaxRetries; ++attempt) {
      g_ble_indicate_status = 0;
      g_ble_indicate_waiting = true;

      const int rc = ble_send_payload_raw(chunk, true);

      if (rc == 0) {
        if (ble_wait_for_indicate_ack(kBleDumpIndicateAckTimeoutMs)) {
          return true;
        }
      } else {
        g_ble_indicate_waiting = false;
        g_ble_indicate_status = rc;
      }

      if (attempt < kBleDumpIndicateMaxRetries) {
        g_ble_dump_xfer.retries++;

        const int bi = attempt < kBleDumpNotifyMaxRetries
                         ? attempt
                         : (kBleDumpNotifyMaxRetries - 1);

        vTaskDelay(pdMS_TO_TICKS(kBleDumpNotifyBackoffMs[bi]));
      }
    }

    return false;
  }

  int attempt = 0;

  for (; attempt <= kBleDumpNotifyMaxRetries; ++attempt) {
    const int rc = ble_send_payload_raw(chunk, false);
    if (rc == 0) break;

    if (attempt < kBleDumpNotifyMaxRetries) {
      g_ble_dump_xfer.retries++;
      vTaskDelay(pdMS_TO_TICKS(kBleDumpNotifyBackoffMs[attempt]));
    }
  }

  if (attempt > kBleDumpNotifyMaxRetries) {
    return false;
  }

  if (attempt > 0) {
    int next_pace = g_ble_dump_xfer.notify_pace_ms + (attempt * 2);
    if (next_pace > kBleDumpNotifyPaceMaxMs) {
      next_pace = kBleDumpNotifyPaceMaxMs;
    }
    g_ble_dump_xfer.notify_pace_ms = next_pace;
  } else if (g_ble_dump_xfer.notify_pace_ms > kBleDumpNotifyPaceMinMs) {
    g_ble_dump_xfer.notify_pace_ms--;
  }

  vTaskDelay(pdMS_TO_TICKS(g_ble_dump_xfer.notify_pace_ms));
  return true;
}

static bool ble_dump_send_line(const std::string& raw) {
  std::string line = raw;

  while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
    line.pop_back();
  }

  std::string payload = line + "\n";

  const size_t max_chunk = ble_dump_max_chunk_len();

  for (size_t pos = 0; pos < payload.size(); pos += max_chunk) {
    const size_t remain = payload.size() - pos;
    const size_t n = remain > max_chunk ? max_chunk : remain;

    std::string chunk = payload.substr(pos, n);

    if (!ble_dump_send_chunk(chunk)) {
      g_ble_dump_xfer.failed_lines++;
      return false;
    }
  }

  return true;
}
static void apply_ble_enabled_policy(bool runtime_apply) {
  g_time_osr = 2;  // 6kHz sample rate enables time_osr=2 even with BLE
  g_freq_osr = 1;
  if (!runtime_apply) return;

  if (!g_ble_enabled) {
    if (g_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
      ble_gap_terminate(g_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    if (g_ble_synced) {
      ble_gap_adv_stop();
    }
    g_ble_last_payload.clear();
    g_ble_last_screen_valid = false;
    g_ble_last_line7.clear();
    g_ble_status_clock_slot_sent = -1;
    g_ble_gps_slot_sent = -1;
    g_ble_last_tick_slot = -1;
    g_ble_last_tick_sec = -1;
    g_ble_waterfall_slot_idx = -1;
    g_ble_waterfall_header = ble_blank_waterfall_header();
    g_ble_text_mode = false;
    g_ble_qso_pick_mode = false;
    g_ble_dump_in_progress = false;
    if (ble_cmd_queue) xQueueReset(ble_cmd_queue);
    return;
  }

  g_ble_force_send = true;
  g_ble_last_screen_valid = false;
  g_ble_status_clock_slot_sent = -1;
  g_ble_gps_slot_sent = -1;
  g_ble_last_tick_slot = -1;
  g_ble_last_tick_sec = -1;
  g_ble_waterfall_slot_idx = -1;
  g_ble_waterfall_header = ble_blank_waterfall_header();
  if (g_ble_synced && g_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
    ble_app_advertise();
  }
}

static std::string ble_mac_suffix() {
  uint8_t mac[6] = {};
  if (esp_efuse_mac_get_default(mac) != ESP_OK) {
    return "0000";
  }
  char out[5];
  std::snprintf(out, sizeof(out), "%02X%02X", mac[4], mac[5]);
  return std::string(out);
}

static std::string ble_sanitize_callsign(const std::string& call) {
  std::string out;
  out.reserve(call.size());
  for (unsigned char ch : call) {
    if (std::isalnum(ch)) {
      out.push_back(static_cast<char>(std::toupper(ch)));
    } else if (ch == '/' || ch == '_' || ch == '-') {
      out.push_back('_');
    }
    if (out.size() >= 12) break;
  }
  while (!out.empty() && out.back() == '_') out.pop_back();
  return out;
}

static void ble_update_name_from_station(bool restart_adv) {
  std::string suffix = ble_sanitize_callsign(g_call);
  if (suffix.empty()) suffix = ble_mac_suffix();
  std::string desired = std::string("Mini-FT8-") + suffix;
  if (desired.size() > 24) desired.resize(24);
  if (desired.empty()) desired = "Mini-FT8";

  if (desired == g_ble_adv_name) return;
  g_ble_adv_name = desired;
  ble_svc_gap_device_name_set(g_ble_adv_name.c_str());

  if (restart_adv && g_ble_enabled && g_ble_synced && g_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
    ble_app_advertise();
  }
}

static const char* ble_page_label(UIMode mode) {
  switch (mode) {
    case UIMode::RX: return "RX";
    case UIMode::TX: return "TX";
    case UIMode::BAND: return "BAND";
    case UIMode::MENU: return "MENU";
    case UIMode::MSC: return "MSC";
    case UIMode::DEBUG: return "DELETE";
    case UIMode::STATUS: return "STATUS";
    case UIMode::QSO: return "QSO";
    case UIMode::GPS: return "GPS";
  }
  return "PAGE";
}

static void ble_page_meta(int& cur, int& total) {
  cur = 1;
  total = 1;
  switch (ui_mode) {
    case UIMode::RX:
      ui_get_rx_page_info(cur, total);
      break;
    case UIMode::TX:
      total = page_count(autoseq_queue_size(), 5);
      cur = tx_page + 1;
      break;
    case UIMode::BAND:
      total = page_count((int)g_bands.size(), 6);
      cur = band_page + 1;
      break;
    case UIMode::MENU:
      total = 3;
      cur = menu_page + 1;
      break;
    case UIMode::DEBUG:
      total = page_count((int)g_d_lines.size(), 6);
      cur = d_page + 1;
      break;
    case UIMode::QSO:
      total = page_count((int)g_q_lines.size(), 6);
      cur = q_page + 1;
      break;
    default:
      break;
  }
  if (total < 1) total = 1;
  if (cur < 1) cur = 1;
  if (cur > total) cur = total;
}

static std::string ble_meta_line() {
  int cur = 1;
  int total = 1;
  ble_page_meta(cur, total);

  char meta[96];
  const char* label = ble_page_label(ui_mode);
  if (ui_mode == UIMode::QSO && g_ble_qso_pick_mode) {
    label = "Fetch";
  }
  const char up = (cur > 1) ? 'u' : '-';
  const char down = (cur < total) ? 'v' : '-';
  std::snprintf(meta, sizeof(meta), "[%s %c%c]", label, up, down);
  return std::string(meta);
}

static const char* menu_edit_label(int idx) {
  switch (idx) {
    case 3:  return "Call";
    case 4:  return "Grid";
    case 7:  return "Cursor";
    case 9:  return "IgnoreList";
    case 10: return "Comment";
    case 15: return "RTC Comp";
    case 17: return "Max Retry";
    default: return "Edit";
  }
}

static std::string ble_menu_long_edit_label() {
  switch (menu_long_kind) {
    case LONG_FT:
      return "F";
    case LONG_COMMENT:
      return "C";
    case LONG_ACTIVE:
      return "ActiveBand";
    case LONG_IGNORE:
      return "IgnoreList";
    case LONG_NONE:
    default:
      return "Edit";
  }
}

static std::string ble_text_mode_line7() {
  std::string item = "Edit";

  if (menu_long_edit) {
    item = ble_menu_long_edit_label();
  } else if (menu_edit_idx >= 0) {
    item = menu_edit_label(menu_edit_idx);
  } else if (band_edit_idx >= 0 && band_edit_idx < (int)g_bands.size()) {
    item = std::string("Band ") + g_bands[band_edit_idx].name;
  } else if (status_edit_idx == 4) {
    item = "Date";
  } else if (status_edit_idx == 5) {
    item = "Time";
  }

  return std::string("[Edit ") + item + "]";
}

static void ble_slot_second_now(int64_t& slot_idx, int& sec, bool& even_slot) {
  int64_t now_ms = rtc_now_ms();
  int64_t slot_ms = now_ms % 15000;
  if (slot_ms < 0) slot_ms += 15000;
  slot_idx = (now_ms - slot_ms) / 15000;
  sec = (int)(slot_ms / 1000);
  if (sec < 0) sec = 0;
  if (sec > 14) sec = 14;
  even_slot = ((slot_idx & 1) == 0);
}

static std::string ble_timing_token(int sec, bool even_slot, bool txing) {
  if (sec == 0) return "|";
  if (sec == 4) return "4";
  if (sec == 8) return "8";
  if (sec == 12) return "12";
  if (txing && (sec == 2 || sec == 6 || sec == 10 || sec == 14)) return "o";
  return even_slot ? ":" : ".";
}

static std::string ble_blank_waterfall_header() {
  return "|                           |";
}

static void ble_update_waterfall_header_if_due(int64_t slot_idx, int sec) {
  if (sec != 12) return;
  if (g_ble_waterfall_slot_idx == slot_idx) return;
  g_ble_waterfall_slot_idx = slot_idx;

  uint8_t row[UAC_WATERFALL_ROW_WIDTH] = {};
  if (!uac_get_latest_waterfall_row(row, sizeof(row))) {
    g_ble_waterfall_header = ble_blank_waterfall_header();
    return;
  }

  constexpr int kBleWfBins = 27;
  uint8_t bins[kBleWfBins] = {};
  for (int i = 0; i < kBleWfBins; ++i) {
    int start = (int)((int64_t)i * UAC_WATERFALL_ROW_WIDTH / kBleWfBins);
    int end = (int)((int64_t)(i + 1) * UAC_WATERFALL_ROW_WIDTH / kBleWfBins);
    if (end <= start) end = start + 1;
    uint8_t vmax = 0;
    for (int x = start; x < end && x < UAC_WATERFALL_ROW_WIDTH; ++x) {
      if (row[x] > vmax) vmax = row[x];
    }
    bins[i] = vmax;
  }

  uint8_t min_v = 255;
  uint8_t max_v = 0;
  for (int i = 0; i < kBleWfBins; ++i) {
    if (bins[i] < min_v) min_v = bins[i];
    if (bins[i] > max_v) max_v = bins[i];
  }

  static const char kChars[4] = {' ', '.', ':', '!'};
  char chars[kBleWfBins + 1];
  chars[kBleWfBins] = '\0';
  if (max_v <= min_v) {
    for (int i = 0; i < kBleWfBins; ++i) chars[i] = ' ';
  } else {
    const int span = (int)max_v - (int)min_v;
    for (int i = 0; i < kBleWfBins; ++i) {
      const int scaled = (int)bins[i] - (int)min_v;
      int level = (scaled * 3 + (span / 2)) / span;  // 0..3, rounded
      if (level < 0) level = 0;
      if (level > 3) level = 3;
      chars[i] = kChars[level];
    }
  }

  g_ble_waterfall_header.clear();
  g_ble_waterfall_header.reserve(kBleWfBins + 2);
  g_ble_waterfall_header.push_back('|');
  g_ble_waterfall_header.append(chars, kBleWfBins);
  g_ble_waterfall_header.push_back('|');
}

static void ble_start_qso_pick_mode() {
  if (g_ble_qso_pick_mode) return;
  g_ble_qso_return_mode = ui_mode;
  g_ble_qso_pick_mode = true;
  g_ble_dump_in_progress = false;
  g_q_show_entries = false;
  g_q_page_view = QPageView::Default;
  q_page = 0;
  enter_mode(UIMode::QSO);
  qso_draw_page();
  g_ble_force_send = true;
}

static void ble_cancel_qso_pick_mode() {
  if (!g_ble_qso_pick_mode) return;
  g_ble_qso_pick_mode = false;
  g_ble_dump_in_progress = false;
  if (ble_cmd_queue) xQueueReset(ble_cmd_queue);
  enter_mode(g_ble_qso_return_mode);
  g_ble_force_send = true;
}

static void ble_dump_qso_file(const std::string& file_name) {
  g_ble_dump_in_progress = true;
  const bool use_indicate = g_ble_tx_indicate_enabled;
  ble_dump_reset_transfer_state(use_indicate);
  std::string full_path = std::string("/storage/") + file_name;
  if (!use_indicate) {
    (void)ble_dump_send_line("fallback notify mode (best effort)");
  }
  (void)ble_dump_send_line(std::string("\n--- <") + file_name + "> ---");

  FILE* f = fopen(full_path.c_str(), "r");
  if (!f) {
    (void)ble_dump_send_line("Open fail");
  } else {
    char line[512];
    while (fgets(line, sizeof(line), f)) {
      g_ble_dump_xfer.file_lines++;
      (void)ble_dump_send_line(line);
    }
    fclose(f);
  }
  (void)ble_dump_send_line("--- EOF ---");
  {
    char summary[160];
    std::snprintf(summary, sizeof(summary),
                  "F summary mode=%s mtu=%u lines=%d retries=%d failed=%d",
                  use_indicate ? "INDICATE" : "NOTIFY",
                  (unsigned)g_ble_dump_xfer.mtu,
                  g_ble_dump_xfer.file_lines,
                  g_ble_dump_xfer.retries,
                  g_ble_dump_xfer.failed_lines);
    (void)ble_dump_send_line(summary);
  }

  g_ble_dump_in_progress = false;
  g_ble_indicate_waiting = false;
  g_ble_dump_xfer.active = false;
  if (ui_mode == UIMode::QSO) {
    qso_draw_page();
  }
  g_ble_force_send = true;
}

static void ble_try_dump_qso_file_by_key(char key) {
  if (key < '1' || key > '6') return;
  int idx = q_page * 6 + (key - '1');
  if (idx < 0 || idx >= (int)g_q_files.size()) return;
  if (!g_ble_enabled || g_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;
  ble_dump_qso_file(g_q_files[idx]);
}

static bool ble_status_clock_only_delta(const std::vector<std::string>& lines,
                                        const std::string& line7) {
  if (!g_ble_last_screen_valid) return false;
  if (line7 != g_ble_last_line7) return false;
  bool changed = false;
  for (int i = 0; i < 6; ++i) {
    if (lines[i] == g_ble_last_screen_lines[i]) continue;
    if (i != 4 && i != 5) return false;  // STATUS line5/6 only (Date/Time)
    changed = true;
  }
  return changed;
}

static bool ble_gps_clock_only_delta(const std::vector<std::string>& lines,
                                     const std::string& line7) {
  if (!g_ble_last_screen_valid) return false;
  if (line7 != g_ble_last_line7) return false;
  for (int i = 0; i < 6; ++i) {
    if (lines[i] != g_ble_last_screen_lines[i]) return true;
  }
  return false;
}

static void ble_mirror_tick() {
  // Text-terminal screen mirror is removed. The native BLE service
  // (ble_native.cpp) now pushes state changes via its own EVENTS /
  // RX_LIST / QSO_QUEUE / RADIO_STREAM characteristics.
  return;
  // Dead code below kept temporarily for reference; will be pruned
  // once step 5 confirms no regressions. Unreachable, zero cost.
  if (!g_ble_enabled) return;
  if (g_ble_dump_in_progress) return;
  if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;

  std::vector<std::string> lines;
  ui_get_visible_text_lines(lines);
  while ((int)lines.size() < 6) lines.push_back("");

  const std::string line7 = g_ble_text_mode ? ble_text_mode_line7() : ble_meta_line();

  bool snapshot_changed = g_ble_force_send || !g_ble_last_screen_valid || (line7 != g_ble_last_line7);
  if (!snapshot_changed) {
    for (int i = 0; i < 6; ++i) {
      if (lines[i] != g_ble_last_screen_lines[i]) {
        snapshot_changed = true;
        break;
      }
    }
  }
  if (!snapshot_changed) return;

  int64_t status_slot_idx = -1;
  if (!g_ble_force_send &&
      ui_mode == UIMode::STATUS &&
      !g_ble_text_mode &&
      ble_status_clock_only_delta(lines, line7)) {
      int sec = 0;
      bool even_slot = true;
      ble_slot_second_now(status_slot_idx, sec, even_slot);
      if (g_ble_status_clock_slot_sent == status_slot_idx) {
        return;
      }
  }

  int64_t gps_slot_idx = -1;
  if (!g_ble_force_send &&
      ui_mode == UIMode::GPS &&
      !g_ble_text_mode &&
      ble_gps_clock_only_delta(lines, line7)) {
      int sec = 0;
      bool even_slot = true;
      ble_slot_second_now(gps_slot_idx, sec, even_slot);
      if (g_ble_gps_slot_sent == gps_slot_idx) {
        return;
      }
  }

  std::string screen_key;
  screen_key.reserve(256);
  for (int i = 0; i < 6; ++i) {
    screen_key += lines[i];
    screen_key.push_back('\n');
  }
  screen_key += line7;

  g_ble_force_send = false;
  g_ble_last_payload = screen_key;
  for (int i = 0; i < 6; ++i) {
    g_ble_last_screen_lines[i] = lines[i];
  }
  g_ble_last_line7 = line7;
  g_ble_last_screen_valid = true;

  if (ui_mode == UIMode::STATUS && !g_ble_text_mode) {
    if (status_slot_idx < 0) {
      int sec = 0;
      bool even_slot = true;
      ble_slot_second_now(status_slot_idx, sec, even_slot);
    }
    g_ble_status_clock_slot_sent = status_slot_idx;
  }
  if (ui_mode == UIMode::GPS && !g_ble_text_mode) {
    if (gps_slot_idx < 0) {
      int sec = 0;
      bool even_slot = true;
      ble_slot_second_now(gps_slot_idx, sec, even_slot);
    }
    g_ble_gps_slot_sent = gps_slot_idx;
  }

  if (g_ble_last_tick_slot < 0 || g_ble_last_tick_sec < 0) {
    int64_t slot_idx = 0;
    int sec = 0;
    bool even_slot = true;
    ble_slot_second_now(slot_idx, sec, even_slot);
    g_ble_last_tick_slot = slot_idx;
    g_ble_last_tick_sec = sec;
  }

  std::string out;
  out.reserve(screen_key.size() + 120);
  out += "\n";
  out += std::string(29, '=');
  out += "\n";
  out += g_ble_waterfall_header.empty() ? ble_blank_waterfall_header() : g_ble_waterfall_header;
  out += "\n";
  out += "---.----+----.----+----.----+";
  out += "\n";
  out += screen_key;
  ble_notify_payload(out);
}

static void ble_countdown_tick() {
  // Terminal-mode countdown token removed; the native app derives the
  // slot clock from its own RTC synced via set_rtc RPC.
  return;
  if (!g_ble_enabled) return;
  if (g_ble_dump_in_progress) return;
  if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;

  const uint32_t seq = g_ble_decode_event_seq;
  if (seq != g_ble_decode_event_seq_seen) {
    g_ble_decode_event_seq_seen = seq;
    if (!g_ble_text_mode) {
      char buf[24];
      std::snprintf(buf, sizeof(buf), "[D:%d]", g_ble_decode_event_count);
      ble_notify_payload(std::string(buf));
    }
  }

  int64_t slot_idx = 0;
  int sec = 0;
  bool even_slot = true;
  ble_slot_second_now(slot_idx, sec, even_slot);
  ble_update_waterfall_header_if_due(slot_idx, sec);

  if (g_ble_last_tick_slot == slot_idx && g_ble_last_tick_sec == sec) return;
  if (g_ble_last_tick_slot < 0 || g_ble_last_tick_sec < 0) {
    g_ble_last_tick_slot = slot_idx;
    g_ble_last_tick_sec = sec;
    return;
  }

  g_ble_last_tick_slot = slot_idx;
  g_ble_last_tick_sec = sec;
  ble_notify_payload(ble_timing_token(sec, even_slot, g_tx_active));
}

static void ble_on_sync(void) {
  int rc = ble_hs_util_ensure_addr(0);
  if (rc != 0) {
    ESP_LOGE(BT_TAG, "ensure addr failed: %d", rc);
    return;
  }
  rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
  if (rc != 0) {
    ESP_LOGE(BT_TAG, "infer auto addr failed: %d", rc);
    return;
  }
  g_ble_synced = true;
  ble_update_name_from_station(false);
  ble_app_advertise();
}

static void nimble_host_task(void* param) {
  (void)param;
  nimble_port_run();
  nimble_port_freertos_deinit();
}

static void ble_app_advertise(void) {
  if (!g_ble_enabled) return;
  if (!g_ble_synced) return;
  if (g_conn_handle != BLE_HS_CONN_HANDLE_NONE) return;

  struct ble_gap_adv_params adv{};
  adv.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv.disc_mode = BLE_GAP_DISC_MODE_GEN;

  // Primary AD carries flags + the 128-bit service UUID so iOS can pre-filter
  // (a custom 128-bit UUID + flags is already 21 of the 31 AD bytes, leaving
  // no room for the name). The complete name travels in the scan response.
  static ble_uuid128_t svc_uuid = BLE_UUID128_INIT(BLE_NATIVE_SVC_UUID);

  struct ble_hs_adv_fields fields{};
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.uuids128 = &svc_uuid;
  fields.num_uuids128 = 1;
  fields.uuids128_is_complete = 1;

  const std::string name = g_ble_adv_name.empty() ? std::string("Mini-FT8") : g_ble_adv_name;
  struct ble_hs_adv_fields rsp_fields{};
  rsp_fields.name = (uint8_t*)name.c_str();
  rsp_fields.name_len = name.size();
  rsp_fields.name_is_complete = 1;

  ble_gap_adv_stop();
  int rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    ESP_LOGE(BT_TAG, "adv_set_fields failed: %d", rc);
    return;
  }
  rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
  if (rc != 0) {
    ESP_LOGE(BT_TAG, "adv_rsp_set_fields failed: %d", rc);
    return;
  }
  rc = ble_gap_adv_start(g_own_addr_type, nullptr, BLE_HS_FOREVER, &adv, gap_cb, nullptr);
  if (rc != 0) {
    ESP_LOGE(BT_TAG, "adv_start failed: %d", rc);
  } else {
    ESP_LOGI(BT_TAG, "Advertising as %s", name.c_str());
  }
}

#else  // ENABLE_BLE
static bool ble_pop_input(BleUiInput& out) { (void)out; return false; }
static void ble_update_name_from_station(bool restart_adv) { (void)restart_adv; }
static void apply_ble_enabled_policy(bool runtime_apply) {
  (void)runtime_apply;
  g_time_osr = 2;  // 6kHz sample rate enables time_osr=2 always
  g_freq_osr = 1;
}
static void ble_mirror_tick() {}
static void ble_countdown_tick() {}
static void init_bluetooth(void) {}
#endif // ENABLE_BLE

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

static void load_station_data() {
  // Sync only Station.txt from SD to SPIFFS (no legacy fallback).
  sync_station_txt_from_sd_to_spiffs();

  // Load-time defaults for runtime settings.
  g_rtc_comp = kRtcCompFixed;
  g_autoseq_max_retry = AUTOSEQ_MAX_RETRY;
  g_ble_enabled = true;
  g_gps_baud = 115200;
  g_grid_saved_manual = g_grid;
  g_grid_from_gps = false;
  g_grid_gps_display8.clear();

  FILE* f = fopen(STATION_FILE, "r");
  if (!f) {
    autoseq_set_max_retry(g_autoseq_max_retry);
    apply_ble_enabled_policy(false);
    return;
  }
  char line[128];
  while (fgets(line, sizeof(line), f)) {
    int idx = -1;
    int val = 0;
    if (sscanf(line, "band%d=%d", &idx, &val) == 2) {
      if (idx >= 0 && idx < (int)g_bands.size()) {
        g_bands[idx].freq = val;
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
    } else if (sscanf(line, "radio=%d", &val) == 1) {
      if (val == (int)RadioType::KH1) g_radio = RadioType::KH1;
      else g_radio = RadioType::QMX;
    } else if (sscanf(line, "gps_baud=%d", &val) == 1) {
      g_gps_baud = normalize_gps_baud_value(val);
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
    } else if (sscanf(line, "ble_enabled=%d", &val) == 1) {
      g_ble_enabled = (val != 0);
    } else if (sscanf(line, "rtc_comp=%d", &val) == 1) {
      g_rtc_comp = clamp_rtc_comp_value(val);
    } else {
      long long epoch_tmp = 0;
      if (sscanf(line, "rtc_sleep_epoch=%lld", &epoch_tmp) == 1) {
        g_rtc_sleep_epoch = (time_t)epoch_tmp;
      }
    }
  }
  fclose(f);
  autoseq_set_max_retry(g_autoseq_max_retry);
  // Try hardware RTC first (persists through deep sleep), fall back to saved strings
  if (!rtc_init_from_hw()) {
    ESP_LOGI(TAG, "Hardware RTC not valid, using saved time strings");
    rtc_set_from_strings();
  }
  rebuild_active_bands();
  rebuild_ignore_prefixes();
  g_beacon = BeaconMode::OFF; // force off on load
  apply_ble_enabled_policy(false);
}

void save_station_data() {
  FILE* f = fopen(STATION_FILE, "w");
  if (!f) {
    ESP_LOGE(TAG, "Failed to open %s for write: errno=%d (%s)",
             STATION_FILE, errno, strerror(errno));
    return;
  }
  for (size_t i = 0; i < g_bands.size(); ++i) {
    fprintf(f, "band%u=%d\n", (unsigned)i, g_bands[i].freq);
  }
  // Beacon is not persisted (stays OFF on reload)
  fprintf(f, "offset=%d\n", g_offset_hz);
  fprintf(f, "band_sel=%d\n", g_band_sel);
  fprintf(f, "date=%s\n", g_date.c_str());
  fprintf(f, "time=%s\n", g_time.c_str());
  fprintf(f, "cq_type=%d\n", (int)g_cq_type);
  fprintf(f, "cq_ft=%s\n", g_cq_freetext.c_str());
  fprintf(f, "skiptx1=%d\n", g_skip_tx1 ? 1 : 0);
  fprintf(f, "free_text=%s\n", g_free_text.c_str());
  fprintf(f, "call=%s\n", g_call.c_str());
  fprintf(f, "grid=%s\n", g_grid_saved_manual.c_str());
  fprintf(f, "offset_src=%d\n", (int)g_offset_src);
  fprintf(f, "radio=%d\n", (int)canonical_radio_type(g_radio));
  fprintf(f, "gps_baud=%d\n", normalize_gps_baud_value(g_gps_baud));
  fprintf(f, "comment1=%s\n", g_comment1.c_str());
  fprintf(f, "ignore_prefixes=%s\n", g_ignore_prefix_text.c_str());
  fprintf(f, "rxtx_log=%d\n", g_rxtx_log ? 1 : 0);
  fprintf(f, "active_bands=%s\n", g_active_band_text.c_str());
  fprintf(f, "rtc_sleep_epoch=%lld\n", (long long)g_rtc_sleep_epoch);
  fprintf(f, "rtc_comp=%d\n", g_rtc_comp);
  fprintf(f, "autoseq_max_retry=%d\n", g_autoseq_max_retry);
  fprintf(f, "ble_enabled=%d\n", g_ble_enabled ? 1 : 0);
  fclose(f);
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
  if (new_mode != UIMode::QSO) {
    g_ble_qso_pick_mode = false;
    g_ble_dump_in_progress = false;
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
      if (g_ble_qso_pick_mode) {
        qso_load_fetch_file_list();
      } else {
        qso_load_file_list();
      }
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
  }
}

#if ENABLE_BLE
static void ble_enter_text_mode() {
  g_ble_text_mode = true;
}

static void ble_exit_text_mode() {
  g_ble_text_mode = false;
}

static bool ble_text_target_active() {
  return menu_long_edit ||
         menu_edit_idx >= 0 ||
         band_edit_idx >= 0 ||
         status_edit_idx == 4 ||
         status_edit_idx == 5;
}

static void ble_commit_text_input(const BleUiInput& input) {
  std::string value = ble_trim_trailing_crlf(input.data, input.len);

  if (menu_long_edit) {
    if (menu_long_kind != LONG_COMMENT) {
      ascii_upper_inplace(value);
    }
    if (menu_long_kind == LONG_IGNORE && value.size() > kIgnorePrefixTextMaxLen) {
      value.resize(kIgnorePrefixTextMaxLen);
    }
    menu_long_buf = value;
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
    ble_exit_text_mode();
    return;
  }

  if (menu_edit_idx >= 0) {
    if (menu_edit_idx != 10) {
      ascii_upper_inplace(value);
    }
    if (menu_edit_idx == 7 && value.size() > 10) value.resize(10);
    if (menu_edit_idx == 15 && value.size() > 11) value.resize(11);
    if (menu_edit_idx == 17 && value.size() > 10) value.resize(10);
    menu_edit_buf = value;
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
        ble_notify_payload("ERROR: use GRID AA00/AA00aa/AA00aa00");
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
    if (menu_edit_idx == 3) {
      ble_update_name_from_station(true);
    }
    if (should_save) {
      save_station_data();
    }
    menu_edit_idx = -1;
    menu_edit_buf.clear();
    draw_menu_view();
    ble_exit_text_mode();
    return;
  }

  if (band_edit_idx >= 0) {
    if (value.size() > 10) value.resize(10);
    band_edit_buffer = value;
    if (!band_edit_buffer.empty()) {
      char* end = nullptr;
      long v = std::strtol(band_edit_buffer.c_str(), &end, 10);
      if (end != band_edit_buffer.c_str() && *end == '\0') {
        g_bands[band_edit_idx].freq = (int)v;
        save_station_data();
      }
    }
    band_edit_idx = -1;
    band_edit_buffer.clear();
    draw_band_view();
    ble_exit_text_mode();
    return;
  }

  if (status_edit_idx == 4 || status_edit_idx == 5) {
    const size_t max_len = status_edit_buffer.size();
    if (value.size() > max_len) value.resize(max_len);
    status_edit_buffer = value;
    if (status_edit_idx == 4) {
      const std::string old_date = g_date;
      const std::string normalized = normalize_date_ymd(status_edit_buffer);
      if (!normalized.empty()) {
        g_date = normalized;
        if (rtc_set_from_strings()) {
          g_time_synced_from_gps = false;
          rtc_sync_to_hw();  // Persist to hardware RTC
          save_station_data();
        } else {
          g_date = old_date;
          ble_notify_payload("ERROR: use DATE YYYY-MM-DD");
        }
      } else {
        g_date = old_date;
        ble_notify_payload("ERROR: use DATE YYYY-MM-DD");
      }
    } else {
      g_time = normalize_time_hms(status_edit_buffer);
      g_time_synced_from_gps = false;
      save_station_data();
      rtc_set_from_strings();
      rtc_sync_to_hw();  // Persist to hardware RTC
    }
    status_edit_idx = -1;
    status_cursor_pos = -1;
    status_edit_buffer.clear();
    draw_status_view();
    ble_exit_text_mode();
    return;
  }

  ble_exit_text_mode();
}
#endif

// "No QMX in N seconds → fall back to USB Serial JTAG / CONTROL mode" timer.
// Set when begin_usb_host_mode arms it; cleared once we either see a QMX
// enumerate (mic or CDC handle) or after we've fallen back. Used by the
// main loop's periodic check below.
static bool    g_qmx_detect_active     = false;
static int64_t g_qmx_detect_deadline_ms = 0;
static constexpr int64_t kQmxDetectTimeoutMs = 10000;

// Switch to MSC (USB Mass Storage) mode. One-way transition: the
// operator copies logs off via the host PC and reboots Mini-FT8 to
// resume operating. esp_bt_controller_mem_release(BLE) inside
// nimble_teardown is irreversible, and we re-claim USB-OTG as a
// device port (was host) for the duration.
static void enter_msc_mode(const char* reason) {
  g_qmx_detect_active = false;
  ESP_LOGI(TAG, "Entering MSC mode: %s", reason);
  debug_log_line("Entering MSC mode");

  // Drop the audio path (releases QMX + USB host), then BLE.
  audio_source_stop();
  vTaskDelay(pdMS_TO_TICKS(200));
  nimble_teardown();

  // Drain any pending Station.txt save that the BLE RPC layer queued
  // before teardown — once we unmount, those writes can't land.
  if (g_config_save_pending) {
    g_config_save_pending = false;
    save_station_data();
  }

  // Unmount FATFS so the host PC has exclusive access to the WL
  // partition via MSC. The unmount internally wl_unmounts, so we
  // re-mount the WL layer fresh below.
  unmount_storage();

  // Re-bind the partition through wear_levelling for MSC. TinyUSB MSC
  // does raw block I/O against this handle.
  const esp_partition_t* part = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "storage");
  if (!part) {
    ESP_LOGE(TAG, "MSC: storage partition not found");
    enter_mode(UIMode::MSC);
    return;
  }
  wl_handle_t wl = WL_INVALID_HANDLE;
  if (wl_mount(part, &wl) != ESP_OK || wl == WL_INVALID_HANDLE) {
    ESP_LOGE(TAG, "MSC: wl_mount failed");
    enter_mode(UIMode::MSC);
    return;
  }

  // Install MSC class driver, register the WL-backed storage as LUN 0,
  // then bring the USB device up — the host PC enumerates it as a
  // removable drive.
  tinyusb_msc_driver_config_t msc_drv_cfg = {};
  if (tinyusb_msc_install_driver(&msc_drv_cfg) != ESP_OK) {
    ESP_LOGE(TAG, "MSC: install_driver failed");
    enter_mode(UIMode::MSC);
    return;
  }
  tinyusb_msc_storage_config_t msc_storage_cfg = {};
  msc_storage_cfg.medium.wl_handle = wl;
  tinyusb_msc_storage_handle_t msc_storage_hdl = nullptr;
  if (tinyusb_msc_new_storage_spiflash(&msc_storage_cfg,
                                       &msc_storage_hdl) != ESP_OK) {
    ESP_LOGE(TAG, "MSC: new_storage_spiflash failed");
    enter_mode(UIMode::MSC);
    return;
  }
  const tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
  if (tinyusb_driver_install(&tusb_cfg) != ESP_OK) {
    ESP_LOGE(TAG, "MSC: tinyusb_driver_install failed");
    enter_mode(UIMode::MSC);
    return;
  }
  ESP_LOGI(TAG, "MSC ready: PC will see Mini-FT8 as a USB drive");

  enter_mode(UIMode::MSC);
  ui_force_redraw_rx();
}

// Perform the STATUS -> '2' action: start the UAC audio source that feeds
// the QMX (or KH1) into the decoder, and sync CAT to the currently selected
// band. Shared between the on-device keypress path and the 1 s splash
// auto-dismiss on boot.
static void begin_usb_host_mode() {
  // The status-view feedback only makes sense when we're actually on the
  // STATUS page (manual S -> '2' path). The splash auto-dismiss lands on
  // RX, so skip the status redraws to avoid painting over the RX view.
  const bool on_status_page = (ui_mode == UIMode::STATUS);
  if (on_status_page) {
    status_edit_idx = 1;
    draw_status_view();
  }
  if (canonical_radio_type(g_radio) == RadioType::KH1 && !g_kh1_connected) {
    g_kh1_connected = true;
    apply_radio_profile_binding();
  }
  if (!audio_source_is_streaming()) {
    debug_log_line("UAC2 start");
    apply_radio_profile_binding();
    debug_log_line("UAC2 bind");
    if (!audio_source_start()) {
      debug_log_line("UAC2 afail");
    } else {
      debug_log_line("UAC2 aok");
      g_decode_enabled = true;
      ui_set_paused(false);
      ui_clear_waterfall();
      esp_err_t rc = radio_control_on_audio_start();
      debug_log_line(rc == ESP_OK ? "UAC2 catok" : "UAC2 catng");
      // Headless-friendly fallback: arm a 10 s "no-QMX" timer when the
      // selected radio is QMX. If neither the UAC mic nor the CDC-ACM
      // endpoint enumerates by then, the main loop calls audio_source_stop
      // and drops into CONTROL so the PC sees the USB Serial JTAG terminal.
      // Gated on QMX so a future "KH1 via Cardputer I2S mic" backend (which
      // never enumerates a USB device) doesn't trip the fallback.
      if (canonical_radio_type(g_radio) == RadioType::QMX) {
        g_qmx_detect_deadline_ms = esp_timer_get_time() / 1000 + kQmxDetectTimeoutMs;
        g_qmx_detect_active = true;
      }
    }
  }
  int freq_hz = g_bands[g_band_sel].freq * 1000;
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

static wl_handle_t s_storage_wl_handle = WL_INVALID_HANDLE;

static bool storage_is_mounted() {
  return s_storage_wl_handle != WL_INVALID_HANDLE;
}

static esp_err_t mount_storage() {
  if (storage_is_mounted()) return ESP_OK;
  esp_vfs_fat_mount_config_t mount_config = {};
  mount_config.format_if_mount_failed = true;
  // Mini-FT8 only ever has one or two files open at once (Station.txt,
  // a daily log). Drop max_files from the prior 5 → 3 for a small
  // additional saving — pairs with CONFIG_FATFS_PER_FILE_CACHE=n in
  // sdkconfig (the bigger win, freed ~16 KB heap).
  mount_config.max_files = 3;
  // 4 KB clusters on a 1 MB partition → 256 clusters total. Keeps the
  // FAT table small (FAT12) and the wear_levelling state proportional —
  // tighter than the default but still way more capacity than a season
  // of ADIF logs.
  mount_config.allocation_unit_size = 4096;
  // Use a local handle and only commit on success. ESP-IDF's mount fn has
  // incomplete cleanup when wl_mount succeeds but a later step (vfs_register,
  // s_f_mount_rw, ctx calloc) ENOMEMs — the fail: label does not wl_unmount,
  // leaving the WL layer half-initialised. If we let s_storage_wl_handle
  // hold that leaked value, a retry sees corrupt state, IDF returns ESP_OK
  // without actually registering /storage with VFS, and every fopen there
  // fails with ENOENT. Recover by wl_unmount-ing on failure ourselves.
  wl_handle_t handle = WL_INVALID_HANDLE;
  esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(
      "/storage", "storage", &mount_config, &handle);
  if (err == ESP_OK) {
    s_storage_wl_handle = handle;
  } else {
    if (handle != WL_INVALID_HANDLE) {
      wl_unmount(handle);
    }
    s_storage_wl_handle = WL_INVALID_HANDLE;
  }
  return err;
}

static void unmount_storage() {
  if (!storage_is_mounted()) return;
  esp_vfs_fat_spiflash_unmount_rw_wl("/storage", s_storage_wl_handle);
  s_storage_wl_handle = WL_INVALID_HANDLE;
}

static void app_task_core0(void* /*param*/) {
  // Mount FATFS first, while heap is fresh — WL grabs its 4 KB sector
  // buffer before USB host or NimBLE start fragmenting things. Stays
  // mounted for the rest of the run; saves write straight through.
  ESP_ERROR_CHECK(mount_storage());

  // Initialize mutexes for thread-safe operations
  log_mutex = xSemaphoreCreateMutex();

  ui_init();
  hashtable_init();

  // Initialize autoseq engine
  autoseq_init();

  // Initialize the functional-core API (creates internal sync primitives).
  // After this, core_api.h consumers (Cardputer UI, future BLE server) can
  // safely call core_get_*, core_cmd_*, and register callbacks.
  core_init();

  // Register the Cardputer UI as a core_api consumer. The callbacks just set
  // the existing dirty flags — the UI main loop drains them on each tick.
  // Trivial handlers only (spec in docs/NATIVE_CLIENT_ARCHITECTURE.md).
  core_on_rx_changed    ([]{ g_rx_dirty = true; });
  core_on_qso_changed   ([]{ g_tx_view_dirty = true; });
  // config changes redraw whatever view is showing them (MENU/STATUS);
  // set both dirty flags so the next UI tick re-evaluates.
  core_on_config_changed([]{ g_rx_dirty = true; g_tx_view_dirty = true; });
  
// Cabrillo Field Day log callback (implemented in autoseq.cpp; declared here to avoid header churn)
using CabrilloFdLogCallback = void (*)(const std::string& dxcall, const std::string& their_fd_exchange);
extern void autoseq_set_cabrillo_fd_callback(CabrilloFdLogCallback cb);

autoseq_set_adif_callback(log_adif_entry);
autoseq_set_cabrillo_fd_callback(log_cabrillo_fd_entry);


  ui_mode = UIMode::RX;
  load_station_data();
  // NimBLE inits here, into a clean heap, so its BT controller can grab
  // big contiguous pool chunks. Deferring it until after USB host
  // enumerated left the controller silently failing to reach BLE_HS_SYNC
  // (advertising never started) — the fragmented post-USB heap shape
  // doesn't satisfy the controller's allocation pattern even when total
  // free bytes look sufficient. Living with NimBLE-first means USB host
  // has to fit alongside its fragments; the s_usb_buffer halving (4608
  // -> 2304) frees enough DMA-capable headroom for CDC pipe alloc.
  init_bluetooth();
  apply_ble_enabled_policy(true);
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
  {
    uart_dev_t *hw = UART_LL_GET_HW(0);
    uint8_t scratch[64];
    while (uart_ll_get_rxfifo_len(hw) > 0) {
      uint32_t n = uart_ll_get_rxfifo_len(hw);
      if (n > 64) n = 64;
      uart_ll_read_rxfifo(hw, scratch, n);
    }
  }

  // UI loop
  char last_key = 0;
  while (true) {
    M5Cardputer.update();
    M5Cardputer.Keyboard.updateKeysState();
    auto &state = M5Cardputer.Keyboard.keysState();
    char c = 0;
    bool c_from_ble = false;
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
    if (c == 0 && s_key_inject_queue) {
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
    if (s_uart_mirror_fire) {
      uart_mirror_dump_screen();
      s_uart_mirror_fire = false;
    }
    if (g_uart_mirror_pending) {
      g_uart_mirror_pending = false;
      s_uart_mirror_fire = true;  // fire on the next iteration
    }
#endif
    if (c == 0) {
      BleUiInput ble_input{};
      if (ble_pop_input(ble_input)) {
#if ENABLE_BLE
        if (!g_ble_enabled || g_ble_dump_in_progress) {
          c_from_ble = false;
        } else {
          c_from_ble = true;
          last_key = 0;  // allow repeated BLE commands without local debounce suppression
          if (g_ble_text_mode) {
            ble_commit_text_input(ble_input);
          } else {
            c = ble_parse_ui_command(ble_input.data, ble_input.len);
            if (c == 0) c_from_ble = false;
          }
        }
#endif
      }
    }

    // BLE remote UI push model: always compare and send latest 7-line snapshot when changed.
    ble_mirror_tick();
    ble_countdown_tick();
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
    }
    // Startup splash: show for kStartupAutoDismissMs, then auto-enter
    // USB-host mode (= STATUS -> '2'). A direct-mode key during the splash
    // window still short-circuits — most usefully 'c', which drops into the
    // USB serial terminal instead of starting the QMX audio path.
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
          // No key within the window — auto-start USB host mode and land
          // on the RX page.
          g_startup_active = false;
          save_station_data();
          enter_mode(UIMode::RX);
          begin_usb_host_mode();
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

    rtc_tick();
    update_countdown();
    consume_cdc_initial_sync();  // auto-sync VFO on first QMX connect (every iter)
    check_slot_boundary();  // TX trigger at slot boundary (matching reference architecture)
    tx_tick();              // Process TX state machine (single-threaded, non-blocking)

    // Drain deferred config saves requested by core_api RPC dispatch
    // (BLE writes land on a 4 KB task stack that can't carry FATFS work
    // buffers; the RPC sets the flag and we run the save here).
    if (g_config_save_pending) {
      g_config_save_pending = false;
      save_station_data();
    }

    // No-QMX fallback: if begin_usb_host_mode armed the timer and nothing
    // has enumerated by the deadline, tear USB host down and switch to
    // CONTROL so a button-less StampS3Bat can still expose the USB serial
    // terminal to the host PC. Detection clears the timer immediately.
    if (g_qmx_detect_active) {
      if (audio_source_qmx_detected()) {
        g_qmx_detect_active = false;
        ESP_LOGI(TAG, "QMX detected; staying in USB host mode");
      } else if (esp_timer_get_time() / 1000 >= g_qmx_detect_deadline_ms) {
        enter_msc_mode("no QMX after 10s");
      }
    }

  // MSC mode is one-way — esp_bt_controller_mem_release (added in
  // Stage 2) and TinyUSB device init (Stage 3) cannot be undone without
  // a reset. Any keypress while in MSC reboots so the operator returns
  // to normal operation cleanly.
  if (ui_mode == UIMode::MSC) {
    if (c != 0 && c != last_key && !c_from_ble) {
      ESP_LOGI(TAG, "Keypress in MSC mode -> reboot");
      vTaskDelay(pdMS_TO_TICKS(100));  // let log flush
      esp_restart();
    }
    last_key = c;
    vTaskDelay(pdMS_TO_TICKS(20));
    continue;
  }

    // Global TX cancel (Esc/` in RX/TX/Status when not editing)
    if (c == '`' &&
        (ui_mode == UIMode::RX || ui_mode == UIMode::TX || ui_mode == UIMode::STATUS) &&
        status_edit_idx == -1) {
      core_cmd_cancel_tx();  // routes through core_api — same effect, plus BLE notify
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
  int cur_status_sync_sig = audio_source_is_streaming() ? 1 : 0;
  if (canonical_radio_type(g_radio) == RadioType::KH1) {
    cur_status_sync_sig |= 2;
    if (g_kh1_connected) cur_status_sync_sig |= 8;
    if (g_kh1_connected && radio_control_ready()) cur_status_sync_sig |= 4;
  }
  if (ui_mode == UIMode::STATUS && cur_status_sync_sig != last_status_sync_sig) {
    draw_status_view();
  }
  if (ui_mode != UIMode::STATUS) {
    last_status_sync_sig = -1;
  } else {
    last_status_sync_sig = cur_status_sync_sig;
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
      else if (c == 'f' || c == 'F') {
        cancel_status_edit();
        if (ui_mode == UIMode::QSO && g_ble_qso_pick_mode) {
          g_ble_qso_return_mode = UIMode::RX;
          ble_cancel_qso_pick_mode();
        } else {
          ble_start_qso_pick_mode();
        }
        switched = true;
      }
      else if (c == 'q' || c == 'Q') { cancel_status_edit(); enter_mode(ui_mode == UIMode::QSO ? UIMode::RX : UIMode::QSO); switched = true; }
      else if (c == 'c' || c == 'C') {
#if ENABLE_BLE
        if (c_from_ble) {
          // BLE 'C' is intentionally ignored.
          switched = true;
        } else
#endif
        {
          // 'C' is a one-way trigger into MSC mode (ignored if already
          // there — MSC's main-loop branch handles its own keypresses).
          if (ui_mode != UIMode::MSC) {
            cancel_status_edit();
            enter_msc_mode("user pressed C");
          }
          switched = true;
        }
      }
      else if (c == 'd' || c == 'D') { cancel_status_edit(); enter_mode(ui_mode == UIMode::DEBUG ? UIMode::RX : UIMode::DEBUG); switched = true; }
      else if (c == 's' || c == 'S') { cancel_status_edit(); enter_mode(ui_mode == UIMode::STATUS ? UIMode::RX : UIMode::STATUS); switched = true; }
      else if (c == 'g' || c == 'G') { cancel_status_edit(); enter_mode(ui_mode == UIMode::GPS ? UIMode::RX : UIMode::GPS); switched = true; }
    }

  if (!switched && c) {
    // Mode-specific handling
    switch (ui_mode) {
      case UIMode::GPS: break;
      case UIMode::RX: {
        int sel = ui_handle_rx_key(c);
        if (sel >= 0 && core_cmd_tap_rx(sel)) {
          // TX-state arming now lives inside core_cmd_tap_rx so both the
          // Cardputer key path and the BLE tap_rx RPC arm immediately.
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
            if (c >= '0' && c <= '9') { band_edit_buffer.push_back(c); draw_band_view(); }
            else if (c == 0x08 || c == 0x7f) {
              if (!band_edit_buffer.empty()) { band_edit_buffer.pop_back(); draw_band_view(); }
            } else if (c == '\r' || c == '\n') {
              if (!band_edit_buffer.empty()) {
                int val = std::stoi(band_edit_buffer);
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
                band_edit_buffer = std::to_string(g_bands[idx].freq);
                draw_band_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              }
            }
          }
          break;
        }
        case UIMode::STATUS: {
        if (status_edit_idx == -1) {
          if (c == '1') { g_status_beacon_temp = (BeaconMode)(((int)g_status_beacon_temp + 1) % 3); draw_status_view(); }
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
                  int freq_hz = g_bands[g_band_sel].freq * 1000;
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
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              }
              else if (c == '6') {
                status_edit_idx = 5; status_edit_buffer = g_time; status_cursor_pos = 0; while (status_cursor_pos < (int)status_edit_buffer.size() && (status_edit_buffer[status_cursor_pos] == ':')) status_cursor_pos++; draw_status_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
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
                  g_time_synced_from_gps = false;
                  save_station_data();
                  rtc_set_from_strings();
                  rtc_sync_to_hw();  // Persist to hardware RTC
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
              std::string path = std::string("/storage/") + deleted;
              if (unlink(path.c_str()) == 0) {
                debug_log_line(std::string("Deleted: ") + deleted);
              } else {
                debug_log_line(std::string("Delete failed: ") + deleted);
              }
              delete_load_file_list();
              int max_page = 0;
              if (!g_d_lines.empty()) {
                max_page = ((int)g_d_lines.size() - 1) / 6;
              }
              if (d_page > max_page) d_page = max_page;
              ui_draw_list(g_d_lines, d_page, -1);
            }
          }
          break;
        }
        case UIMode::QSO: {
#if ENABLE_BLE
          if (g_ble_qso_pick_mode) {
            if (c == ';') {
              if (q_page > 0) { q_page--; qso_draw_page(); }
            } else if (c == '.') {
              if ((q_page + 1) * 6 < (int)g_q_lines.size()) { q_page++; qso_draw_page(); }
            } else if (c >= '1' && c <= '6') {
              ble_try_dump_qso_file_by_key(c);
            } else if (c == '`') {
              ble_cancel_qso_pick_mode();
            }
            break;
          }
#endif
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
                qso_load_entries(g_q_current_file);
                g_q_show_entries = true;
                q_page = 0;
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
              if (q_page > 0) { q_page--; qso_draw_page(); }
            } else if (c == '.') {
              if ((q_page + 1) * 6 < (int)g_q_lines.size()) { q_page++; qso_draw_page(); }
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
                if (menu_edit_idx == 3) {
                  ble_update_name_from_station(true);
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
                int64_t now_slot = rtc_now_ms() / 15000;
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
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              } else if (c == '4') {
                menu_edit_idx = 3; // Call (line index 3)
                menu_edit_buf = g_call;
                draw_menu_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              } else if (c == '5') {
                menu_edit_idx = 4; // Grid (line index 4)
                menu_edit_buf = g_grid;
                draw_menu_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              } else if (c == '6') {
                ESP_LOGI(TAG, "Entering deep sleep (GPIO0 wake)");
                // Save current accurate time for compensation after wake-up
                if (rtc_valid) {
                  g_rtc_sleep_epoch = rtc_epoch_base +
                      (esp_timer_get_time() / 1000 - rtc_ms_start) / 1000;
                  rtc_sync_to_hw();  // Sync to hardware RTC
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
#if ENABLE_BLE
                  if (c_from_ble) ble_enter_text_mode();
#endif
                } else if (c == '3') {
                  g_radio = (canonical_radio_type(g_radio) == RadioType::KH1)
                              ? RadioType::QMX
                              : RadioType::KH1;
                  apply_radio_profile_binding();
                  save_station_data();
                  draw_menu_view();
                } else if (c == '4') {
                  menu_long_edit = true;
                  menu_long_kind = LONG_IGNORE;
                  menu_long_buf = g_ignore_prefix_text;
                  menu_long_backup = g_ignore_prefix_text;
                  draw_menu_view();
#if ENABLE_BLE
                  if (c_from_ble) ble_enter_text_mode();
#endif
                } else if (c == '5') {
                  menu_long_edit = true;
                  menu_long_kind = LONG_COMMENT;
                  menu_long_buf = g_comment1;
                  menu_long_backup = g_comment1;
                  draw_menu_view();
#if ENABLE_BLE
                  if (c_from_ble) ble_enter_text_mode();
#endif
                } else if (c == '6') {
                  g_ble_enabled = !g_ble_enabled;
                  apply_ble_enabled_policy(true);
                  save_station_data();
                  draw_menu_view();
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
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              } else if (c == '4') {
                menu_edit_idx = 15; // RTC Comp line
                menu_edit_buf = std::to_string(g_rtc_comp);
                draw_menu_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              } else if (c == '5') {
                CopyLogsResult copy_res = copy_logs_spiffs_to_sd_overwrite();
                menu_flash_idx = 16; // abs index of page 2 line 5
                menu_flash_deadline = rtc_now_ms() + 500;
                if (copy_res.missed_count <= 0) {
                  menu_copy_feedback_text = "Copied OK";
                } else {
                  char fb[20];
                  snprintf(fb, sizeof(fb), "Missed %d", copy_res.missed_count);
                  menu_copy_feedback_text = fb;
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
                  debug_log_line("Copied SPIFFS files to SD");
                }

                draw_menu_view();
              } else if (c == '6') {
                menu_edit_idx = 17; // Max Retry line
                menu_edit_buf = std::to_string(g_autoseq_max_retry);
                draw_menu_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              }
            }
          }
          break;
        }
      }
    }

#if ENABLE_BLE
    if (g_ble_text_mode && !ble_text_target_active()) {
      ble_exit_text_mode();
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

extern "C" void app_main(void) {
  // Run the main application loop on core0.
  xTaskCreatePinnedToCore(app_task_core0, "app_core0", APP_CORE0_STACK_BYTES, nullptr, 5, nullptr, 0);
}
static void draw_status_line(int idx, const std::string& text, bool highlight) {
  const int line_h = 19;
  const int start_y = 18 + 3 + 3; // WATERFALL_H + COUNTDOWN_H + gap
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
