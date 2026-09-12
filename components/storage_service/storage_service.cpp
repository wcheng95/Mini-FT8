#include "storage_service.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <new>
#include <sys/stat.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"
#include "wear_levelling.h"

namespace {

constexpr char kPartitionLabel[] = "fatfs";
constexpr char kBasePath[] = "/storage";
constexpr char kStationFile[] = "Station.txt";
constexpr char kStationTemp[] = "Station.tmp";
constexpr char kSdBasePath[] = "/sdcard";
constexpr gpio_num_t kSdMiso = GPIO_NUM_39;
constexpr gpio_num_t kSdMosi = GPIO_NUM_14;
constexpr gpio_num_t kSdClock = GPIO_NUM_40;
constexpr gpio_num_t kSdChipSelect = GPIO_NUM_12;
// LoRa-1262 module CS shares GPIO5 with the Cardputer debug UART RX path.
constexpr gpio_num_t kLora1262ChipSelect = GPIO_NUM_5;

const char* TAG = "storage_service";
const char* COPY_TAG = "STORAGE_COPY";

StaticSemaphore_t s_mutex_buffer;
SemaphoreHandle_t s_mutex;
StorageOwner s_owner = StorageOwner::UNAVAILABLE;
wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
tinyusb_msc_storage_handle_t s_msc_storage;
bool s_tinyusb_installed;
size_t s_open_streams;
bool s_station_sync_attempted;
sdmmc_card_t* s_sd_card;
bool s_sd_mounted;
bool s_lora_cs_held_for_sd;

enum class MountTransitionResult : uint8_t {
    NONE,
    STARTED,
    COMPLETE,
    FAILED,
};

volatile MountTransitionResult s_mount_transition_result = MountTransitionResult::NONE;
volatile tinyusb_msc_mount_point_t s_mount_transition_point =
    TINYUSB_MSC_STORAGE_MOUNT_APP;

class StorageGuard {
public:
    StorageGuard() : held_(s_mutex && xSemaphoreTakeRecursive(s_mutex, portMAX_DELAY) == pdTRUE) {}
    ~StorageGuard() {
        if (held_) {
            xSemaphoreGiveRecursive(s_mutex);
        }
    }
    bool held() const { return held_; }

private:
    bool held_;
};

bool firmware_owns_storage_locked() {
    return s_owner == StorageOwner::FIRMWARE && s_msc_storage != nullptr;
}

bool wl_mount_error_allows_repair(esp_err_t err) {
    return err != ESP_ERR_NO_MEM && err != ESP_ERR_INVALID_ARG;
}

esp_err_t mount_wear_levelling_with_repair(const esp_partition_t* partition, bool* repaired) {
    if (repaired) {
        *repaired = false;
    }

    s_wl_handle = WL_INVALID_HANDLE;
    esp_err_t err = wl_mount(partition, &s_wl_handle);
    if (err == ESP_OK) {
        return ESP_OK;
    }

    s_wl_handle = WL_INVALID_HANDLE;
    ESP_LOGE(TAG,
             "wear levelling mount failed for partition '%s' addr=0x%08lx size=0x%08lx: %s",
             partition->label,
             static_cast<unsigned long>(partition->address),
             static_cast<unsigned long>(partition->size),
             esp_err_to_name(err));

    if (partition->readonly || !wl_mount_error_allows_repair(err)) {
        ESP_LOGE(TAG, "not erasing internal FATFS partition after wl_mount failure");
        return err;
    }

    ESP_LOGW(TAG,
             "erasing internal FATFS partition '%s' addr=0x%08lx size=0x%08lx after wl_mount failure",
             partition->label,
             static_cast<unsigned long>(partition->address),
             static_cast<unsigned long>(partition->size));
    esp_err_t erase_err = esp_partition_erase_range(partition, 0, partition->size);
    ESP_LOGI(TAG, "internal FATFS partition erase result: %s", esp_err_to_name(erase_err));
    if (erase_err != ESP_OK) {
        s_wl_handle = WL_INVALID_HANDLE;
        return erase_err;
    }

    ESP_LOGI(TAG, "retrying wear levelling mount after internal FATFS erase");
    err = wl_mount(partition, &s_wl_handle);
    ESP_LOGI(TAG, "wear levelling retry result: %s", esp_err_to_name(err));
    if (err != ESP_OK) {
        s_wl_handle = WL_INVALID_HANDLE;
        return err;
    }

    if (repaired) {
        *repaired = true;
    }
    return ESP_OK;
}

const char* storage_owner_name(StorageOwner owner) {
    switch (owner) {
        case StorageOwner::UNAVAILABLE:
            return "unavailable";
        case StorageOwner::FIRMWARE:
            return "firmware";
        case StorageOwner::USB_HOST:
            return "usb_host";
        case StorageOwner::TRANSITION:
            return "transition";
    }
    return "unknown";
}

bool normalize_name(const std::string& input, std::string& name) {
    name = input;
    const std::string prefix = std::string(kBasePath) + "/";
    if (name.compare(0, prefix.size(), prefix) == 0) {
        name.erase(0, prefix.size());
    }
    if (name.empty() || name.front() == '/' || name.find('\\') != std::string::npos ||
        name.find("..") != std::string::npos) {
        return false;
    }
    return true;
}

bool build_path(const std::string& input, std::string& path) {
    std::string name;
    if (!normalize_name(input, name)) {
        return false;
    }
    path = std::string(kBasePath) + "/" + name;
    return true;
}

bool sync_file(FILE* file) {
    return file && fflush(file) == 0 && fsync(fileno(file)) == 0;
}

bool flush_all_locked() {
    errno = 0;
    const int rc = fflush(nullptr);
    const int err_no = errno;
    ESP_LOGI(COPY_TAG, "pre-copy flush: rc=%d errno=%d (%s)",
             rc, err_no, strerror(err_no));
    return rc == 0;
}

bool write_atomic_locked(const std::string& input, const std::string& content) {
    if (!firmware_owns_storage_locked()) {
        return false;
    }

    std::string name;
    std::string final_path;
    if (!normalize_name(input, name) || !build_path(name, final_path)) {
        return false;
    }
    const std::string temp_name = (name == kStationFile) ? kStationTemp : name + ".tmp";
    std::string temp_path;
    if (!build_path(temp_name, temp_path)) {
        return false;
    }

    FILE* file = fopen(temp_path.c_str(), "wb");
    if (!file) {
        return false;
    }

    bool ok = content.empty() || fwrite(content.data(), 1, content.size(), file) == content.size();
    ok = ok && sync_file(file);
    if (fclose(file) != 0) {
        ok = false;
    }
    if (!ok) {
        unlink(temp_path.c_str());
        return false;
    }

    if (unlink(final_path.c_str()) != 0 && errno != ENOENT) {
        unlink(temp_path.c_str());
        return false;
    }
    if (rename(temp_path.c_str(), final_path.c_str()) != 0) {
        ESP_LOGE(TAG, "rename failed: %s -> %s", temp_path.c_str(), final_path.c_str());
        unlink(temp_path.c_str());
        return false;
    }
    return true;
}

bool list_files_locked(std::vector<std::string>& files) {
    files.clear();
    if (!firmware_owns_storage_locked()) {
        return false;
    }

    DIR* dir = opendir(kBasePath);
    if (!dir) {
        return false;
    }

    while (dirent* entry = readdir(dir)) {
        const char* name = entry->d_name;
        if (!name || name[0] == '.') {
            continue;
        }
        std::string path;
        if (!build_path(name, path)) {
            continue;
        }
        struct stat info {};
        if (stat(path.c_str(), &info) == 0 && S_ISREG(info.st_mode)) {
            files.emplace_back(name);
        }
    }
    closedir(dir);
    return true;
}

void hold_lora1262_cs_deselected() {
    // Explicitly deselect the LoRa-1262 cap's CS pin to prevent SPI bus
    // collisions with the SD card while the shared bus is active.
    gpio_reset_pin(kLora1262ChipSelect);
    gpio_set_direction(kLora1262ChipSelect, GPIO_MODE_OUTPUT);
    gpio_set_level(kLora1262ChipSelect, 1);
    s_lora_cs_held_for_sd = true;
    ESP_LOGI(COPY_TAG, "LoRa-1262 CS (GPIO%d) held HIGH for SD access",
             static_cast<int>(kLora1262ChipSelect));
}

void release_lora1262_cs_hold() {
    if (!s_lora_cs_held_for_sd) {
        return;
    }
    // Leave the pin as a floating input. Callers that need UART RX on G5
    // (GNSS_LoRa:OFF) must re-apply their pin policy after SD access ends.
    gpio_reset_pin(kLora1262ChipSelect);
    gpio_set_direction(kLora1262ChipSelect, GPIO_MODE_INPUT);
    gpio_set_pull_mode(kLora1262ChipSelect, GPIO_FLOATING);
    gpio_intr_disable(kLora1262ChipSelect);
    s_lora_cs_held_for_sd = false;
    ESP_LOGI(COPY_TAG, "LoRa-1262 CS (GPIO%d) released after SD access",
             static_cast<int>(kLora1262ChipSelect));
}

esp_err_t mount_sd_locked() {
    if (s_sd_mounted && s_sd_card) {
        return ESP_OK;
    }

    hold_lora1262_cs_deselected();

    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = kSdMosi;
    bus_config.miso_io_num = kSdMiso;
    bus_config.sclk_io_num = kSdClock;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 4096;

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        release_lora1262_cs_hold();
        return err;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = kSdChipSelect;
    slot_config.host_id = SPI2_HOST;

    esp_vfs_fat_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 5000;
    err = esp_vfs_fat_sdspi_mount(kSdBasePath, &host, &slot_config, &mount_config, &s_sd_card);
    if (err != ESP_OK) {
        s_sd_card = nullptr;
        s_sd_mounted = false;
        spi_bus_free(SPI2_HOST);
        release_lora1262_cs_hold();
        return err;
    }

    s_sd_mounted = true;
    return ESP_OK;
}

void unmount_sd_locked() {
    if (s_sd_mounted && s_sd_card) {
        esp_vfs_fat_sdcard_unmount(kSdBasePath, s_sd_card);
        s_sd_card = nullptr;
        s_sd_mounted = false;
    }
    spi_bus_free(SPI2_HOST);
    release_lora1262_cs_hold();
}

struct CopyFileStatus {
    esp_err_t err = ESP_OK;
    int err_no = 0;
    const char* stage = "ok";
};

CopyFileStatus copy_error(esp_err_t err, int err_no, const char* stage) {
    CopyFileStatus status;
    status.err = err;
    status.err_no = err_no;
    status.stage = stage;
    return status;
}

CopyFileStatus copy_file_locked(const std::string& source, const std::string& destination) {
    FILE* input = fopen(source.c_str(), "rb");
    if (!input) {
        return copy_error(ESP_FAIL, errno, "open-src");
    }
    FILE* output = fopen(destination.c_str(), "wb");
    if (!output) {
        const int err_no = errno;
        fclose(input);
        return copy_error(ESP_FAIL, err_no, "open-dst");
    }

    uint8_t buffer[4096];
    CopyFileStatus status;
    while (true) {
        const size_t count = fread(buffer, 1, sizeof(buffer), input);
        if (count > 0 && fwrite(buffer, 1, count, output) != count) {
            status = copy_error(ESP_FAIL, errno, "write");
            break;
        }
        if (count < sizeof(buffer)) {
            if (ferror(input)) {
                status = copy_error(ESP_FAIL, errno, "read");
            }
            break;
        }
    }
    if (status.err == ESP_OK) {
        if (fflush(output) != 0) {
            status = copy_error(ESP_FAIL, errno, "flush-dst");
        } else if (fsync(fileno(output)) != 0) {
            status = copy_error(ESP_FAIL, errno, "fsync-dst");
        }
    }
    if (fclose(output) != 0 && status.err == ESP_OK) {
        status = copy_error(ESP_FAIL, errno, "close-dst");
    }
    fclose(input);
    return status;
}

CopyFileStatus copy_file_retry_locked(const std::string& name,
                                      const std::string& source,
                                      const std::string& destination,
                                      const char* pass_name,
                                      int attempts = 5) {
    CopyFileStatus result = copy_error(ESP_FAIL, 0, "not-run");
    for (int attempt = 0; attempt < attempts; ++attempt) {
        result = copy_file_locked(source, destination);
        if (result.err == ESP_OK) {
            break;
        }
        ESP_LOGW(COPY_TAG,
                 "copy attempt failed pass=%s file=%s attempt=%d/%d err=%s errno=%d (%s) stage=%s",
                 pass_name, name.c_str(), attempt + 1, attempts,
                 esp_err_to_name(result.err), result.err_no, strerror(result.err_no),
                 result.stage);
        if (attempt + 1 < attempts) {
            vTaskDelay(pdMS_TO_TICKS(80));
        }
    }
    return result;
}

struct CopyFileRecord {
    std::string name;
    bool copied = false;
    CopyFileStatus last_status = copy_error(ESP_FAIL, 0, "not-run");
};

CopyFileRecord& copy_record_for(std::vector<CopyFileRecord>& records,
                                const std::string& name) {
    for (CopyFileRecord& record : records) {
        if (record.name == name) {
            return record;
        }
    }
    records.push_back(CopyFileRecord {});
    records.back().name = name;
    return records.back();
}

bool source_file_size_locked(const std::string& source, long long& size) {
    struct stat info {};
    if (stat(source.c_str(), &info) != 0 || !S_ISREG(info.st_mode)) {
        size = -1;
        return false;
    }
    size = static_cast<long long>(info.st_size);
    return true;
}

CopyFileStatus copy_named_file_locked(const std::string& name,
                                      const char* pass_name,
                                      int attempts) {
    std::string source;
    if (!build_path(name, source)) {
        ESP_LOGW(COPY_TAG, "copy skipped pass=%s file=%s invalid source name",
                 pass_name, name.c_str());
        return copy_error(ESP_ERR_INVALID_ARG, EINVAL, "build-src");
    }

    long long source_size = -1;
    if (!source_file_size_locked(source, source_size)) {
        const int err_no = errno;
        ESP_LOGW(COPY_TAG,
                 "copy skipped pass=%s file=%s source=%s stat failed errno=%d (%s)",
                 pass_name, name.c_str(), source.c_str(), err_no, strerror(err_no));
        return copy_error(ESP_FAIL, err_no, "stat-src");
    }

    const std::string destination = std::string(kSdBasePath) + "/" + name;
    ESP_LOGI(COPY_TAG,
             "copy start pass=%s file=%s size=%lld source=%s destination=%s attempts=%d",
             pass_name, name.c_str(), source_size, source.c_str(), destination.c_str(),
             attempts);
    CopyFileStatus status =
        copy_file_retry_locked(name, source, destination, pass_name, attempts);
    if (status.err == ESP_OK) {
        ESP_LOGI(COPY_TAG, "copy ok pass=%s file=%s size=%lld destination=%s",
                 pass_name, name.c_str(), source_size, destination.c_str());
    } else {
        ESP_LOGE(COPY_TAG,
                 "copy failed pass=%s file=%s size=%lld destination=%s err=%s errno=%d (%s) stage=%s",
                 pass_name, name.c_str(), source_size, destination.c_str(),
                 esp_err_to_name(status.err), status.err_no, strerror(status.err_no),
                 status.stage);
    }
    return status;
}

void set_copy_status(std::vector<CopyFileRecord>& records,
                     const std::string& name,
                     const CopyFileStatus& status) {
    CopyFileRecord& record = copy_record_for(records, name);
    record.last_status = status;
    if (status.err == ESP_OK) {
        record.copied = true;
    }
}

void copy_file_record_locked(std::vector<CopyFileRecord>& records,
                             const std::string& name,
                             const char* pass_name,
                             int attempts) {
    const CopyFileStatus status = copy_named_file_locked(name, pass_name, attempts);
    set_copy_status(records, name, status);
}

void copy_missing_pass_locked(std::vector<CopyFileRecord>& records,
                              const char* pass_name,
                              int attempts) {
    const size_t record_count = records.size();
    for (size_t i = 0; i < record_count; ++i) {
        if (!records[i].copied) {
            copy_file_record_locked(records, records[i].name, pass_name, attempts);
        }
    }
}

void copy_active_file_if_present_locked(std::vector<CopyFileRecord>& records,
                                        const std::string& name,
                                        const char* pass_name,
                                        int attempts) {
    if (name.empty()) {
        return;
    }

    std::string source;
    long long source_size = -1;
    if (!build_path(name, source) || !source_file_size_locked(source, source_size)) {
        ESP_LOGI(COPY_TAG, "active retry skipped pass=%s file=%s exists=0",
                 pass_name, name.c_str());
        return;
    }

    copy_file_record_locked(records, name, pass_name, attempts);
}

void log_missed_files(const std::vector<std::string>& missed_files) {
    if (missed_files.empty()) {
        ESP_LOGI(COPY_TAG, "missed files: <none>");
        return;
    }

    std::string missed;
    for (const std::string& name : missed_files) {
        if (!missed.empty()) {
            missed += ",";
        }
        missed += name;
    }
    ESP_LOGW(COPY_TAG, "missed files: %s", missed.c_str());
}

void log_copy_summary(const StorageCopyResult& result) {
    log_missed_files(result.missed_files);
    ESP_LOGI(COPY_TAG, "end copied=%d missed=%d err=%s",
             result.copied_count, result.missed_count, esp_err_to_name(result.err));
}

bool cabrillo_ensure_header_locked(const std::string& path,
                                   const std::string& mycall,
                                   const std::string& location) {
    struct stat info {};
    if (stat(path.c_str(), &info) == 0 && info.st_size > 0) {
        return true;
    }

    FILE* file = fopen(path.c_str(), "wb");
    if (!file) {
        return false;
    }
    bool ok = fprintf(file, "START-OF-LOG: 3.0\n") >= 0;
    ok = ok && fprintf(file, "CREATED-BY: Mini-FT8\n") >= 0;
    ok = ok && fprintf(file, "CONTEST: ARRL-FIELD-DAY\n") >= 0;
    ok = ok && fprintf(file, "CALLSIGN: %s\n", mycall.c_str()) >= 0;
    ok = ok && fprintf(file, "CATEGORY-OPERATOR: SINGLE-OP\n") >= 0;
    ok = ok && fprintf(file, "CATEGORY-TRANSMITTER: ONE\n") >= 0;
    ok = ok && fprintf(file, "CATEGORY-ASSISTED: NON-ASSISTED\n") >= 0;
    ok = ok && fprintf(file, "CATEGORY-BAND: ALL\n") >= 0;
    ok = ok && fprintf(file, "CATEGORY-MODE: MIXED\n") >= 0;
    ok = ok && fprintf(file, "CATEGORY-POWER: LOW\n") >= 0;
    ok = ok && fprintf(file, "CATEGORY-STATION: PORTABLE\n") >= 0;
    ok = ok && fprintf(file, "LOCATION: %s\n", location.c_str()) >= 0;
    ok = ok && fprintf(file, "OPERATORS: %s\n", mycall.c_str()) >= 0;
    ok = ok && fprintf(file, "END-OF-LOG:\n") >= 0;
    ok = ok && sync_file(file);
    if (fclose(file) != 0) {
        ok = false;
    }
    return ok;
}

bool cabrillo_truncate_end_marker(FILE* file) {
    if (!file || fseek(file, 0, SEEK_END) != 0) {
        return false;
    }
    const long file_end = ftell(file);
    if (file_end <= 0) {
        return false;
    }

    constexpr long kMaxTail = 256;
    const long tail_start = file_end > kMaxTail ? file_end - kMaxTail : 0;
    if (fseek(file, tail_start, SEEK_SET) != 0) {
        return false;
    }

    std::string tail(static_cast<size_t>(file_end - tail_start), '\0');
    tail.resize(fread(tail.data(), 1, tail.size(), file));
    size_t line_end = tail.size();
    while (line_end > 0 && (tail[line_end - 1] == '\n' || tail[line_end - 1] == '\r')) {
        --line_end;
    }
    if (line_end == 0) {
        return false;
    }
    size_t line_start = tail.rfind('\n', line_end - 1);
    line_start = line_start == std::string::npos ? 0 : line_start + 1;
    if (tail.substr(line_start, line_end - line_start) != "END-OF-LOG:") {
        return false;
    }

    const int descriptor = fileno(file);
    const long truncate_at = tail_start + static_cast<long>(line_start);
    return descriptor >= 0 && ftruncate(descriptor, truncate_at) == 0 &&
           fseek(file, 0, SEEK_END) == 0;
}

void storage_event_callback(tinyusb_msc_storage_handle_t,
                            tinyusb_msc_event_t* event,
                            void*) {
    if (!event) {
        return;
    }

    s_mount_transition_point = event->mount_point;
    switch (event->id) {
        case TINYUSB_MSC_EVENT_MOUNT_START:
            s_mount_transition_result = MountTransitionResult::STARTED;
            break;
        case TINYUSB_MSC_EVENT_MOUNT_COMPLETE:
            s_mount_transition_result = MountTransitionResult::COMPLETE;
            break;
        case TINYUSB_MSC_EVENT_MOUNT_FAILED:
        case TINYUSB_MSC_EVENT_FORMAT_REQUIRED:
        case TINYUSB_MSC_EVENT_FORMAT_FAILED:
            s_mount_transition_result = MountTransitionResult::FAILED;
            break;
    }
    ESP_LOGI(TAG, "MSC storage event=%d mount=%d", event->id, event->mount_point);
}

esp_err_t set_storage_mount_point_locked(tinyusb_msc_mount_point_t mount_point) {
    s_mount_transition_result = MountTransitionResult::NONE;
    s_mount_transition_point = mount_point;

    const esp_err_t err = tinyusb_msc_set_storage_mount_point(s_msc_storage, mount_point);
    if (err != ESP_OK) {
        return err;
    }
    if (s_mount_transition_result != MountTransitionResult::COMPLETE ||
        s_mount_transition_point != mount_point) {
        ESP_LOGE(TAG, "FATFS ownership transition to mount=%d was not confirmed",
                 mount_point);
        return ESP_FAIL;
    }
    return ESP_OK;
}

}  // namespace

struct StorageStream {
    FILE* file = nullptr;
};

esp_err_t storage_service_init() {
    if (!s_mutex) {
        s_mutex = xSemaphoreCreateRecursiveMutexStatic(&s_mutex_buffer);
        if (!s_mutex) {
            return ESP_ERR_NO_MEM;
        }
    }

    StorageGuard guard;
    if (!guard.held()) {
        return ESP_FAIL;
    }
    if (s_owner != StorageOwner::UNAVAILABLE) {
        return ESP_OK;
    }

    const esp_partition_t* partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, kPartitionLabel);
    if (!partition) {
        ESP_LOGE(TAG, "FATFS partition '%s' not found", kPartitionLabel);
        return ESP_ERR_NOT_FOUND;
    }

    bool repaired_wl_mount = false;
    esp_err_t err = mount_wear_levelling_with_repair(partition, &repaired_wl_mount);
    if (err != ESP_OK) {
        s_wl_handle = WL_INVALID_HANDLE;
        ESP_LOGE(TAG, "wear levelling mount failed: %s", esp_err_to_name(err));
        return err;
    }

    tinyusb_msc_driver_config_t driver_config = {};
    driver_config.user_flags.auto_mount_off = 1;
    driver_config.callback = storage_event_callback;
    err = tinyusb_msc_install_driver(&driver_config);
    if (err != ESP_OK) {
        wl_unmount(s_wl_handle);
        s_wl_handle = WL_INVALID_HANDLE;
        ESP_LOGE(TAG, "MSC driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    tinyusb_msc_storage_config_t storage_config = {};
    storage_config.medium.wl_handle = s_wl_handle;
    storage_config.fat_fs.base_path = const_cast<char*>(kBasePath);
    storage_config.fat_fs.config.format_if_mount_failed = true;
    storage_config.fat_fs.config.max_files = 8;
    storage_config.fat_fs.config.allocation_unit_size = 4096;
    storage_config.mount_point = TINYUSB_MSC_STORAGE_MOUNT_APP;
    err = tinyusb_msc_new_storage_spiflash(&storage_config, &s_msc_storage);
    if (err != ESP_OK) {
        tinyusb_msc_uninstall_driver();
        wl_unmount(s_wl_handle);
        s_wl_handle = WL_INVALID_HANDLE;
        s_msc_storage = nullptr;
        ESP_LOGE(TAG, "FATFS storage creation failed: %s", esp_err_to_name(err));
        return err;
    }

    s_owner = StorageOwner::FIRMWARE;
    ESP_LOGI(TAG, "initialized owner: firmware owns %s on partition '%s'",
             kBasePath, kPartitionLabel);
    if (repaired_wl_mount) {
        ESP_LOGW(TAG, "internal storage reformatted after wear levelling mount failure");
    }
    return ESP_OK;
}

StorageOwner storage_service_owner() {
    StorageGuard guard;
    return guard.held() ? s_owner : StorageOwner::UNAVAILABLE;
}

bool storage_service_firmware_available() {
    StorageGuard guard;
    return guard.held() && firmware_owns_storage_locked();
}

bool storage_service_usb_drive_enabled() {
    StorageGuard guard;
    return guard.held() && s_owner == StorageOwner::USB_HOST;
}

size_t storage_service_open_stream_count() {
    StorageGuard guard;
    return guard.held() ? s_open_streams : 0;
}

esp_err_t storage_service_set_usb_drive_enabled(bool enabled) {
    StorageGuard guard;
    if (!guard.held() || !s_msc_storage) {
        return ESP_ERR_INVALID_STATE;
    }

    if (enabled) {
        if (s_owner == StorageOwner::USB_HOST) {
            return ESP_OK;
        }
        if (s_owner != StorageOwner::FIRMWARE) {
            return ESP_ERR_INVALID_STATE;
        }
        if (s_open_streams != 0) {
            ESP_LOGW(TAG, "USB Drive blocked by %u open storage stream(s)",
                     static_cast<unsigned>(s_open_streams));
            return ESP_ERR_INVALID_STATE;
        }

        s_owner = StorageOwner::TRANSITION;
        esp_err_t err = set_storage_mount_point_locked(TINYUSB_MSC_STORAGE_MOUNT_USB);
        if (err != ESP_OK) {
            s_owner = s_mount_transition_result == MountTransitionResult::NONE
                          ? StorageOwner::FIRMWARE
                          : StorageOwner::UNAVAILABLE;
            return err;
        }

        const tinyusb_config_t tinyusb_config = TINYUSB_DEFAULT_CONFIG();
        err = tinyusb_driver_install(&tinyusb_config);
        if (err != ESP_OK) {
            const esp_err_t rollback =
                set_storage_mount_point_locked(TINYUSB_MSC_STORAGE_MOUNT_APP);
            s_owner = rollback == ESP_OK ? StorageOwner::FIRMWARE : StorageOwner::UNAVAILABLE;
            return err;
        }

        s_tinyusb_installed = true;
        s_owner = StorageOwner::USB_HOST;
        ESP_LOGI(TAG, "USB Drive ON: PC owns FATFS");
        return ESP_OK;
    }

    if (s_owner == StorageOwner::FIRMWARE) {
        return ESP_OK;
    }
    if (s_owner != StorageOwner::USB_HOST || !s_tinyusb_installed) {
        return ESP_ERR_INVALID_STATE;
    }

    s_owner = StorageOwner::TRANSITION;
    esp_err_t err = tinyusb_driver_uninstall();
    if (err != ESP_OK) {
        s_owner = StorageOwner::USB_HOST;
        return err;
    }
    s_tinyusb_installed = false;

    err = set_storage_mount_point_locked(TINYUSB_MSC_STORAGE_MOUNT_APP);
    if (err != ESP_OK) {
        s_owner = StorageOwner::UNAVAILABLE;
        return err;
    }

    s_owner = StorageOwner::FIRMWARE;
    ESP_LOGI(TAG, "USB Drive OFF: firmware owns FATFS");
    return ESP_OK;
}

bool storage_file_exists(const std::string& name) {
    StorageGuard guard;
    std::string path;
    struct stat info {};
    return guard.held() && firmware_owns_storage_locked() && build_path(name, path) &&
           stat(path.c_str(), &info) == 0 && S_ISREG(info.st_mode);
}

bool storage_file_remove(const std::string& name) {
    StorageGuard guard;
    std::string path;
    return guard.held() && firmware_owns_storage_locked() && build_path(name, path) &&
           unlink(path.c_str()) == 0;
}

bool storage_file_list(std::vector<std::string>& files) {
    StorageGuard guard;
    return guard.held() && list_files_locked(files);
}

bool storage_file_read_text(const std::string& name, std::string& content) {
    StorageGuard guard;
    content.clear();
    std::string path;
    if (!guard.held() || !firmware_owns_storage_locked() || !build_path(name, path)) {
        return false;
    }

    FILE* file = fopen(path.c_str(), "rb");
    if (!file) {
        return false;
    }
    char buffer[512];
    while (true) {
        const size_t count = fread(buffer, 1, sizeof(buffer), file);
        if (count > 0) {
            content.append(buffer, count);
        }
        if (count < sizeof(buffer)) {
            break;
        }
    }
    const bool ok = ferror(file) == 0;
    fclose(file);
    return ok;
}

bool storage_file_write_atomic(const std::string& name, const std::string& content) {
    StorageGuard guard;
    return guard.held() && write_atomic_locked(name, content);
}

bool storage_file_append(const std::string& name,
                         const std::string& content,
                         const std::string& header_if_new,
                         bool sync_to_flash) {
    StorageGuard guard;
    std::string path;
    if (!guard.held() || !firmware_owns_storage_locked() || !build_path(name, path)) {
        return false;
    }

    bool need_header = false;
    if (!header_if_new.empty()) {
        struct stat info {};
        need_header = stat(path.c_str(), &info) != 0 || info.st_size == 0;
    }

    FILE* file = fopen(path.c_str(), "ab");
    if (!file) {
        return false;
    }
    bool ok = !need_header ||
              fwrite(header_if_new.data(), 1, header_if_new.size(), file) ==
                  header_if_new.size();
    ok = ok && (content.empty() ||
                fwrite(content.data(), 1, content.size(), file) == content.size());
    if (ok && sync_to_flash) {
        ok = sync_file(file);
    }
    if (fclose(file) != 0) {
        ok = false;
    }
    return ok;
}

bool storage_file_append_cabrillo(const std::string& mycall,
                                  const std::string& location,
                                  const std::string& qso_line) {
    StorageGuard guard;
    std::string path;
    if (!guard.held() || !firmware_owns_storage_locked() ||
        !build_path("fieldday.txt", path) ||
        !cabrillo_ensure_header_locked(path, mycall, location)) {
        return false;
    }

    FILE* file = fopen(path.c_str(), "r+b");
    if (!file) {
        file = fopen(path.c_str(), "a+b");
    }
    if (!file) {
        return false;
    }

    cabrillo_truncate_end_marker(file);
    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        return false;
    }
    const long end = ftell(file);
    if (end > 0 && fseek(file, -1, SEEK_END) == 0) {
        const int last = fgetc(file);
        fseek(file, 0, SEEK_END);
        if (last != '\n') {
            fputc('\n', file);
        }
    }
    bool ok = fprintf(file, "%s\nEND-OF-LOG:\n", qso_line.c_str()) >= 0;
    ok = ok && sync_file(file);
    if (fclose(file) != 0) {
        ok = false;
    }
    return ok;
}

StorageStream* storage_stream_open(const std::string& name, StorageOpenMode mode) {
    StorageGuard guard;
    std::string path;
    if (!guard.held() || !firmware_owns_storage_locked() || !build_path(name, path)) {
        return nullptr;
    }

    const char* open_mode = "rb";
    if (mode == StorageOpenMode::WRITE_TRUNCATE) {
        open_mode = "wb";
    } else if (mode == StorageOpenMode::APPEND) {
        open_mode = "ab";
    }

    StorageStream* stream = new (std::nothrow) StorageStream;
    if (!stream) {
        return nullptr;
    }
    stream->file = fopen(path.c_str(), open_mode);
    if (!stream->file) {
        delete stream;
        return nullptr;
    }
    ++s_open_streams;
    return stream;
}

size_t storage_stream_read(StorageStream* stream, void* data, size_t size) {
    StorageGuard guard;
    if (!guard.held() || !firmware_owns_storage_locked() || !stream || !stream->file) {
        return 0;
    }
    return fread(data, 1, size, stream->file);
}

bool storage_stream_read_line(StorageStream* stream, char* line, size_t line_size) {
    StorageGuard guard;
    return guard.held() && firmware_owns_storage_locked() && stream && stream->file &&
           line && line_size > 0 && fgets(line, static_cast<int>(line_size), stream->file);
}

size_t storage_stream_write(StorageStream* stream, const void* data, size_t size) {
    StorageGuard guard;
    if (!guard.held() || !firmware_owns_storage_locked() || !stream || !stream->file) {
        return 0;
    }
    return fwrite(data, 1, size, stream->file);
}

bool storage_stream_seek(StorageStream* stream, long offset, int whence) {
    StorageGuard guard;
    return guard.held() && firmware_owns_storage_locked() && stream && stream->file &&
           fseek(stream->file, offset, whence) == 0;
}

long storage_stream_tell(StorageStream* stream) {
    StorageGuard guard;
    if (!guard.held() || !firmware_owns_storage_locked() || !stream || !stream->file) {
        return -1;
    }
    return ftell(stream->file);
}

long storage_stream_size(StorageStream* stream) {
    StorageGuard guard;
    if (!guard.held() || !firmware_owns_storage_locked() || !stream || !stream->file) {
        return -1;
    }
    const long current = ftell(stream->file);
    if (current < 0 || fseek(stream->file, 0, SEEK_END) != 0) {
        return -1;
    }
    const long size = ftell(stream->file);
    fseek(stream->file, current, SEEK_SET);
    return size;
}

bool storage_stream_sync(StorageStream* stream) {
    StorageGuard guard;
    return guard.held() && firmware_owns_storage_locked() && stream && stream->file &&
           sync_file(stream->file);
}

void storage_stream_close(StorageStream* stream) {
    if (!stream) {
        return;
    }
    StorageGuard guard;
    if (guard.held()) {
        if (stream->file) {
            fclose(stream->file);
            stream->file = nullptr;
        }
        if (s_open_streams > 0) {
            --s_open_streams;
        }
    }
    delete stream;
}

bool storage_sync_station_from_sd() {
    StorageGuard guard;
    if (!guard.held() || !firmware_owns_storage_locked()) {
        return false;
    }
    if (s_station_sync_attempted) {
        return true;
    }
    s_station_sync_attempted = true;

    std::string internal_path;
    struct stat st = {};
    if (build_path(kStationFile, internal_path) &&
        stat(internal_path.c_str(), &st) == 0 &&
        S_ISREG(st.st_mode) &&
        st.st_size > 0) {
        ESP_LOGI(TAG, "Internal Station.txt exists; skipping SD import");
        return true;
    }

    if (mount_sd_locked() != ESP_OK) {
        ESP_LOGI(TAG, "SD not mounted; using internal Station.txt");
        return false;
    }

    const std::string source = std::string(kSdBasePath) + "/" + kStationFile;
    FILE* file = fopen(source.c_str(), "rb");
    if (!file) {
        ESP_LOGI(TAG, "Station.txt not found on SD");
        unmount_sd_locked();
        return false;
    }

    std::string content;
    char buffer[512];
    while (true) {
        const size_t count = fread(buffer, 1, sizeof(buffer), file);
        if (count > 0) {
            content.append(buffer, count);
        }
        if (count < sizeof(buffer)) {
            break;
        }
    }
    const bool read_ok = ferror(file) == 0;
    fclose(file);
    const bool write_ok = read_ok && write_atomic_locked(kStationFile, content);
    unmount_sd_locked();

    ESP_LOGI(TAG, "%s Station.txt from SD", write_ok ? "Imported" : "Failed to import");
    return write_ok;
}

bool storage_service_flush_all() {
    StorageGuard guard;
    if (!guard.held() || !firmware_owns_storage_locked()) {
        return false;
    }
    return flush_all_locked();
}

StorageCopyResult storage_copy_all_to_sd(const std::string& priority_file,
                                         const std::string& priority_file2) {
    StorageCopyResult result {};
    StorageGuard guard;
    ESP_LOGI(COPY_TAG,
             "begin owner=%s open_streams=%u tinyusb=%d sd_mounted=%d source_base=%s sd_base=%s",
             storage_owner_name(s_owner), static_cast<unsigned>(s_open_streams),
             s_tinyusb_installed, s_sd_mounted, kBasePath, kSdBasePath);

    if (!guard.held()) {
        result.err = ESP_ERR_INVALID_STATE;
        result.status = StorageCopyStatus::STORAGE_BUSY;
        result.missed_count = 1;
        result.missed_files.push_back("<storage lock>");
        ESP_LOGE(COPY_TAG, "storage lock unavailable");
        log_copy_summary(result);
        return result;
    }

    if (!firmware_owns_storage_locked() || s_open_streams != 0) {
        result.err = ESP_ERR_INVALID_STATE;
        result.status = StorageCopyStatus::STORAGE_BUSY;
        result.missed_count = 1;
        result.missed_files.push_back("<storage busy>");
        ESP_LOGW(COPY_TAG, "copy blocked owner=%s open_streams=%u",
                 storage_owner_name(s_owner), static_cast<unsigned>(s_open_streams));
        log_copy_summary(result);
        return result;
    }

    const bool flush_ok = flush_all_locked();
    ESP_LOGI(COPY_TAG, "active log flush result=%d", flush_ok);

    std::vector<std::string> files;
    if (!list_files_locked(files)) {
        result.err = ESP_FAIL;
        result.status = StorageCopyStatus::LIST_FAILED;
        result.missed_count = 1;
        result.missed_files.push_back("<list failed>");
        ESP_LOGE(COPY_TAG, "source file list failed base=%s", kBasePath);
        log_copy_summary(result);
        return result;
    }
    std::sort(files.begin(), files.end());
    ESP_LOGI(COPY_TAG, "source file discovery count=%u",
             static_cast<unsigned>(files.size()));

    const esp_err_t mount_err = mount_sd_locked();
    ESP_LOGI(COPY_TAG, "SD mount result=%s", esp_err_to_name(mount_err));
    if (mount_err != ESP_OK) {
        result.err = mount_err;
        result.status = StorageCopyStatus::SD_MOUNT_FAILED;
        result.missed_count = std::max(1, static_cast<int>(files.size()));
        result.missed_files = files;
        if (result.missed_files.empty()) {
            result.missed_files.push_back("<sd mount>");
        }
        ESP_LOGE(COPY_TAG, "SD mount failed err=%s missed_count=%d",
                 esp_err_to_name(mount_err), result.missed_count);
        log_copy_summary(result);
        return result;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    std::vector<CopyFileRecord> records;
    records.reserve(files.size() + 2);
    for (const std::string& name : files) {
        copy_file_record_locked(records, name, "first", name == priority_file ? 6 : 5);
    }

    vTaskDelay(pdMS_TO_TICKS(120));
    copy_missing_pass_locked(records, "retry-missed", 5);

    copy_active_file_if_present_locked(records, priority_file, "final-qso", 6);
    copy_active_file_if_present_locked(records, priority_file2, "final-rt", 6);

    for (const CopyFileRecord& record : records) {
        if (record.copied) {
            ++result.copied_count;
        } else {
            ++result.missed_count;
            result.missed_files.push_back(record.name);
            ESP_LOGE(COPY_TAG,
                     "missed file=%s err=%s errno=%d (%s) stage=%s",
                     record.name.c_str(), esp_err_to_name(record.last_status.err),
                     record.last_status.err_no, strerror(record.last_status.err_no),
                     record.last_status.stage);
        }
    }

    unmount_sd_locked();
    result.err = result.missed_count == 0 ? ESP_OK : ESP_FAIL;
    result.status = result.missed_count == 0 ? StorageCopyStatus::OK
                                             : StorageCopyStatus::COPY_FAILED;
    log_copy_summary(result);
    return result;
}
