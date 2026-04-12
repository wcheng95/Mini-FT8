#include "gps.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <cmath>
#include <vector>

static const char* TAG = "GPS";
static gps_state_t s_gps_state = {};
static SemaphoreHandle_t s_gps_mutex = NULL;

gps_state_t gps_get_state() {
    gps_state_t snapshot = {};
    if (s_gps_mutex) {
        xSemaphoreTake(s_gps_mutex, portMAX_DELAY);
        snapshot = s_gps_state;
        xSemaphoreGive(s_gps_mutex);
    }
    return snapshot;
}

static std::string lat_lon_to_grid(double lat, double lon) {
    if (lon < -180.0 || lon > 180.0 || lat < -90.0 || lat > 90.0) return "";
    lon += 180.0;
    lat += 90.0;
    std::string grid = "    ";
    grid[0] = 'A' + (int)(lon / 20.0);
    grid[1] = 'A' + (int)(lat / 10.0);
    lon = std::fmod(lon, 20.0);
    lat = std::fmod(lat, 10.0);
    grid[2] = '0' + (int)(lon / 2.0);
    grid[3] = '0' + (int)(lat / 1.0);
    return grid;
}

static std::vector<std::string> split_csv(const std::string& s) {
    std::vector<std::string> tokens;
    size_t start = 0, end = 0;
    while ((end = s.find(',', start)) != std::string::npos) {
        tokens.push_back(s.substr(start, end - start));
        start = end + 1;
    }
    tokens.push_back(s.substr(start));
    return tokens;
}

static double parse_nmea_coord(const std::string& val_str, const std::string& dir_str) {
    if (val_str.empty() || dir_str.empty()) return 0.0;
    double val = atof(val_str.c_str());
    int degrees = (int)(val / 100);
    double minutes = val - (degrees * 100);
    double result = degrees + (minutes / 60.0);
    if (dir_str[0] == 'S' || dir_str[0] == 'W') {
        result = -result;
    }
    return result;
}

static void gps_task(void* arg) {
    uint8_t data[256];
    std::string line_buffer;

    while (1) {
        int len = uart_read_bytes(UART_NUM_1, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = data[i];
                if (c == '\n' || c == '\r') {
                    if (!line_buffer.empty()) {
                        // NMEA Sentence found (starts with $)
                        if (line_buffer[0] == '$') {
                            xSemaphoreTake(s_gps_mutex, portMAX_DELAY);
                            s_gps_state.last_rx_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                            
                            std::vector<std::string> parts = split_csv(line_buffer);
                            
                            // Check RMC for Time, Date, and Location
                            if (parts[0].find("RMC") != std::string::npos && parts.size() >= 10) {
                                if (parts[2] == "A") {
                                    s_gps_state.valid_fix = true;
                                    s_gps_state.latitude = parse_nmea_coord(parts[3], parts[4]);
                                    s_gps_state.longitude = parse_nmea_coord(parts[5], parts[6]);
                                    s_gps_state.grid_square = lat_lon_to_grid(s_gps_state.latitude, s_gps_state.longitude);
                                    
                                    if (parts[1].length() >= 6) {
                                        s_gps_state.time_utc = parts[1].substr(0,2) + ":" + parts[1].substr(2,2) + ":" + parts[1].substr(4,2);
                                    }
                                    if (parts[9].length() >= 6) {
                                        s_gps_state.date_utc = "20" + parts[9].substr(4,2) + "-" + parts[9].substr(2,2) + "-" + parts[9].substr(0,2);
                                    }
                                } else {
                                    s_gps_state.valid_fix = false;
                                }
                            } 
                            // Check GGA for Satellites
                            else if (parts[0].find("GGA") != std::string::npos && parts.size() >= 8) {
                                if (!parts[7].empty()) {
                                    s_gps_state.satellites = atoi(parts[7].c_str());
                                }
                            }
                            xSemaphoreGive(s_gps_mutex);
                        }
                        line_buffer.clear();
                    }
                } else {
                    line_buffer += c;
                    if (line_buffer.size() > 120) line_buffer.clear(); // Safety line length cap
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void gps_init() {
    s_gps_mutex = xSemaphoreCreateMutex();
    
    uart_config_t uart_config = {};
    uart_config.baud_rate = 9600;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity    = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    
    ESP_LOGI(TAG, "Initializing GPS UART1 on Port A");
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    
    // M5Cardputer Port A (Grove Connector)
    // GPIO 1 -> RX (GT-U7 TX)
    // GPIO 2 -> TX (GT-U7 RX - Optional)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 2, 1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Spin up background parser
    xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 5, NULL, 1);
}
