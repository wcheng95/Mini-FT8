#include "radio_control_backend.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char* TAG = "RADIO_KH1";

static constexpr uart_port_t k_kh1_uart_num = UART_NUM_1;
static constexpr gpio_num_t k_kh1_uart_tx_pin = GPIO_NUM_1;
static constexpr int k_kh1_uart_baud = 9600;
static bool s_uart_inited = false;

static int s_rx_freq10 = 0;
static int s_tx_freq10 = 0;
static int s_tx_base_hz = 1500;
static bool s_tx_active = false;

static constexpr int k_fo_map[8] = {0, 6, 13, 19, 25, 31, 38, 44};

static int hz_to_10hz(int hz) {
    return (hz + 5) / 10;
}

static esp_err_t kh1_uart_init(void) {
    if (s_uart_inited) return ESP_OK;

    uart_config_t cfg = {};
    cfg.baud_rate = k_kh1_uart_baud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#ifdef UART_SCLK_REF_TICK
    cfg.source_clk = UART_SCLK_REF_TICK;
#else
    cfg.source_clk = UART_SCLK_DEFAULT;
#endif

    esp_err_t err = uart_driver_install(k_kh1_uart_num, 512, 0, 0, nullptr, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    err = uart_param_config(k_kh1_uart_num, &cfg);
    if (err != ESP_OK) return err;

    err = uart_set_pin(k_kh1_uart_num, k_kh1_uart_tx_pin, UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    // Proven setup requires inverted serial level for KH1 CAT adapter path.
    err = uart_set_line_inverse(k_kh1_uart_num, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
    if (err != ESP_OK) return err;

    gpio_set_drive_capability(k_kh1_uart_tx_pin, GPIO_DRIVE_CAP_3);
    s_uart_inited = true;
    ESP_LOGI(TAG, "KH1 UART ready: UART%d TX=G%d %d 8N1 inverted",
             (int)k_kh1_uart_num, (int)k_kh1_uart_tx_pin, k_kh1_uart_baud);
    return ESP_OK;
}

static bool kh1_ready(void) {
    return kh1_uart_init() == ESP_OK;
}

static esp_err_t kh1_send_cmd(const char* cmd, uint32_t timeout_ms) {
    if (kh1_uart_init() != ESP_OK) return ESP_ERR_INVALID_STATE;
    ESP_LOGD(TAG, "CAT>%s", cmd);
    int written = uart_write_bytes(k_kh1_uart_num, cmd, strlen(cmd));
    if (written <= 0) return ESP_FAIL;
    return uart_wait_tx_done(k_kh1_uart_num, pdMS_TO_TICKS(timeout_ms));
}

static esp_err_t kh1_sync_frequency_mode(int freq_hz) {
    s_rx_freq10 = hz_to_10hz(freq_hz);

    esp_err_t err = kh1_send_cmd("MD2;", 200);
    if (err != ESP_OK) return err;

    char fa[24];
    snprintf(fa, sizeof(fa), "FA%07d;", s_rx_freq10);
    err = kh1_send_cmd(fa, 200);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "KH1 sync ok band_freq=%dHz (FA=%07d)", freq_hz, s_rx_freq10);
    }
    return err;
}

static esp_err_t kh1_begin_tx(int freq_hz, int tx_base_hz) {
    s_rx_freq10 = hz_to_10hz(freq_hz);
    s_tx_base_hz = tx_base_hz;
    s_tx_freq10 = s_rx_freq10 + hz_to_10hz(tx_base_hz);

    esp_err_t err = kh1_send_cmd("MD2;", 200);
    if (err != ESP_OK) return err;

    char fa[24];
    snprintf(fa, sizeof(fa), "FA%07d;", s_tx_freq10);
    err = kh1_send_cmd(fa, 200);
    if (err != ESP_OK) return err;

    err = kh1_send_cmd("HK1;", 200);
    if (err == ESP_OK) {
        s_tx_active = true;
        ESP_LOGI(TAG, "KH1 TX start band=%dHz base=%dHz txFA=%07d",
                 freq_hz, tx_base_hz, s_tx_freq10);
    }
    return err;
}

static esp_err_t kh1_set_tone_hz(float tone_hz) {
    if (!s_tx_active) return ESP_ERR_INVALID_STATE;

    int tone_idx = (int)lrintf((tone_hz - (float)s_tx_base_hz) / 6.25f);
    if (tone_idx < 0) tone_idx = 0;
    if (tone_idx > 7) tone_idx = 7;

    int fo = k_fo_map[tone_idx];
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "FO%02d;", fo);
    return kh1_send_cmd(cmd, 10);
}

static esp_err_t kh1_end_tx(void) {
    if (!s_tx_active) return ESP_OK;

    esp_err_t err = kh1_send_cmd("HK0;", 200);
    if (err != ESP_OK) return err;

    char fa[24];
    snprintf(fa, sizeof(fa), "FA%07d;", s_rx_freq10);
    err = kh1_send_cmd(fa, 200);
    if (err != ESP_OK) return err;

    err = kh1_send_cmd("FO99;", 200);
    if (err == ESP_OK) {
        s_tx_active = false;
        ESP_LOGI(TAG, "KH1 TX stop restore FA=%07d", s_rx_freq10);
    }
    return err;
}

static esp_err_t kh1_set_tune(bool enable, int freq_hz, int tone_hz) {
    if (!enable) {
        return kh1_end_tx();
    }

    s_rx_freq10 = hz_to_10hz(freq_hz);
    s_tx_freq10 = s_rx_freq10 + hz_to_10hz(tone_hz);

    esp_err_t err = kh1_send_cmd("MD2;", 200);
    if (err != ESP_OK) return err;
    char fa[24];
    snprintf(fa, sizeof(fa), "FA%07d;", s_tx_freq10);
    err = kh1_send_cmd(fa, 200);
    if (err != ESP_OK) return err;
    err = kh1_send_cmd("HK1;", 200);
    if (err != ESP_OK) return err;
    s_tx_active = true;

    return kh1_send_cmd("FO00;", 200);
}

static esp_err_t kh1_on_audio_start(void) {
    if (!kh1_ready()) return ESP_ERR_INVALID_STATE;

    esp_err_t err = kh1_send_cmd("MD2;", 200);
    if (err != ESP_OK) return err;
    kh1_send_cmd("MNPWR;", 200);
    kh1_send_cmd("MP020;", 200);
    kh1_send_cmd("SW4T;", 200);
    ESP_LOGI(TAG, "KH1 CAT backend initialized with proven command set");
    return ESP_OK;
}

static const radio_control_ops_t k_ops = {
    .name = "kh1_cat",
    .ready = kh1_ready,
    .on_audio_start = kh1_on_audio_start,
    .sync_frequency_mode = kh1_sync_frequency_mode,
    .begin_tx = kh1_begin_tx,
    .set_tone_hz = kh1_set_tone_hz,
    .end_tx = kh1_end_tx,
    .set_tune = kh1_set_tune,
    .set_time = nullptr,
};

const radio_control_ops_t* radio_control_kh1_get_ops(void) {
    return &k_ops;
}
