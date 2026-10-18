#include "radio_control_backend.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "kh1_tone_map.h"

static const char* TAG = "RADIO_KH1";

static constexpr uart_port_t k_kh1_uart_num = UART_NUM_1;
static constexpr gpio_num_t k_kh1_uart_tx_pin = GPIO_NUM_2;
static constexpr gpio_num_t k_kh1_uart_rx_pin = GPIO_NUM_1;
static constexpr int k_kh1_uart_baud = 9600;
static bool s_uart_inited = false;
static bool s_kh1_enabled = false;
static bool s_power_seq_done = true;

static int s_rx_freq10 = 0;
static int s_tx_freq10 = 0;
static int s_tx_base_hz = 1500;
static int s_current_fa10 = -1;
static bool s_tx_active = false;

static constexpr int k_diag_key_ms = 750;
static constexpr int k_diag_tone_ms = 160;

static int hz_to_10hz(int hz) {
    return (hz + 5) / 10;
}

static esp_err_t kh1_uart_init(void) {
    if (!s_kh1_enabled) return ESP_ERR_INVALID_STATE;
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

    err = uart_set_pin(k_kh1_uart_num, k_kh1_uart_tx_pin, k_kh1_uart_rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    // Proven setup requires inverted serial level for KH1 CAT adapter path.
    err = uart_set_line_inverse(k_kh1_uart_num, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
    if (err != ESP_OK) return err;

    gpio_set_drive_capability(k_kh1_uart_tx_pin, GPIO_DRIVE_CAP_3);
    s_uart_inited = true;
    ESP_LOGI(TAG, "KH1 UART ready: UART%d TX=G%d RX=G%d %d 8N1 inverted",
             (int)k_kh1_uart_num, (int)k_kh1_uart_tx_pin, (int)k_kh1_uart_rx_pin, k_kh1_uart_baud);
    return ESP_OK;
}

static bool kh1_ready(void) {
    if (!s_kh1_enabled) return false;
    return kh1_uart_init() == ESP_OK;
}

static esp_err_t kh1_send_cmd(const char* cmd, uint32_t timeout_ms) {
    if (!s_kh1_enabled) return ESP_ERR_INVALID_STATE;
    if (kh1_uart_init() != ESP_OK) return ESP_ERR_INVALID_STATE;
    ESP_LOGD(TAG, "CAT>%s", cmd);
    int written = uart_write_bytes(k_kh1_uart_num, cmd, strlen(cmd));
    if (written <= 0) return ESP_FAIL;
    return uart_wait_tx_done(k_kh1_uart_num, pdMS_TO_TICKS(timeout_ms));
}

static esp_err_t kh1_set_fa_if_changed(int freq10, const char* reason, bool* out_sent = nullptr) {
    if (out_sent) *out_sent = false;
    // Cache only reflects CAT changes made here; STATUS sync or reconnect
    // re-establishes state after manual KH1 VFO changes.
    if (s_current_fa10 == freq10) {
        ESP_LOGI(TAG, "KH1 FA skip %s FA=%07d", reason ? reason : "", freq10);
        return ESP_OK;
    }

    char fa[24];
    snprintf(fa, sizeof(fa), "FA%07d;", freq10);
    esp_err_t err = kh1_send_cmd(fa, 200);
    if (err == ESP_OK) {
        s_current_fa10 = freq10;
        if (out_sent) *out_sent = true;
        ESP_LOGI(TAG, "KH1 FA sent %s FA=%07d", reason ? reason : "", freq10);
    }
    return err;
}

static esp_err_t kh1_send_fa_forced(int freq10, const char* reason) {
    char fa[24];
    snprintf(fa, sizeof(fa), "FA%07d;", freq10);
    esp_err_t err = kh1_send_cmd(fa, 200);
    if (err == ESP_OK) {
        s_current_fa10 = freq10;
        ESP_LOGI(TAG, "KH1 FA forced %s FA=%07d", reason ? reason : "", freq10);
    }
    return err;
}

static esp_err_t kh1_sync_frequency_mode(int freq_hz) {
    s_rx_freq10 = hz_to_10hz(freq_hz);

    esp_err_t err = kh1_send_cmd("MD2;", 200);
    if (err != ESP_OK) return err;
    kh1_send_cmd("MNPWR;", 200);
    kh1_send_cmd("MP020;", 200);
    kh1_send_cmd("SW4T;", 200);

    err = kh1_set_fa_if_changed(s_rx_freq10, "sync");
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "KH1 sync ok band_freq=%dHz (FA=%07d)", freq_hz, s_rx_freq10);
    }
    return err;
}

static esp_err_t kh1_begin_tx(int freq_hz, int tx_base_hz) {
    s_rx_freq10 = hz_to_10hz(freq_hz);
    s_tx_base_hz = tx_base_hz;
    s_tx_freq10 = s_rx_freq10 + hz_to_10hz(tx_base_hz);

    esp_err_t err = kh1_set_fa_if_changed(s_tx_freq10, "tx");
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

    int fo = kh1_fo_from_tone_hz(tone_hz, (float)s_tx_base_hz);
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "FO%02d;", fo);
    return kh1_send_cmd(cmd, 10);
}

static esp_err_t kh1_end_tx(void) {
    if (!s_tx_active) return ESP_OK;

    esp_err_t err = kh1_send_cmd("HK0;", 200);
    if (err != ESP_OK) return err;

    s_tx_active = false;

    // RX decode expects KH1 back on the band dial frequency after every TX.
    err = kh1_send_fa_forced(s_rx_freq10, "tx-rx");
    esp_err_t fo_err = kh1_send_cmd("FO99;", 200);
    if (err != ESP_OK) {
        if (fo_err != ESP_OK) {
            ESP_LOGW(TAG, "KH1 TX stop restore FA failed and FO99 also failed: FA=%s FO=%s",
                     esp_err_to_name(err), esp_err_to_name(fo_err));
        }
        return err;
    }
    if (fo_err != ESP_OK) {
        return fo_err;
    }

    ESP_LOGI(TAG, "KH1 TX stop restore FA=%07d", s_rx_freq10);
    return ESP_OK;
}

static esp_err_t kh1_set_tune(bool enable, int freq_hz, int tone_hz) {
    if (!enable) {
        return kh1_end_tx();
    }

    s_rx_freq10 = hz_to_10hz(freq_hz);
    s_tx_freq10 = s_rx_freq10 + hz_to_10hz(tone_hz);

    // Tune must always send one FA so KH1/ATU can react before key-down.
    // Unlike FT8 TX, do not cache-skip this command.
    char fa[24];
    snprintf(fa, sizeof(fa), "FA%07d;", s_tx_freq10);
    esp_err_t err = kh1_send_cmd(fa, 200);
    if (err != ESP_OK) return err;
    s_current_fa10 = s_tx_freq10;

    err = kh1_send_cmd("HK1;", 200);
    if (err == ESP_OK) {
        s_tx_active = true;
        ESP_LOGI(TAG, "KH1 tune key down FA=%07d", s_tx_freq10);
    }
    return err;
}

static esp_err_t kh1_diag_send_fa(int freq10, const char* reason, bool* out_sent = nullptr) {
    return kh1_set_fa_if_changed(freq10, reason, out_sent);
}

static esp_err_t kh1_diag_send_fo_index(int tone_idx, uint32_t timeout_ms) {
    if (tone_idx < 0) tone_idx = 0;
    if (tone_idx > 7) tone_idx = 7;
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "FO%02d;", kh1_fo_from_delta_hz(6.25f * (float)tone_idx));
    return kh1_send_cmd(cmd, timeout_ms);
}

esp_err_t radio_control_kh1_diag_test(char test_key, int freq_hz, int offset_hz, bool* out_fa_sent) {
    if (test_key >= 'A' && test_key <= 'Z') {
        test_key = (char)(test_key - 'A' + 'a');
    }
    if (out_fa_sent) *out_fa_sent = false;
    if (!kh1_ready()) return ESP_ERR_INVALID_STATE;

    const int rx_freq10 = hz_to_10hz(freq_hz);
    const int tx_freq10 = rx_freq10 + hz_to_10hz(offset_hz);
    esp_err_t err = ESP_OK;

    switch (test_key) {
    case 'u':
        ESP_LOGI(TAG, "KH1 diag u: FA%07d if changed; wait %dms; repeat;",
                 tx_freq10, k_diag_key_ms);
        err = kh1_diag_send_fa(tx_freq10, "diag-u", out_fa_sent);
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(k_diag_key_ms));
        return kh1_diag_send_fa(tx_freq10, "diag-u-repeat");

    case 'i':
        ESP_LOGI(TAG, "KH1 diag i: HK1; wait %dms; HK0;", k_diag_key_ms);
        err = kh1_send_cmd("HK1;", 200);
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(k_diag_key_ms));
        return kh1_send_cmd("HK0;", 200);

    case 'j':
        ESP_LOGI(TAG, "KH1 diag j: FA%07d if changed; HK1; wait %dms; HK0; FA%07d if changed;",
                 tx_freq10, k_diag_key_ms, rx_freq10);
        err = kh1_diag_send_fa(tx_freq10, "diag-j-tx");
        if (err != ESP_OK) return err;
        err = kh1_send_cmd("HK1;", 200);
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(k_diag_key_ms));
        err = kh1_send_cmd("HK0;", 200);
        if (err != ESP_OK) return err;
        return kh1_diag_send_fa(rx_freq10, "diag-j-rx");

    case 'k':
        ESP_LOGI(TAG, "KH1 diag k: FA%07d if changed; HK1; FO00; wait %dms; HK0; FA%07d if changed; FO99;",
                 tx_freq10, k_diag_key_ms, rx_freq10);
        err = kh1_diag_send_fa(tx_freq10, "diag-k-tx");
        if (err != ESP_OK) return err;
        err = kh1_send_cmd("HK1;", 200);
        if (err != ESP_OK) return err;
        err = kh1_send_cmd("FO00;", 200);
        if (err != ESP_OK) {
            kh1_send_cmd("HK0;", 200);
            kh1_diag_send_fa(rx_freq10, "diag-k-rx");
            kh1_send_cmd("FO99;", 200);
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(k_diag_key_ms));
        err = kh1_send_cmd("HK0;", 200);
        if (err != ESP_OK) return err;
        err = kh1_diag_send_fa(rx_freq10, "diag-k-rx");
        if (err != ESP_OK) return err;
        return kh1_send_cmd("FO99;", 200);

    case 'l':
        ESP_LOGI(TAG, "KH1 diag l: FA%07d if changed; HK1; 79 timed FOxx; HK0; FA%07d if changed; FO99;",
                 tx_freq10, rx_freq10);
        err = kh1_diag_send_fa(tx_freq10, "diag-l-tx");
        if (err != ESP_OK) return err;
        err = kh1_send_cmd("HK1;", 200);
        if (err != ESP_OK) return err;
        for (int i = 0; i < 79; ++i) {
            err = kh1_diag_send_fo_index(i & 7, 10);
            if (err != ESP_OK) break;
            vTaskDelay(pdMS_TO_TICKS(k_diag_tone_ms));
        }
        if (err != ESP_OK) {
            kh1_send_cmd("HK0;", 200);
            kh1_diag_send_fa(rx_freq10, "diag-l-rx");
            kh1_send_cmd("FO99;", 200);
            return err;
        }
        err = kh1_send_cmd("HK0;", 200);
        if (err != ESP_OK) return err;
        err = kh1_diag_send_fa(rx_freq10, "diag-l-rx");
        if (err != ESP_OK) return err;
        return kh1_send_cmd("FO99;", 200);

    default:
        return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t kh1_on_audio_start(void) {
    if (!kh1_ready()) return ESP_ERR_INVALID_STATE;

    esp_err_t err = kh1_send_cmd("MD2;", 200);
    if (err != ESP_OK) return err;
    
    if (!s_power_seq_done) {
        
        for (int i = 0; i < 25; ++i) {
            err = kh1_send_cmd("ENAD;", 200);
            if (err != ESP_OK) return err;
        }
        for (int i = 0; i < 5; ++i) {
            err = kh1_send_cmd("ENAU;", 200);
            if (err != ESP_OK) return err;
        }
        s_power_seq_done = true;
    }
    
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
    .start_tx_waveform = nullptr,
};

const radio_control_ops_t* radio_control_kh1_get_ops(void) {
    return &k_ops;
}

void radio_control_kh1_set_enabled(bool enabled) {
    if (s_kh1_enabled == enabled) return;
    s_kh1_enabled = enabled;
    if (!enabled) {
        s_tx_active = false;
        s_current_fa10 = -1;
        if (s_uart_inited) {
            uart_wait_tx_done(k_kh1_uart_num, pdMS_TO_TICKS(50));
            uart_flush_input(k_kh1_uart_num);
            uart_driver_delete(k_kh1_uart_num);
            s_uart_inited = false;
        }
    }
    ESP_LOGI(TAG, "KH1 CAT %s", enabled ? "enabled" : "disabled");
}

bool radio_control_kh1_is_enabled(void) {
    return s_kh1_enabled;
}
