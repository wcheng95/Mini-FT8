#include "radio_control_backend.h"

#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "stream_uac.h"

static const char* TAG = "RADIO_QDX";

static bool qdx_ready(void) {
    return cat_cdc_ready();
}

static esp_err_t qdx_send_cmd(const char* command, uint32_t timeout_ms) {
    if (!cat_cdc_ready()) return ESP_ERR_INVALID_STATE;
    return cat_cdc_send(reinterpret_cast<const uint8_t*>(command),
                        strlen(command), timeout_ms);
}

static esp_err_t qdx_sync_frequency_mode(int freq_hz) {
    esp_err_t err = qdx_send_cmd("MD6;", 200);
    if (err != ESP_OK) return err;
    err = qdx_send_cmd("FR0;", 200);
    if (err != ESP_OK) return err;
    err = qdx_send_cmd("FT0;", 200);
    if (err != ESP_OK) return err;

    char command[32];
    snprintf(command, sizeof(command), "FA%011d;", freq_hz);
    err = qdx_send_cmd(command, 200);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "QDX sync ok freq=%d", freq_hz);
    }
    return err;
}

static esp_err_t qdx_begin_tx(int freq_hz, int tx_base_hz) {
    (void)freq_hz;
    (void)tx_base_hz;

    esp_err_t err = qdx_send_cmd("MD6;", 200);
    if (err != ESP_OK) return err;
    err = qdx_send_cmd("TX;", 200);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "QDX TX start");
    }
    return err;
}

static esp_err_t qdx_set_tone_hz(float tone_hz) {
    (void)tone_hz;
    return ESP_OK;
}

static esp_err_t qdx_end_tx(void) {
    uac_tx_end();
    esp_err_t err = qdx_send_cmd("RX;", 200);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "QDX TX stop");
    }
    return err;
}

static esp_err_t qdx_set_tune(bool enable, int freq_hz, int tone_hz) {
    if (!enable) {
        return qdx_end_tx();
    }

    esp_err_t err = qdx_sync_frequency_mode(freq_hz);
    if (err != ESP_OK) return err;
    err = qdx_send_cmd("TX;", 200);
    if (err != ESP_OK) return err;
    if (!uac_tx_begin_tune(static_cast<float>(tone_hz))) {
        qdx_send_cmd("RX;", 200);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static esp_err_t qdx_on_audio_start(void) {
    ESP_LOGI(TAG, "QDX CAT/UAC backend initialized");
    return ESP_OK;
}

static esp_err_t qdx_start_tx_waveform(float base_hz,
                                       const uint8_t* symbols,
                                       size_t symbol_count,
                                       float tone_spacing_hz,
                                       float symbol_period_s,
                                       uint32_t samples_per_symbol_48k) {
    (void)symbol_period_s;
    return uac_tx_begin_cpfsk(base_hz, symbols, symbol_count, tone_spacing_hz,
                              samples_per_symbol_48k)
               ? ESP_OK
               : ESP_ERR_INVALID_STATE;
}

static esp_err_t qdx_set_time(int hour, int minute, int second) {
    char command[32];
    snprintf(command, sizeof(command), "TM%02d%02d%02d;",
             hour, minute, second);
    return qdx_send_cmd(command, 200);
}

static const radio_control_ops_t k_ops = {
    .name = "qdx",
    .ready = qdx_ready,
    .on_audio_start = qdx_on_audio_start,
    .sync_frequency_mode = qdx_sync_frequency_mode,
    .begin_tx = qdx_begin_tx,
    .set_tone_hz = qdx_set_tone_hz,
    .end_tx = qdx_end_tx,
    .set_tune = qdx_set_tune,
    .set_time = qdx_set_time,
    .start_tx_waveform = qdx_start_tx_waveform,
};

const radio_control_ops_t* radio_control_qdx_get_ops(void) {
    return &k_ops;
}
