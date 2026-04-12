#include "radio_control_backend.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "esp_log.h"

#include "stream_uac.h"

static const char* TAG = "RADIO_QMX";

static bool qmx_ready(void) {
    return cat_cdc_ready();
}

static esp_err_t qmx_send_cmd(const char* cmd, uint32_t timeout_ms) {
    if (!cat_cdc_ready()) return ESP_ERR_INVALID_STATE;
    return cat_cdc_send(reinterpret_cast<const uint8_t*>(cmd), strlen(cmd), timeout_ms);
}

static esp_err_t qmx_sync_frequency_mode(int freq_hz) {
    char fa[32];
    snprintf(fa, sizeof(fa), "FA%011d;", freq_hz);

    esp_err_t err = qmx_send_cmd(fa, 200);
    if (err != ESP_OK) return err;

    const char* md = "MD6;";
    err = qmx_send_cmd(md, 200);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "QMX sync ok freq=%d", freq_hz);
    }
    return err;
}

static esp_err_t qmx_begin_tx(int freq_hz, int tx_base_hz) {
    (void)freq_hz;
    (void)tx_base_hz;

    const char* md = "MD6;";
    esp_err_t err = qmx_send_cmd(md, 200);
    if (err != ESP_OK) return err;

    const char* tx = "TX;";
    err = qmx_send_cmd(tx, 200);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "QMX TX start");
    }
    return err;
}

static esp_err_t qmx_set_tone_hz(float tone_hz) {
    int ta_int = (int)lrintf(tone_hz);
    float frac = tone_hz - (float)ta_int;
    int ta_frac = (int)lrintf(frac * 100.0f);

    char ta[16];
    snprintf(ta, sizeof(ta), "TA%04d.%02d;", ta_int, ta_frac);
    return qmx_send_cmd(ta, 10);
}

static esp_err_t qmx_end_tx(void) {
    const char* rx = "RX;";
    esp_err_t err = qmx_send_cmd(rx, 200);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "QMX TX stop");
    }
    return err;
}

static esp_err_t qmx_set_tune(bool enable, int freq_hz, int tone_hz) {
    if (!enable) {
        return qmx_end_tx();
    }

    esp_err_t err = qmx_sync_frequency_mode(freq_hz);
    if (err != ESP_OK) return err;

    const char* tx = "TX;";
    err = qmx_send_cmd(tx, 200);
    if (err != ESP_OK) return err;

    return qmx_set_tone_hz((float)tone_hz);
}

static esp_err_t qmx_on_audio_start(void) {
    ESP_LOGI(TAG, "QMX CAT backend initialized");
    return ESP_OK;
}

static esp_err_t qmx_set_time(int hour, int minute, int second) {
    char tm[32];
    snprintf(tm, sizeof(tm), "TM%02d%02d%02d;", hour, minute, second);
    esp_err_t err = qmx_send_cmd(tm, 200);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "QMX CAT time set: %02d:%02d:%02d", hour, minute, second);
    }
    return err;
}

static const radio_control_ops_t k_ops = {
    .name = "qmx",
    .ready = qmx_ready,
    .on_audio_start = qmx_on_audio_start,
    .sync_frequency_mode = qmx_sync_frequency_mode,
    .begin_tx = qmx_begin_tx,
    .set_tone_hz = qmx_set_tone_hz,
    .end_tx = qmx_end_tx,
    .set_tune = qmx_set_tune,
    .set_time = qmx_set_time,
};

const radio_control_ops_t* radio_control_qmx_get_ops(void) {
    return &k_ops;
}
