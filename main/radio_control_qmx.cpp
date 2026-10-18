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
    const char* md = "MD6;";
    esp_err_t err = qmx_send_cmd(md, 200);
    if (err != ESP_OK) return err;

    err = qmx_send_cmd("FR0;", 200);
    if (err != ESP_OK) return err;

    err = qmx_send_cmd("FT0;", 200);
    if (err != ESP_OK) return err;

    char fa[32];
    snprintf(fa, sizeof(fa), "FA%011d;", freq_hz);
    err = qmx_send_cmd(fa, 200);
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
    // Three correctness fixes for the QMX TA command, all producing invalid
    // commands the radio silently rejects:
    //   1. Use floorf (not lrintf) for ta_int so frac is always in [0.0, 1.0).
    //      lrintf rounds to nearest; 1520.83 -> ta_int=1521, frac=-0.17, which
    //      formats as "TA1521.-17;" — invalid.
    //   2. Clamp ta_frac to 99. lrintf(0.995 * 100) rounds up to 100 which
    //      overflows %02d producing "TA1234.100;" — invalid. FT4's 20.8333 Hz
    //      tone spacing makes this triggerable in normal operation.
    //   3. Clamp ta_int to [0, 9999], the QMX TA command's documented range.
    //      Defensive against any upstream bug producing an out-of-range tone,
    //      and gives the compiler a tight upper bound so %04d is provably
    //      4 chars and a 16-byte buffer suffices ("TA9999.99;\0" = 11 bytes).
    int ta_int = (int)floorf(tone_hz);
    if (ta_int < 0) ta_int = 0;
    if (ta_int > 9999) ta_int = 9999;
    float frac = tone_hz - (float)ta_int;     // always in [0.0, 1.0)
    int ta_frac = (int)lrintf(frac * 100.0f); // 0..100 before clamp
    if (ta_frac > 99) ta_frac = 99;           // clamp: %02d is 2 digits max

    char ta[32];
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
    .start_tx_waveform = nullptr,
};

const radio_control_ops_t* radio_control_qmx_get_ops(void) {
    return &k_ops;
}
