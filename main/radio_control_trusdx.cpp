// (tr)uSDX (DL2MAN) — Kenwood TS-480 subset CAT with in-band audio
// streaming. All I/O goes through stream_trusdx; keying (";TX0;") is done
// when the waveform starts so samples follow the command immediately.
#include "radio_control_backend.h"

#include <cstdio>

#include "esp_log.h"
#include "stream_trusdx.h"

static const char* TAG = "RADIO_TRUSDX";

static bool trusdx_ready(void) {
    return trusdx_cat_ready();
}

static esp_err_t trusdx_sync_frequency_mode(int freq_hz) {
    esp_err_t err = trusdx_cat_send("MD2;", 200);
    if (err != ESP_OK) return err;
    char command[32];
    snprintf(command, sizeof(command), "FA%011d;", freq_hz);
    err = trusdx_cat_send(command, 200);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "truSDX sync ok freq=%d", freq_hz);
    }
    return err;
}

static esp_err_t trusdx_begin_tx(int freq_hz, int tx_base_hz) {
    (void)freq_hz;
    (void)tx_base_hz;
    return trusdx_cat_ready() ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static esp_err_t trusdx_set_tone_hz(float tone_hz) {
    (void)tone_hz;  // tones are synthesized into the CAT audio stream
    return ESP_OK;
}

static esp_err_t trusdx_end_tx(void) {
    return trusdx_tx_end();
}

static esp_err_t trusdx_set_tune(bool enable, int freq_hz, int tone_hz) {
    if (!enable) {
        return trusdx_tx_end();
    }
    esp_err_t err = trusdx_sync_frequency_mode(freq_hz);
    if (err != ESP_OK) return err;
    return trusdx_tx_begin_tune(static_cast<float>(tone_hz)) ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static esp_err_t trusdx_on_audio_start(void) {
    ESP_LOGI(TAG, "truSDX CAT-stream backend initialized");
    return ESP_OK;
}

static esp_err_t trusdx_start_tx_waveform(float base_hz,
                                          const uint8_t* symbols,
                                          size_t symbol_count,
                                          float tone_spacing_hz,
                                          float symbol_period_s,
                                          uint32_t samples_per_symbol_48k) {
    (void)samples_per_symbol_48k;  // stream runs at 11520 Hz, not 48 kHz
    return trusdx_tx_begin_cpfsk(base_hz, symbols, symbol_count, tone_spacing_hz,
                                 symbol_period_s)
               ? ESP_OK
               : ESP_ERR_INVALID_STATE;
}

static const radio_control_ops_t k_ops = {
    .name = "trusdx",
    .ready = trusdx_ready,
    .on_audio_start = trusdx_on_audio_start,
    .sync_frequency_mode = trusdx_sync_frequency_mode,
    .begin_tx = trusdx_begin_tx,
    .set_tone_hz = trusdx_set_tone_hz,
    .end_tx = trusdx_end_tx,
    .set_tune = trusdx_set_tune,
    .set_time = nullptr,
    .start_tx_waveform = trusdx_start_tx_waveform,
};

const radio_control_ops_t* radio_control_trusdx_get_ops(void) {
    return &k_ops;
}
