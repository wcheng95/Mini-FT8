#include "radio_control.h"

#include "radio_control_backend.h"

#include "esp_log.h"

static const char* TAG = "RADIO_CTRL";

static radio_control_backend_t s_backend = RADIO_CONTROL_QMX;

static const radio_control_ops_t* current_ops(void) {
    switch (s_backend) {
    case RADIO_CONTROL_KH1_CAT:
        return radio_control_kh1_get_ops();
    case RADIO_CONTROL_QDX:
        return radio_control_qdx_get_ops();
    case RADIO_CONTROL_FX4CR:
        return radio_control_fx4cr_get_ops();
    case RADIO_CONTROL_TRUSDX:
        return radio_control_trusdx_get_ops();
    case RADIO_CONTROL_QMX:
    default:
        return radio_control_qmx_get_ops();
    }
}

void radio_control_set_backend(radio_control_backend_t backend) {
    s_backend = backend;
    const radio_control_ops_t* ops = current_ops();
    ESP_LOGI(TAG, "Selected radio control backend=%s", ops ? ops->name : "unknown");
}

radio_control_backend_t radio_control_get_backend(void) {
    return s_backend;
}

const char* radio_control_backend_name(radio_control_backend_t backend) {
    switch (backend) {
    case RADIO_CONTROL_QMX:
        return "qmx";
    case RADIO_CONTROL_KH1_CAT:
        return "kh1_cat";
    case RADIO_CONTROL_QDX:
        return "qdx";
    case RADIO_CONTROL_FX4CR:
        return "fx4cr";
    case RADIO_CONTROL_TRUSDX:
        return "trusdx";
    default:
        return "unknown";
    }
}

bool radio_control_ready(void) {
    const radio_control_ops_t* ops = current_ops();
    if (!ops || !ops->ready) return false;
    return ops->ready();
}

esp_err_t radio_control_on_audio_start(void) {
    const radio_control_ops_t* ops = current_ops();
    if (!ops || !ops->on_audio_start) return ESP_ERR_INVALID_STATE;
    return ops->on_audio_start();
}

esp_err_t radio_control_sync_frequency_mode(int freq_hz) {
    const radio_control_ops_t* ops = current_ops();
    if (!ops || !ops->sync_frequency_mode) return ESP_ERR_INVALID_STATE;
    return ops->sync_frequency_mode(freq_hz);
}

esp_err_t radio_control_begin_tx(int freq_hz, int tx_base_hz) {
    const radio_control_ops_t* ops = current_ops();
    if (!ops || !ops->begin_tx) return ESP_ERR_INVALID_STATE;
    return ops->begin_tx(freq_hz, tx_base_hz);
}

esp_err_t radio_control_set_tone_hz(float tone_hz) {
    const radio_control_ops_t* ops = current_ops();
    if (!ops || !ops->set_tone_hz) return ESP_ERR_INVALID_STATE;
    return ops->set_tone_hz(tone_hz);
}

esp_err_t radio_control_end_tx(void) {
    const radio_control_ops_t* ops = current_ops();
    if (!ops || !ops->end_tx) return ESP_ERR_INVALID_STATE;
    return ops->end_tx();
}

esp_err_t radio_control_set_tune(bool enable, int freq_hz, int tone_hz) {
    const radio_control_ops_t* ops = current_ops();
    if (!ops || !ops->set_tune) return ESP_ERR_INVALID_STATE;
    return ops->set_tune(enable, freq_hz, tone_hz);
}

esp_err_t radio_control_set_time(int hour, int minute, int second) {
    const radio_control_ops_t* ops = current_ops();
    if (!ops || !ops->set_time) return ESP_ERR_NOT_SUPPORTED;
    return ops->set_time(hour, minute, second);
}

bool radio_control_uses_tx_waveform(void) {
    const radio_control_ops_t* ops = current_ops();
    return ops && ops->start_tx_waveform;
}

esp_err_t radio_control_start_tx_waveform(float base_hz,
                                          const uint8_t* symbols,
                                          size_t symbol_count,
                                          float tone_spacing_hz,
                                          float symbol_period_s,
                                          uint32_t samples_per_symbol_48k) {
    const radio_control_ops_t* ops = current_ops();
    if (!ops || !ops->start_tx_waveform) return ESP_ERR_NOT_SUPPORTED;
    return ops->start_tx_waveform(base_hz, symbols, symbol_count, tone_spacing_hz,
                                  symbol_period_s, samples_per_symbol_48k);
}
