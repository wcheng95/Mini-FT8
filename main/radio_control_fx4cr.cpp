// FX-4CR (BG2FX) with F5BUD V2 firmware.
//
// USB: internal hub -> C-Media UAC1 codec (RX + TX audio) and a CH340
// USB-serial bridge for CAT. CAT emulates a Kenwood TS-590S(G) at 115200 8N1.
// TX audio is only taken from USB in DATA-L/DATA-U, so the radio menu
// "Digital mode (CAT)" must be DATA (the default) for MD2 to select DATA-U.
//
// The ESP32-S3 USB host has 8 channels and this radio leaves only one free
// (see stream_uac.cpp). CAT is therefore sent in bursts (uac_cat_batch: mic
// paused, bridge opened, sent, closed) and never overlaps the TX audio:
//   TX:  [TX;] (mic stays paused) -> speaker stream ... speaker off -> [RX;] -> mic
#include "radio_control_backend.h"

#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stream_uac.h"

static const char* TAG = "RADIO_FX4CR";

// True while we have keyed the radio (TX;). end_tx() is also called as a
// precaution before every band sync; skipping RX; when not keyed saves a
// CAT burst, i.e. one RX audio pause.
static bool s_keyed = false;

// TS-590 "TX1;" = DATA SEND (audio from USB/ACC2); plain "TX;" is SEND with
// the mic as audio source. F5BUD documents WSJT-X with rig TS-590S and
// mode Data/Pkt, for which Hamlib keys with TX1.
static const char k_tx_cmd[] = "TX1;";

static bool fx4cr_ready(void) {
    return cat_cdc_ready();
}

// MD2 = USB, which F5BUD maps to DATA-U when "Digital mode (CAT)" = DATA.
// DA1 additionally requests TS-590 data mode; ignored if unsupported.
// Split: unlike a real TS-590, on F5BUD "FR0;" turns split OFF and "FT0;"
// turns it ON (RX on VFO A, TX on VFO B) - tested on the radio. Send FR0
// (clears a split left from earlier) and never FT. Only VFO A is set, so
// VFO B stays free for the operator (e.g. a voice calling frequency).
static esp_err_t fx4cr_send_sync(int freq_hz, bool then_tx, bool resume_rx) {
    char fa[32];
    snprintf(fa, sizeof(fa), "FA%011d;", freq_hz);
    const char* cmds[] = {"MD2;", "DA1;", "FR0;", fa, k_tx_cmd};
    return uac_cat_batch(cmds, then_tx ? 5 : 4, resume_rx);
}

static esp_err_t fx4cr_sync_frequency_mode(int freq_hz) {
    esp_err_t err = fx4cr_send_sync(freq_hz, false, true);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "FX-4CR sync ok freq=%d", freq_hz);
    }
    return err;
}

static esp_err_t fx4cr_begin_tx(int freq_hz, int tx_base_hz) {
    (void)tx_base_hz;
    // Keep the mic paused: the TX audio stream needs its host channel next.
    // Exactly the tune burst (mode, VFO, frequency, TX1), which keys reliably.
    esp_err_t err = fx4cr_send_sync(freq_hz, true, false);
    if (err != ESP_OK) {
        uac_rx_resume();
        return err;
    }
    s_keyed = true;
    ESP_LOGI(TAG, "FX-4CR TX start");
    return ESP_OK;
}

static esp_err_t fx4cr_set_tone_hz(float tone_hz) {
    (void)tone_hz;  // TX tones are synthesized into the UAC OUT stream
    return ESP_OK;
}

static esp_err_t fx4cr_end_tx(void) {
    if (!s_keyed) {
        uac_tx_end_ex(true);  // nothing keyed: just make sure RX audio runs
        return ESP_OK;
    }
    uac_tx_end_ex(false);  // speaker off, mic still paused
    // Unkeying must never be throttled or dropped: bypass the back-off and
    // retry, or the radio would stay in TX.
    const char* cmds[] = {"RX;"};
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < 3 && err != ESP_OK; ++attempt) {
        if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(100));
        err = uac_cat_batch_ex(cmds, 1, true, true);  // then mic back on
    }
    if (err == ESP_OK) {
        s_keyed = false;
        ESP_LOGI(TAG, "FX-4CR TX stop");
    } else {
        uac_rx_resume();
    }
    return err;
}

static esp_err_t fx4cr_set_tune(bool enable, int freq_hz, int tone_hz) {
    if (!enable) {
        return fx4cr_end_tx();
    }
    esp_err_t err = fx4cr_send_sync(freq_hz, true, false);
    if (err != ESP_OK) {
        uac_rx_resume();
        return err;
    }
    s_keyed = true;
    if (!uac_tx_begin_tune(static_cast<float>(tone_hz))) {
        fx4cr_end_tx();
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static esp_err_t fx4cr_on_audio_start(void) {
    ESP_LOGI(TAG, "FX-4CR CAT/UAC backend initialized");
    return ESP_OK;
}

static esp_err_t fx4cr_start_tx_waveform(float base_hz,
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

static const radio_control_ops_t k_ops = {
    .name = "fx4cr",
    .ready = fx4cr_ready,
    .on_audio_start = fx4cr_on_audio_start,
    .sync_frequency_mode = fx4cr_sync_frequency_mode,
    .begin_tx = fx4cr_begin_tx,
    .set_tone_hz = fx4cr_set_tone_hz,
    .end_tx = fx4cr_end_tx,
    .set_tune = fx4cr_set_tune,
    .set_time = nullptr,  // TS-590 CAT has no clock command
    .start_tx_waveform = fx4cr_start_tx_waveform,
};

const radio_control_ops_t* radio_control_fx4cr_get_ops(void) {
    return &k_ops;
}
