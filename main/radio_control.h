#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RADIO_CONTROL_QMX = 0,
    RADIO_CONTROL_KH1_CAT = 1,
    RADIO_CONTROL_QDX = 2,
    RADIO_CONTROL_FX4CR = 3,
    RADIO_CONTROL_TRUSDX = 4,
} radio_control_backend_t;

void radio_control_set_backend(radio_control_backend_t backend);
radio_control_backend_t radio_control_get_backend(void);
const char* radio_control_backend_name(radio_control_backend_t backend);

bool radio_control_ready(void);

esp_err_t radio_control_on_audio_start(void);
esp_err_t radio_control_sync_frequency_mode(int freq_hz);
esp_err_t radio_control_begin_tx(int freq_hz, int tx_base_hz);
esp_err_t radio_control_set_tone_hz(float tone_hz);
esp_err_t radio_control_end_tx(void);
esp_err_t radio_control_set_tune(bool enable, int freq_hz, int tone_hz);
esp_err_t radio_control_set_time(int hour, int minute, int second);

// Radios that take TX as a host-generated audio waveform (QDX, FX-4CR,
// truSDX) instead of per-symbol tone commands (QMX TA, KH1 FO).
bool radio_control_uses_tx_waveform(void);
// Start the waveform after radio_control_begin_tx(). samples_per_symbol_48k
// is the exact symbol length at 48 kHz (ProtocolConfig::samples_per_symbol).
esp_err_t radio_control_start_tx_waveform(float base_hz,
                                          const uint8_t* symbols,
                                          size_t symbol_count,
                                          float tone_spacing_hz,
                                          float symbol_period_s,
                                          uint32_t samples_per_symbol_48k);

#ifdef __cplusplus
}
#endif
