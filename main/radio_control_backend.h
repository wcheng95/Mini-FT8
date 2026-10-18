#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    const char* name;
    bool (*ready)(void);
    esp_err_t (*on_audio_start)(void);
    esp_err_t (*sync_frequency_mode)(int freq_hz);
    esp_err_t (*begin_tx)(int freq_hz, int tx_base_hz);
    esp_err_t (*set_tone_hz)(float tone_hz);
    esp_err_t (*end_tx)(void);
    esp_err_t (*set_tune)(bool enable, int freq_hz, int tone_hz);
    esp_err_t (*set_time)(int hour, int minute, int second);
    // Optional: host-synthesized TX audio. NULL for tone-command radios.
    esp_err_t (*start_tx_waveform)(float base_hz,
                                   const uint8_t* symbols,
                                   size_t symbol_count,
                                   float tone_spacing_hz,
                                   float symbol_period_s,
                                   uint32_t samples_per_symbol_48k);
} radio_control_ops_t;

const radio_control_ops_t* radio_control_qmx_get_ops(void);
const radio_control_ops_t* radio_control_qdx_get_ops(void);
const radio_control_ops_t* radio_control_kh1_get_ops(void);
const radio_control_ops_t* radio_control_fx4cr_get_ops(void);
const radio_control_ops_t* radio_control_trusdx_get_ops(void);
void radio_control_kh1_set_enabled(bool enabled);
bool radio_control_kh1_is_enabled(void);
esp_err_t radio_control_kh1_diag_test(char test_key, int freq_hz, int offset_hz, bool* out_fa_sent);
