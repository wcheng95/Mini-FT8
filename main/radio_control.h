#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RADIO_CONTROL_QMX = 0,
    RADIO_CONTROL_KH1_CAT = 1,
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

#ifdef __cplusplus
}
#endif
