#pragma once

// ============================================================================
// stream_trusdx.h
//
// (tr)uSDX (DL2MAN / PE1NNZ firmware >= 2.00u) over its on-board CH340
// USB-serial bridge. The radio has no USB sound card: RX and TX audio are
// streamed inside the Kenwood CAT link ("CAT streaming", UA command).
//
//   RX:  radio -> host  "US" + U8 samples @ ~7812.5 Hz, terminated by ';'
//   TX:  host -> radio  ";TX0;" + U8 samples @ 11520 Hz, ended by ";RX;"
//
// Sample bytes never equal ';' (0x3B): the radio sends 0x3C instead, and the
// host must send 0x3A. The Cardputer must supply 5 V on USB-C (PORTA 5VIN).
// ============================================================================

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

bool trusdx_stream_start(void);
void trusdx_stream_stop(void);
bool trusdx_stream_is_streaming(void);
bool trusdx_stream_detected(void);

const char* trusdx_get_status_string(void);
const char* trusdx_get_debug_line1(void);
const char* trusdx_get_debug_line2(void);

// CAT link is open and the radio has finished its post-open reset delay.
bool trusdx_cat_ready(void);
// Send one CAT command such as "FA00014074000;". While TX audio is
// streaming a leading ';' is added so the command is not taken as audio.
esp_err_t trusdx_cat_send(const char* cmd, uint32_t timeout_ms);

// Key the radio (";TX0;") and stream an FT8/FT4 CPFSK waveform at 11520 Hz.
bool trusdx_tx_begin_cpfsk(float base_hz,
                           const uint8_t* symbols,
                           size_t symbol_count,
                           float tone_spacing_hz,
                           float symbol_period_s);
// Key the radio and stream a constant tone until trusdx_tx_end().
bool trusdx_tx_begin_tune(float tone_hz);
// Stop streaming, let the radio drain, then return to RX (";RX;").
esp_err_t trusdx_tx_end(void);

#ifdef __cplusplus
}
#endif
