#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "ft8_audio_pipeline.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UAC_SAMPLE_RATE 48000
#define UAC_BIT_RESOLUTION 24
#define UAC_CHANNELS 2

#define UAC_BUFFER_SIZE 4000
#define UAC_BUFFER_THRESHOLD 600

typedef enum {
    UAC_STATE_IDLE,
    UAC_STATE_WAITING,
    UAC_STATE_CONNECTED,
    UAC_STATE_STREAMING,
    UAC_STATE_ERROR,
} uac_stream_state_t;

typedef enum {
    UAC_PROFILE_QMX = 0,
    UAC_PROFILE_GENERIC_USB = 1,
    // FX-4CR: internal hub with a C-Media UAC1 codec plus a USB-serial
    // bridge (CH34x / CP210x / CDC-ACM) for Kenwood CAT at 115200 baud.
    UAC_PROFILE_FX4CR = 2,
} uac_stream_profile_t;

#define UAC_WATERFALL_ROW_WIDTH FT8_AUDIO_WATERFALL_ROW_WIDTH

uac_stream_state_t uac_get_state(void);
bool uac_is_streaming(void);
bool uac_start(void);
bool uac_start_with_profile(uac_stream_profile_t profile);
void uac_stop(void);
bool uac_qmx_detected(void);

const char* uac_get_status_string(void);
const char* uac_get_debug_line1(void);
const char* uac_get_debug_line2(void);
bool uac_get_latest_waterfall_row(uint8_t* out_row, int out_len);

bool cat_cdc_ready(void);
// FX-4CR STATUS text: "CAT none", "CAT seen vvvv:pppp", "CAT vvvv:pppp",
// "CAT ok vvvv:pppp" after a successful burst, or "CATerr <error>".
const char* uac_get_cat_status(void);
esp_err_t cat_cdc_send(const uint8_t* data, size_t len, uint32_t timeout_ms);

bool uac_tx_begin_cpfsk(float base_hz,
                        const uint8_t* symbols,
                        size_t symbol_count,
                        float tone_spacing_hz,
                        uint32_t samples_per_symbol);
bool uac_tx_begin_tune(float tone_hz);
void uac_tx_end(void);
// As uac_tx_end(); with resume_rx = false the FX-4CR mic stays paused so a
// following CAT batch (RX;) can use its host channel.
void uac_tx_end_ex(bool resume_rx);
// Send CAT commands as one burst. FX-4CR: pauses the mic, opens the bridge,
// sends, closes it; resume_rx restarts the mic afterwards. Others: plain send.
esp_err_t uac_cat_batch(const char* const* cmds, size_t count, bool resume_rx);
// urgent = true ignores the failure back-off (used for RX; to unkey).
esp_err_t uac_cat_batch_ex(const char* const* cmds, size_t count, bool resume_rx, bool urgent);
// Restart an FX-4CR mic paused for TX / CAT (no-op if not paused).
void uac_rx_resume(void);

#ifdef __cplusplus
}
#endif
