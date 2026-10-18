#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AUDIO_SOURCE_QMX_UAC = 0,
    AUDIO_SOURCE_USB_UAC_GENERIC = 1,
    AUDIO_SOURCE_KH1_MIC = 2,
    AUDIO_SOURCE_FX4CR_UAC = 3,     // C-Media UAC behind the FX-4CR hub
    AUDIO_SOURCE_TRUSDX_CAT = 4,    // audio streamed inside truSDX CAT
} audio_source_backend_t;

void audio_source_set_backend(audio_source_backend_t backend);
audio_source_backend_t audio_source_get_backend(void);
const char* audio_source_backend_name(audio_source_backend_t backend);

bool audio_source_start(void);
void audio_source_stop(void);

bool audio_source_is_streaming(void);
bool audio_source_qmx_detected(void);
const char* audio_source_get_status_string(void);
const char* audio_source_get_debug_line1(void);
const char* audio_source_get_debug_line2(void);
bool audio_source_get_latest_waterfall_row(uint8_t* out_row, int out_len);

#ifdef __cplusplus
}
#endif
