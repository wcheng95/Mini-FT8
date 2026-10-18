#include "audio_source.h"

#include "ft8_audio_pipeline.h"
#include "stream_mic.h"
#include "stream_uac.h"
#include "stream_trusdx.h"

#include "esp_log.h"

static const char* TAG = "AUDIO_SRC";
static audio_source_backend_t s_backend = AUDIO_SOURCE_QMX_UAC;
static audio_source_backend_t s_active_backend = AUDIO_SOURCE_QMX_UAC;
static bool s_have_active_backend = false;

static bool backend_is_uac(audio_source_backend_t backend) {
    return backend == AUDIO_SOURCE_QMX_UAC || backend == AUDIO_SOURCE_USB_UAC_GENERIC ||
           backend == AUDIO_SOURCE_FX4CR_UAC;
}

void audio_source_set_backend(audio_source_backend_t backend) {
    s_backend = backend;
}

audio_source_backend_t audio_source_get_backend(void) {
    return s_backend;
}

const char* audio_source_backend_name(audio_source_backend_t backend) {
    switch (backend) {
    case AUDIO_SOURCE_QMX_UAC:
        return "qmx_uac";
    case AUDIO_SOURCE_USB_UAC_GENERIC:
        return "usb_uac_generic";
    case AUDIO_SOURCE_KH1_MIC:
        return "kh1_mic";
    case AUDIO_SOURCE_FX4CR_UAC:
        return "fx4cr_uac";
    case AUDIO_SOURCE_TRUSDX_CAT:
        return "trusdx_cat";
    default:
        return "unknown";
    }
}

bool audio_source_start(void) {
    bool ok = false;
    if (backend_is_uac(s_backend)) {
        uac_stream_profile_t profile = UAC_PROFILE_QMX;
        if (s_backend == AUDIO_SOURCE_USB_UAC_GENERIC) {
            profile = UAC_PROFILE_GENERIC_USB;
        } else if (s_backend == AUDIO_SOURCE_FX4CR_UAC) {
            profile = UAC_PROFILE_FX4CR;
        }
        ESP_LOGI(TAG, "Start audio source backend=%s", audio_source_backend_name(s_backend));
        ok = uac_start_with_profile(profile);
    } else if (s_backend == AUDIO_SOURCE_KH1_MIC) {
        ESP_LOGI(TAG, "Start audio source backend=%s", audio_source_backend_name(s_backend));
        ok = mic_stream_start();
    } else if (s_backend == AUDIO_SOURCE_TRUSDX_CAT) {
        ESP_LOGI(TAG, "Start audio source backend=%s", audio_source_backend_name(s_backend));
        ok = trusdx_stream_start();
    }

    if (ok) {
        s_active_backend = s_backend;
        s_have_active_backend = true;
    }
    return ok;
}

void audio_source_stop(void) {
    audio_source_backend_t backend = s_have_active_backend ? s_active_backend : s_backend;
    if (backend_is_uac(backend)) {
        uac_stop();
    } else if (backend == AUDIO_SOURCE_KH1_MIC) {
        mic_stream_stop();
    } else if (backend == AUDIO_SOURCE_TRUSDX_CAT) {
        trusdx_stream_stop();
    }
    s_have_active_backend = false;
}

bool audio_source_is_streaming(void) {
    audio_source_backend_t backend = s_have_active_backend ? s_active_backend : s_backend;
    if (backend_is_uac(backend)) {
        return uac_is_streaming();
    }
    if (backend == AUDIO_SOURCE_KH1_MIC) {
        return mic_stream_is_streaming();
    }
    if (backend == AUDIO_SOURCE_TRUSDX_CAT) {
        return trusdx_stream_is_streaming();
    }
    return false;
}

bool audio_source_qmx_detected(void) {
    audio_source_backend_t backend = s_have_active_backend ? s_active_backend : s_backend;
    if (backend == AUDIO_SOURCE_TRUSDX_CAT) {
        return trusdx_stream_detected();
    }
    return uac_qmx_detected();
}

const char* audio_source_get_status_string(void) {
    audio_source_backend_t backend = s_have_active_backend ? s_active_backend : s_backend;
    if (backend_is_uac(backend)) {
        return uac_get_status_string();
    }
    if (backend == AUDIO_SOURCE_KH1_MIC) {
        return mic_stream_get_status_string();
    }
    if (backend == AUDIO_SOURCE_TRUSDX_CAT) {
        return trusdx_get_status_string();
    }
    return "Idle";
}

const char* audio_source_get_debug_line1(void) {
    audio_source_backend_t backend = s_have_active_backend ? s_active_backend : s_backend;
    if (backend_is_uac(backend)) {
        return uac_get_debug_line1();
    }
    if (backend == AUDIO_SOURCE_KH1_MIC) {
        return mic_stream_get_debug_line1();
    }
    if (backend == AUDIO_SOURCE_TRUSDX_CAT) {
        return trusdx_get_debug_line1();
    }
    return "";
}

const char* audio_source_get_debug_line2(void) {
    audio_source_backend_t backend = s_have_active_backend ? s_active_backend : s_backend;
    if (backend_is_uac(backend)) {
        return uac_get_debug_line2();
    }
    if (backend == AUDIO_SOURCE_KH1_MIC) {
        return mic_stream_get_debug_line2();
    }
    if (backend == AUDIO_SOURCE_TRUSDX_CAT) {
        return trusdx_get_debug_line2();
    }
    return "";
}

bool audio_source_get_latest_waterfall_row(uint8_t* out_row, int out_len) {
    return ft8_audio_pipeline_get_latest_waterfall_row(out_row, out_len);
}
