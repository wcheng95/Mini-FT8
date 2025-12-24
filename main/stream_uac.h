#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// UAC audio format requirements (strict)
#define UAC_SAMPLE_RATE     48000
#define UAC_BIT_RESOLUTION  24
#define UAC_CHANNELS        2

// UAC buffer configuration
#define UAC_BUFFER_SIZE     16000   // Ringbuffer size in bytes
#define UAC_BUFFER_THRESHOLD 1000   // ~3.5ms at 48kHz stereo 24-bit

// UAC streaming state
typedef enum {
    UAC_STATE_IDLE,         // Not initialized or stopped
    UAC_STATE_WAITING,      // Waiting for device connection
    UAC_STATE_CONNECTED,    // Device connected, ready to stream
    UAC_STATE_STREAMING,    // Actively streaming audio
    UAC_STATE_ERROR,        // Error state
} uac_stream_state_t;

// Get current UAC streaming state
uac_stream_state_t uac_get_state(void);

// Check if UAC streaming is active
bool uac_is_streaming(void);

// Start UAC streaming tasks (call when user presses 'H')
// This starts USB host tasks and waits for device connection
// Returns true on success, false on failure
bool uac_start(void);

// Stop UAC streaming and cleanup
// Call when exiting Host mode
void uac_stop(void);

// Get current device info as a status string
// Returns pointer to static buffer, do not free
const char* uac_get_status_string(void);

#ifdef __cplusplus
}
#endif
