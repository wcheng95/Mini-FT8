#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Decimation factor: 48kHz -> 12kHz
#define RESAMPLE_FACTOR 4

// FIR filter length (taps)
#define FIR_TAPS 32

// Resampler state for maintaining filter history between calls
typedef struct {
    float history[FIR_TAPS];  // Filter delay line
    int history_idx;          // Current position in delay line
} resample_state_t;

// Initialize resampler state
void resample_init(resample_state_t* state);

// Convert 24-bit stereo samples to mono float
// Input: 24-bit LE stereo samples (6 bytes per sample pair: L0 L1 L2 R0 R1 R2)
// Output: Mono float samples in range [-1.0, 1.0]
// Returns: Number of mono samples written
int convert_24bit_stereo_to_mono_float(
    const uint8_t* in,      // Input buffer (24-bit stereo, little-endian)
    float* out,             // Output buffer (mono float)
    int num_stereo_samples  // Number of stereo sample pairs
);

// Decimate 48kHz mono float samples to 12kHz with anti-aliasing
// Input: 48kHz mono float samples
// Output: 12kHz mono float samples
// Returns: Number of output samples written (in_samples / RESAMPLE_FACTOR)
// Note: in_samples must be a multiple of RESAMPLE_FACTOR
int resample_48k_to_12k(
    resample_state_t* state,
    const float* in,        // Input buffer (48kHz mono)
    float* out,             // Output buffer (12kHz mono)
    int in_samples          // Number of input samples
);

// Combined conversion: 24-bit/48kHz/stereo -> 12kHz mono float
// This is the main entry point for UAC audio processing
// Input: Raw USB audio data (24-bit LE stereo at 48kHz)
// Output: 12kHz mono float samples ready for FT8 processing
// Returns: Number of 12kHz samples written
int uac_to_ft8_samples(
    resample_state_t* state,
    const uint8_t* in,      // USB audio buffer (24-bit stereo)
    float* out,             // Output buffer (12kHz mono float)
    int num_stereo_samples  // Number of stereo sample pairs at 48kHz
);

#ifdef __cplusplus
}
#endif
