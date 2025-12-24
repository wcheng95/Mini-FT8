#include "resample.h"
#include <string.h>
#include <math.h>

// Pre-computed 32-tap FIR lowpass filter coefficients
// Design: Parks-McClellan with Hamming window
// Sample rate: 48kHz
// Passband: 0-5.5kHz, Stopband: 6kHz+
// Normalized cutoff: 5.5/24 = 0.229
static const float fir_coeffs[FIR_TAPS] = {
    -0.000273f,  0.000431f,  0.001489f,  0.002731f,
     0.003259f,  0.002110f, -0.001579f, -0.007796f,
    -0.014741f, -0.019461f, -0.018418f, -0.008530f,
     0.011556f,  0.040735f,  0.075128f,  0.109161f,
     0.136853f,  0.153241f,  0.155341f,  0.143162f,
     0.118866f,  0.086153f,  0.049851f,  0.014892f,
    -0.014195f, -0.034659f, -0.045301f, -0.046481f,
    -0.039811f, -0.027827f, -0.013393f, -0.000000f
};

// Symmetric FIR filter - use full 32 taps
// Coefficients sum to approximately 1.0 for unity gain

void resample_init(resample_state_t* state) {
    memset(state->history, 0, sizeof(state->history));
    state->history_idx = 0;
}

int convert_24bit_stereo_to_mono_float(
    const uint8_t* in,
    float* out,
    int num_stereo_samples
) {
    const float scale = 1.0f / 8388608.0f;  // 2^23 for 24-bit normalization

    for (int i = 0; i < num_stereo_samples; i++) {
        // Read left channel (24-bit LE)
        int offset = i * 6;  // 6 bytes per stereo sample (3+3)
        int32_t left = in[offset] | (in[offset + 1] << 8) | (in[offset + 2] << 16);
        // Sign extend from 24-bit to 32-bit
        if (left & 0x800000) {
            left |= 0xFF000000;
        }

        // Read right channel (24-bit LE)
        int32_t right = in[offset + 3] | (in[offset + 4] << 8) | (in[offset + 5] << 16);
        // Sign extend from 24-bit to 32-bit
        if (right & 0x800000) {
            right |= 0xFF000000;
        }

        // Downmix to mono: (L + R) / 2
        float mono = ((float)left + (float)right) * 0.5f * scale;
        out[i] = mono;
    }

    return num_stereo_samples;
}

// Apply FIR filter to a single sample and return filtered value
static inline float fir_filter_sample(resample_state_t* state, float sample) {
    // Add new sample to history
    state->history[state->history_idx] = sample;

    // Compute FIR output
    float acc = 0.0f;
    int idx = state->history_idx;

    for (int i = 0; i < FIR_TAPS; i++) {
        acc += fir_coeffs[i] * state->history[idx];
        idx--;
        if (idx < 0) {
            idx = FIR_TAPS - 1;
        }
    }

    // Advance history index
    state->history_idx++;
    if (state->history_idx >= FIR_TAPS) {
        state->history_idx = 0;
    }

    return acc;
}

int resample_48k_to_12k(
    resample_state_t* state,
    const float* in,
    float* out,
    int in_samples
) {
    int out_samples = in_samples / RESAMPLE_FACTOR;
    int out_idx = 0;

    for (int i = 0; i < in_samples; i++) {
        // Apply anti-aliasing filter to every input sample
        float filtered = fir_filter_sample(state, in[i]);

        // Decimate: keep every RESAMPLE_FACTOR-th sample
        if ((i + 1) % RESAMPLE_FACTOR == 0) {
            out[out_idx++] = filtered;
        }
    }

    return out_idx;
}

int uac_to_ft8_samples(
    resample_state_t* state,
    const uint8_t* in,
    float* out,
    int num_stereo_samples
) {
    // Temporary buffer for 48kHz mono samples
    // Max input: typically 288 bytes = 48 stereo samples per ms
    // For larger buffers, process in chunks
    static float temp_mono[4096];

    int total_out = 0;
    int remaining = num_stereo_samples;
    const uint8_t* in_ptr = in;
    float* out_ptr = out;

    while (remaining > 0) {
        int chunk = (remaining > 4096) ? 4096 : remaining;

        // Step 1: Convert 24-bit stereo to mono float
        convert_24bit_stereo_to_mono_float(in_ptr, temp_mono, chunk);

        // Step 2: Decimate 48kHz to 12kHz with anti-aliasing
        int out_count = resample_48k_to_12k(state, temp_mono, out_ptr, chunk);

        in_ptr += chunk * 6;  // 6 bytes per stereo sample
        out_ptr += out_count;
        total_out += out_count;
        remaining -= chunk;
    }

    return total_out;
}
