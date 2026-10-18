#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DDS_FS_HZ 48000
#define DDS_CPFSK_MAX_SYMBOLS 105u

void dds_init(void);
void dds_set_freq_hz(double frequency_hz);
void dds_reset_phase(void);

bool dds_cpfsk_begin(double base_hz,
                     const uint8_t* symbols,
                     size_t symbol_count,
                     double tone_spacing_hz,
                     uint32_t samples_per_symbol);
void dds_cpfsk_end(void);

// Render packed 24-bit stereo PCM (six bytes per frame).
void dds_render_24bit_stereo(uint8_t* out, unsigned frames);

// Render packed little-endian PCM at DDS_FS_HZ. bit_resolution is 16 or 24,
// channels is 1 or 2; every channel carries the same sample.
void dds_render_pcm(uint8_t* out, unsigned frames,
                    unsigned bit_resolution, unsigned channels);

#ifdef __cplusplus
}
#endif
