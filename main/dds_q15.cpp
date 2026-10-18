#include "dds_q15.h"

#include <atomic>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static constexpr unsigned QTABLE_BITS = 8;
static constexpr unsigned FRAC_BITS = 10;
static constexpr unsigned QTABLE_SIZE = 1u << QTABLE_BITS;
static constexpr unsigned VISIBLE_BITS = 2 + QTABLE_BITS + FRAC_BITS;
static constexpr unsigned PHASE_BITS = 64;

static int16_t s_sin_quarter[QTABLE_SIZE + 1];
static std::atomic<uint64_t> s_inc{0};
static uint64_t s_phase = 0;

static uint64_t s_cpfsk_incs[DDS_CPFSK_MAX_SYMBOLS];
static size_t s_cpfsk_symbol_count = 0;
static uint32_t s_cpfsk_samples_per_symbol = 0;
static uint32_t s_cpfsk_sample_counter = 0;
static std::atomic<bool> s_cpfsk_active{false};

static inline uint64_t inc_from_hz(double frequency_hz) {
    long double numerator = static_cast<long double>(frequency_hz)
                          * static_cast<long double>(1ULL << 63) * 2.0L;
    return static_cast<uint64_t>(
        llroundl(numerator / static_cast<long double>(DDS_FS_HZ)));
}

void dds_init(void) {
    const double step = (M_PI / 2.0) / static_cast<double>(QTABLE_SIZE);
    for (unsigned n = 0; n <= QTABLE_SIZE; ++n) {
        double value = sin(step * static_cast<double>(n));
        int32_t q15 = static_cast<int32_t>(lround(value * 32767.0));
        if (q15 > 32767) q15 = 32767;
        if (q15 < -32768) q15 = -32768;
        s_sin_quarter[n] = static_cast<int16_t>(q15);
    }
}

void dds_set_freq_hz(double frequency_hz) {
    if (frequency_hz < 0.0) frequency_hz = 0.0;
    s_inc.store(inc_from_hz(frequency_hz), std::memory_order_relaxed);
}

void dds_reset_phase(void) {
    s_phase = 0;
}

bool dds_cpfsk_begin(double base_hz,
                     const uint8_t* symbols,
                     size_t symbol_count,
                     double tone_spacing_hz,
                     uint32_t samples_per_symbol) {
    if (!symbols || symbol_count == 0 ||
        symbol_count > DDS_CPFSK_MAX_SYMBOLS ||
        base_hz < 0.0 || tone_spacing_hz <= 0.0 ||
        samples_per_symbol == 0) {
        return false;
    }

    s_cpfsk_active.store(false, std::memory_order_release);
    for (size_t i = 0; i < symbol_count; ++i) {
        const double frequency_hz =
            base_hz + tone_spacing_hz * static_cast<double>(symbols[i]);
        s_cpfsk_incs[i] = inc_from_hz(frequency_hz);
    }

    s_phase = 0;
    s_cpfsk_symbol_count = symbol_count;
    s_cpfsk_samples_per_symbol = samples_per_symbol;
    s_cpfsk_sample_counter = 0;
    s_inc.store(s_cpfsk_incs[0], std::memory_order_relaxed);
    s_cpfsk_active.store(true, std::memory_order_release);
    return true;
}

void dds_cpfsk_end(void) {
    s_cpfsk_active.store(false, std::memory_order_release);
}

static inline int16_t sin_q15_from_phase(uint64_t phase) {
    uint32_t value =
        static_cast<uint32_t>(phase >> (PHASE_BITS - VISIBLE_BITS));
    uint32_t quadrant = value >> (QTABLE_BITS + FRAC_BITS);
    uint32_t position =
        value & ((1u << (QTABLE_BITS + FRAC_BITS)) - 1u);
    uint32_t index = position >> FRAC_BITS;
    uint32_t fraction = position & ((1u << FRAC_BITS) - 1u);

    if (quadrant & 1u) {
        index = QTABLE_SIZE -
                ((position + ((1u << FRAC_BITS) - 1u)) >> FRAC_BITS);
        fraction = (-fraction) & ((1u << FRAC_BITS) - 1u);
    }

    int32_t sample;
    if (index >= QTABLE_SIZE) {
        sample = s_sin_quarter[QTABLE_SIZE];
    } else {
        int32_t y0 = s_sin_quarter[index];
        int32_t y1 = s_sin_quarter[index + 1];
        int32_t interpolated =
            (y1 - y0) * static_cast<int32_t>(fraction);
        sample = y0 +
                 ((interpolated + (1 << (FRAC_BITS - 1))) >> FRAC_BITS);
    }

    if (quadrant >= 2u) sample = -sample;
    return static_cast<int16_t>(sample);
}

// Writes one PCM frame (same sample on every channel), little-endian.
static inline void emit_frame(uint8_t* out, int16_t sample_q15,
                              unsigned bytes_per_sample, unsigned channels) {
    int32_t sample = static_cast<int32_t>(sample_q15);
    if (bytes_per_sample == 3) sample <<= 8;
    for (unsigned ch = 0; ch < channels; ++ch) {
        for (unsigned b = 0; b < bytes_per_sample; ++b) {
            *out++ = static_cast<uint8_t>((sample >> (8 * b)) & 0xFF);
        }
    }
}

void dds_render_pcm(uint8_t* out, unsigned frames,
                    unsigned bit_resolution, unsigned channels) {
    const unsigned bytes_per_sample = bit_resolution / 8;
    const unsigned frame_bytes = bytes_per_sample * channels;
    uint64_t phase = s_phase;

    if (s_cpfsk_active.load(std::memory_order_acquire)) {
        const uint32_t samples_per_symbol = s_cpfsk_samples_per_symbol;
        const uint64_t total_samples =
            static_cast<uint64_t>(s_cpfsk_symbol_count) * samples_per_symbol;
        uint32_t counter = s_cpfsk_sample_counter;

        while (frames > 0) {
            if (counter >= total_samples) {
                memset(out, 0, static_cast<size_t>(frames) * frame_bytes);
                frames = 0;
                break;
            }

            size_t symbol_index = counter / samples_per_symbol;
            uint32_t position_in_symbol =
                counter - static_cast<uint32_t>(symbol_index) * samples_per_symbol;
            uint32_t samples_left = samples_per_symbol - position_in_symbol;
            unsigned render_count =
                (frames < samples_left) ? frames : samples_left;
            uint64_t increment = s_cpfsk_incs[symbol_index];
            s_inc.store(increment, std::memory_order_relaxed);

            for (unsigned i = 0; i < render_count; ++i) {
                emit_frame(out, sin_q15_from_phase(phase), bytes_per_sample, channels);
                out += frame_bytes;
                phase += increment;
            }
            counter += render_count;
            frames -= render_count;
        }

        s_cpfsk_sample_counter = counter;
        s_phase = phase;
        return;
    }

    uint64_t increment = s_inc.load(std::memory_order_relaxed);
    for (unsigned n = 0; n < frames; ++n) {
        emit_frame(out, sin_q15_from_phase(phase), bytes_per_sample, channels);
        out += frame_bytes;
        phase += increment;
    }
    s_phase = phase;
}

void dds_render_24bit_stereo(uint8_t* out, unsigned frames) {
    dds_render_pcm(out, frames, 24, 2);
}
