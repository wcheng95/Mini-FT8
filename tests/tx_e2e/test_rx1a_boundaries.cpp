// test_rx1a_boundaries.cpp
//
// RX-1A boundary regression anchors for MiniFT8-V3 cleanup.
//
// Algorithm baseline (MiniFT8-V2 source, before RX-1A test-only commits):
//   5bd3ef98f72388a850bebad04bd7300b90edb63c
//
// Hard invariants frozen here:
//   1. known PCM -> exact compact waterfall bytes (via FNV-1a-64 fingerprint)
//   2. waterfall -> production-policy decoder -> exact unique protocol payload
//   3. fixed protocol payload -> protocol type + canonical decoded text
//
// Deliberately NOT hard-frozen here:
//   candidate score/order, V2 SNR, and V2's weak generic field-offset model.
// Those are diagnostics/baselines, not the future V3 structural contract.

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "decode_helper.h"

extern "C" {
#include "../../components/ft8_lib/common/monitor.h"
#include "../../components/ft8_lib/ft8/decode.h"
#include "../../components/ft8_lib/ft8/message.h"
}

#ifndef RX1A_GOLDEN_DIR
#define RX1A_GOLDEN_DIR "golden"
#endif

static int g_failures = 0;

static void failf(const char* label, const char* detail)
{
    std::fprintf(stderr, "FAIL [%s] %s\n", label, detail);
    ++g_failures;
}

static uint64_t fnv1a64(const void* data, size_t len)
{
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint64_t h = UINT64_C(14695981039346656037);
    for (size_t i = 0; i < len; ++i)
    {
        h ^= p[i];
        h *= UINT64_C(1099511628211);
    }
    return h;
}

static bool payload_equal(const ftx_message_t& msg,
                          const std::array<uint8_t, FTX_PAYLOAD_LENGTH_BYTES>& expected)
{
    return std::memcmp(msg.payload, expected.data(), FTX_PAYLOAD_LENGTH_BYTES) == 0;
}

static std::array<uint8_t, FTX_PAYLOAD_LENGTH_BYTES> payload_copy(const ftx_message_t& msg)
{
    std::array<uint8_t, FTX_PAYLOAD_LENGTH_BYTES> out{};
    std::memcpy(out.data(), msg.payload, FTX_PAYLOAD_LENGTH_BYTES);
    return out;
}

struct MonitorGolden
{
    const char* filename;
    ftx_protocol_t protocol;
    int sample_rate;
    int block_size;
    int subblock_size;
    int nfft;
    int min_bin;
    int max_bin;
    int num_blocks;
    int num_bins;
    int time_osr;
    int freq_osr;
    int block_stride;
    size_t active_bytes;
    uint64_t waterfall_fnv1a64;
    std::array<uint8_t, FTX_PAYLOAD_LENGTH_BYTES> payload;
    const char* decoded_text;
};

static const MonitorGolden kMonitorCases[] = {
    {
        "ft8_cq_w1xyz_fn42.wav", FTX_PROTOCOL_FT8,
        6000, 960, 480, 960, 32, 465, 85, 433, 2, 1, 866,
        73610, UINT64_C(0x18BE1E838FD9C6AF),
        {0x00,0x00,0x00,0x20,0x60,0x16,0x50,0x0A,0x19,0x88},
        "CQ W1XYZ FN42"
    },
    {
        "ft4_cq_w1xyz_fn42.wav", FTX_PROTOCOL_FT4,
        6000, 288, 144, 288, 9, 140, 125, 131, 2, 1, 262,
        32750, UINT64_C(0xCBDD2509276E5030),
        {0x00,0x00,0x00,0x20,0x60,0x16,0x50,0x0A,0x19,0x88},
        "CQ W1XYZ FN42"
    },
};

static void check_int(const char* label, const char* field, long long got, long long expected)
{
    if (got != expected)
    {
        char msg[160];
        std::snprintf(msg, sizeof(msg), "%s: got=%lld expected=%lld", field, got, expected);
        failf(label, msg);
    }
}

static void test_monitor_and_decode(const MonitorGolden& g)
{
    char path[512];
    std::snprintf(path, sizeof(path), "%s/%s", RX1A_GOLDEN_DIR, g.filename);

    std::vector<float> pcm;
    int fs = 0;
    if (read_wav(path, pcm, &fs) != 0)
    {
        failf(g.filename, "cannot read golden WAV");
        return;
    }

    monitor_config_t cfg{};
    cfg.f_min = 200.0f;
    cfg.f_max = 2900.0f;
    cfg.sample_rate = fs;
    cfg.time_osr = 2;
    cfg.freq_osr = 1;
    cfg.protocol = g.protocol;

    monitor_t mon{};
    monitor_init(&mon, &cfg);
    if (mon.wf.mag == nullptr || mon.block_size <= 0)
    {
        failf(g.filename, "monitor_init failed");
        monitor_free(&mon);
        return;
    }
    monitor_reset(&mon);

    for (int i = 0; i + mon.block_size <= static_cast<int>(pcm.size()); i += mon.block_size)
        monitor_process(&mon, pcm.data() + i);

    check_int(g.filename, "sample_rate", fs, g.sample_rate);
    check_int(g.filename, "block_size", mon.block_size, g.block_size);
    check_int(g.filename, "subblock_size", mon.subblock_size, g.subblock_size);
    check_int(g.filename, "nfft", mon.nfft, g.nfft);
    check_int(g.filename, "min_bin", mon.min_bin, g.min_bin);
    check_int(g.filename, "max_bin", mon.max_bin, g.max_bin);
    check_int(g.filename, "num_blocks", mon.wf.num_blocks, g.num_blocks);
    check_int(g.filename, "num_bins", mon.wf.num_bins, g.num_bins);
    check_int(g.filename, "time_osr", mon.wf.time_osr, g.time_osr);
    check_int(g.filename, "freq_osr", mon.wf.freq_osr, g.freq_osr);
    check_int(g.filename, "block_stride", mon.wf.block_stride, g.block_stride);

    const size_t active_bytes = static_cast<size_t>(mon.wf.num_blocks) *
                                static_cast<size_t>(mon.wf.block_stride) *
                                sizeof(WF_ELEM_T);
    check_int(g.filename, "active_bytes", static_cast<long long>(active_bytes),
              static_cast<long long>(g.active_bytes));

    const uint64_t wf_hash = fnv1a64(mon.wf.mag, active_bytes);
    if (wf_hash != g.waterfall_fnv1a64)
    {
        char msg[160];
        std::snprintf(msg, sizeof(msg),
                      "waterfall hash: got=%016llX expected=%016llX",
                      static_cast<unsigned long long>(wf_hash),
                      static_cast<unsigned long long>(g.waterfall_fnv1a64));
        failf(g.filename, msg);
    }

    constexpr int kCandidateCapacity = 50;
    constexpr int kMinScore = 5;
    constexpr int kMaxLdpcIterations = 25;
    ftx_candidate_t candidates[kCandidateCapacity]{};
    const int num_candidates = ftx_find_candidates(
        &mon.wf, kCandidateCapacity, candidates, kMinScore);

    std::vector<std::array<uint8_t, FTX_PAYLOAD_LENGTH_BYTES>> unique_payloads;
    char decoded_text[FTX_MAX_MESSAGE_LENGTH]{};

    for (int i = 0; i < num_candidates; ++i)
    {
        ftx_message_t msg{};
        ftx_decode_status_t status{};
        if (!ftx_decode_candidate(&mon.wf, &candidates[i], kMaxLdpcIterations,
                                  &msg, &status))
            continue;

        bool duplicate = false;
        for (const auto& existing : unique_payloads)
        {
            if (payload_equal(msg, existing))
            {
                duplicate = true;
                break;
            }
        }
        if (duplicate)
            continue;

        unique_payloads.push_back(payload_copy(msg));

        if (unique_payloads.size() == 1)
        {
            ftx_message_offsets_t offsets{};
            const ftx_message_rc_t rc = ftx_message_decode(
                &msg, nullptr, decoded_text, &offsets);
            if (rc != FTX_MESSAGE_RC_OK)
                failf(g.filename, "first unique payload failed message decode");
        }
    }

    check_int(g.filename, "unique_payload_count",
              static_cast<long long>(unique_payloads.size()), 1);

    if (!unique_payloads.empty() && unique_payloads[0] != g.payload)
        failf(g.filename, "decoded protocol payload differs from frozen value");

    if (std::strcmp(decoded_text, g.decoded_text) != 0)
    {
        char msg[192];
        std::snprintf(msg, sizeof(msg), "decoded text: got=\"%s\" expected=\"%s\"",
                      decoded_text, g.decoded_text);
        failf(g.filename, msg);
    }

    monitor_free(&mon);
}

// Minimal deterministic lookup needed by the frozen DXpedition payload.
// First 10 payload bits encode h10=0x0C9 for KH1/KH7Z.
static bool codec_hash_lookup(ftx_callsign_hash_type_t type, uint32_t hash, char* callsign)
{
    if (type == FTX_CALLSIGN_HASH_10_BITS && hash == 0x0C9u)
    {
        std::strcpy(callsign, "KH1/KH7Z");
        return true;
    }
    return false;
}

static void codec_hash_save(const char*, uint32_t)
{
}

static ftx_callsign_hash_interface_t g_codec_hash_if = {
    .lookup_hash = codec_hash_lookup,
    .save_hash = codec_hash_save,
};

struct CodecGolden
{
    const char* label;
    std::array<uint8_t, FTX_PAYLOAD_LENGTH_BYTES> payload;
    uint8_t i3;
    uint8_t n3;
    ftx_message_type_t type;
    const char* text;
};

static const CodecGolden kCodecCases[] = {
    {
        "standard_cq",
        {0x00,0x00,0x00,0x20,0x60,0x16,0x50,0x0A,0x19,0x88},
        1, 6, FTX_MESSAGE_TYPE_STANDARD, "CQ W1XYZ FN42"
    },
    {
        "arrl_fd",
        {0x0C,0x16,0x90,0xC5,0x36,0xB4,0x26,0x81,0x76,0xC0},
        0, 3, FTX_MESSAGE_TYPE_ARRL_FD, "W6ABC AG6AQ R 1B SCV"
    },
    {
        "dxpedition",
        {0x32,0x6C,0x13,0x7B,0xC6,0xA1,0x85,0x27,0x70,0x40},
        0, 1, FTX_MESSAGE_TYPE_DXPEDITION,
        "K1ABC RR73; W9XYZ <KH1/KH7Z> -08"
    },
    {
        "nonstd_cq",
        {0x00,0x00,0x3E,0x4A,0x34,0xA8,0x6E,0xEB,0x84,0x60},
        4, 1, FTX_MESSAGE_TYPE_NONSTD_CALL, "CQ PJ4/KA1ABC"
    },
    {
        "free_text_cq_shape",
        {0x2C,0x91,0x49,0x5D,0x9F,0x3A,0x73,0x11,0x94,0x00},
        0, 0, FTX_MESSAGE_TYPE_FREE_TEXT, "CQ POTA W1XYZ"
    },
};

static void test_codec_case(const CodecGolden& g)
{
    ftx_message_t msg{};
    std::memcpy(msg.payload, g.payload.data(), FTX_PAYLOAD_LENGTH_BYTES);

    check_int(g.label, "i3", ftx_message_get_i3(&msg), g.i3);
    check_int(g.label, "n3", ftx_message_get_n3(&msg), g.n3);
    check_int(g.label, "message_type", ftx_message_get_type(&msg), g.type);

    char text[FTX_MAX_MESSAGE_LENGTH]{};
    ftx_message_offsets_t offsets{};
    const ftx_message_rc_t rc = ftx_message_decode(
        &msg, &g_codec_hash_if, text, &offsets);
    if (rc != FTX_MESSAGE_RC_OK)
    {
        failf(g.label, "message decode returned error");
        return;
    }

    if (std::strcmp(text, g.text) != 0)
    {
        char detail[192];
        std::snprintf(detail, sizeof(detail), "text: got=\"%s\" expected=\"%s\"",
                      text, g.text);
        failf(g.label, detail);
    }
}

int main()
{
    std::printf("RX-1A boundary golden test\n");
    std::printf("algorithm baseline: 5bd3ef98f72388a850bebad04bd7300b90edb63c\n");

    for (const auto& c : kMonitorCases)
        test_monitor_and_decode(c);

    for (const auto& c : kCodecCases)
        test_codec_case(c);

    if (g_failures == 0)
    {
        std::printf("PASS: all RX-1A boundary anchors match\n");
        return 0;
    }

    std::fprintf(stderr, "FAIL: %d RX-1A boundary checks failed\n", g_failures);
    return 1;
}
