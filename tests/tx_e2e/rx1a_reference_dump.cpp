// rx1a_reference_dump.cpp
//
// One-time/diagnostic reference dumper for MiniFT8-V3 RX-1A.
// It records stable boundary evidence from MiniFT8-V2 without changing
// decoder/DSP behavior:
//
//   PCM -> monitor/waterfall fingerprint
//   waterfall -> production-policy candidate decode -> exact payload(s)
//   payload -> message type/rendered protocol semantics
//
// The values emitted by this program are intended to be frozen into the
// MiniShell RX-1A golden manifest and boundary regression tests.  Do not use
// this tool to silently regenerate expected values after decoder changes.

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

static const char* proto_name(ftx_protocol_t proto)
{
    return (proto == FTX_PROTOCOL_FT4) ? "FT4" : "FT8";
}

static const char* type_name(ftx_message_type_t type)
{
    switch (type)
    {
    case FTX_MESSAGE_TYPE_FREE_TEXT:   return "FREE_TEXT";
    case FTX_MESSAGE_TYPE_DXPEDITION:  return "DXPEDITION";
    case FTX_MESSAGE_TYPE_EU_VHF:      return "EU_VHF";
    case FTX_MESSAGE_TYPE_ARRL_FD:     return "ARRL_FD";
    case FTX_MESSAGE_TYPE_TELEMETRY:   return "TELEMETRY";
    case FTX_MESSAGE_TYPE_CONTESTING:  return "CONTESTING";
    case FTX_MESSAGE_TYPE_STANDARD:    return "STANDARD";
    case FTX_MESSAGE_TYPE_ARRL_RTTY:   return "ARRL_RTTY";
    case FTX_MESSAGE_TYPE_NONSTD_CALL: return "NONSTD_CALL";
    case FTX_MESSAGE_TYPE_WWROF:       return "WWROF";
    default:                            return "UNKNOWN";
    }
}

static void payload_hex(const ftx_message_t& msg, char out[FTX_PAYLOAD_LENGTH_BYTES * 2 + 1])
{
    for (int i = 0; i < FTX_PAYLOAD_LENGTH_BYTES; ++i)
        std::snprintf(out + (2 * i), 3, "%02X", msg.payload[i]);
    out[FTX_PAYLOAD_LENGTH_BYTES * 2] = '\0';
}

static bool payload_equal(const ftx_message_t& msg,
                          const std::array<uint8_t, FTX_PAYLOAD_LENGTH_BYTES>& other)
{
    return std::memcmp(msg.payload, other.data(), FTX_PAYLOAD_LENGTH_BYTES) == 0;
}

static std::array<uint8_t, FTX_PAYLOAD_LENGTH_BYTES> payload_copy(const ftx_message_t& msg)
{
    std::array<uint8_t, FTX_PAYLOAD_LENGTH_BYTES> out{};
    std::memcpy(out.data(), msg.payload, FTX_PAYLOAD_LENGTH_BYTES);
    return out;
}

static int dump_monitor_decode_case(const char* filename, ftx_protocol_t proto)
{
    char path[512];
    std::snprintf(path, sizeof(path), "%s/%s", RX1A_GOLDEN_DIR, filename);

    std::vector<float> pcm;
    int fs = 0;
    if (read_wav(path, pcm, &fs) != 0)
        return 1;

    monitor_config_t cfg{};
    cfg.f_min = 200.0f;
    cfg.f_max = 2900.0f;
    cfg.sample_rate = fs;
    cfg.time_osr = 2;
    cfg.freq_osr = 1;
    cfg.protocol = proto;

    monitor_t mon{};
    monitor_init(&mon, &cfg);
    if (mon.wf.mag == nullptr || mon.block_size <= 0 || mon.wf.block_stride <= 0)
    {
        std::fprintf(stderr, "RX1A: monitor_init failed for %s\n", filename);
        monitor_free(&mon);
        return 1;
    }
    monitor_reset(&mon);

    const int n_samples = static_cast<int>(pcm.size());
    for (int i = 0; i + mon.block_size <= n_samples; i += mon.block_size)
        monitor_process(&mon, pcm.data() + i);

    const size_t active_elems = static_cast<size_t>(mon.wf.num_blocks) *
                                static_cast<size_t>(mon.wf.block_stride);
    const size_t active_bytes = active_elems * sizeof(WF_ELEM_T);
    const uint64_t wf_hash = fnv1a64(mon.wf.mag, active_bytes);

    std::printf(
        "RX1A_MONITOR case=%s proto=%s fs=%d block=%d subblock=%d nfft=%d "
        "min_bin=%d max_bin=%d blocks=%d bins=%d time_osr=%d freq_osr=%d "
        "stride=%d elem_size=%zu bytes=%zu fnv1a64=%016llX\n",
        filename, proto_name(proto), fs, mon.block_size, mon.subblock_size,
        mon.nfft, mon.min_bin, mon.max_bin, mon.wf.num_blocks, mon.wf.num_bins,
        mon.wf.time_osr, mon.wf.freq_osr, mon.wf.block_stride,
        sizeof(WF_ELEM_T), active_bytes,
        static_cast<unsigned long long>(wf_hash));

    constexpr int kCandidateCapacity = 50;
    constexpr int kMinScore = 5;
    constexpr int kMaxLdpcIterations = 25;
    ftx_candidate_t candidates[kCandidateCapacity]{};
    const int num_candidates = ftx_find_candidates(
        &mon.wf, kCandidateCapacity, candidates, kMinScore);

    if (num_candidates > 0)
    {
        const ftx_candidate_t& top = candidates[0];
        std::printf(
            "RX1A_SEARCH case=%s candidates=%d capacity=%d min_score=%d "
            "top_score=%d top_time_offset=%d top_freq_offset=%d "
            "top_time_sub=%u top_freq_sub=%u\n",
            filename, num_candidates, kCandidateCapacity, kMinScore,
            static_cast<int>(top.score), static_cast<int>(top.time_offset),
            static_cast<int>(top.freq_offset), static_cast<unsigned>(top.time_sub),
            static_cast<unsigned>(top.freq_sub));
    }
    else
    {
        std::printf(
            "RX1A_SEARCH case=%s candidates=0 capacity=%d min_score=%d\n",
            filename, kCandidateCapacity, kMinScore);
    }

    decode_clear_hashes();
    std::vector<std::array<uint8_t, FTX_PAYLOAD_LENGTH_BYTES>> unique_payloads;

    for (int i = 0; i < num_candidates; ++i)
    {
        ftx_message_t msg{};
        ftx_decode_status_t status{};
        if (!ftx_decode_candidate(&mon.wf, &candidates[i], kMaxLdpcIterations,
                                  &msg, &status))
            continue;

        bool duplicate = false;
        for (const auto& p : unique_payloads)
        {
            if (payload_equal(msg, p))
            {
                duplicate = true;
                break;
            }
        }
        if (duplicate)
            continue;

        unique_payloads.push_back(payload_copy(msg));

        char text[FTX_MAX_MESSAGE_LENGTH]{};
        ftx_message_offsets_t offsets{};
        const ftx_message_rc_t rc = ftx_message_decode(
            &msg, decode_get_hash_if(), text, &offsets);
        char hex[FTX_PAYLOAD_LENGTH_BYTES * 2 + 1];
        payload_hex(msg, hex);
        const ftx_message_type_t type = ftx_message_get_type(&msg);

        std::printf(
            "RX1A_DECODE case=%s unique_index=%zu candidate_index=%d "
            "payload=%s crc14=%04X type=%s(%d) rc=%d text=\"%s\" "
            "score=%d time_offset=%d freq_offset=%d time_sub=%u freq_sub=%u "
            "ldpc_errors=%d crc_extracted=%04X crc_calculated=%04X\n",
            filename, unique_payloads.size() - 1, i, hex,
            static_cast<unsigned>(msg.hash), type_name(type), static_cast<int>(type),
            static_cast<int>(rc), text, static_cast<int>(candidates[i].score),
            static_cast<int>(candidates[i].time_offset),
            static_cast<int>(candidates[i].freq_offset),
            static_cast<unsigned>(candidates[i].time_sub),
            static_cast<unsigned>(candidates[i].freq_sub), status.ldpc_errors,
            static_cast<unsigned>(status.crc_extracted),
            static_cast<unsigned>(status.crc_calculated));
    }

    std::printf("RX1A_DECODE_SUMMARY case=%s unique_payloads=%zu max_ldpc_iterations=%d\n",
                filename, unique_payloads.size(), kMaxLdpcIterations);

    monitor_free(&mon);
    return unique_payloads.empty() ? 1 : 0;
}

static void dump_codec_result(const char* label, const ftx_message_t& msg)
{
    char hex[FTX_PAYLOAD_LENGTH_BYTES * 2 + 1];
    payload_hex(msg, hex);

    char text[FTX_MAX_MESSAGE_LENGTH]{};
    ftx_message_offsets_t offsets{};
    const ftx_message_rc_t rc = ftx_message_decode(
        &msg, decode_get_hash_if(), text, &offsets);
    const ftx_message_type_t type = ftx_message_get_type(&msg);

    std::printf(
        "RX1A_CODEC case=%s payload=%s i3=%u n3=%u type=%s(%d) rc=%d "
        "text=\"%s\" fields=%d@%d,%d@%d,%d@%d\n",
        label, hex,
        static_cast<unsigned>(ftx_message_get_i3(&msg)),
        static_cast<unsigned>(ftx_message_get_n3(&msg)),
        type_name(type), static_cast<int>(type), static_cast<int>(rc), text,
        static_cast<int>(offsets.types[0]), static_cast<int>(offsets.offsets[0]),
        static_cast<int>(offsets.types[1]), static_cast<int>(offsets.offsets[1]),
        static_cast<int>(offsets.types[2]), static_cast<int>(offsets.offsets[2]));
}

static int dump_codec_vectors()
{
    int failures = 0;
    ftx_message_t msg{};
    ftx_message_rc_t rc;

    decode_clear_hashes();
    ftx_message_init(&msg);
    rc = ftx_message_encode_std(&msg, decode_get_hash_if(), "CQ", "W1XYZ", "FN42");
    if (rc == FTX_MESSAGE_RC_OK) dump_codec_result("standard_cq", msg); else ++failures;

    decode_clear_hashes();
    ftx_message_init(&msg);
    rc = ftx_message_encode_arrl_fd(
        &msg, decode_get_hash_if(), "W6ABC", "AG6AQ", "R 1B SCV");
    if (rc == FTX_MESSAGE_RC_OK) dump_codec_result("arrl_fd", msg); else ++failures;

    decode_clear_hashes();
    ftx_message_init(&msg);
    rc = ftx_message_encode_dxpedition(
        &msg, decode_get_hash_if(), "K1ABC RR73; W9XYZ KH1/KH7Z -08");
    if (rc == FTX_MESSAGE_RC_OK) dump_codec_result("dxpedition", msg); else ++failures;

    decode_clear_hashes();
    ftx_message_init(&msg);
    rc = ftx_message_encode_nonstd(
        &msg, decode_get_hash_if(), "CQ", "PJ4/KA1ABC", "");
    if (rc == FTX_MESSAGE_RC_OK) dump_codec_result("nonstd_cq", msg); else ++failures;

    decode_clear_hashes();
    ftx_message_init(&msg);
    rc = ftx_message_encode_free(&msg, "CQ POTA W1XYZ");
    if (rc == FTX_MESSAGE_RC_OK) dump_codec_result("free_text_cq_shape", msg); else ++failures;

    // Document the V2 type-0.6 classification gap without treating it as a
    // golden target.  n3 occupies payload bits 71..73; i3 remains zero.
    ftx_message_t gap{};
    gap.payload[8] = 0x80; // n3 bit 2 = 1
    gap.payload[9] = 0x80; // n3 bits 1..0 = 2 -> n3 == 6, i3 == 0
    std::printf("RX1A_KNOWN_GAP case=type_0_6 i3=%u n3=%u get_type=%s(%d) expected_future=CONTESTING\n",
                static_cast<unsigned>(ftx_message_get_i3(&gap)),
                static_cast<unsigned>(ftx_message_get_n3(&gap)),
                type_name(ftx_message_get_type(&gap)),
                static_cast<int>(ftx_message_get_type(&gap)));

    return failures;
}

int main()
{
    std::printf("RX1A_REFERENCE_SOURCE MiniFT8-V2 algorithm_baseline=5bd3ef98f72388a850bebad04bd7300b90edb63c\n");

    int failures = 0;
    failures += dump_monitor_decode_case("ft8_cq_w1xyz_fn42.wav", FTX_PROTOCOL_FT8);
    failures += dump_monitor_decode_case("ft4_cq_w1xyz_fn42.wav", FTX_PROTOCOL_FT4);
    failures += dump_codec_vectors();

    std::printf("RX1A_REFERENCE_DONE failures=%d\n", failures);
    return failures == 0 ? 0 : 1;
}
