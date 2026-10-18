#pragma once

// ============================================================================
// station_types.h
//
// Enums and plain types describing Mini-FT8 station state. Shared between
// main.cpp (owner of the runtime globals) and core_api.cpp (exposing the
// functional-core API). Kept separate so core_api.cpp can access the
// underlying globals directly without main.cpp having to expose its
// internal headers.
// ============================================================================

#include <string>

// Beacon (automatic CQ) mode.
enum class BeaconMode { OFF = 0, EVEN, ODD };

// CQ prefix variants.
enum class CqType { CQ, CQSOTA, CQPOTA, CQQRP, CQFD, CQFREETEXT };

// How the TX audio offset is chosen for new QSOs.
enum class OffsetSrc { RANDOM, CURSOR, RX };

// Supported radios. Keep explicit values stable for station.txt radio= saves.
enum class RadioType {
    NONE = 0,
    TRUSDX = 1,
    QMX = 2,
    KH1_USBC = 3,
    KH1 = KH1_USBC, // Backward-compatible alias for old KH1 USB-C mode.
    KH1_MIC = 4,
    QDX = 5,
    FX4CR = 6,
};

// One entry in the band list (name + frequency in kHz).
// float to support FT4 frequencies with .5 kHz offsets (e.g. 40m FT4 = 7047.5 kHz).
struct BandItem {
    const char* name;
    float freq;  // kHz
};
