#pragma once

#include <string>
#include <stdint.h>

struct gps_state_t {
    bool valid_fix;
    int satellites;
    std::string time_utc;    // "HH:MM:SS"
    std::string date_utc;    // "YYYY-MM-DD"
    double latitude;
    double longitude;
    std::string grid_square; // "CM97"
    uint32_t last_sync_ms;   // Millisecond tick of last successful active fix and clock sync
    uint32_t last_rx_ms;     // Millisecond tick of last received valid NMEA sentence
};

// Initialize UART on Port A (G1/G2) and start the GPS parsing background task
void gps_init();

// Fetch thread-safe copy of current GPS telemetry
gps_state_t gps_get_state();
