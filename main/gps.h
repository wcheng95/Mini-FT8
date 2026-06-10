#pragma once

#include <stdint.h>
#include <string>

#include "driver/gpio.h"
#include "driver/uart.h"

struct gps_state_t {
    bool valid_fix = false;
    int satellites = 0;          // sats USED in current solution (from GGA)
    int satellites_in_view = 0;  // sats VISIBLE across all constellations (from GSV)
    int gga_fix_quality = 0;     // 0=no fix, 1=GPS, 2=DGPS, 6=estimated (GGA field 6)
    std::string time_utc;    // "HH:MM:SS"
    std::string date_utc;    // "YYYY-MM-DD"
    double latitude = 0.0;
    double longitude = 0.0;
    std::string grid_square; // "CM97"
    uint32_t last_rx_ms = 0; // last received decodable NMEA sentence
    uint32_t total_rx_bytes = 0; // raw bytes received since gps_start
    uint32_t started_at_ms = 0;  // tick when most recent gps_start() ran
    int active_baud = 0;
    bool baud_locked = false;
    bool running = false;
};

// Hardware binding for a GPS source: which UART peripheral, pins, and baud policy.
// PORTA (AT6668 unit, Grove): UART_NUM_1 on G1/G2, auto-baud 9600/115200.
// CAP-1262 stacking cap (ATGM336H): UART_NUM_2 on G15/G13, fixed 115200.
struct gps_pins_t {
    uart_port_t uart;
    gpio_num_t  rx;
    gpio_num_t  tx;
    int         default_baud;
    bool        auto_baud;
};

// Start GPS parser on the given UART/pins. Idempotent: a second call is a no-op
// unless gps_stop() has been called.
void gps_start(const gps_pins_t& pins);

// Stop GPS parser and release the UART driver.
void gps_stop();

// Periodic housekeeping hook (lightweight; safe to call each loop).
void gps_tick();

// Current state snapshot.
gps_state_t gps_get_state();

// One-shot event: returns true once when auto-baud locks to a new baud.
bool gps_take_baud_update(int* out_baud);
