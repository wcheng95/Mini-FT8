#pragma once
#include <cstddef>
#include <cstdint>
#include "esp_err.h"

esp_err_t mic_input_init();
void mic_input_start();
void mic_input_stop();

// Fetch exactly n float samples at 12 kHz into out[]; blocks until filled.
bool mic_input_get_block(float* out, size_t n);
