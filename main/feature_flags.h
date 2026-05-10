#pragma once

// ---------------------------------------------------------------------------
// Build-time feature flags for Mini-FT8.
//
// Override at cmake time:
//   idf.py build -DENABLE_FT4=1 -DENABLE_BLE=0
// or via sdkconfig.defaults.<variant> (see PR-3).
// ---------------------------------------------------------------------------

// ── 1. Protocol selection ────────────────────────────────────────────────────
// Mutually exclusive: exactly one protocol per binary (mini-ft8 vs mini-ft4).
// Both enabled → compile error.  Neither enabled → compile error.

#ifndef ENABLE_FT8
#define ENABLE_FT8 1   // default: FT8 binary
#endif

#ifndef ENABLE_FT4
#define ENABLE_FT4 0   // set to 1 (and ENABLE_FT8=0) for the FT4 binary
#endif

#if ENABLE_FT8 && ENABLE_FT4
#error "ENABLE_FT8 and ENABLE_FT4 cannot both be 1 — build one binary per protocol."
#endif

#if !ENABLE_FT8 && !ENABLE_FT4
#error "Exactly one of ENABLE_FT8 or ENABLE_FT4 must be 1."
#endif

// ── 2. Optional features ─────────────────────────────────────────────────────

// ENABLE_BLE — compile in NimBLE controller + GATT UI service.
// Default on for FT8. Must be 0 for the FT4 binary (DRAM constraint — see below).
#ifndef ENABLE_BLE
#define ENABLE_BLE 1
#endif

// ── 3. Cross-feature constraints ─────────────────────────────────────────────

#if ENABLE_BLE && ENABLE_FT4
#error "ENABLE_BLE and ENABLE_FT4 cannot both be 1 — NimBLE consumes ~50 KB DRAM needed by the FT4 decoder."
#endif

// ── 4. Per-variant sizing ────────────────────────────────────────────────────

// STREAM_TASK_STACK_BYTES — stack for the ft8_audio_pipeline_run() task, which
// calls decode_monitor_results() inline at the end of each slot.
// FT4's LDPC decode is ~2x heavier (105 symbols vs 79, 20.83 Hz tone spacing
// vs 6.25 Hz), so it needs more headroom. FT8 keeps the original 8 KB.
// Enabling ENABLE_FT4 in PR-3 automatically selects the right size here.
#if ENABLE_FT4
#define STREAM_TASK_STACK_BYTES 16384
#else
#define STREAM_TASK_STACK_BYTES  8192
#endif
