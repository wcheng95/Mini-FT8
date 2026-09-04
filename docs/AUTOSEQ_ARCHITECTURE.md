# Autoseq Architecture: Reference Project Design

This document describes the single-threaded state machine architecture used in the
DX-FT8 reference project and how Mini-FT8 implements the same pattern.

## Core Principle: Three Events, Strict Ordering

The reference project is a **single-threaded state machine** with exactly 3 events:

```
Event 1: 79 symbols received → start decoding
Event 2: decoding finished → autoseq parses decodes, populates TX message
Event 3: 15s slot boundary → check parity, TX if message ready, tick if was_txing
```

**Critical: Event 2 ALWAYS happens before Event 3.** This ordering eliminates race conditions.

## Event Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         15-SECOND SLOT TIMELINE                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  SLOT N (RX)              │  SLOT N+1 (TX)           │  SLOT N+2 (RX)   │
│  ─────────────────────────┼──────────────────────────┼─────────────────  │
│                           │                          │                   │
│  [Receive 79 symbols]     │  [Transmit if ready]     │  [Receive again]  │
│           │               │         │                │                   │
│           ▼               │         │                │                   │
│  Event 1: Decode start    │         │                │                   │
│           │               │         │                │                   │
│           ▼               │         │                │                   │
│  Event 2: Decode done     │         │                │                   │
│    - autoseq_on_decodes() │         │                │                   │
│    - Check TX ready       │         │                │                   │
│    - Set QSO_xmit flag    │         │                │                   │
│           │               │         │                │                   │
│           └───────────────┼────►Event 3: Slot boundary                   │
│                           │    - if was_txing: tick()│                   │
│                           │    - if QSO_xmit &&      │                   │
│                           │      correct_parity:     │                   │
│                           │      start TX            │                   │
│                           │         │                │                   │
│                           │         ▼                │                   │
│                           │    [TX runs ~12.8s]      │                   │
│                           │         │                │                   │
│                           │         └────────────────┼──► tick() at      │
│                           │                          │    next boundary  │
└─────────────────────────────────────────────────────────────────────────┘
```

## Reference Project Code Pattern (main.c)

```c
// EVENT 3: Slot boundary detection and TX trigger
int current_slot = ft8_time / 15000 % 2;
if (current_slot != slot_state) {
    slot_state ^= 1;

    // If we were transmitting, call tick to pop completed TX
    if (was_txing) {
        autoseq_tick();  // Pops the entry that was just transmitted
    }
    was_txing = 0;
}

// TX trigger: check if we should transmit in this slot
if (QSO_xmit && target_slot == slot_state && FT_8_counter < 29) {
    setup_to_transmit_on_next_DSP_Flag();
    QSO_xmit = 0;
    was_txing = 1;
}

// EVENT 2: After decode finishes (decode_flag set by decoder)
if (decode_flag) {
    // Only process decodes when NOT transmitting
    if (!was_txing) {
        autoseq_on_decodes(new_decoded, master_decoded);

        // Check if TX is ready from autoseq
        if (autoseq_get_next_tx(autoseq_txbuf)) {
            queue_custom_text(autoseq_txbuf);
            QSO_xmit = 1;
        }
        // ELSE IF beacon on, add CQ
        else if (Beacon_On) {
            autoseq_start_cq();
            autoseq_get_next_tx(autoseq_txbuf);
            queue_custom_text(autoseq_txbuf);
            QSO_xmit = 1;
            target_slot = slot_state ^ 1;
        }
    }
    decode_flag = 0;
}
```

## CQ/Beacon Design: Short-Lived Entries

**Key insight: CQ entries are SHORT-LIVED.** They exist in the queue for exactly ONE transmission.

```
1. Beacon is ON, no active QSO
2. Decode callback: autoseq_get_next_tx() returns false (queue empty)
3. Since Beacon_On: autoseq_start_cq() adds CQ entry
   - state = CALLING
   - next_tx = TX6 (CQ message)
   - dxcall = "CQ"
4. autoseq_get_next_tx() now returns true (CQ ready)
5. Set QSO_xmit = 1, target_slot = opposite parity
6. At slot boundary: TX starts, was_txing = 1
7. TX completes (12.8 seconds)
8. Next slot boundary: autoseq_tick() is called
9. tick() sees state=CALLING, transitions to IDLE, pops entry
10. Queue is now empty
11. Next decode: back to step 2, new CQ is added
```

**Why short-lived?**
- Makes beacon flag changes take effect immediately
- If beacon is turned OFF, the CQ in queue transmits once, then tick() removes it
- No stale CQ entries accumulating

## Queue Sorting Rules

The queue is sorted with this priority (descending):
```
IDLE (6)      → highest priority (gets popped first)
SIGNOFF (5)
ROGERS (4)
ROGER_REPORT (3)
REPORT (2)
REPLYING (1)
CALLING (0)   → lowest priority (CQ stays at bottom)
```

**Rationale:**
- IDLE entries are popped immediately by sort_and_clean()
- More advanced QSOs (closer to completion) get priority
- CQ (CALLING) stays at bottom, only transmitted when no active QSO

## autoseq_tick() Behavior

Called ONCE per TX slot, at slot boundary AFTER TX completes:

```c
void autoseq_tick(void) {
    if (queue_size == 0) return;

    ctx_t *ctx = &ctx_queue[0];  // Always operates on FRONT entry

    switch (ctx->state) {
        case AS_REPLYING:
        case AS_REPORT:
        case AS_ROGER_REPORT:
        case AS_ROGERS:
            // Increment retry counter or transition to IDLE if max retries
            if (ctx->retry_counter < ctx->retry_limit) {
                ctx->next_tx = TX_for_state;
                ctx->retry_counter++;
            } else {
                ctx->state = AS_IDLE;
            }
            break;

        case AS_CALLING:   // CQ - only once (controlled by beacon)
        case AS_SIGNOFF:   // 73 - only once
            ctx->state = AS_IDLE;
            ctx->next_tx = TX_UNDEF;
            break;
    }

    if (ctx->state == AS_IDLE) {
        pop();  // Remove from queue
    }
}
```

**Key points:**
- tick() always operates on queue[0] (the front)
- CALLING (CQ) immediately transitions to IDLE and gets popped
- This is why CQ is short-lived: one TX, one tick, gone

## Why This Design Prevents Race Conditions

**Problem with immediate TX scheduling:**
```
1. Decode finishes, TX is scheduled with delay
2. During delay, ANOTHER decode happens
3. New decode adds entries, queue is re-sorted
4. Original TX entry is no longer at queue[0]
5. tick() operates on wrong entry
6. Bug: CQ never gets popped, or wrong entry gets popped
```

**Reference design solution:**
```
1. Decode finishes, just SET FLAGS (QSO_xmit, target_slot)
2. Another decode happens - flags might be updated, that's fine
3. At slot boundary: check flags, start TX if conditions met
4. TX runs for entire slot
5. Next slot boundary: tick() operates on queue[0]
6. Since Event 2 always before Event 3, queue is stable when tick() runs
```

## Mini-FT8 Implementation

Mini-FT8 now follows the same pattern:

```cpp
// State machine variables
static volatile bool g_qso_xmit = false;        // TX is pending
static volatile int g_target_slot_parity = 0;   // Parity for TX
static volatile bool g_was_txing = false;       // For tick timing
static int g_last_slot_parity = -1;             // Slot boundary detection (parity only)

// Called from main loop every tick
static void check_slot_boundary() {
    int64_t slot_idx = now_ms / 15000;
    int slot_parity = slot_idx & 1;

    // Detect slot boundary (parity change)
    if (slot_parity != g_last_slot_parity) {
        g_last_slot_parity = slot_parity;

        // If we were transmitting, call tick
        if (g_was_txing) {
            autoseq_tick(slot_idx, slot_parity, 0);
            g_was_txing = false;
        }
    }

    // TX trigger
    if (g_qso_xmit && g_target_slot_parity == slot_parity &&
        slot_ms < 4000 && s_tx_task_handle == NULL) {
        g_qso_xmit = false;
        tx_start_immediate(skip_tones);
    }
}

// In decode_monitor_results (Event 2)
if (!g_was_txing) {
    if (!to_me.empty()) {
        autoseq_on_decodes(to_me);
    }

    AutoseqTxEntry pending;
    if (autoseq_fetch_pending_tx(pending)) {
        g_qso_xmit = true;
        g_target_slot_parity = pending.slot_id & 1;
    } else if (g_beacon != BeaconMode::OFF) {
        enqueue_beacon_cq();
        if (autoseq_fetch_pending_tx(pending)) {
            g_qso_xmit = true;
            g_target_slot_parity = pending.slot_id & 1;
        }
    }
}
```

## Audio Sample-Driven Timing (Critical Design Detail)

The reference project uses **audio sample timing** to drive all RX/TX symbol processing,
NOT wall clock timing. This is critical for proper synchronization.

### Reference Design: I2S-Driven Event Loop

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    AUDIO SAMPLE TIMING ARCHITECTURE                      │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  I2S DMA Interrupt (32kHz sample rate)                                   │
│  ├── Fires every 40ms (1280 samples per half-buffer)                    │
│  ├── Sets DSP_Flag in ISR                                               │
│  └── Main loop checks DSP_Flag                                          │
│                                                                          │
│  Symbol Timing (4 × 40ms = 160ms per symbol)                            │
│  ├── frame_counter increments on each DSP_Flag                          │
│  ├── When frame_counter == 2, send next TX tone                         │
│  └── 79 tones × 160ms = 12,640ms TX duration                           │
│                                                                          │
│  RX Symbol Counting                                                      │
│  ├── FT_8_counter increments per 160ms of audio                         │
│  ├── At 91 symbols (14,560ms), decode is triggered                      │
│  └── decode_flag is set for main loop                                   │
│                                                                          │
│  Slot Boundary (HAL_GetTick based, 15s intervals)                       │
│  ├── ft8_time = HAL_GetTick() - start_time                              │
│  ├── current_slot = ft8_time / 15000 % 2                                │
│  └── When slot changes: reset counters, call tick if was_txing         │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Key Insight: TX Duration is Guaranteed by Sample Counting

In the reference design:
1. **TX starts** when `setup_to_transmit_on_next_DSP_Flag()` is called
2. **Each DSP_Flag** (40ms) advances `frame_counter`
3. **Every 4 DSP_Flags** (160ms), a tone is sent
4. **After 79 tones**, TX automatically stops: `ft8_xmit_counter == 79`
5. **Slot boundary detection** happens AFTER TX completes because:
   - 79 tones × 160ms = 12,640ms
   - Slot boundary at 15,000ms
   - TX completes ~2,360ms before slot boundary

### Reference Code: TX Tone Timing

```c
if (DSP_Flag) {
    DSP_Flag = 0;
    I2S2_RX_ProcessBuffer(buff_offset);

    if (xmit_flag) {
        if (ft8_xmit_delay >= 28) {  // ~1.1s startup delay
            if ((ft8_xmit_counter < 79) && (frame_counter == 2)) {
                set_FT8_Tone(tones[ft8_xmit_counter]);
                ft8_xmit_counter++;
            }
            if (ft8_xmit_counter == 79) {
                ft8_receive_sequence();
                receive_sequence();
                xmit_flag = 0;
            }
        } else {
            ++ft8_xmit_delay;
        }
    }
}
```

### Why This Matters for Mini-FT8

**Problem with wall clock timing:**
- TX starts at slot boundary (t=0ms)
- `tx_tick()` uses `rtc_now_ms()` to check if 160ms elapsed
- Slot boundary detected at t=15000ms via wall clock
- BUT: Wall clock and audio timing can drift apart
- If TX hasn't sent all 79 tones when slot boundary hits, `tick()` is called too early

**Solution: Sample-Driven Timing**
- Count UAC audio samples instead of wall clock time
- 48kHz × 0.160s = 7,680 samples per symbol
- 79 symbols × 7,680 = 606,720 samples for full TX
- TX completion is guaranteed before slot boundary detection

### Mini-FT8 Implementation Options

**Option 1: Sample-Counted TX**
```cpp
// In UAC audio callback (fires every block of samples)
static int g_tx_sample_count = 0;
static int g_tx_current_tone = 0;

void on_audio_block(int samples_received) {
    if (g_tx_active) {
        g_tx_sample_count += samples_received;
        if (g_tx_sample_count >= 7680) {  // 160ms at 48kHz
            g_tx_sample_count -= 7680;
            send_next_tone();
            g_tx_current_tone++;
            if (g_tx_current_tone >= 79) {
                tx_complete();
            }
        }
    }
}
```

**Option 2: Block-Counted TX**
```cpp
// Block size = 1920 samples at 48kHz = 40ms per block
// 4 blocks = 160ms = 1 symbol
static int g_tx_block_count = 0;

void on_audio_block() {
    if (g_tx_active) {
        g_tx_block_count++;
        if (g_tx_block_count >= 4) {  // 4 × 40ms = 160ms
            g_tx_block_count = 0;
            send_next_tone();
        }
    }
}
```

## Summary: The Golden Rules

1. **Decode sets flags, slot boundary triggers TX** - Never start TX immediately after decode
2. **tick() at slot boundary AFTER TX** - Not during TX, not after decode
3. **CQ is short-lived** - Added when needed, transmitted once, popped by tick
4. **Queue[0] is always the active entry** - tick() and fetch_pending_tx() both use queue[0]
5. **Event 2 before Event 3** - Decode processing completes before TX decision is made
6. **Audio samples drive symbol timing** - RX/TX duration determined by sample count, not wall clock
7. **Slot boundary uses wall clock** - 15s intervals based on system tick/RTC

## Execution model on the ESP32: the owner task

The reference design is single-threaded, and as of 2026-08-28 the port is
too — for autoseq. Every call into `autoseq_*`, and every touch of the
armed-TX state (`g_pending_tx`, `g_qso_xmit`, `g_target_slot_parity`),
happens on the core-0 main loop. Nothing else may call in.

Before this, three tasks did: the decode task on core 1 applied decodes and
armed the TX, the main loop ticked and fired it, and the BLE RPC task
touched/dropped/cleared. There were no locks. The defences were ordering
flags (`g_was_txing`, `g_decode_in_progress`, `g_decode_applied_slot_idx`),
which are conventions rather than enforcement: `volatile` orders nothing
across cores on Xtensa, the `std::string`s inside `g_pending_tx` were copied
on one core while read on the other, and `g_decode_applied_slot_idx` was a
64-bit `volatile` whose read isn't even atomic on a 32-bit core. The
comments in `check_slot_boundary` record an actual race hit on hardware.

Now:

- **Decode task** builds the small `to_me` vector (pure work on its own
  `s_dec`) and posts it — always, even when empty — with the slot index.
- **BLE RPC task** posts touch / clear / drop / freetext / config commands.
  "ok" means *accepted*; the effect shows up in the next `QSO_QUEUE`
  snapshot. Posting never blocks and never waits.
- **Main loop** drains the queue at the top of `check_slot_boundary()`,
  applies decodes (the beacon block that `host_mock` mirrors), arms or
  re-arms, advances the decode barrier, ticks, fires. Single writer, single
  reader, for all of it.
- **Other tasks read a published snapshot.** After any change (autoseq's
  generation counter, or the armed-TX state), the owner copies the active
  contexts and the armed entry into `AutoseqOwnerSnapshot` under a small
  mutex; `core_qso_*` reads that. The mutex guards a *copy*, not autoseq.

This is also what makes the host harness honest: it was always
single-threaded, so it could model the state machine but never the races.
With the firmware single-threaded for autoseq, the harness's model and the
firmware's execution are the same shape.

Still outside this model, noted for a later pass: the BLE `tap_rx` path
reads the decode task's static RX list (`ui_get_rx_entry`) to build its
message, and station config globals (`g_call`, `g_grid`, `g_beacon`, …) are
written by the RPC task and read on the main loop. Both are pre-existing
and narrower than what this change closed.
