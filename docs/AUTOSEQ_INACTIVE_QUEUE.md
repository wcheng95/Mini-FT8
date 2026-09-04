# Autoseq Inactive Queue: Preserving QSO Metadata Across Retry Exhaustion

## Problem Statement

The autoseq engine stores all per-QSO metadata (dxcall, dxgrid, snr_tx, snr_rx,
logged, is_fd) in queue entries (`QsoContext`). When a context exhausts its retries,
it transitions to IDLE and is popped from the queue, destroying all metadata.

If DX is more patient than us and continues retrying, their message creates a
"reincarnated" context with default values (snr_tx=-99, logged=false). This
reincarnated context can complete the QSO and log an ADIF entry with wrong metadata.

### The Concrete Bug: REPORT + TX3 Reincarnation

```
T=0:00   We send TX2 (rst_sent=-12), retry 1
T=0:30   DX sends TX3 (R-08) — we don't decode (QRM)
T=1:00   We send TX2, retry 2
T=1:30   DX sends TX3 — we don't decode
  ...
T=2:30   Retries exhausted → IDLE → pop. snr_tx=-12 is gone forever.
T=3:00   DX sends TX3 — propagation improves, we decode!
         → on_decode: no matching ctx → new ctx (snr_tx=-99)
         → REPORT+TX3 → ROGERS → log_qso_if_needed
         → ADIF entry has rst_sent=-99 ← WRONG, no prior correct entry exists
```

### Root Cause

Our retry window (~2.5 min) can be shorter than DX's retry window (WSJT-X: up to
7+ min with 15 retries). After our context is popped, DX is still actively working
the QSO. The queue's lifetime is bounded by OUR retry limit, but the QSO's real
lifetime is bounded by DX's.

### Safety Invariant Analysis

Every pop from the queue falls into one of these cases:

| State at exhaustion | Logged? | DX received our TX | DX did NOT receive our TX |
|---|---|---|---|
| CALLING (CQ) | No | DX sends TX1 → normal new QSO | Nothing happens |
| REPLYING (TX1) | No | DX sends TX2 → new ctx, fresh exchange, consistent | DX resends CQ → not addressed to us |
| REPORT (TX2) | No | DX sends TX3 → new ctx, **snr_tx=-99 (BUG)** | DX resends TX2/TX1 → new ctx, snr_tx from msg.snr, OK |
| ROGER_REPORT (TX3) | No | DX sends TX4 → signoff guard filters it | DX resends TX2 → new ctx, fresh exchange, OK |
| ROGERS (TX4) | **Yes** | DX sends TX4/TX5 → signoff guard filters it | DX resends TX3 → **duplicate ADIF, snr_tx=-99** (original correct entry exists) |
| SIGNOFF (TX5) | **Yes** | QSO done | DX sends TX4/TX5 → signoff guard filters it |

The **REPORT + DX received** row is the critical gap: no prior correct ADIF entry
exists, and the reincarnated context has lost snr_tx.

## Solution: Active + Inactive Queue

Instead of popping contexts when retries exhaust, move them to an **inactive zone**
at the back of the queue. The single `s_queue[]` array serves two logical partitions:

```
s_queue[0 .. s_active_count-1]   → Active zone: has retries, gets scheduled by tick()
s_queue[s_active_count .. s_queue_size-1] → Inactive zone: retries exhausted, metadata preserved
```

### Key Design Rules

1. **Retry exhaustion → inactive, not IDLE.** When tick() exhausts retries, the
   context is marked `inactive = true` and moved to the back via sort. It is NOT
   popped. All metadata (snr_tx, snr_rx, dxgrid, logged, is_fd) is preserved.

2. **on_decode finds inactive contexts.** The existing dxcall-matching loop in
   on_decode() already scans the full queue. When DX retries, the dormant context
   is found, generate_response() advances its state, `inactive` is cleared, and
   it moves back to the active zone with fresh retries.

3. **Eviction only when safe.** A context can be evicted (popped) only if:
   - `logged == true` — ADIF has the correct entry, safe to discard
   - State is CALLING or REPLYING — no reports exchanged yet. CALLING is a bare
     CQ. REPLYING means we only sent TX1 (grid); if DX retries with TX2, a fresh
     context gets correct snr_tx from msg.snr, so nothing valuable is lost.
   - Contexts in REPORT or beyond (sent TX2+) must be preserved — they carry the
     snr_tx we actually transmitted, which would be lost on eviction.

4. **Reject only R+report (TX3) from unknown DX.** If DX sends TX3 (R+report)
   and has no context in either active or inactive zone, reject it — TX3 skips
   the snr_tx latch, producing wrong ADIF metadata. All other message types are
   allowed for new context creation:
   - TX1 (grid): new QSO start, correct metadata
   - TX2 (report): DX skipped TX1 (WSJT-X "skip TX1" feature), snr_tx set from
     msg.snr, snr_rx from report text — correct metadata
   - FD exchange: equivalent to TX2 in Field Day mode — correct metadata
   - TX4/TX5 (RR73/73): filtered separately by signoff guard

5. **Time-based expiry.** Inactive contexts expire after a configurable window.
   Expired inactive contexts are evicted regardless of logged state — if we haven't
   heard from DX in that long, the QSO is truly dead.

### Queue Layout: Dynamic Boundary

The active and inactive zones share a single array and grow from opposite ends.
No fixed partition — both zones expand until their pointers meet.

```
s_queue[]:
 index 0                                               index AUTOSEQ_MAX_QUEUE-1
 ┌──────────────────────┬────────────┬──────────────────────┐
 │   Active zone        │  (free)    │   Inactive zone      │
 │   grows →            │            │            ← grows   │
 └──────────────────────┴────────────┴──────────────────────┘
       s_active_count ──┘            └── s_inactive_start
```

- `s_active_count`: number of active entries at the front (grows right)
- `s_inactive_start`: index of first inactive entry at the back (grows left)
- Free space: `s_inactive_start - s_active_count`
- When both pointers meet (free space = 0), queue is full:
  evict the oldest inactive entry (rightmost) to make room.

Queue size: **120 entries.** This accommodates a busy 1-hour activation.
QsoContext is ~80 bytes with std::string SSO, so 120 entries ≈ 10 KB —
negligible on ESP32-S3 (512 KB SRAM).

### Sort Order

Active entries are sorted by state priority (same as before):
```
  IDLE > SIGNOFF > ROGERS > ROGER_REPORT > REPORT > REPLYING > CALLING
```

Inactive entries are stored at the back in insertion order (oldest at the
rightmost end). When eviction is needed, the oldest (rightmost) entry is
removed first.

### Modified tick() Behavior

```
Before:  retry exhausted → state = IDLE → pop_front()
After:   retry exhausted + state > REPLYING → move to inactive zone at back
         retry exhausted + state <= REPLYING → evict (no reports exchanged)
```

tick() only processes the front of the active zone.

### Modified on_decode() Behavior

```
Before:  no matching ctx + not signoff → append_ctx + generate_response(override=true)
After:   no matching ctx → check message type:
           TX4/TX5 (RR73/73): reject (signoff guard, as before)
           TX3 (R+report): reject (snr_tx would be -99, reincarnation bug)
           TX1/TX2/FD exchange: allow (fresh context gets correct metadata)
```

### Reincarnation Eliminated

The reincarnation problem disappears because:
- Dormant contexts stay in the queue with all metadata
- on_decode finds them by dxcall, reactivates them
- Mid-QSO messages from unknown DX are rejected (no memory-less reincarnation)
- The only new contexts created are for TX1 (fresh QSO starts) where default
  metadata is correct

## Invariant: Active entries always have a valid next_tx

The autoseq engine maintains this invariant for every active queue entry:

> **If `ctx` is in the active zone and `state != IDLE`, then `next_tx != TX_UNDEF`.**

This invariant is what lets `autoseq_fetch_pending_tx()` safely return the TX
at `s_queue[0]` without checking for degenerate states. Violating it causes a
hard deadlock: `fetch_pending_tx` returns false, no TX fires, `tick()` never
runs to advance the state, and the dead entry blocks every subsequent TX —
including beacon CQs sorted behind it.

### How the invariant was maintained (pre-inactive-queue)

1. `set_state()` is the only legitimate way to change `state`, and it always
   writes `next_tx` at the same time.
2. The override path in `generate_response()` sets `next_tx = TX_UNDEF` as a
   "clean slate" but is always immediately followed by a state machine
   transition that restores a valid `next_tx`.
3. `autoseq_tick()` refreshes `next_tx` idempotently based on `state` on every
   call (within retry limit).
4. For an existing active ctx passed to `generate_response(override=false)`,
   the invariant holds by induction: if the state machine transitions, the new
   `set_state` maintains it; if it falls to `default`, `next_tx` was already
   valid from the prior tick or transition.

### How the inactive queue almost broke it

`move_to_inactive()` clobbers `next_tx = TX_UNDEF` before parking an entry.
When DX retries and `on_decode()` reactivates the dormant context, the
reactivated entry lands in the active zone with `next_tx == TX_UNDEF`.
`generate_response(override=false)` then runs the state machine; if the
(state, rcvd) combination falls to `default: return false`, `next_tx` is
*never* restored. The active zone now contains an entry violating the
invariant, and the queue deadlocks.

### Fix: make `generate_response` locally total

Every state handler in `generate_response()` must explicitly handle every
possible `rcvd` type — there is no implicit `default` that silently leaves
`next_tx` stale. For `(state, rcvd)` combinations that don't advance the QSO,
the handler refreshes `next_tx` to the state's canonical value and returns
false. The canonical mapping:

| State | Canonical next_tx | Meaning |
|---|---|---|
| CALLING | TX6 | Send CQ |
| REPLYING | TX1 | Send our grid |
| REPORT | TX2 | Send our report |
| ROGER_REPORT | TX3 | Send R+report |
| ROGERS | TX4 | Send RR73 |
| SIGNOFF | TX5 | Send 73 |

The semantics of every "no-op" case is identical: **"DX sent something that
doesn't advance our state — they probably didn't decode our last message.
Keep sending what matches our current state."** Encoding this in the
switch/case (rather than a centralized fallback) makes every state handler
self-contained and locally reasonable — when you read the REPORT case, you
see exactly what happens for every possible input, including the ones that
don't advance the QSO.

The invariant is restored even across the inactive queue boundary: if
`reactivate()` brings back an entry with stale `next_tx`, the very next call
to `generate_response()` repairs it unconditionally.

**Note on CALLING (CQ):** The CALLING state handler does NOT refresh `next_tx`
in its default case, because CALLING is unreachable via reactivation by
construction (CQ ctxs have dxcall="CQ", never enter the inactive zone, and
the override path always transitions immediately). See inline comment in
`generate_response()` for the full proof.

### Post-mortem 2026-08-28: the theorem had a hole

RT260828, 20:10:30–20:28:32 UTC: after VE7MGU's retries ran out, the node
went silent for 18 minutes with the beacon on, until the operator re-toggled
it. The reconstruction (host_mock/test_contest_r_grid.json replays it):

1. VE7MGU was in a grid-exchange contest mode and acknowledged our report
   with `R CN89` — a legitimate TX3 the classifier didn't know, so every
   ack read as `TX_NONE` and we retried TX2 until exhaustion.
2. Exhausted → `move_to_inactive` (next_tx := TX_NONE). VE7MGU called
   again → `reactivate()` → `generate_response()` → parse → `TX_NONE` →
   **early return, above the state switch**. The "repairs it unconditionally"
   sentence above was true of the switch and false of the function: the
   one input that skips the switch is exactly the input a station that
   can't be parsed keeps sending.
3. Dead head at queue[0] (REPORT, next_tx=TX_NONE): the buffer is empty,
   the beacon CQ is appended behind it, `refresh_tx_msg_buffer` only looks
   at queue[0], tick never runs because nothing transmits. Permanent.

Fixes: `R <grid>` classifies as TX3; the `TX_NONE` early return now
refreshes `next_tx` to the canonical value first (`canonical_next_tx`);
and `on_decode` only reactivates a parked context for a message it can act
on — waking on noise would also reset the retry counter and let a station
that never decodes us drive an unbounded retry storm. The harness now
asserts two invariants on every period: an active QSO head is schedulable,
and a beacon never goes silent on its own parity. It also prints the same
`Q` queue-snapshot lines the firmware now writes to the RxTx log, so the
next one of these is read off the log rather than reconstructed.

## Formal design notes

### Orthogonality of logical position and scheduling state

A QsoContext has two independent coordinates:

- **Logical position** (`state`): where we are in the FT8 protocol exchange.
  Maps to the conversational order TX6 → TX1 → TX2 → TX3 → TX4 → TX5.
  Active ctxs always have a well-defined position in {CALLING..SIGNOFF}.

- **Scheduling state**: whether the ctx is actively participating in TX
  scheduling. Three modes: active (in active zone, `tick()` processes it,
  `fetch_pending_tx` can select it), inactive (in inactive zone, dormant,
  preserving metadata for reactivation), or eliminated (popped from queue).

These two dimensions are **orthogonal**. A ctx at logical position REPORT can
be active (retrying TX2) or inactive (parked after retry exhaustion). Its
logical position doesn't change when it moves between active and inactive —
only its scheduling state does. When reactivated, it resumes at the same
logical position with the same metadata.

### Semantics of TX_UNDEF

`TX_UNDEF` in the `next_tx` field means **"this context has no TX to schedule
and should be eliminated from the active zone on the next cleanup."** It is a
scheduling-layer tombstone, not a logical position.

Specifically: `(next_tx == TX_UNDEF) ⇔ (state == IDLE, about to be popped)`.

TX_UNDEF is NOT a protocol position. It should never appear on an active ctx
with state != IDLE. The idempotent `generate_response()` state machine
enforces this by refreshing `next_tx` to the canonical value for the current
state whenever the state machine runs.

**Pragmatic exception:** `autoseq_start_cq()` sets `next_tx = TX6` on CALLING
ctxs so that CQ text can be generated through the same `format_tx_text()`
path as QSO messages. This reuses the TX-generation pipeline uniformly for
all message types, including the broadcast initiator (CQ). CALLING ctxs are
one-shot (tick immediately transitions to IDLE → pop) and never retried or
moved to inactive, so this does not violate the scheduling semantics of
TX_UNDEF.

### Future direction: TLA+ formalization

The current switch/case state machine encodes both protocol progression rules
and operational compliance heuristics (e.g., not logging QSOs without report
exchange, handling DX intent for liveness). A future TLA+ specification would
formalize these as:

- **Safety**: the next_tx invariant, the ADIF logging correctness invariant
- **Liveness**: every QSO with a responsive DX eventually completes
- **Fairness**: round-robin retry ensures all active QSOs make progress

The signed-distance metric (Δ = dx_pos − our_pos in conversational order)
provides a candidate simplification: advance when 0 ≤ Δ ≤ 2, stay when
Δ < 0, reject when Δ > 2. However, the current explicit rules also encode
operational nuances (e.g., ethical compliance, DX intent inference) that
would need to be captured as additional TLA+ constraints before the
distance-based formulation can fully replace the case-by-case rules.
