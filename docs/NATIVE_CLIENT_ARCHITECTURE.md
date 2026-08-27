# Native Client Architecture

Design rationale for the headless-node + mobile-app direction. Complements
`main/core_api.h` (the actual API surface) by explaining *why* the pieces
look the way they do.

## Goals

1. Run Mini-FT8 on a **headless ESP32-S3** node (e.g. StampS3Bat) with no
   local display, controlled entirely from a phone.
2. Offer a **modern native-feeling mobile UI** that doesn't mirror the
   6-line, text-only Cardputer layout.
3. Keep **one source of truth** for all station state on the node; the
   mobile app is a thin viewer/controller, not a second brain.
4. Support the Cardputer local UI and the mobile UI **simultaneously**
   without each knowing about the other.

## Non-goals

- No offline operation on the phone. If the phone disconnects, the node
  keeps operating; when it reconnects it re-reads a snapshot.
- No cross-device sync (single node ↔ single phone).
- Not a transport-agnostic abstraction. First transport is BLE. A WiFi
  transport can be added later; the core API is transport-neutral but
  the BLE profile is BLE-specific.
- No on-device UI framework portability — the Cardputer UI remains M5GFX.

---

## System topology

```
                           ┌───────────────────────┐
                           │ Mini-FT8 node (ESP32) │
                           │                       │
    Flutter mobile app ─────► BLE server           │
    (iOS + Android)          │      │              │
                             │      ▼              │
                             │  ┌─────────────┐    │
       Cardputer local UI ───┼─►│   core API  │◄───┼─ autoseq / decode /
       (M5GFX renderer)      │  └─────────────┘    │   DSP / radio_control
                             │                     │
                             └─────────────────────┘
```

`core_api.h` is the single functional-core boundary. Everything above it
(Cardputer UI, BLE server) is a **consumer**. Everything below it
(autoseq, decode, DSP, radio IO) is **implementation**.

### Why a core API at all?

Today, the Cardputer UI reaches directly into globals (`g_rx_lines`,
`g_bands`, `g_beacon`, `autoseq_queue_size()`, etc.) and mutates them
directly. This works for one consumer but doesn't compose — you can't
plug a second consumer (BLE server) in without one of:

- Duplicating the access logic → two code paths to keep in sync
- Having the BLE server mutate the same globals → concurrency nightmares
- Serializing everything into text and routing through the existing
  terminal protocol → forces the mobile UI to look like a terminal,
  which is exactly what we're trying to escape

Introducing `core_api.h` resolves all three:
- Both consumers call the same functions → one code path
- Mutations happen through guarded commands → concurrency is local to
  the implementation, invisible to consumers
- The mobile client is free to present state however it wants

---

## Data-flow model: notify + pull

All stateful information follows the same pattern:

```
┌──── State ────┐         ┌─── Consumer ──┐
│   (in core)   │────────►│   (e.g. BLE   │
└───────────────┘  notify │    server)    │
        ▲                  └───────┬───────┘
        │                          │
        │  core_get_*() ◄──────────┘
        │                      snapshot pull
        │
        │  core_cmd_*()
        │
    ┌───┴───────────────┐
    │     Consumer      │
    └───────────────────┘
```

1. Consumer calls `core_get_*()` once at startup — initial snapshot.
2. Consumer registers a change callback via `core_on_*_changed()`.
3. When state mutates, core fires the callback.
4. Consumer responds by calling `core_get_*()` again and re-rendering.

### Why notify + pull, not push-with-payload?

Push-with-payload would mean: callback receives the new state as an
argument (`cb(const StationConfig&)`), so the consumer can render
directly without a pull.

We chose **notify + pull** because:

- **Single serialization point**: with pull, only the consumer decides
  format (Cardputer = in-memory structs, BLE = JSON bytes). With push,
  core has to copy/pack state once per callback, whether needed or not.
- **Coalescing**: if a burst of 5 updates happens, the consumer only
  re-reads once after the last callback. Push would deliver 5 full
  payloads.
- **Parity with Cardputer's existing dirty-flag idiom**: today's
  `g_rx_dirty = true` + later snapshot read is exactly this pattern,
  just without function names. The refactor is surgical, not disruptive.
- **Lossy BLE tolerance**: GATT `notify` is unreliable. If a
  notification is dropped, the consumer stays stale until the next
  change triggers another notify. Because every state has an
  eventually-consistent self-repair (RX updates every 15 s, radio every
  ~160 ms, config on user action), no special retry/ack logic is
  needed — the next notification always reconciles.

### The one exception: waterfall

Waterfall is **streaming**, not stateful. There's no "current waterfall"
to snapshot — data only exists at the instant it's produced. The
`core_on_waterfall_row` callback carries the row directly because:
- No snapshot semantics exist
- The receiver always wants the latest row, not the most recent
  historical one
- Rate is high (~6 Hz) so double-dispatch overhead matters

---

## Thread-safety contract

Mutations originate from two task contexts:

| Task | Mutations it triggers |
|---|---|
| `app_task_core0` (main UI loop) | Config changes, RX taps, manual TX commands, band change |
| `stream_uac_task` (audio decode) | RX list updates (new decode), waterfall rows |

Consumers run on their own tasks:

| Consumer | Task |
|---|---|
| Cardputer UI | `app_task_core0` |
| BLE server | NimBLE host task + a BLE TX helper task |

Rules:

1. **All `core_get_*()` accessors are thread-safe.** Implementation uses
   a mutex per state domain; short critical sections, no blocking I/O.
2. **Callbacks fire in the task that caused the mutation.** A config
   change from the Cardputer UI calls your callback on the main task; a
   new decode calls it on the audio task.
3. **Callback handlers must be trivial.** No filesystem I/O, no BLE
   sends, no blocking. The expected pattern is: set a flag or enqueue a
   message on your own task's queue, return immediately.
4. **Commands (`core_cmd_*()`) are reentrant.** Internally they may
   acquire the same mutex your callback released; ordering doesn't
   matter because commands finish their mutation before returning.

The main reason for rule 3: if a consumer's callback blocks, it stalls
the task that produced the mutation. An audio-task stall drops decode
samples. A main-task stall freezes the Cardputer UI.

---

## BLE transport layer (v1)

The BLE profile is an implementation detail above `core_api`, but it's
tightly enough designed to be worth documenting here.

### GATT characteristics

| Name | Dir | Mode | Purpose |
|---|---|---|---|
| `EVENTS` | S→C | notify | Change event pings (`{"e":"rx"}`, etc.) |
| `RX_LIST` | S | read | Snapshot: decoded messages |
| `QSO_QUEUE` | S | read | Snapshot: active QSOs + next TX |
| `CONFIG` | S | read | Snapshot: station config (incl. band) |
| `RADIO_STREAM` | S→C | notify | Waterfall row + swr/pwr/ptt (lossy) |
| `RPC_REQ` | C→S | write | Typed command request (JSON) |
| `RPC_RESP` | S→C | notify | Per-request response (JSON) |
| `ADIF_STREAM` | S→C | indicate | Chunked reliable log download |

### Worked-QSO log

Logs live one file per UTC day at `/storage/YYYYMMDD.txt`, written by
`log_adif_entry()`. Reading them back goes through **one parser**, in
`core_api.cpp` — both the Cardputer viewer and the BLE server call it, so the
two cannot drift about what a record means. The viewer used to carry its own
field scanner; that scanner ended values at the first space, which truncated
any value containing one (the default comment is `MiniFT8 /Radio`).

| RPC | Args | Response |
|---|---|---|
| `log_days` | — | `{"n":<total days>,"d":["YYYYMMDD:<entries>",…]}` newest first |
| `log_read` | `{"d":"YYYYMMDD","from":N,"n":M}` all optional | `{"n":<will send>,"total":<that day>}` |
| `log_abort` | — | stops an in-flight stream |

`log_read` then streams that many entries over `LOG_STREAM`, **one entry per
indication**; a zero-length indication ends it. An ADIF record has a schema
and a bounded size, so each entry fits a single MTU — no chunk index, no
reassembly on the client, and nothing for a client to re-parse.

```json
{"i":0,"t":"142530","c":"W1ABC","g":"FN42","m":"FT8",
 "f":"14.074","b":"20m","s":-12,"r":-8,"x":"MiniFT8 /Radio"}
```

`s`/`r` are omitted when the node logged no report, keeping "absent" distinct
from a real 0 dB.

**Entry indices are the cursor.** A day file is append-only, so an entry's
index never changes once written. A client holding entries 0..N-1 asks for
`from = N` and receives only what was logged since. Indices are stable across
node reboots and immune to clock changes — which byte offsets also are, but
timestamps are not, and the node's RTC gets pushed from the phone and GPS.

`d` is capped (`kLogDaysMax`, currently 10) so the response fits the 256-byte
RPC buffer; `n` carries the true count so truncation is detectable.

Two ordering constraints:

- **Subscribe to `LOG_STREAM` before `log_read`.** The firmware only pumps
  while a subscriber exists and starts as soon as the RPC returns.
- **One stream at a time.** A second `log_read` answers `log busy` until the
  first finishes, is aborted, or the link drops. Unlike a file handle there is
  nothing to leak: `core_log_read` opens and closes per batch.

### Why 8 characteristics, not one

We could multiplex everything through one characteristic with a type
tag. Per-stream characteristics instead because:

- **Per-stream QoS**: waterfall uses `notify` (lossy); ADIF uses
  `indicate` (reliable with per-chunk ack). Can't mix modes on one
  characteristic.
- **Per-stream subscription**: a client that only wants the radio
  stream (e.g. a waterfall-only viewer) subscribes to one
  characteristic and ignores the rest. The firmware doesn't send what
  nobody's listening to.
- **Per-stream sizing**: `CONFIG` is ~500 bytes (fine as a single
  GATT read). `RADIO_STREAM` rows are ~640 bytes each at 6 Hz. `ADIF`
  can be hundreds of KB chunked.
- **Clearer wire semantics**: the characteristic UUID *is* the event
  type. No discriminator byte to manage.

### Wire format: JSON

All characteristics exchange UTF-8 JSON. Binary blobs (waterfall
magnitudes, ADIF file bytes) travel as base64-encoded strings inside
JSON.

Rationale:
- **One format to parse on both sides** (cJSON on ESP-IDF, dart:convert
  on Flutter — both already available).
- **Readable in logs** — every hex-dump is self-explanatory during
  debugging.
- **Extensible**: adding a field never breaks an older client; the
  client ignores unknowns.
- **Within bandwidth budget**: at 6 Hz waterfall + 1 Hz radio status
  + occasional RX/QSO/RPC traffic, total is ~4 KB/s, well under
  BLE LE 1M practical throughput (~25 KB/s).

If any single stream outgrows JSON, its characteristic can be swapped
to packed binary independently without touching the rest of the
profile — that flexibility is the other reason for multiple
characteristics.

### Lossy vs. reliable

- **`notify`** (lossy, fire-and-forget): used for `EVENTS`,
  `RADIO_STREAM`, `RPC_RESP`. Loss is tolerated because:
  - `EVENTS` is self-repairing (next state change re-pings)
  - `RADIO_STREAM` is a continuous stream; a dropped row means a
    one-pixel gap in the waterfall
  - `RPC_RESP` is paired with a client-side timeout per RPC
- **`indicate`** (reliable, per-packet ack): used for `ADIF_STREAM`.
  Losing log bytes would be permanent data loss.
- **`read`** (on-demand pull): used for all snapshots. The GATT
  layer handles fragmentation if the payload exceeds MTU.

### The "event ping" pattern

The `EVENTS` characteristic carries only type tags:

```json
{"e":"rx"}
{"e":"qso"}
{"e":"config"}
```

It does not carry the new state. The client reacts by reading the
matching snapshot characteristic. This is a deliberate two-round-trip
design:

- Avoids the server having to serialize a potentially-large payload
  on every mutation (RX list can be 3 KB+).
- Makes coalescing trivial: if five RX updates happen in rapid
  succession, the client only reads once after the latest event.
- Decouples mutation-side cost from payload-side cost.

The tradeoff is latency (one extra round trip). At BLE 50 ms connection
intervals this is imperceptible to a human user.

---

## Why Flutter on the client

Considered alternatives:
- **Native SwiftUI + Jetpack Compose**: two codebases, two implementations
  of every screen, easy look-and-feel divergence, more maintenance.
- **React Native**: JavaScript runtime overhead, less reliable BLE
  support, non-native look on both platforms.
- **Progressive Web App**: Web Bluetooth has patchy iOS support.

Flutter wins on:
- **Single codebase** for iOS + Android.
- **Pixel-identical rendering**: Flutter paints its own widgets via
  Skia/Impeller, so the waterfall looks the same on both platforms.
- **Mature BLE ecosystem**: `flutter_blue_plus` handles iOS/Android
  BLE quirks (bonding, MTU negotiation, notify subscription).
- **CustomPainter** for the waterfall: efficient pixel-level rendering.

The one cost is Flutter's non-native feel. Mitigated by using the
Material 3 / Cupertino adaptive widgets where the style matters
(buttons, sheets, switches); the custom waterfall and decode list are
visibly app-specific anyway.

---

## Evolution guidelines

### Adding a new config field

1. Add the field to `StationConfig` in `core_api.h`.
2. Add a typed setter (`core_cmd_set_<field>`) in the same header.
3. Implement it in `core_api.cpp` (updates underlying state, calls
   `save_station_data()`, fires `on_config_changed`).
4. Serialize the new field in the `CONFIG` characteristic's JSON.
5. Handle the new JSON key in the Flutter config screen.

Old clients that don't know the new field just ignore it. No version
negotiation needed.

### Removing or renaming a field

Deprecate first: keep the old name working, add the new name, mark the
old as deprecated in comments. When all known clients have been updated
(check Flutter app version in the wild), remove the old name and bump
`core_api_version()` major.

### Changing wire format on one characteristic

Can be done without disturbing others. Bump `core_api_version()` minor.
The Flutter app should already read the version on connect and fall
back gracefully or refuse to connect if incompatible.

### When to bump `core_api_version()`

- **Major**: incompatible change (removed field, changed semantics,
  changed RPC contract).
- **Minor**: additive change to any GATT characteristic's JSON.
- **Patch**: bug fix or non-wire internal change.

The version is exposed through the `CONFIG` snapshot and as
`core_api_version()`, so clients can gate features on it.

---

## What's NOT in core_api (and why)

- **`enter_mode(UIMode::RX)` and friends**: "modes" are a Cardputer UI
  artifact — a 6-line display needs page navigation. The BLE client
  has tabs/sheets that map nowhere onto this. Leaving it out keeps the
  core API genuinely UI-agnostic.
- **Page numbers, scroll positions, selection indices**: purely
  consumer-local rendering state.
- **Sleep / reboot**: the StampS3Bat has no independent battery and we
  aim for code that doesn't need a restart-as-fix.
- **Host USB-serial protocol (CONTROL mode)**: legacy, kept for the
  Cardputer's debugging workflow. Not exposed through core_api.
- **Detailed error reporting**: commands return plain `bool`. Firmware
  and Flutter app are co-developed and tightly coupled; structured
  errors aren't worth the complexity cost at this stage.
