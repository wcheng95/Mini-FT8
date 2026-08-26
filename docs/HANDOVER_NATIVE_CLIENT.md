# Handover — Native Client (firmware → app)

Snapshot for resuming the native-client track on a different machine. Written
after the firmware-side work converged on a stable build; next phase is the
Flutter mobile client.

## Where things stand

- **Branch:** `native_app` (local-only — needs `git push -u origin native_app`
  before the new machine can pull it).
- **HEAD:** `fad5189 stream_uac: move usb_buffer to DMA_ATTR BSS`.
- **Target hardware:** primarily M5StampS3Bat (headless). M5Cardputer still
  builds and runs the same firmware as a debugging UI.
- **Firmware state:** the four-step native-service rollout (core_api.h →
  core_api.cpp → Cardputer UI migration → GATT server) is landed. Memory
  regressions that appeared during that rollout are all resolved; last build
  was field-verified by the user with BLE streaming running.
- **Text-terminal BLE service:** removed (commit `f4898a2`). Reconsidered
  after Wei noted the text path could carry configs; decision (user
  confirmed) is to stay native. Reasoning: binary waterfall stream, typed
  events, per-characteristic QoS choices.

## Authoritative specs (don't duplicate — read these)

| File | Role |
|---|---|
| `main/ble_native.h` | UUIDs, RPC names, event tags, `BleRadioStreamHeader` wire format. **Bump `BLE_NATIVE_VERSION` on any wire change.** |
| `main/core_api.h` | Functional-core API — all types/enums/commands the server and UI consume. |
| `docs/NATIVE_CLIENT_ARCHITECTURE.md` | Design rationale (notify+pull, JSON choice, what's NOT in core_api). |
| `main/ble_native.cpp` | GATT server, RPC dispatcher, waterfall/ADIF TX task. |

## GATT surface at a glance

Service UUID: `F1A4D100-0001-4001-A001-000000000001`. Characteristic UUIDs
differ only in the last byte (see `ble_native.h`):

| NN | Char | Role |
|---|---|---|
| 02 | EVENTS | notify — JSON `{"e":"rx"\|"qso"\|"config"}` tells client to re-read |
| 03 | RX_LIST | read — JSON snapshot of recent decodes |
| 04 | QSO_QUEUE | read — JSON snapshot of queued QSOs + next TX |
| 05 | CONFIG | read — JSON station config |
| 06 | RADIO_STREAM | notify — binary waterfall rows (8-byte header + N bins) |
| 07 | RPC_REQ | write — JSON `{"id":n,"cmd":"…","args":{…}}` |
| 08 | RPC_RESP | notify — JSON `{"id":n,"ok":bool,"err":"…"}` |
| 09 | ADIF_STREAM | indicate — binary ADIF file bytes; zero-length = EOF. `adif_list` enumerates day-logs, `adif_open {d,off}` resumes at a byte offset for incremental sync. See NATIVE_CLIENT_ARCHITECTURE.md. |

Flow: subscribe to EVENTS + RPC_RESP, read RX_LIST / QSO_QUEUE / CONFIG once,
re-read on the matching event tag, subscribe to RADIO_STREAM for live
waterfall, subscribe to ADIF_STREAM only around `adif_open` RPCs.

## Known stubs / deferred work on the firmware

- **SWR / PWR / PTT in RADIO_STREAM header are placeholder constants**
  (1.5 / 2.0 / RX). The header fields exist on the wire; the values aren't
  real yet. See `main/stream_uac.cpp:push_waterfall_latest` and
  `main/core_api.cpp kStubSwr/kStubPwr`. Real polling will need CAT reads
  from the QMX (or KH1) on a low-priority timer — not blocking decode.
- **End-to-end mobile validation is still pending** — GATT surface has been
  poked with Bluetility/nRF Connect, but no real client has exercised the
  full EVENTS → re-read loop under load.

## Next phase: Flutter client

Goals (from `docs/NATIVE_CLIENT_ARCHITECTURE.md`):

- iOS + Android, single Flutter codebase.
- Modern native UI — not a terminal mirror. Real list/detail screens for
  RX list, QSO queue, config form, live waterfall.
- Thin viewer: node owns all state. On disconnect/reconnect, re-read the
  three snapshots.

Suggested first steps once set up on the new machine:

1. `flutter create` an app shell with `flutter_blue_plus` (or whichever
   BLE package is still maintained best when you start).
2. Build a `MiniFt8Client` class that encapsulates: connect, MTU request
   (aim for ≥247 so 441-byte waterfall rows fit), subscribe to EVENTS /
   RPC_RESP / RADIO_STREAM, `readSnapshot('rx'|'qso'|'config')`,
   `rpc(cmd, args)` returning a `Future<Map>` keyed by request id.
3. Smallest useful screen: live RX list. Validates EVENTS → re-read loop.
4. Waterfall render second — uses the binary header, no JSON on the hot
   path.

Keep in mind:

- JSON is `cJSON` on the firmware, `dart:convert` on the client.
- Wire format assumes little-endian (the ESP32 is LE); decode accordingly.
- `BLE_NATIVE_VERSION` in `ble_native.h` is the compatibility marker —
  mirror that constant on the client and reject mismatched majors.

## Machine transfer checklist

On the old machine (this one):

```
git push -u origin native_app
```

On the new machine:

```
git clone <repo>
git checkout native_app
git submodule update --init --recursive   # ft8_lib + any m5 deps
```

ESP-IDF toolchain — confirmed working version from the current builds:
ESP-IDF **v5.5.1** (see boot log prefix in any recent session). Install via
`$IDF_PATH/install.sh` then `. $IDF_PATH/export.sh`. Target is
`esp32s3`.

Build / flash sanity check before starting Flutter work:

```
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

A clean boot log should show:
- `UAC_AFTER_FFT_ALLOC: HEAP: free=… largest=…` followed by successful
  stream task start (no "Buffer allocation failed").
- `BLE_NATIVE: …` lines when a client connects.

If the build errors out on missing components, check
`main/CMakeLists.txt` REQUIRES includes `json` (cJSON) and the ft8_lib
component path under `components/ft8_lib/`.

## Stray files in the working tree at handover

Not tracked, not relevant to the client work, left alone intentionally:

- `20260413.txt`, `RT260411.TXT`, `RT260413.txt` — user's on-air logs.
- `concrete-semantics.pdf`, `semantics.txt` — unrelated reading material.
- `sdkconfig.old` — backup from a UART pin experiment.

These aren't sensitive but aren't worth committing either.

## Fresh-Claude orientation (read this first when resuming)

- Project: Mini-FT8, an ESP32-S3 (M5StampS3Bat / Cardputer) automatic FT8
  QSO maker. User is N6HAN.
- You (Claude) are picking up mid-project. Firmware is landed on branch
  `native_app`; the next phase is a Flutter iOS/Android client that talks
  to the GATT service above.
- Read `docs/NATIVE_CLIENT_ARCHITECTURE.md` for the why, `main/ble_native.h`
  for the wire format, and `main/core_api.h` for the state model. Those
  three files define the contract — everything else is implementation.
- User collaboration preferences are captured in the auto-memory system
  (`MEMORY.md`); honor them.
