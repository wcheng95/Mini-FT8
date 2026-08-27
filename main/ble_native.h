#pragma once

// ============================================================================
// ble_native.h
//
// GATT profile + wire-format constants for the Mini-FT8 "native client"
// protocol. This is a second BLE service that runs alongside the existing
// text-terminal service — a modern native UI (Flutter) talks to it via
// structured reads / writes / notifications.
//
// Design rationale: see docs/NATIVE_CLIENT_ARCHITECTURE.md
// Functional-core surface: see main/core_api.h
//
// The constants below are mirrored by the Flutter client; any change here
// requires a matching Flutter-side update. Bump the version byte on any
// wire-incompatible change.
// ============================================================================

#include <cstdint>
#include <cstddef>

// ---------------------------------------------------------------------------
// Protocol version — bump major on wire-incompatible change.
// ---------------------------------------------------------------------------
#define BLE_NATIVE_VERSION "1.1.0"

// ---------------------------------------------------------------------------
// UUIDs
//
// Base: F1A4D100-0001-4001-A001-0000-0000-0000-NN
// Unique per-chip for Mini-FT8 native API. Characteristics differ only in
// the last byte (NN) so the set groups cleanly in scanner UIs.
// ---------------------------------------------------------------------------

// In NimBLE, 128-bit UUIDs are stored little-endian. These arrays are the
// byte representation, LSB-first.
//
// String form:           F1A4D100-0001-4001-A001-000000000001 (service)
// Byte form (LE):  01 00 00 00 00 00 01 A0 01 40 01 00 00 D1 A4 F1

// Comma-separated 16 bytes (LSB-first). The NimBLE BLE_UUID128_INIT() macro
// wraps this list in its own braces for the .value field of ble_uuid128_t.
#define BLE_NATIVE_UUID_SUFFIX(NN) \
  (NN), 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xA0, \
  0x01, 0x40, 0x01, 0x00, 0x00, 0xD1, 0xA4, 0xF1

// Service
#define BLE_NATIVE_SVC_UUID          BLE_NATIVE_UUID_SUFFIX(0x01)

// Characteristics
#define BLE_NATIVE_CHR_EVENTS        BLE_NATIVE_UUID_SUFFIX(0x02)  // notify
#define BLE_NATIVE_CHR_RX_LIST       BLE_NATIVE_UUID_SUFFIX(0x03)  // read
#define BLE_NATIVE_CHR_QSO_QUEUE     BLE_NATIVE_UUID_SUFFIX(0x04)  // read
#define BLE_NATIVE_CHR_CONFIG        BLE_NATIVE_UUID_SUFFIX(0x05)  // read
#define BLE_NATIVE_CHR_RADIO_STREAM  BLE_NATIVE_UUID_SUFFIX(0x06)  // notify (binary)
#define BLE_NATIVE_CHR_RPC_REQ       BLE_NATIVE_UUID_SUFFIX(0x07)  // write
#define BLE_NATIVE_CHR_RPC_RESP      BLE_NATIVE_UUID_SUFFIX(0x08)  // notify
#define BLE_NATIVE_CHR_LOG_STREAM    BLE_NATIVE_UUID_SUFFIX(0x09)  // indicate (JSON, one entry per packet)

// ---------------------------------------------------------------------------
// Event tags (in EVENTS notifications, JSON format).
//   {"e":"rx"}     — RX list changed; client re-reads RX_LIST
//   {"e":"qso"}    — QSO queue / next TX changed; client re-reads QSO_QUEUE
//   {"e":"config"} — Station config changed; client re-reads CONFIG
// ---------------------------------------------------------------------------
#define BLE_NATIVE_EVT_RX     "rx"
#define BLE_NATIVE_EVT_QSO    "qso"
#define BLE_NATIVE_EVT_CONFIG "config"

// ---------------------------------------------------------------------------
// RPC commands (in RPC_REQ writes, JSON format).
//
//   Request:  {"id":<n>,"cmd":"<name>","args":{ ... }}
//   Response: {"id":<n>,"ok":<bool>,"err":"<msg>"}
//
// Handler exists for each command; missing args → err.
// ---------------------------------------------------------------------------
#define BLE_NATIVE_RPC_TAP_RX          "tap_rx"
#define BLE_NATIVE_RPC_CANCEL_TX       "cancel_tx"
#define BLE_NATIVE_RPC_CLEAR_QSO_QUEUE "clear_qso"
#define BLE_NATIVE_RPC_DROP_QSO        "drop_qso"
#define BLE_NATIVE_RPC_QUEUE_FREETEXT  "queue_freetext"
#define BLE_NATIVE_RPC_SET_BAND        "set_band"
#define BLE_NATIVE_RPC_SET_RADIO       "set_radio"
#define BLE_NATIVE_RPC_SET_CALL        "set_call"
#define BLE_NATIVE_RPC_SET_GRID        "set_grid"
#define BLE_NATIVE_RPC_SET_COMMENT     "set_comment"
#define BLE_NATIVE_RPC_SET_CQ_TYPE     "set_cq_type"
#define BLE_NATIVE_RPC_SET_CQ_FREETEXT "set_cq_freetext"
#define BLE_NATIVE_RPC_SET_BEACON      "set_beacon"
#define BLE_NATIVE_RPC_SET_OFFSET_SRC  "set_offset_src"
#define BLE_NATIVE_RPC_SET_OFFSET_HZ   "set_offset_hz"
#define BLE_NATIVE_RPC_SET_SKIP_TX1    "set_skip_tx1"
#define BLE_NATIVE_RPC_SET_MAX_RETRY   "set_max_retry"
#define BLE_NATIVE_RPC_SET_RTC         "set_rtc"
#define BLE_NATIVE_RPC_SET_RTC_COMP    "set_rtc_comp"
#define BLE_NATIVE_RPC_IGNORE_ADD      "ignore_add"
#define BLE_NATIVE_RPC_IGNORE_REMOVE   "ignore_remove"
#define BLE_NATIVE_RPC_IGNORE_CLEAR    "ignore_clear"
// Worked-QSO log. Access is per entry, not per byte: an ADIF record has a
// schema and a bounded size, so it fits one indication and needs no
// reassembly. The firmware owns the only ADIF parser (core_api.cpp) — clients
// never see the file format.
//
// log_days -> {"n":<total days>,"d":["YYYYMMDD:<entries>",...]} newest first.
//             "d" is capped (kLogDaysMax) to fit the 256-byte RPC response;
//             "n" is the true count, so a client can tell it was truncated.
// log_read   args {"d":"YYYYMMDD","from":N,"n":M} — all optional. "d"
//             defaults to today, "from" to 0, "n" to "all remaining".
//             Responds {"n":<will send>,"total":<entries that day>} then
//             streams that many entries over LOG_STREAM.
//             Entry indices are stable: the day file is append-only, so a
//             client holding 0..N-1 asks for from=N and gets only what was
//             logged since.
// log_abort   stops an in-flight stream.
#define BLE_NATIVE_RPC_LOG_DAYS        "log_days"
#define BLE_NATIVE_RPC_LOG_READ        "log_read"
#define BLE_NATIVE_RPC_LOG_ABORT       "log_abort"

// ---------------------------------------------------------------------------
// RADIO_STREAM packet (binary, little-endian).
//
// One waterfall row may be split across multiple notifications when the row
// would otherwise exceed the negotiated MTU (e.g. on iOS where MTU often
// caps at 247 and a 433-bin row needs 441 bytes). Each notification is a
// "chunk" with a small header that lets the client reassemble the row.
//
//   offset  size  field       meaning
//   ------  ----  ----------  --------------------------------------------
//   0       2     sym         symbol index within 15s slot (0..92)
//   2       2     swr_q8      SWR × 256, little-endian uint16
//   4       2     pwr_q8      Watts × 256, little-endian uint16
//   6       1     ptt         0 = RX, 1 = TX
//   7       1     chunk_info  bits 0..3: chunk_idx (0..chunk_count-1)
//                             bits 4..7: chunk_count (1..15)
//   8       N     mag[N]      waterfall magnitudes for this chunk only
//                             N = (total_payload_bytes - 8). N == 0 when
//                             ptt=1 (PTT frames are always single-chunk).
//
// Reassembly: the client buffers chunks until chunk_idx == chunk_count-1,
// then the concatenated mag[] across all chunks is the full waterfall row
// covering 200..2900 Hz (433 bins at the firmware's current config).
// Drop-tolerant: a missing chunk causes the in-progress row to be discarded
// when the next chunk_idx==0 arrives — same blast radius as one lost row.
// ---------------------------------------------------------------------------

#define BLE_NATIVE_RADIO_STREAM_HEADER_SIZE 8

struct BleRadioStreamHeader {
  uint16_t sym;
  uint16_t swr_q8;
  uint16_t pwr_q8;
  uint8_t  ptt;
  uint8_t  chunk_info;   // (chunk_count << 4) | chunk_idx
} __attribute__((packed));
static_assert(sizeof(BleRadioStreamHeader) == BLE_NATIVE_RADIO_STREAM_HEADER_SIZE,
              "BleRadioStreamHeader packed size mismatch");

// ---------------------------------------------------------------------------
// LOG_STREAM packet (JSON, indicate-mode, reliable).
//
// One worked-QSO entry per indication, so each packet is self-contained and
// well inside MTU — no chunk index, no reassembly. A zero-length indication
// ends the stream. Client must be subscribed before issuing log_read; the
// firmware only pumps while a subscriber exists.
//
//   {"i":0,"t":"142530","c":"W1ABC","g":"FN42","m":"FT8",
//    "f":"14.074","b":"20m","s":-12,"r":-8,"x":"MiniFT8 /Radio"}
//
//   i index within the day   t time_on HHMMSS UTC   c call     g grid
//   m mode                   f freq MHz            b band      x comment
//   s rst_sent   r rst_rcvd  — both omitted when the node logged no report,
//                              keeping "absent" distinct from a real 0 dB.
//
// ---------------------------------------------------------------------------
// Initialization API (firmware only — Flutter clients don't need this).
// ---------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

// Registers the native GATT service with NimBLE. Call ONCE, before
// ble_gatts_count_cfg/ble_gatts_add_svcs in the BLE bring-up path.
// Also starts the TX helper task that pumps notifications.
bool ble_native_init(void);

// Shut the TX helper task down and free its queues. Call BEFORE
// nimble_port_deinit() — otherwise the task body may invoke
// ble_gatts_* against torn-down state and crash.
void ble_native_shutdown(void);

// Notify server of a new connection / disconnect so it can track
// subscription state. Called from gap_cb.
void ble_native_on_connect(uint16_t conn_handle);
void ble_native_on_disconnect(void);

// Update tracked MTU (call from gap_cb on MTU event).
void ble_native_on_mtu(uint16_t mtu);

// Called when the client subscribes/unsubscribes to a notifiable
// characteristic (from gap_cb subscribe event).
void ble_native_on_subscribe(uint16_t attr_handle,
                             bool notify_en, bool indicate_en);

#ifdef __cplusplus
}
#endif
