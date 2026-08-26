// ============================================================================
// ble_native.cpp — GATT server for the Mini-FT8 native mobile client.
//
// Adds a second BLE service (alongside the existing text-terminal service)
// that exposes structured reads, writes, and notifications per the contract
// in ble_native.h and docs/NATIVE_CLIENT_ARCHITECTURE.md.
//
// The server is driven by a dedicated FreeRTOS task that:
//   - pumps EVENTS / RADIO_STREAM notifications out of dirty flags
//   - dispatches RPC_REQ writes to core_cmd_* and emits RPC_RESP
//   - streams ADIF chunks on demand
//
// Consumer callbacks registered on core_api.h set dirty flags / push onto
// queues — they MUST be trivial and fast; all heavy lifting happens on the
// BLE TX task.
// ============================================================================

#include "ble_native.h"
#include "core_api.h"
#include "core_api_internal.h"

#include <cstring>
#include <cstdio>
#include <string>
#include <vector>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "cJSON.h"

static const char* TAG = "BLE_NATIVE";

// ---------------------------------------------------------------------------
// UUIDs
// ---------------------------------------------------------------------------

static const ble_uuid128_t k_svc_uuid         = BLE_UUID128_INIT(BLE_NATIVE_SVC_UUID);
static const ble_uuid128_t k_chr_events       = BLE_UUID128_INIT(BLE_NATIVE_CHR_EVENTS);
static const ble_uuid128_t k_chr_rx_list      = BLE_UUID128_INIT(BLE_NATIVE_CHR_RX_LIST);
static const ble_uuid128_t k_chr_qso_queue    = BLE_UUID128_INIT(BLE_NATIVE_CHR_QSO_QUEUE);
static const ble_uuid128_t k_chr_config       = BLE_UUID128_INIT(BLE_NATIVE_CHR_CONFIG);
static const ble_uuid128_t k_chr_radio_stream = BLE_UUID128_INIT(BLE_NATIVE_CHR_RADIO_STREAM);
static const ble_uuid128_t k_chr_rpc_req      = BLE_UUID128_INIT(BLE_NATIVE_CHR_RPC_REQ);
static const ble_uuid128_t k_chr_rpc_resp     = BLE_UUID128_INIT(BLE_NATIVE_CHR_RPC_RESP);
static const ble_uuid128_t k_chr_adif_stream  = BLE_UUID128_INIT(BLE_NATIVE_CHR_ADIF_STREAM);

// ---------------------------------------------------------------------------
// Connection / subscription state
// ---------------------------------------------------------------------------

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_mtu         = 23;   // negotiate up on connect

static uint16_t s_h_events       = 0;
static uint16_t s_h_radio_stream = 0;
static uint16_t s_h_rpc_resp     = 0;
static uint16_t s_h_adif_stream  = 0;

static bool s_sub_events       = false;
static bool s_sub_radio        = false;
static bool s_sub_rpc_resp     = false;
static bool s_sub_adif         = false;

// ---------------------------------------------------------------------------
// Event dirty flags (set by core_api callbacks, drained by TX task)
// ---------------------------------------------------------------------------

static volatile bool s_evt_dirty_rx     = false;
static volatile bool s_evt_dirty_qso    = false;
static volatile bool s_evt_dirty_config = false;

// Latest waterfall row — zero-copy: we store a pointer into the monitor's
// waterfall buffer (mon.wf.mag) instead of copying 433 bytes. That memory
// is stable until the next slot boundary (each block writes to a distinct
// offset), so the TX task can read from it directly.
//
// Race window: across a slot boundary, the audio task reuses the same
// offsets for the new slot's blocks. If the TX task happens to read
// during that transition (160 ms at 6 Hz × 93 blocks = tiny window at
// block 0 of the new slot), it may see torn data in one row. For a lossy
// waterfall stream this is acceptable — one noisy frame.
struct WaterfallSlot {
  volatile bool  valid    = false;
  int            sym      = 0;
  int            num_bins = 0;
  const uint8_t* mag_ptr  = nullptr;  // borrowed pointer, not owned
  float          swr      = 1.5f;
  float          pwr      = 2.0f;
  bool           ptt      = false;
};
static WaterfallSlot s_wf_slot;

// Most day-logs returned by one adif_list. Bounded by the 256-byte RPC
// response, not by storage — "n" in the response carries the true total.
static constexpr int kAdifListMax = 10;

// ADIF streaming state
struct AdifStreamState {
  volatile bool active   = false;
  int           handle   = -1;
  int           total    = 0;
  bool          eof_sent = false;
};
static AdifStreamState s_adif;

// RPC request/response queue items are POD — std::string can't ride in a
// FreeRTOS queue (xQueueSend memcpys raw bytes, bypassing the copy ctor,
// and you get a double-free when both the sender's copy and the receiver's
// copy destruct). Fixed-size char buffers are safe.
static constexpr int kRpcJsonMax = 256;
struct RpcMsg {
  uint16_t len;
  char     json[kRpcJsonMax];
};
static QueueHandle_t s_rpc_req_queue  = nullptr;
static QueueHandle_t s_rpc_resp_queue = nullptr;

// ---------------------------------------------------------------------------
// TX task handle
// ---------------------------------------------------------------------------

static TaskHandle_t s_tx_task = nullptr;

// ---------------------------------------------------------------------------
// Core-API consumer callbacks (set dirty flags only; no BLE calls here)
// ---------------------------------------------------------------------------

static inline void wake_tx_task() {
  if (s_tx_task) xTaskNotifyGive(s_tx_task);
}

static void on_rx_changed()     { s_evt_dirty_rx     = true; wake_tx_task(); }
static void on_qso_changed()    { s_evt_dirty_qso    = true; wake_tx_task(); }
static void on_config_changed() { s_evt_dirty_config = true; wake_tx_task(); }

static void on_waterfall_row(const WaterfallRow& row) {
  // Zero-copy: stash the pointer instead of copying bytes. Valid until
  // the next slot boundary on the audio task. Worst case a torn read
  // during slot rollover — acceptable for a lossy stream.
  s_wf_slot.valid    = false;          // invalidate while storing
  s_wf_slot.sym      = row.sym;
  s_wf_slot.swr      = row.swr;
  s_wf_slot.pwr      = row.pwr;
  s_wf_slot.ptt      = row.ptt;
  s_wf_slot.num_bins = row.mag ? row.num_bins : 0;
  s_wf_slot.mag_ptr  = row.mag;
  s_wf_slot.valid    = true;
  wake_tx_task();
}

// ---------------------------------------------------------------------------
// JSON serialization helpers
// ---------------------------------------------------------------------------

// RX_LIST wire format (binary, replaces the old JSON encoding):
//
//   u8  version  = 1
//   u8  count    (number of decode rows that follow)
//   per row:
//     i16 snr           (little-endian)
//     i16 offset_hz     (little-endian)
//     u8  slot_id       (0=even, 1=odd; the firmware's slot parity)
//     u8  flags         bit0 = is_cq, bit1 = is_to_me
//     u8  text_len      (0..RX_TEXT_MAX)
//     char text[text_len]
//
// True zero-copy on the firmware side: each row's header is built on a
// few stack bytes, the text comes from the static rx_lines[] array via
// ui_get_rx_entry's stack copy, and the bytes go straight into the
// NimBLE mbuf pool with no intermediate scratch on the main heap.
// Important under heap fragmentation — after a heavy decode pass the
// largest contiguous block can drop below 2 KB and the previous cJSON
// + std::string + std::vector path used to abort.
constexpr uint8_t kRxListWireVersion = 1;

// QSO_QUEUE wire format (binary, replaces the old JSON encoding):
//
//   u8  version       = 1
//   u8  active_count
//   u8  next_tx_valid
//   [next_tx — present only when next_tx_valid != 0]
//     u16 offset_hz   (LE)
//     u8  slot_id     (0=even, 1=odd)
//     u8  retries_remaining
//     u8  call_len
//     char call[call_len]
//     u8  text_len
//     char text[text_len]
//   [active entries — active_count of them]
//     u8  state         (CoreQsoState as u8: 0=CALLING ... 6=IDLE)
//     u8  next_tx       (CoreTxMsg     as u8: 0=NONE 1..6=TX1..TX6 7=FREETEXT)
//     i8  snr_tx        (-99..99; 0x80 reserved for "unset" if ever needed)
//     i8  snr_rx
//     u8  retry
//     u8  retry_max
//     u8  slot_id
//     u8  flags         bit0 = is_fd, bit1 = logged
//     u8  call_len
//     char call[call_len]
//     u8  grid_len
//     char grid[grid_len]
//
// Same zero-copy story as RX_LIST: the rows are streamed directly into
// the NimBLE mbuf, no std::vector / cJSON / std::string in the path.
constexpr uint8_t kQsoQueueWireVersion = 1;

// Helper for the BLE callback to serialise a length-prefixed string into
// the mbuf. Returns 0 on success, BLE_ATT_ERR_INSUFFICIENT_RES on append
// failure (the mbuf pool — separate from main heap — is exhausted).
static int append_lp_string(struct os_mbuf* om, const std::string& s) {
  uint8_t len = (s.size() > 255) ? 255 : (uint8_t)s.size();
  if (os_mbuf_append(om, &len, 1) != 0) return BLE_ATT_ERR_INSUFFICIENT_RES;
  if (len > 0 && os_mbuf_append(om, s.data(), len) != 0) {
    return BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  return 0;
}

static const char* beacon_name(CoreBeaconMode m) {
  switch (m) {
    case CoreBeaconMode::OFF:  return "OFF";
    case CoreBeaconMode::EVEN: return "EVEN";
    case CoreBeaconMode::ODD:  return "ODD";
  }
  return "OFF";
}
static const char* cq_name(CoreCqType t) {
  switch (t) {
    case CoreCqType::CQ:       return "CQ";
    case CoreCqType::SOTA:     return "SOTA";
    case CoreCqType::POTA:     return "POTA";
    case CoreCqType::QRP:      return "QRP";
    case CoreCqType::FD:       return "FD";
    case CoreCqType::FREETEXT: return "FREETEXT";
  }
  return "CQ";
}
static const char* offset_name(CoreOffsetSrc s) {
  switch (s) {
    case CoreOffsetSrc::RX:     return "RX";
    case CoreOffsetSrc::CURSOR: return "CURSOR";
    case CoreOffsetSrc::RANDOM: return "RANDOM";
  }
  return "RANDOM";
}
static const char* radio_name(CoreRadioType r) {
  return (r == CoreRadioType::KH1) ? "KH1" : "QMX";
}

static std::string json_build_config() {
  StationConfig cfg;
  core_get_config(cfg);
  cJSON* root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "apiVer",    core_api_version());
  cJSON_AddStringToObject(root, "bleVer",    BLE_NATIVE_VERSION);
  cJSON_AddStringToObject(root, "call",      cfg.call.c_str());
  cJSON_AddStringToObject(root, "grid",      cfg.grid.c_str());
  cJSON_AddStringToObject(root, "comment",   cfg.comment.c_str());
  cJSON_AddStringToObject(root, "radio",     radio_name(cfg.radio));
  cJSON* bands = cJSON_CreateArray();
  for (uint32_t hz : cfg.bands_hz) {
    cJSON_AddItemToArray(bands, cJSON_CreateNumber((double)hz));
  }
  cJSON_AddItemToObject(root, "bands",     bands);
  cJSON_AddNumberToObject(root, "bandIdx",   cfg.band_idx);
  cJSON_AddStringToObject(root, "cqType",    cq_name(cfg.cq_type));
  cJSON_AddStringToObject(root, "cqFreetext",cfg.cq_freetext.c_str());
  cJSON_AddStringToObject(root, "beacon",    beacon_name(cfg.beacon));
  cJSON_AddStringToObject(root, "offsetSrc", offset_name(cfg.offset_src));
  cJSON_AddNumberToObject(root, "offsetHz",  cfg.offset_hz);
  cJSON_AddBoolToObject  (root, "skipTx1",   cfg.skip_tx1);
  cJSON_AddNumberToObject(root, "maxRetry",  cfg.max_retry);
  cJSON_AddNumberToObject(root, "rtcComp",   cfg.rtc_comp);
  cJSON_AddStringToObject(root, "date",      cfg.date.c_str());
  cJSON_AddStringToObject(root, "time",      cfg.time.c_str());
  cJSON* ig = cJSON_CreateArray();
  for (const auto& p : cfg.ignore_prefixes) {
    cJSON_AddItemToArray(ig, cJSON_CreateString(p.c_str()));
  }
  cJSON_AddItemToObject(root, "ignorePrefixes", ig);
  char* s = cJSON_PrintUnformatted(root);
  std::string out = s ? s : "{}";
  if (s) free(s);
  cJSON_Delete(root);
  return out;
}

// ---------------------------------------------------------------------------
// GATT access callbacks (invoked by NimBLE on read/write)
// ---------------------------------------------------------------------------

static int access_read_str(struct ble_gatt_access_ctxt* ctxt, const std::string& body) {
  return os_mbuf_append(ctxt->om, body.data(), body.size()) == 0
             ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int chr_rx_list_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;

  const int total = ui_get_rx_count();
  uint8_t count = (total < 0) ? 0 : (total > 255 ? 255 : (uint8_t)total);

  uint8_t hdr[2] = { kRxListWireVersion, count };
  if (os_mbuf_append(ctxt->om, hdr, sizeof(hdr)) != 0) {
    return BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  for (uint8_t i = 0; i < count; ++i) {
    RxDecodeEntry e;
    if (!ui_get_rx_entry(i, &e)) continue;

    // RX_TEXT_MAX is 64; bound length to one byte just in case.
    size_t tlen = strnlen(e.text, sizeof(e.text));
    if (tlen > 255) tlen = 255;

    struct __attribute__((packed)) {
      int16_t snr;
      int16_t offset_hz;
      uint8_t slot_id;
      uint8_t flags;
      uint8_t text_len;
    } row{
      .snr       = (int16_t)e.snr,
      .offset_hz = (int16_t)e.offset_hz,
      .slot_id   = (uint8_t)e.slot_id,
      .flags     = (uint8_t)((e.is_cq ? 0x01 : 0) | (e.is_to_me ? 0x02 : 0)),
      .text_len  = (uint8_t)tlen,
    };

    if (os_mbuf_append(ctxt->om, &row, sizeof(row)) != 0 ||
        (tlen > 0 && os_mbuf_append(ctxt->om, e.text, tlen) != 0)) {
      return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
  }
  return 0;
}
static int chr_qso_queue_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;

  const int total = core_qso_active_count();
  uint8_t active_count = (total < 0) ? 0 : (total > 255 ? 255 : (uint8_t)total);

  NextTxEntry nx{};
  core_qso_get_next_tx(nx);

  // Header: version + active_count + next_tx_valid.
  uint8_t hdr[3] = {
    kQsoQueueWireVersion,
    active_count,
    (uint8_t)(nx.valid ? 1 : 0),
  };
  if (os_mbuf_append(ctxt->om, hdr, sizeof(hdr)) != 0) {
    return BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  // next_tx body (when valid): offset, slot, retries, call, text.
  if (nx.valid) {
    struct __attribute__((packed)) {
      uint16_t offset_hz;
      uint8_t  slot_id;
      uint8_t  retries_remaining;
    } nx_hdr{
      .offset_hz         = (uint16_t)(nx.offset_hz < 0 ? 0 : nx.offset_hz),
      .slot_id           = (uint8_t)nx.slot_parity,
      .retries_remaining = (uint8_t)(nx.retries_remaining < 0 ? 0 :
                                     nx.retries_remaining > 255 ? 255 :
                                     nx.retries_remaining),
    };
    if (os_mbuf_append(ctxt->om, &nx_hdr, sizeof(nx_hdr)) != 0) {
      return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    int rc = append_lp_string(ctxt->om, nx.dxcall);
    if (rc != 0) return rc;
    rc = append_lp_string(ctxt->om, nx.text);
    if (rc != 0) return rc;
  }

  // Active entries.
  for (uint8_t i = 0; i < active_count; ++i) {
    QsoEntry q;
    if (!core_qso_get_active(i, q)) continue;

    auto clamp_i8 = [](int v) -> int8_t {
      if (v < -128) return -128;
      if (v >  127) return  127;
      return (int8_t)v;
    };
    auto clamp_u8 = [](int v) -> uint8_t {
      if (v < 0) return 0;
      if (v > 255) return 255;
      return (uint8_t)v;
    };

    struct __attribute__((packed)) {
      uint8_t state;
      uint8_t next_tx;
      int8_t  snr_tx;
      int8_t  snr_rx;
      uint8_t retry;
      uint8_t retry_max;
      uint8_t slot_id;
      uint8_t flags;
    } row{
      .state     = static_cast<uint8_t>(q.state),
      .next_tx   = static_cast<uint8_t>(q.next_tx),
      .snr_tx    = clamp_i8(q.snr_tx),
      .snr_rx    = clamp_i8(q.snr_rx),
      .retry     = clamp_u8(q.retry_counter),
      .retry_max = clamp_u8(q.retry_limit),
      .slot_id   = (uint8_t)q.slot_parity,
      .flags     = (uint8_t)((q.is_fd ? 0x01 : 0) | (q.logged ? 0x02 : 0)),
    };

    if (os_mbuf_append(ctxt->om, &row, sizeof(row)) != 0) {
      return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    int rc = append_lp_string(ctxt->om, q.dxcall);
    if (rc != 0) return rc;
    rc = append_lp_string(ctxt->om, q.dxgrid);
    if (rc != 0) return rc;
  }
  return 0;
}
static int chr_config_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
  return access_read_str(ctxt, json_build_config());
}

static int chr_rpc_req_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
  if (!ctxt->om || !s_rpc_req_queue) return 0;

  // Flatten the mbuf chain into our POD message. Oversized requests are
  // dropped — legitimate RPCs fit well under 256 bytes.
  uint16_t total = OS_MBUF_PKTLEN(ctxt->om);
  if (total == 0 || total >= kRpcJsonMax) return 0;

  RpcMsg msg{};
  uint16_t copied = 0;
  if (ble_hs_mbuf_to_flat(ctxt->om, msg.json, total, &copied) != 0) return 0;
  msg.json[copied] = '\0';
  msg.len = copied;
  xQueueSend(s_rpc_req_queue, &msg, 0);
  if (s_tx_task) xTaskNotifyGive(s_tx_task);
  return 0;
}

// Stubs for notify-only characteristics — they don't handle reads.
static int chr_notify_only_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt*, void*) {
  return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

// ---------------------------------------------------------------------------
// Characteristic + service tables
// ---------------------------------------------------------------------------

static const struct ble_gatt_chr_def k_chrs[] = {
  { &k_chr_events.u,       chr_notify_only_cb, nullptr, nullptr,
    BLE_GATT_CHR_F_NOTIFY, 0, &s_h_events, nullptr },
  { &k_chr_rx_list.u,      chr_rx_list_cb,     nullptr, nullptr,
    BLE_GATT_CHR_F_READ,   0, nullptr, nullptr },
  { &k_chr_qso_queue.u,    chr_qso_queue_cb,   nullptr, nullptr,
    BLE_GATT_CHR_F_READ,   0, nullptr, nullptr },
  { &k_chr_config.u,       chr_config_cb,      nullptr, nullptr,
    BLE_GATT_CHR_F_READ,   0, nullptr, nullptr },
  { &k_chr_radio_stream.u, chr_notify_only_cb, nullptr, nullptr,
    BLE_GATT_CHR_F_NOTIFY, 0, &s_h_radio_stream, nullptr },
  { &k_chr_rpc_req.u,      chr_rpc_req_cb,     nullptr, nullptr,
    BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP, 0, nullptr, nullptr },
  { &k_chr_rpc_resp.u,     chr_notify_only_cb, nullptr, nullptr,
    BLE_GATT_CHR_F_NOTIFY, 0, &s_h_rpc_resp, nullptr },
  { &k_chr_adif_stream.u,  chr_notify_only_cb, nullptr, nullptr,
    BLE_GATT_CHR_F_INDICATE, 0, &s_h_adif_stream, nullptr },
  { nullptr, nullptr, nullptr, nullptr, 0, 0, nullptr, nullptr }
};

static const struct ble_gatt_svc_def k_svcs[] = {
  { BLE_GATT_SVC_TYPE_PRIMARY, &k_svc_uuid.u, nullptr, k_chrs },
  { 0, nullptr, nullptr, nullptr }
};

// ---------------------------------------------------------------------------
// Connection lifecycle (called from gap_cb in main.cpp)
// ---------------------------------------------------------------------------

void ble_native_on_connect(uint16_t conn_handle) {
  s_conn_handle = conn_handle;
  s_sub_events  = s_sub_radio = s_sub_rpc_resp = s_sub_adif = false;
  s_adif.active = false;
  s_adif.handle = -1;
}

void ble_native_on_disconnect(void) {
  s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
  s_sub_events  = s_sub_radio = s_sub_rpc_resp = s_sub_adif = false;
  if (s_adif.active) {
    core_adif_close(s_adif.handle);
    s_adif.active = false;
    s_adif.handle = -1;
  }
}

void ble_native_on_mtu(uint16_t mtu) {
  s_mtu = mtu;
}

void ble_native_on_subscribe(uint16_t attr_handle, bool notify_en, bool indicate_en) {
  if (attr_handle == s_h_events)       s_sub_events   = notify_en;
  if (attr_handle == s_h_radio_stream) s_sub_radio    = notify_en;
  if (attr_handle == s_h_rpc_resp)     s_sub_rpc_resp = notify_en;
  if (attr_handle == s_h_adif_stream)  s_sub_adif     = indicate_en;
}

// ---------------------------------------------------------------------------
// Notification helpers
// ---------------------------------------------------------------------------

static void send_notify(uint16_t handle, const void* data, size_t len) {
  if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || handle == 0) return;
  struct os_mbuf* om = ble_hs_mbuf_from_flat(data, len);
  if (!om) return;
  ble_gatts_notify_custom(s_conn_handle, handle, om);
}
static void send_indicate(uint16_t handle, const void* data, size_t len) {
  if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || handle == 0) return;
  struct os_mbuf* om = ble_hs_mbuf_from_flat(data, len);
  if (!om) return;
  ble_gatts_indicate_custom(s_conn_handle, handle, om);
}

static void send_event_ping(const char* evt_tag) {
  if (!s_sub_events) return;
  char buf[32];
  int n = snprintf(buf, sizeof(buf), "{\"e\":\"%s\"}", evt_tag);
  if (n > 0) send_notify(s_h_events, buf, (size_t)n);
}

static void send_waterfall_row() {
  if (!s_sub_radio || !s_wf_slot.valid) return;
  s_wf_slot.valid = false;  // consume

  // Snapshot the borrowed pointer + metadata. Even if the audio task
  // updates s_wf_slot below, we've already captured what we need.
  const uint8_t* const mag   = s_wf_slot.mag_ptr;
  const int            bins  = s_wf_slot.ptt ? 0 : s_wf_slot.num_bins;
  const uint16_t       sym   = (uint16_t)s_wf_slot.sym;
  const uint16_t       swr_q = (uint16_t)(s_wf_slot.swr * 256.0f);
  const uint16_t       pwr_q = (uint16_t)(s_wf_slot.pwr * 256.0f);
  const uint8_t        ptt   = s_wf_slot.ptt ? 1 : 0;

  if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_h_radio_stream == 0) return;

  const size_t header_sz   = BLE_NATIVE_RADIO_STREAM_HEADER_SIZE;
  const size_t max_payload = (s_mtu > 3) ? (size_t)(s_mtu - 3) : 0;
  if (max_payload <= header_sz) return;
  const int max_bins_per_chunk = (int)(max_payload - header_sz);

  // Compute chunking: split the 433-bin row across chunk_count notifications
  // when it can't fit in one MTU. PTT frames carry no magnitudes but still
  // emit one header-only chunk so chunk_count is always >= 1.
  int chunk_count;
  int chunk_size;
  if (bins <= 0 || !mag) {
    chunk_count = 1;
    chunk_size  = 0;
  } else {
    chunk_count = (bins + max_bins_per_chunk - 1) / max_bins_per_chunk;
    if (chunk_count > 15) chunk_count = 15;   // chunk_info field is 4 bits
    chunk_size = (bins + chunk_count - 1) / chunk_count;
  }

  // Each chunk's body is a pointer into mon.wf.mag — true zero-copy,
  // no scratch buffer.
  for (int c = 0; c < chunk_count; ++c) {
    const int bin_offset = c * chunk_size;
    int this_bins = (bins > 0) ? chunk_size : 0;
    if (bin_offset + this_bins > bins) this_bins = bins - bin_offset;
    if (this_bins < 0) this_bins = 0;

    BleRadioStreamHeader hdr;
    hdr.sym       = sym;
    hdr.swr_q8    = swr_q;
    hdr.pwr_q8    = pwr_q;
    hdr.ptt       = ptt;
    hdr.chunk_info = (uint8_t)(((chunk_count & 0x0F) << 4) | (c & 0x0F));

    struct os_mbuf* om = ble_hs_mbuf_from_flat(&hdr, sizeof(hdr));
    if (!om) return;
    if (this_bins > 0) {
      if (os_mbuf_append(om, mag + bin_offset, (size_t)this_bins) != 0) {
        os_mbuf_free_chain(om);
        return;
      }
    }
    ble_gatts_notify_custom(s_conn_handle, s_h_radio_stream, om);
  }
}

// ---------------------------------------------------------------------------
// RPC dispatch
// ---------------------------------------------------------------------------

namespace {
struct RpcResult { bool ok; std::string err; std::string extra_json; };

RpcResult res_ok()                        { return {true,  {},  {}}; }
RpcResult res_err(const char* m)          { return {false, m,   {}}; }
RpcResult res_ok_extra(std::string extra) { return {true,  {},  std::move(extra)}; }

int arg_int(cJSON* args, const char* key, int dflt) {
  cJSON* v = cJSON_GetObjectItem(args, key);
  return (v && cJSON_IsNumber(v)) ? v->valueint : dflt;
}
bool arg_bool(cJSON* args, const char* key, bool dflt) {
  cJSON* v = cJSON_GetObjectItem(args, key);
  return (v && cJSON_IsBool(v)) ? cJSON_IsTrue(v) : dflt;
}
std::string arg_str(cJSON* args, const char* key) {
  cJSON* v = cJSON_GetObjectItem(args, key);
  return (v && cJSON_IsString(v) && v->valuestring) ? v->valuestring : "";
}

CoreBeaconMode parse_beacon(const std::string& s) {
  if (s == "EVEN") return CoreBeaconMode::EVEN;
  if (s == "ODD")  return CoreBeaconMode::ODD;
  return CoreBeaconMode::OFF;
}
CoreCqType parse_cq(const std::string& s) {
  if (s == "SOTA")     return CoreCqType::SOTA;
  if (s == "POTA")     return CoreCqType::POTA;
  if (s == "QRP")      return CoreCqType::QRP;
  if (s == "FD")       return CoreCqType::FD;
  if (s == "FREETEXT") return CoreCqType::FREETEXT;
  return CoreCqType::CQ;
}
CoreOffsetSrc parse_offset(const std::string& s) {
  if (s == "RX")     return CoreOffsetSrc::RX;
  if (s == "CURSOR") return CoreOffsetSrc::CURSOR;
  return CoreOffsetSrc::RANDOM;
}
CoreRadioType parse_radio(const std::string& s) {
  if (s == "KH1") return CoreRadioType::KH1;
  return CoreRadioType::QMX;
}

RpcResult dispatch_rpc(const std::string& cmd, cJSON* args) {
  if (cmd == BLE_NATIVE_RPC_TAP_RX) {
    return core_cmd_tap_rx(arg_int(args, "idx", -1)) ? res_ok() : res_err("tap_rx failed");
  }
  if (cmd == BLE_NATIVE_RPC_CANCEL_TX) {
    return core_cmd_cancel_tx() ? res_ok() : res_err("cancel_tx failed");
  }
  if (cmd == BLE_NATIVE_RPC_CLEAR_QSO_QUEUE) {
    return core_cmd_clear_qso_queue() ? res_ok() : res_err("clear failed");
  }
  if (cmd == BLE_NATIVE_RPC_DROP_QSO) {
    return core_cmd_drop_qso(arg_int(args, "idx", -1)) ? res_ok() : res_err("drop failed");
  }
  if (cmd == BLE_NATIVE_RPC_QUEUE_FREETEXT) {
    return core_cmd_queue_freetext(arg_str(args, "text")) ? res_ok() : res_err("ft rejected");
  }
  if (cmd == BLE_NATIVE_RPC_SET_BAND) {
    return core_cmd_set_band(arg_int(args, "idx", -1)) ? res_ok() : res_err("band idx");
  }
  if (cmd == BLE_NATIVE_RPC_SET_RADIO) {
    return core_cmd_set_radio(parse_radio(arg_str(args, "radio"))) ? res_ok() : res_err("set_radio");
  }
  if (cmd == BLE_NATIVE_RPC_SET_CALL) {
    return core_cmd_set_call(arg_str(args, "call")) ? res_ok() : res_err("set_call");
  }
  if (cmd == BLE_NATIVE_RPC_SET_GRID) {
    return core_cmd_set_grid(arg_str(args, "grid")) ? res_ok() : res_err("set_grid");
  }
  if (cmd == BLE_NATIVE_RPC_SET_COMMENT) {
    return core_cmd_set_comment(arg_str(args, "comment")) ? res_ok() : res_err("set_comment");
  }
  if (cmd == BLE_NATIVE_RPC_SET_CQ_TYPE) {
    return core_cmd_set_cq_type(parse_cq(arg_str(args, "type"))) ? res_ok() : res_err("set_cq");
  }
  if (cmd == BLE_NATIVE_RPC_SET_CQ_FREETEXT) {
    return core_cmd_set_cq_freetext(arg_str(args, "text")) ? res_ok() : res_err("set_cq_ft");
  }
  if (cmd == BLE_NATIVE_RPC_SET_BEACON) {
    return core_cmd_set_beacon(parse_beacon(arg_str(args, "mode"))) ? res_ok() : res_err("set_beacon");
  }
  if (cmd == BLE_NATIVE_RPC_SET_OFFSET_SRC) {
    return core_cmd_set_offset_src(parse_offset(arg_str(args, "src"))) ? res_ok() : res_err("offset_src");
  }
  if (cmd == BLE_NATIVE_RPC_SET_OFFSET_HZ) {
    return core_cmd_set_offset_hz(arg_int(args, "hz", -1)) ? res_ok() : res_err("offset_hz");
  }
  if (cmd == BLE_NATIVE_RPC_SET_SKIP_TX1) {
    return core_cmd_set_skip_tx1(arg_bool(args, "skip", false)) ? res_ok() : res_err("skip_tx1");
  }
  if (cmd == BLE_NATIVE_RPC_SET_MAX_RETRY) {
    return core_cmd_set_max_retry(arg_int(args, "n", -1)) ? res_ok() : res_err("max_retry");
  }
  if (cmd == BLE_NATIVE_RPC_SET_RTC) {
    cJSON* v = cJSON_GetObjectItem(args, "ms");
    int64_t ms = (v && cJSON_IsNumber(v)) ? (int64_t)v->valuedouble : 0;
    return core_cmd_set_rtc(ms) ? res_ok() : res_err("set_rtc");
  }
  if (cmd == BLE_NATIVE_RPC_SET_RTC_COMP) {
    return core_cmd_set_rtc_comp(arg_int(args, "ppm", 0)) ? res_ok() : res_err("rtc_comp");
  }
  if (cmd == BLE_NATIVE_RPC_IGNORE_ADD) {
    return core_cmd_ignore_add(arg_str(args, "prefix")) ? res_ok() : res_err("ignore_add");
  }
  if (cmd == BLE_NATIVE_RPC_IGNORE_REMOVE) {
    return core_cmd_ignore_remove(arg_str(args, "prefix")) ? res_ok() : res_err("ignore_rm");
  }
  if (cmd == BLE_NATIVE_RPC_IGNORE_CLEAR) {
    return core_cmd_ignore_clear() ? res_ok() : res_err("ignore_clear");
  }
  if (cmd == BLE_NATIVE_RPC_ADIF_LIST) {
    // Newest-first day-logs with sizes, so the client can diff against its
    // own cursors and fetch only what changed. The whole response has to fit
    // kRpcJsonMax (256 B) including the {"id":N,"ok":true...} envelope, so
    // cap the array and report the true total separately in "n".
    CoreAdifFileInfo files[kAdifListMax];
    int total = core_adif_list(files, kAdifListMax);
    int shown = total < kAdifListMax ? total : kAdifListMax;

    // Budget: envelope is at most ~22 B, closing brace 1, NUL 1. Stop early
    // if an entry would not fit rather than emitting truncated JSON.
    constexpr size_t kExtraBudget = kRpcJsonMax - 32;
    std::string extra = ",\"n\":" + std::to_string(total) + ",\"f\":[";
    for (int i = 0; i < shown; ++i) {
      char item[32];
      snprintf(item, sizeof(item), "%s\"%s:%d\"",
               i ? "," : "", files[i].date, files[i].size);
      if (extra.size() + strlen(item) + 1 > kExtraBudget) break;
      extra += item;
    }
    extra += "]";
    return res_ok_extra(std::move(extra));
  }
  if (cmd == BLE_NATIVE_RPC_ADIF_OPEN) {
    if (s_adif.active) return res_err("adif busy");
    // Both args optional: "d" defaults to today, "off" to 0. A non-zero
    // offset resumes an interrupted or incremental download — day-logs are
    // append-only, so bytes before the cursor can never have changed.
    const std::string date = arg_str(args, "d");
    const int         off  = arg_int(args, "off", 0);
    CoreAdifHandle h = core_adif_open(date.empty() ? nullptr : date.c_str(), off);
    if (h.id < 0) return res_err("adif open failed");
    s_adif.handle   = h.id;
    s_adif.total    = h.total;
    s_adif.eof_sent = false;
    s_adif.active   = true;
    char extra[48];
    snprintf(extra, sizeof(extra), ",\"size\":%d,\"off\":%d", h.total, h.offset);
    return res_ok_extra(extra);
  }
  if (cmd == BLE_NATIVE_RPC_ADIF_CLOSE) {
    if (s_adif.active) {
      core_adif_close(s_adif.handle);
      s_adif.active = false;
      s_adif.handle = -1;
    }
    return res_ok();
  }
  return res_err("unknown cmd");
}

void handle_rpc(const std::string& body) {
  cJSON* root = cJSON_Parse(body.c_str());
  if (!root) { ESP_LOGW(TAG, "rpc: bad json"); return; }

  cJSON* id_j   = cJSON_GetObjectItem(root, "id");
  cJSON* cmd_j  = cJSON_GetObjectItem(root, "cmd");
  cJSON* args_j = cJSON_GetObjectItem(root, "args");
  int id = (id_j && cJSON_IsNumber(id_j)) ? id_j->valueint : 0;
  std::string cmd = (cmd_j && cJSON_IsString(cmd_j) && cmd_j->valuestring)
                        ? cmd_j->valuestring : "";

  RpcResult r = cmd.empty() ? res_err("no cmd") : dispatch_rpc(cmd, args_j);

  // Build the response JSON by hand — small, predictable, no cJSON allocations.
  RpcMsg resp{};
  int n;
  if (r.ok) {
    if (r.extra_json.empty())
      n = snprintf(resp.json, sizeof(resp.json), "{\"id\":%d,\"ok\":true}", id);
    else
      n = snprintf(resp.json, sizeof(resp.json), "{\"id\":%d,\"ok\":true%s}",
                   id, r.extra_json.c_str());
  } else {
    n = snprintf(resp.json, sizeof(resp.json), "{\"id\":%d,\"ok\":false,\"err\":\"%s\"}",
                 id, r.err.c_str());
  }
  if (n > 0) {
    if (n >= (int)sizeof(resp.json)) n = sizeof(resp.json) - 1;
    resp.len = (uint16_t)n;
    if (s_rpc_resp_queue) xQueueSend(s_rpc_resp_queue, &resp, 0);
    if (s_tx_task) xTaskNotifyGive(s_tx_task);
  }

  cJSON_Delete(root);
}
}  // namespace

// ---------------------------------------------------------------------------
// TX task — pumps notifications and dispatches RPC requests.
// ---------------------------------------------------------------------------

static void pump_adif() {
  if (!s_adif.active || !s_sub_adif) return;
  // Chunk size = MTU - 3, capped at 200 for reliable indicate pacing.
  size_t chunk = s_mtu > 3 ? (size_t)(s_mtu - 3) : 20;
  if (chunk > 200) chunk = 200;

  std::vector<uint8_t> buf;
  if (core_adif_read(s_adif.handle, buf, chunk)) {
    if (!buf.empty()) send_indicate(s_h_adif_stream, buf.data(), buf.size());
    wake_tx_task();  // keep streaming without waiting for the poll timeout
  } else if (!s_adif.eof_sent) {
    // Zero-length indication = EOF marker.
    send_indicate(s_h_adif_stream, nullptr, 0);
    s_adif.eof_sent = true;
    core_adif_close(s_adif.handle);
    s_adif.active = false;
    s_adif.handle = -1;
  }
}

static volatile bool s_tx_task_exit = false;

static void tx_task_main(void*) {
  while (true) {
    if (s_tx_task_exit) {
      s_tx_task = nullptr;
      vTaskDelete(NULL);
      return;
    }
    // Wait for a notification (event, waterfall, RPC). ADIF streaming
    // progresses by self-notifying on each chunk. Fallback 250 ms
    // timeout guarantees forward progress even if a notify is ever lost.
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));

    // Event pings: drain dirty flags into one notification each.
    if (s_evt_dirty_rx)     { s_evt_dirty_rx     = false; send_event_ping(BLE_NATIVE_EVT_RX); }
    if (s_evt_dirty_qso)    { s_evt_dirty_qso    = false; send_event_ping(BLE_NATIVE_EVT_QSO); }
    if (s_evt_dirty_config) { s_evt_dirty_config = false; send_event_ping(BLE_NATIVE_EVT_CONFIG); }

    // Waterfall: send the latest row if any.
    send_waterfall_row();

    // RPC requests: dispatch, response gets enqueued.
    RpcMsg req;
    while (s_rpc_req_queue && xQueueReceive(s_rpc_req_queue, &req, 0) == pdTRUE) {
      handle_rpc(std::string(req.json, req.len));
    }

    // RPC responses: drain to RPC_RESP characteristic.
    if (s_sub_rpc_resp) {
      RpcMsg resp;
      while (s_rpc_resp_queue && xQueueReceive(s_rpc_resp_queue, &resp, 0) == pdTRUE) {
        send_notify(s_h_rpc_resp, resp.json, resp.len);
      }
    }

    // ADIF streaming: pump one chunk per cycle (paces reliably under indicate).
    pump_adif();
  }
}

// ---------------------------------------------------------------------------
// Public init
// ---------------------------------------------------------------------------

bool ble_native_init(void) {
  if (s_tx_task) return true;  // already initialized

  // Small queues — RPC traffic is human-paced, 4 in-flight is plenty.
  // Each RpcMsg is 258 bytes, so depth × size is the heap cost.
  s_rpc_req_queue  = xQueueCreate(4, sizeof(RpcMsg));   // ~1 KB
  s_rpc_resp_queue = xQueueCreate(4, sizeof(RpcMsg));   // ~1 KB
  if (!s_rpc_req_queue || !s_rpc_resp_queue) {
    ESP_LOGE(TAG, "queue create failed");
    return false;
  }

  int rc = ble_gatts_count_cfg(k_svcs);
  if (rc != 0) { ESP_LOGE(TAG, "count_cfg rc=%d", rc); return false; }
  rc = ble_gatts_add_svcs(k_svcs);
  if (rc != 0) { ESP_LOGE(TAG, "add_svcs rc=%d", rc); return false; }

  core_on_rx_changed    (on_rx_changed);
  core_on_qso_changed   (on_qso_changed);
  core_on_config_changed(on_config_changed);
  core_on_waterfall_row (on_waterfall_row);

  // Priority 3 (below app_task_core0's 5) so the local UI never gets
  // starved by BLE TX work. Pinned to core 1 so it runs alongside the
  // audio task but doesn't contend with the main UI loop on core 0.
  // 4 KB stack. The save_station_data fprintf chain (22 fprintfs into
  // a SPIFFS file) is the deep frame on this task — deferred to the
  // main task via g_config_save_pending so it never runs here. With
  // that out of the path, 4 KB is comfortable.
  BaseType_t ok = xTaskCreatePinnedToCore(tx_task_main, "ble_native", 4096,
                                          nullptr, 3, &s_tx_task, 1);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "tx task create failed");
    return false;
  }
  ESP_LOGI(TAG, "native BLE service registered, version %s", BLE_NATIVE_VERSION);
  return true;
}

void ble_native_shutdown(void) {
  if (s_tx_task) {
    s_tx_task_exit = true;
    xTaskNotifyGive(s_tx_task);
    // Wait briefly for the task to clear its handle and self-delete.
    for (int i = 0; i < 50 && s_tx_task != nullptr; ++i) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  if (s_rpc_req_queue)  { vQueueDelete(s_rpc_req_queue);  s_rpc_req_queue  = nullptr; }
  if (s_rpc_resp_queue) { vQueueDelete(s_rpc_resp_queue); s_rpc_resp_queue = nullptr; }
}
