Also see [PaperFT8](https://github.com/wcheng95/PaperFT8)

# First POTA activation (v1.0 2025-12-31)
![First POTA Activation](IMG_6087.jpeg)

## Mini-FT8 Release Notice
Mini-FT8 is built on Karlis Goba’s ft8_lib. It’s also a joint adventure between Zhenxing (N6HAN) and Wei (AG6AQ), with inspiration from DXFT8 by Barb (WB2CBA) and Charley (W5BAA). It has been a great learning platform for me, and I hope you find it just as fun to use. It supports QMX and KH1.

### Thanks

- The [DX FT8](https://github.com/WB2CBA/DX-FT8-FT8-MULTIBAND-TABLET-TRANSCEIVER) team — Barb (WB2CBA), Charley (W5BAA), and Paul (G8KIG) — for the inspiration.
- Zhenxing (N6HAN) — for helping build the audio/DSP path (UAC) and autoseq. This project would not have been possible without his help.
- Karlis Goba — for [ft8_lib](https://github.com/kgoba/ft8_lib). Thanks also to Shawn Rutledge for non-standard callsign support.
- OpenAI and Anthropic — for their incredible coding assistance.
  
### Hardware
(I have no affiliation with the vendors.)
  - Must order: https://shop.m5stack.com/products/m5stack-cardputer-adv-version-esp32-s3 or from digikey: https://www.digikey.com/en/products/detail/m5stack-technology-co-ltd/K132-ADV/27685158
  - Optional: [https://shop.m5stack.com/products/gps-bds-unit-v1-1-at6668](https://shop.m5stack.com/products/gps-bds-unit-v1-1-at6668) (for Date/Time/Grid, other GPS modules work too)
  - For KH1 TX : https://shop.m5stack.com/products/4pin-buckled-grove-cable, for a custmized serial cable
  - For KH1 RX: [USB C Microphone Adapter](https://www.amazon.com/dp/B0FWC9ZFC4?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1), Other adapters may also work, but this one is confirmed. (V2.0.2 Supports Cardputer direct microphone in, so the USB-C adapter becomes **optional, choose KH1-Mic**)

73, Wei AG6AQ

# Mini-FT8 Operation Manual (V2.0.4)
(Temporarily disabled BLE in V2.0.4, if you need it, please choose V2.0.4 in kh1_mic branch)

## Quick Mode Map

| Key | Mode | Purpose |
|---|---|---|
| `R` | RX | View decoded messages and tap one to start a QSO. |
| `T` | TX Queue | View and manage the transmit queue. |
| `S` | STATUS | Access beacon, connect/sync, band step, tune, and date/time functions. |
| `G` | GPS | View GPS telemetry and synchronization status. |
| `M` | MENU P1 | Configure core station and operator settings. |
| `N` | MENU P2 | Configure radio, input, and comment settings. |
| `O` | MENU P3 | Configure logging, active bands, RTC, copy-to-SD, and retry settings. |
| `Q` | QSO | Browse QSO and log files, and view entries. |
| `F` | Fetch | Browse files and fetch or dump the selected file to the BLE client. |
| `D` | Delete Files | Browse and delete files stored in SPIFFS. |
| `B` | BAND | Edit per-band frequencies. |
| `C` | Connect | Enter USB serial command mode. (removed in V2.0.4)|
| `P` | Performance | View A Simple Performance Monitor. (added in V2.0.4)|

## Global Keys and Navigation

- `R` / `T` / `B` / `S` / `G` / `Q` / `F` / `D` / `C`: switch to the selected mode. Press the same mode key again to return to `RX`.
- `M` / `N` / `O`: jump to MENU page 1 / 2 / 3. Press the current page key again to return to `RX`.
- `` ` ``: cancel TX globally in `RX`, `TX`, and `STATUS` when not editing.
- `▲` / `▼`: page up / page down in `RX`, `TX`, `BAND`, `MENU`, `QSO`, `Fetch`, and `Delete`.
- `◀` / `▶`: move left / right in `QSO-SNR`, `STATUS` date/time, `MENU P2` (N->2).
- `1`..`6`: always select the currently visible row in the active mode.

### BLE Terminal

 You can operate Mini-FT8 entirely from Bluetooth(BLE)
 
 Tested: App Store: BLE Terminal HM-10 by Gopi Gadhiya
 
- `E`: cancel current TX 
- `u` / `v`: page up / page down
- `z` / `x`: move left / right (`QSO-SNR`)

## BLE Screen Layout

1. Text waterfall frame:
   ```text
   =============================  (29 `=` frame boundary)
   |                           |  (27 bins inside the bars)
   ----.----+----.----+----.---+  (29-character scale with 500 Hz tick marks)
   ```
2. Line 7 meta/edit line:
   - normal: [MODE uv] (`u` / `v` show page-up / page-down availability; `-` means not available)
   - text edit mode: [Edit <item>] (edit in the BLE terminal, then press Enter to save; ESC is not available)

Notes:
- Waterfall bins use `space`, `.`, `:`, and `!` to indicate signal strength, about 100 Hz per bin.
- Counter symbols: `|` (slot boundary), `4`, `8`, `12`, `:` (even slot), `.` (odd slot), `o` (TX indicator).
- Decoded message count: [D:n]

## Per-Mode Controls

- ` acts as ESC where applicable.
- Text Edit: Backspace deletes, ` cancels, Enter saves.
  
| Mode | Item | Notes |
|---|---|---|
| `R` (RX) | `1..6` | Select a decoded line to reply to. CQ messages are sorted from strongest to weakest. If selected within 4 s, TX starts immediately. |
|  | `▲` `▼` | Page up/down is available when line 1 or line 6 is cyan. |
| `T` (TX Queue) | `1` | Rotate the queue to the next same-parity entry. |
|  | `2..6` | Drop the queue item on the current page. |
|  | `` ` `` | Cancel TX immediately. |
| `G` (GPS) |  | View live GPS telemetry including 3D fix, satellites, UTC time, grid square, and last synchronization age. The first line shows the active source. |
|  | `1` | Cycle GPS source: PORTA (Grove AT6668) → CAP (LoRa-1262 stacking cap on G15/G13) → Off. Persists across reboots. |
| `S` (STATUS) | `1` | Cycle Beacon mode. Applies when leaving STATUS mode. |
|  | `2` | Run connect/sync now; starts audio and follows the CAT sync path. |
|  | `3` | Step to the next active band. Applies after key 2 is pressed or when leaving STATUS. |
|  | `4` | Toggle Tune. |
|  | `5` | Edit Date (in place). `G` indicates date/time is synced from GPS. |
|  | `6` | Edit Time (in place). |
| `M` (MENU P1) | `1` | Cycle CQ Type. For CQ FD, enter operating class and ARRL/RAC section in FreeText, for example `1B SCV`. |
|  | `2` | Send FreeText once. |
|  | `3` | Edit FreeText (Long Edit). Used for SOTAMAT, park/summit reference, ARRL Field Day exchange, CQ modifiers (`CQ EU`, `CQ ASIA`), and similar text. |
|  | `4` | Edit Call (in place). |
|  | `5` | Edit Grid (in place). Supports 4/6/8-character grid. If GPS is available, the GPS grid is shown and used, but not saved. |
|  | `6` | Enter Sleep. Shows battery info. |
| `N` (MENU P2) | `1` | Select offset source: Random / RX / Fixed. Random values are within 500-2500 Hz. |
|  | `2` | Edit fixed cursor offset (in place). Enter directly or use `▲` `▼` `◀` `▶`. |
|  | `3` | Select radio (`QMX` / `KH1`). |
|  | `4` | Edit ignore list (Long Edit). Prefixes are separated by spaces; maximum 64 characters. |
|  | `5` | Edit comment (Long Edit). Used for ADIF logging. Supports `/Radio` and `/Grid` macro expansion. |
|  | `6` | Turn BLE on/off. Device name is `Mini-FT8-<callsign>`. |
| `O` (MENU P3) | `1` | Turn RxTx log on/off. Note: RxTxLog has been renamed to `RT[YYMMDD].txt`. |
|  | `2` | Turn SkipTX1 on/off. Skips `dxcall mycall mygrid` and replies with the SNR report. |
|  | `3` | Edit active bands (Long Edit). Used by STATUS -> Band. |
|  | `4` | Edit RTC compensation (in place). |
|  | `5` | Copy files to SD. Feedback is `Copied OK` or `Missed [n]`. |
|  | `6` | Edit max retry (in place). Accepts any natural number or `0`. |
| `Q` (QSO) | `1..6` | Open the selected ADIF file. |
|  | `◀` `▶` | Switch columns (Default view or SNR view). |
| `F` (Fetch) | `1..6` | Select and send a file over BLE. |
| `D` (Delete Files) | `1..6` | Delete the selected file immediately, without confirmation. |
| `B` (BAND) | `1..6` | Choose a band slot to edit. |
| `C` (Connect) |  | USB serial command mode for host commands. Available only before connecting to a radio. Type `help` on the PC to list host-side commands. (removed in V2.0.4)|
| `P` (PERFORMANCE) | | A Simple Performance Monitor. (added in V2.0.4) |

## Download Logs

- Use SD
  - Insert a FAT/FAT32-formatted SD card.
  - In MENU P3 (`O`), press `5` (Copy files to SD). All files will be copied to the SD card.
  - If the result shows `Missed`, a reboot will usually fix it.

- Use BLE
  - In the BLE Terminal, send `F` and choose the file.
  - Use "Send Log File" in the BLE Terminal to save or email it.

- Use `pc_terminal.py`
  - On the M5 Cardputer, press `C` to enter communication mode.
  - On the PC, run `python .\pc_terminal.py COM11` for interactive use with multiple commands.
  - On the PC, run `python .\pc_terminal.py COM11 read 20260113.txt` for a single command.

## GPS Connections

Two GPS sources are supported, selectable from the GPS view (`G`, press `1` to cycle). The chosen source is persisted in `Station.txt` (`gps_source=`).

- **PORTA** (default): any NMEA UART module wired to the Grove PORTA. Both 9600 and 115200 baud are supported (auto-detected). **Make sure the micro switch is on the left.** Once Mini-FT8 gets its time/grid, the GPS can be removed, this is important for KH1.
- **CAP**: the M5Stack [Cap LoRa-1262](https://docs.m5stack.com/en/cap/Cap_LoRa-1262) stacking cap (ATGM336H GNSS). No wiring needed — it plugs onto the Cardputer ADV's top header. Fixed 115200 baud, lives on UART2/G15/G13, so it coexists with KH1 CAT (which uses UART1/G1/G2). The LoRa side is not used by this firmware.

While waiting for a fix, the GPS view shows a diagnostic line `NoFix Q:N V:N` followed by `Up:Ns Rx:NkB L:N`:

- **Q** — fix quality reported by the chip (0 = no fix, 1 = GPS, 2 = DGPS).
- **V** — satellites visible across all constellations (from GSV).
- **Up** — seconds since GPS started; cold start is typically 30–60 s with sky view, 5+ min near a window.
- **Rx / L** — bytes received and parser lock (both should climb; `L1` means NMEA is being decoded).

If `V` stays at 0 even outside, the antenna isn't hearing satellites — most often a sky-view problem, especially with the CAP's small patch antenna.

```text
┌──────────────────┐                 ┌─────────────────────────────┐
│ GPS              │                 │ Cardputer ADV               │
│                  │                 │ PORTA                       │
│ GND ─────────────┼─────────────────┤ GND                         │
│ VDD ─────────────┼─────────────────┤ 5V                          │
│ RX  ─────────────┼<──(Not Used)────┤ TX (G2)                     │
│ TX  ─────────────┼────────────────>┤ RX (G1)                     │
└──────────────────┘                 │                             │
                                     │ SW: 5VOUT (Left)            │
                                     └─────────────────────────────┘
```
## KH1 Connections
![KH1 Cables](kh1_cables.jpeg)

 - TX Only ([sotamat](https://sotamat.com/))
```text
┌──────────────────┐                 ┌────────────────────────────┐
│ KH1 RS232        │                 │ Cardputer ADV              │
│                  │                 │ PORTA                      │
│ GND ─────────────┼─────────────────┤ GND                        │
│                  │                 │ 5V (NC)                    │
│ Tip(Rx) ─────────┼<────────────────┤ TX (G2)                    │
│ Ring(TX) ────────┼───(Not Used)───>┤ RX (G1)                    │
└──────────────────┘                 │                            │
                                     │ SW: NA                     │
                                     └────────────────────────────┘
```
- TX + RX (FT8 QSO) (V2.0.2 Supports Cardputer direct microphone in, so the **USB-C adapter becomes optional, choose KH1-Mic**)
  - Use a USB-C audio/mic adapter for RX. Tested adapter: Amazon `B0FWC9ZFC4`. Other adapters may also work, but this one is confirmed.
  - Supply 5 V to PORTA; otherwise, the USB-C OTG port will not be powered. **Make sure the micro swithch is on the right**
```text
┌──────────────────┐
│ Power Cable      │
│ GND ─────────────┼─────────┐
│ 5V  ─────────────┼─────┐   │
└──────────────────┘     |   |
┌──────────────────┐     |   |       ┌────────────────────────────┐
│ KH1 RS232        │     |   |       │ Cardputer ADV              │
│                  │     |   |       │ PORTA                      │
│ GND ─────────────┼─────)───┴───────┤ GND                        │
│                  │     └───────────┤ 5V                         │
│ Tip(Rx) ─────────┼<────────────────┤ TX (G2)                    │
│ Ring(TX) ────────┼── (Not Used)───>┤ RX (G1)                    │
└──────────────────┘                 │                            │
                                     │ SW: 5VIN (Right)           │
                                     └────────────────────────────┘
```

- Mini-FT8 automatically sets KH1 TX power to 2 W.
- For best RX performance, reduce AF volume to `05` or `06`.
