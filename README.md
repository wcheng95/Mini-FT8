Subscribe to [https://freelists.org/list/qrp-portable](https://freelists.org/list/qrp-portable) for announcements, discussions, and updates about my Mini-series apps for the Cardputer ADV.

### Dean, KD3AN had added support for IC705: https://github.com/hamrec/cp705
### Michael, 2E0MIK had added support for FTX-1: https://github.com/mapoby/Mini-FT8

# First POTA activation (v1.0 2025-12-31)
![First POTA Activation](IMG_6087.jpeg)

## Mini-FT8 Release Notice
Mini-FT8 is built on Karlis Goba’s ft8_lib. It’s also a joint adventure between Zhenxing (N6HAN) and Wei (AG6AQ), with inspiration from DXFT8 by Barb (WB2CBA) and Charley (W5BAA). It has been a great learning platform for me, and I hope you find it just as fun to use. It supports FT8 and FT4 with QMX, QDX, and KH1.

### Thanks

- The [DX FT8](https://github.com/WB2CBA/DX-FT8-FT8-MULTIBAND-TABLET-TRANSCEIVER) team — Barb (WB2CBA), Charley (W5BAA), and Paul (G8KIG) — for the inspiration.
- Zhenxing (N6HAN) — for helping build the audio/DSP path (UAC) and autoseq. This project would not have been possible without his help.
- Karlis Goba — for [ft8_lib](https://github.com/kgoba/ft8_lib). Thanks also to Shawn Rutledge for non-standard callsign support.
- OpenAI and Anthropic — for their incredible coding assistance.
  
### Hardware
(I have no affiliation with the vendors.)
  - Must order: https://shop.m5stack.com/products/m5stack-cardputer-adv-version-esp32-s3 or from digikey: https://www.digikey.com/en/products/detail/m5stack-technology-co-ltd/K132-ADV/27685158
  - Optional: [https://shop.m5stack.com/products/gps-bds-unit-v1-1-at6668](https://shop.m5stack.com/products/gps-bds-unit-v1-1-at6668) (PORTA GPS for Date/Time/Grid; other UART NMEA GPS modules work too)
  - Optional: M5Stack LoRa-1262 cap GNSS (set `GNSS_LoRa:ON`; only the GNSS is used)
  - Optional: DS3231 RTC module on I2C `G8/G9` (for retained UTC date/time without GPS)
  - For KH1 TX: https://shop.m5stack.com/products/4pin-buckled-grove-cable, for a custom serial cable
  - For KH1-USBC RX: [USB-C microphone adapter](https://www.amazon.com/dp/B0FWC9ZFC4?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1). Other adapters may also work, but this one is confirmed. KH1-MIC uses the Cardputer built-in microphone, so the USB-C adapter is optional.

73, Wei AG6AQ

# Mini-FT8 Operation Manual

## Quick Mode Map

| Key | Mode | Purpose |
|---|---|---|
| `R` | RX | View decoded messages and tap one to start a QSO. |
| `T` | TX Queue | View and manage the transmit queue. |
| `S` | STATUS | Access beacon, connect/sync, band step, tune, and date/time functions. |
| `G` | GPS | View GPS telemetry and synchronization status. |
| `M` | MENU P1 | Configure core station and operator settings. |
| `N` | MENU P2 | Configure radio, input, and comment settings. |
| `O` | MENU P3 | Configure logging, active bands, GNSS LoRa GPS, copy-to-SD, and retry settings. |
| `Q` | QSO | Browse QSO and log files, and view entries. |
| `D` | Delete Files | Browse and delete files stored in internal FATFS. |
| `B` | BAND | Edit per-band frequencies. |
| `C` | USB Drive | Toggle internal FATFS ownership between Mini-FT8 and the PC. |
| `P` | Performance | View A Simple Performance Monitor. (added in V2.0.4)|

## Global Keys and Navigation

- `R` / `T` / `B` / `S` / `G` / `Q` / `D` / `C`: switch to the selected mode. Press the same mode key again to return to `RX`.
- `M` / `N` / `O`: jump to MENU page 1 / 2 / 3. Press the current page key again to return to `RX`.
- `` ` ``: cancel TX globally in `RX`, `TX`, and `STATUS` when not editing.
- `▲` / `▼`: page up / page down in `RX`, `TX`, `BAND`, `MENU`, `QSO`, and `Delete`.
- `◀` / `▶`: move left / right in `QSO-SNR`, `STATUS` date/time, `MENU P2` (N->2).
- `1`..`6`: always select the currently visible row in the active mode.

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
| `G` (GPS) |  | View live GPS telemetry including active source, 3D fix, satellites, UTC time, grid square, and last synchronization age. |
| `S` (STATUS) | `1` | Cycle Beacon mode. Applies when leaving STATUS mode. |
|  | `2` | Run connect/sync now; starts audio and follows the CAT sync path. |
|  | `3` | Step to the next active band. Applies after key 2 is pressed or when leaving STATUS. |
|  | `4` | Toggle Tune. |
|  | `5` | Edit Date (in place). On the Time line, `G` means GPS time and `R` means DS3231 RTC time. |
|  | `6` | Edit Time (in place). |
| `M` (MENU P1) | `1` | Cycle CQ Type. For CQ FD, enter operating class and ARRL/RAC section in FreeText, for example `1B SCV`. |
|  | `2` | Send FreeText once. |
|  | `3` | Edit FreeText (Long Edit). Used for SOTAMAT, park/summit reference, ARRL Field Day exchange, CQ modifiers (`CQ EU`, `CQ ASIA`), and similar text. |
|  | `4` | Edit Call (in place). |
|  | `5` | Edit Grid (in place). Supports 4/6/8-character grid. If GPS is available, the GPS grid is shown and used, but not saved. |
|  | `6` | Enter Sleep. Shows battery info. |
| `N` (MENU P2) | `1` | Select offset source: Random / RX / Fixed. Random values are within 500-2500 Hz. |
|  | `2` | Edit fixed cursor offset (in place). Enter directly or use `▲` `▼` `◀` `▶`. |
|  | `3` | Select radio (`QMX` / `QDX` / `FX-4CR` / `truSDX` / `KH1-USBC` / `KH1-MIC`). |
|  | `4` | Edit ignore list (Long Edit). Prefixes are separated by spaces; maximum 64 characters. |
|  | `5` | Edit comment (Long Edit). Used for ADIF logging. Supports `/Radio` and `/Grid` macro expansion. |
|  | `6` | Select FT8 / FT4 protocol. Reboot to apply the change. |
| `O` (MENU P3) | `1` | Turn RxTx log on/off. Note: RxTxLog has been renamed to `RT[YYMMDD].txt`. |
|  | `2` | Turn SkipTX1 on/off. Skips `dxcall mycall mygrid` and replies with the SNR report. |
|  | `3` | Edit active bands (Long Edit). Used by STATUS -> Band. |
|  | `4` | Toggle `GNSS_LoRa`. `OFF` uses PORTA GPS; `ON` uses the LoRa-1262 cap GNSS. |
|  | `5` | Copy files to SD. Feedback is `Copied OK` or `Missed [n]`. |
|  | `6` | Edit max retry (in place). Accepts any natural number or `0`. |
| `Q` (QSO) | `1..6` | Open the selected ADIF file. |
|  | `◀` `▶` | Switch columns (Default view or SNR view). |
| `D` (Delete Files) | `1..6` | Delete the selected file immediately, without confirmation. |
| `B` (BAND) | `1..6` | Choose a band slot to edit. |
| `C` (USB Drive) |  | Stop radio audio and expose FATFS to the PC. Safely eject it on the PC, then press `C` again to remount storage and return to RX. |
| `P` (PERFORMANCE) | | A Simple Performance Monitor. (added in V2.0.4) |

## Download Logs

- Mini-FT8 and Mini-CW share the `fatfs` partition. Their files can coexist,
  and current M5Launcher installs/reinstalls can switch between the applications
  while preserving an existing compatible FATFS partition. Both applications
  use 512-byte FATFS and wear-levelling sectors.

- Use SD
  - Insert a FAT/FAT32-formatted SD card.
  - In MENU P3 (`O`), press `5` (Copy files to SD). All files will be copied to the SD card.
  - If the result shows `Missed`, a reboot will usually fix it.

## GPS Connections

Mini-FT8 supports two GPS sources selected from MENU P3 (`O -> 4`):

- `GNSS_LoRa:OFF` uses the PORTA GPS wiring below. Both 9600 and 115200 baud GPS modules are supported and auto-detected. **Make sure the micro switch is on the left.** Once Mini-FT8 gets its time/grid, the GPS can be removed, this is important for KH1.
- `GNSS_LoRa:ON` uses the M5Stack LoRa-1262 cap GNSS on UART2 (`RX=G15`, `TX=G13`) at 115200 baud. The LoRa/SX1262 radio side is not used. This source can keep running while KH1 CAT uses PORTA/UART1.

When `GNSS_LoRa` is `ON`, the physical G4/G5 debug UART path is disabled and the pins are left as floating inputs to avoid conflicts. USB Serial/JTAG host commands still work.

The GPS view shows the active source on its first line.
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

## DS3231 RTC Connections

Mini-FT8 can use an optional DS3231 module as an external UTC clock. Connect it
to the Cardputer Adv shared I2C bus: `SDA=G8`, `SCL=G9`, plus module power and
ground. On boot, a valid DS3231 time is used before the ESP RTC or saved
`Station.txt` time. Status `S -> 6` appends `R` when the active time came from
the DS3231, and appends `G` after a full GPS time sync. GPS and manual time
updates write the DS3231 when it is present; FT8 decode fine corrections do not.

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
- TX + RX (FT8/FT4 QSO)
  - Choose `KH1-USBC` for USB-C audio adapter RX. Tested adapter: Amazon `B0FWC9ZFC4`. Other adapters may also work, but this one is confirmed.
  - Choose `KH1-MIC` for Cardputer microphone RX. No USB-C audio adapter is needed.
  - For `KH1-USBC`, supply 5 V to PORTA; otherwise, the USB-C OTG port will not be powered. **Make sure the micro switch is on the right**
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

## FX-4CR (F5BUD firmware) Connections

The FX-4CR's USB port leads to internal USB hubs. Behind them are a C-Media USB sound card (RX and TX audio),
a CH340 USB-serial bridge for CAT (Kenwood TS-590S protocol, 115200 baud) and a Bluetooth module.

- Select `FX-4CR` in MENU P2 (`N` -> `3`), then connect with `S` -> `2`.
- Power the radio from its own 12 V supply (XT60) as usual. The USB port does not power the radio.
- A USB host must provide 5 V on the cable for the radio's hub, sound card and serial bridge. Supply 5 V from an external source (power bank/adapter) to PORTA and **set the micro switch to the right** (5VIN), the same as for KH1-USBC. The Cardputer passes this 5 V through to USB-C.
- Use the radio's own USB cable: TRRS-to-USB-A on V/VR units, USB-C on R units. Add a USB-A (female) to USB-C (male) OTG adapter for the Cardputer.
- Radio settings:
  - `Digital mode (CAT)` = `DATA` (the default). Mini-FT8 sends `MD2;`, which the F5BUD firmware maps to DATA-U. Only in DATA-L/U does the radio take TX audio from USB.
  - Mini-FT8 holds RTS and DTR low, so `RTS as PTT` does not key the radio.
  - Adjust `Aux-Input-Level` / `Aux-Output-Level` (Usb & Bt) for a clean TX signal and a sensible RX level.
- TX keys with CAT `TX1;` (data send) and plays a 48 kHz FT8/FT4 waveform on the USB sound card, the same as QDX. `RX;` ends the transmission.
- The ESP32-S3 USB host has only 8 channels, and the radio's hubs and devices permanently use 7 of them. Mini-FT8 therefore takes turns with the last one:
  - CAT commands are sent in short bursts, at connect, at each band change and at TX start/end. RX audio pauses for a fraction of a second during each burst.
  - The Bluetooth module's audio device is ignored.
- STATUS line 2 shows the CAT state: `CAT ok 1a86:7523` after a successful command burst, or `CATerr <error>`. After a failure, retries back off from 3 s up to 60 s, and the automatic sync stops after 3 attempts so RX keeps decoding. Press `S` -> `2` to retry.

```text
┌───────────────────┐   USB (hub: C-Media audio + serial bridge)   ┌─────────────────────┐
│ FX-4CR            ├──────────────────────────────────────────────┤ USB-C  Cardputer ADV│
│                   │                                               │                     │
└───────────────────┘                   5 V supply ─────────────────┤ PORTA 5V / GND      │
                                                                    │ SW: 5VIN (Right)    │
                                                                    └─────────────────────┘
```

## (tr)uSDX Connections

The (tr)uSDX has no USB sound card. Its Micro-USB port is a CH340G serial bridge to the ATmega. Mini-FT8 uses
the firmware's **CAT audio streaming**:
- RX audio arrives as 8-bit samples at about 7.8 kHz inside the CAT link, enabled with `UA2;`, which also mutes the radio speaker.
- TX audio is sent back as 8-bit samples at 11.52 kHz after `TX0;`.

- Requires truSDX firmware **2.00u or newer**; CAT runs at 115200 baud from 2.00t on.
- Select `truSDX` in MENU P2 (`N` -> `3`).
- **Always power the radio from its own 12 V supply.** Without it, the truSDX runs the whole radio from USB 5 V (about 0.5 W out, several hundred mA in TX), and that current would flow through the Cardputer's PORTA and USB-C path.
- Supply 5 V from an external source to PORTA and **set the micro switch to the right** (5VIN). With 12 V connected, this only feeds the CH340 serial bridge, a few tens of mA.
- To keep the radio's USB current off the Cardputer entirely, put a self-powered USB hub between the Cardputer and the radio. Hub support is enabled in this build.
- Cable: Micro-USB (male) to USB-C (male) OTG cable or adapter.
- After the port opens, Mini-FT8 waits 3 s, because opening the CH340 resets the ATmega. It then sends `MD2;UA2;` and the current band frequency.
- Radio menu recommendations:
  - `TX Drive` = `8` (constant amplitude).
  - Semi-QSK `OFF`.
  - Calibrate `Ref freq` for accurate frequency.
- Keep the TX offset between about 200 and 2300 Hz; truSDX TX audio bandwidth is about 2.4 kHz.
- Decoding works from 8-bit audio, but the dynamic range is only about 46 dB. Keep the RX gain/attenuator so the band noise is not clipping.
- STATUS line 2 shows `truSDX rx <rate>Hz` once streaming. The RX sample rate is measured continuously; it differs between firmware builds, e.g. about 7936 Hz on 2.00x.

```text
┌───────────────────┐        Micro-USB (CH340G, CAT + audio)        ┌─────────────────────┐
│ (tr)uSDX          ├───────────────────────────────────────────────┤ USB-C  Cardputer ADV│
│                   │                                               │                     │
└───────────────────┘                   5 V supply ─────────────────┤ PORTA 5V / GND      │
                                                                    │ SW: 5VIN (Right)    │
                                                                    └─────────────────────┘
```
