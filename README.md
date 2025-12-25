# First milestone (v0.1 2025-12-24)
With help from Zhenxing, N6HAN, we have our first on air ft8 message decoded on M5 Cardputer
![First on air FT8 decoded message](IMG_5208.png)

# MiniFT8 (ESP32-S3 / M5 Cardputer)

Offline FT8 decoder/utility for M5 Cardputer (ESP32-S3) with menu-driven UI, software RTC, host file upload, and WAV streaming decode.

## Build & Flash

Requires ESP-IDF (tested v5.5.1) and M5GFX/M5Cardputer components vendored in `components/`.

```sh
idf.py build
idf.py -p COM11 flash  # adjust port
```

## Filesystem

SPIFFS partition size: ~0.93 MB (`partitions.csv`). Place files under `/spiffs`, e.g. `/spiffs/kfs40m.wav`. You can upload via USB host mode command `WRITEBIN <file> <size> <crc32_hex>` (see Host mode below).

## UI & Controls

- Modes: `R` RX, `T` TX, `B` Band, `M` Menu, `S` Status, `D` Debug, `L` List, `H` Host.
- RX: press `x` to stream/decode `/spiffs/kfs40m.wav`; select lines `1-6` to enqueue replies (brief flash feedback).
- TX: `;`/`.` page; `e` encodes/logs tones; toggle delete on lines `2-6`.
- Menu (2 pages): CQ type, SkipTX1, Send FreeText (queues without leaving M, flashes briefly), Call/Grid, Offset source, Radio, Antenna, Comment, Battery. Long edits (`4` for FreeText, `4` on page2 for Comment) show dedicated editor. Uppercasing for call/grid/FreeText. Offset/Radio/etc. with `1/2/...`.
- Status: edit Date/Time with digit overwrite and `/ ,` for cursor move; tune toggle; beacon/offset/band. Time updates per second without flicker.
- Host: USB serial (CDC/JTAG). Commands: `WRITEBIN`, `WRITE/APPEND`, `READ/DELETE`, `LIST/INFO/HELP`, `EXIT`. Binary upload with per-chunk CRC/ACK; echoes first/last bytes and CRC on success.
- Debug/List: paged views.
- Countdown bar: green even slots, red odd; waterfall displays yellow gradient.

## Streaming Decode

Press `x` in RX to stream `/spiffs/kfs40m.wav` in real-time 160 ms frames; starts at next 15 s boundary (aligned to software RTC), decodes after ~80 blocks (~12.8 s). Continues through longer files, decoding each slot in sequence.

## RTC

Software RTC settable in Status page; ticks independently of GPS; used for slot alignment and display.

## Notes

- USB is dual-role; you must switch roles (don’t host and connect to PC simultaneously).
- Battery line shows percent and icon; charging indicated with bolt and “CHG”.
- dot -Tpng -Gdpi=200 flow.dot -o flow.png `choco install graphviz`
