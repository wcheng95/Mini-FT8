#!/usr/bin/env bash
# Dump the node's storage partition (FATFS: ADIF logs, RT logs, Station.txt)
# to a timestamped file BEFORE flashing. Run this every time, before
# idf.py flash — the partition survives a flash, but it does not survive a
# mount failure at the next boot, and on 2026-09-13 a reflash cost a day's
# POTA log that way.
#
#   . ~/esp/idf.sh && tools/dump_storage.sh [port] [out_dir]
#
# Reads flash through the ROM bootloader: the app never runs, so nothing is
# written to the partition while we read it. Do NOT boot the node or mount
# it over MSC first if you're trying to recover — macOS writes .fseventsd
# and friends onto a freshly mounted volume, straight over the old data.
set -euo pipefail

PORT="${1:-$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)}"
OUT="${2:-$HOME/mini_ft8_storage_dumps}"
[ -n "$PORT" ] || { echo "no usbmodem port — is the node plugged in (and not in MSC mode)?" >&2; exit 1; }

# From partitions.csv: storage, data, fat, 0x190000, 1M
OFFSET=0x190000
SIZE=0x100000

mkdir -p "$OUT"
FILE="$OUT/storage_$(date +%Y%m%d_%H%M%S).bin"
esptool.py --chip esp32s3 --port "$PORT" read_flash "$OFFSET" "$SIZE" "$FILE"
echo "storage partition -> $FILE"
echo "recover files with: python3 tools/carve_storage.py $FILE <out_dir>"
