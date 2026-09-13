#!/usr/bin/env python3
"""Recover Mini-FT8 log files from a raw dump of the storage partition.

The storage partition is FATFS (FAT12, 4 KB sectors = 4 KB clusters) behind
ESP-IDF's wear-levelling layer. When the firmware reformats it (mount failure
with format_if_mount_failed=true), f_mkfs rewrites only the boot sector, the
FAT and the root directory. The data clusters are untouched — the filenames,
sizes and cluster chains are gone, the bytes are not.

So this doesn't try to parse FAT structures at all. It scans every 4 KB block
of the dump for text that matches the files the firmware writes:

  RT log lines   T|R|Q [YYYYMMDD HHMMSS][freq] ...
  ADIF records   <call:N>... <eor>
  ADIF header    ADIF EXPORT / <eoh>
  Station.txt    key=value lines

and reassembles them by their own timestamps. Wear levelling means physical
block order says nothing about logical order, and the RT log and ADIF of one
day were appended alternately, so their clusters interleave — both are
handled by sorting on the timestamps every line carries.

A record that straddles two clusters shows up as a tail fragment in one block
and a head fragment in another; the two are re-joined when their
concatenation parses as a valid line.

Usage:
    esptool.py --port /dev/cu.usbmodemXXXX read_flash 0x190000 0x100000 storage.bin
    python3 tools/carve_storage.py storage.bin recovered/

Prints what it found; writes RTyymmdd.TXT and yyyymmdd.txt files that the
firmware would accept back (copy them onto the MSC volume).
"""
import os
import re
import sys
from collections import defaultdict

BLOCK = 4096

RT_LINE   = re.compile(rb'^([TRQL]) \[(\d{8}) (\d{6})\]\[(\d+\.\d{3})\] (.*)$')
ADIF_REC  = re.compile(rb'<call:\d+>.*?<eor>', re.DOTALL | re.IGNORECASE)
ADIF_DATE = re.compile(rb'<qso_date:\d+>(\d{8})', re.IGNORECASE)
ADIF_TIME = re.compile(rb'<time_on:\d+>(\d{6})', re.IGNORECASE)
STATION   = re.compile(rb'^[a-z_]+=.*$', re.IGNORECASE)

PRINTABLE = set(range(0x20, 0x7f)) | {0x09, 0x0a, 0x0d}


def is_texty(block: bytes) -> bool:
    """A cluster that ever held one of our text files: mostly printable,
    or erased flash (0xFF) after a printable prefix."""
    stripped = block.rstrip(b'\xff').rstrip(b'\x00')
    if not stripped:
        return False
    good = sum(1 for b in stripped if b in PRINTABLE)
    return good / len(stripped) > 0.95


def main(dump_path: str, out_dir: str) -> int:
    data = open(dump_path, 'rb').read()
    print(f'dump: {len(data)} bytes, {len(data) // BLOCK} blocks')

    rt_lines: dict[bytes, tuple] = {}       # dedupe on full line
    adif_recs: dict[bytes, tuple] = {}
    station: list[bytes] = []
    heads: list[bytes] = []                 # block-start fragments (no line start)
    tails: list[bytes] = []                 # block-end fragments (no newline)
    texty = 0

    def take_line(line: bytes) -> bool:
        m = RT_LINE.match(line)
        if m:
            kind, date, time, freq, rest = m.groups()
            rt_lines[line] = (date, time, kind, rest)
            return True
        if STATION.match(line) and b'<' not in line:
            station.append(line)
            return True
        return False

    for i in range(0, len(data), BLOCK):
        block = data[i:i + BLOCK]
        if not is_texty(block):
            continue
        texty += 1
        text = block.rstrip(b'\xff').rstrip(b'\x00')

        # ADIF records first — they're self-delimiting and may contain newlines
        for m in ADIF_REC.finditer(text):
            rec = m.group(0)
            d = ADIF_DATE.search(rec)
            t = ADIF_TIME.search(rec)
            adif_recs[rec] = (d.group(1) if d else b'00000000',
                              t.group(1) if t else b'000000')

        lines = text.split(b'\n')
        # A block that starts mid-line: its first "line" is a head fragment
        # unless it happens to be a complete line on its own.
        first, last = lines[0], lines[-1]
        middle = lines[1:-1] if len(lines) > 1 else []

        if not take_line(first) and first and not ADIF_REC.search(first):
            heads.append(first)
        for ln in middle:
            take_line(ln)
        if last:                                   # no trailing newline
            if not take_line(last) and not ADIF_REC.search(last):
                tails.append(last)

    print(f'text blocks: {texty}')
    print(f'complete RT lines: {len(rt_lines)}, ADIF records: {len(adif_recs)}, '
          f'station lines: {len(station)}')
    print(f'fragments: {len(tails)} tails, {len(heads)} heads')

    # Re-join cluster-straddling lines. A tail is the unterminated end of a
    # block, a head the line-less start of another; concatenating the right
    # pair restores the line. Only unambiguous pairs are accepted — a tail
    # like "R [20260906 0048" would "validate" against many heads — so a
    # tail must match exactly one head and that head exactly one tail.
    def parses(cand):
        return RT_LINE.match(cand) is not None or ADIF_REC.search(cand) is not None
    matches = {ti: [hi for hi, h in enumerate(heads) if parses(t + h)]
               for ti, t in enumerate(tails)}
    head_hits = defaultdict(list)
    for ti, hs in matches.items():
        for hi in hs:
            head_hits[hi].append(ti)
    joined = ambiguous = 0
    for ti, hs in matches.items():
        if len(hs) != 1 or len(head_hits[hs[0]]) != 1:
            if hs: ambiguous += 1
            continue
        cand = tails[ti] + heads[hs[0]]
        m = RT_LINE.match(cand)
        if m:
            kind, date, time, freq, rest = m.groups()
            rt_lines[cand] = (date, time, kind, rest)
            joined += 1
            continue
        m = ADIF_REC.search(cand)
        if m and m.group(0) not in adif_recs:
            rec = m.group(0)
            d = ADIF_DATE.search(rec); tt = ADIF_TIME.search(rec)
            adif_recs[rec] = (d.group(1) if d else b'00000000',
                              tt.group(1) if tt else b'000000')
            joined += 1
    print(f'ambiguous fragment pairs left unjoined: {ambiguous}')
    print(f'rejoined across clusters: {joined}')

    os.makedirs(out_dir, exist_ok=True)

    # RT logs: one file per day, sorted by timestamp. Sort is stable, and
    # lines carry no sequence number, so same-second lines keep dump order.
    by_day = defaultdict(list)
    for line, (date, time, kind, rest) in rt_lines.items():
        by_day[date].append((time, line))
    for date, items in sorted(by_day.items()):
        items.sort(key=lambda x: x[0])
        name = f'RT{date[2:].decode()}.TXT'
        with open(os.path.join(out_dir, name), 'wb') as f:
            for _, line in items:
                f.write(line + b'\n')
        n_tx = sum(1 for _, l in items if l.startswith(b'T '))
        print(f'  {name}: {len(items)} lines ({n_tx} TX)')

    # ADIF: one file per day, header + records sorted by time.
    by_day = defaultdict(list)
    for rec, (date, time) in adif_recs.items():
        by_day[date].append((time, rec))
    for date, items in sorted(by_day.items()):
        items.sort(key=lambda x: x[0])
        name = f'{date.decode()}.txt'
        with open(os.path.join(out_dir, name), 'wb') as f:
            f.write(b'ADIF EXPORT\n<eoh>\n')
            for _, rec in items:
                f.write(rec + b'\n')
        calls = [re.search(rb'<call:\d+>(\S+)', r, re.I).group(1).decode()
                 for _, r in items]
        print(f'  {name}: {len(items)} QSOs — {", ".join(calls)}')

    if station:
        with open(os.path.join(out_dir, 'STATION.TXT.recovered'), 'wb') as f:
            f.write(b'\n'.join(dict.fromkeys(station)) + b'\n')
        print(f'  STATION.TXT.recovered: {len(set(station))} lines')

    return 0


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1], sys.argv[2]))
