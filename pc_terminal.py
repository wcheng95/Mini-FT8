#!/usr/bin/env python3
"""
Minimal PC terminal for Mini-FT8 control mode over USB serial (COMx).
Supports:
  - help/list/info commands (prints response)
  - read <remote_path>  -> saves to local file (same basename unless --output)
  - write <local_path>  -> uploads to remote using WRITEBIN protocol

Protocol reference (from device firmware):
  WRITEBIN <file> <size> <crc32_hex>
    Then stream binary in chunks (<=512 bytes) each followed by 4-byte CRC32 LE.
    Device replies with "ACK <total_written>" per chunk.
  READ/INFO/LIST/HELP
    Text responses terminated by prompt "MINIFT8> ".
"""

import argparse
import os
import sys
import time
import zlib
import serial

PROMPT = b"MINIFT8> "
CHUNK = 512


def read_until_prompt(ser, echo=True, timeout=5.0):
    """Read lines until prompt seen or timeout; return list of lines (bytes, without CRLF)."""
    lines = []
    deadline = time.monotonic() + timeout
    while True:
        if time.monotonic() > deadline:
            break
        b = ser.readline()
        if not b:
            continue
        b = b.rstrip(b"\r\n")
        if b == PROMPT.rstrip():
            break
        lines.append(b)
        if echo:
            print(b.decode(errors="replace"))
        # Some firmwares echo prompt on same line
        if b.endswith(PROMPT.rstrip()):
            break
    return lines


def send_line(ser, line: str):
    ser.write(line.encode() + b"\n")
    ser.flush()


def cmd_simple(ser, cmd: str):
    send_line(ser, cmd)
    read_until_prompt(ser, echo=True, timeout=5.0)


def cmd_read(ser, remote: str, local: str):
    send_line(ser, f"READ {remote}")
    lines = read_until_prompt(ser, echo=False, timeout=15.0)
    with open(local, "wb") as f:
        for ln in lines:
            f.write(ln + b"\n")
    print(f"Saved {len(lines)} lines to {local}")


def cmd_write(ser, local: str, remote: str):
    size = os.path.getsize(local)
    with open(local, "rb") as f:
        data = f.read()
    crc_all = zlib.crc32(data) & 0xFFFFFFFF
    send_line(ser, f"WRITEBIN {remote} {size} {crc_all:08X}")

    sent = 0
    view = memoryview(data)
    while sent < size:
        chunk = view[sent:sent + CHUNK]
        crc = zlib.crc32(chunk) & 0xFFFFFFFF
        ser.write(chunk.tobytes() + crc.to_bytes(4, "little"))
        ser.flush()
        # Wait for ACK line
        line = ser.readline().rstrip(b"\r\n")
        if line.startswith(b"ACK"):
            sent = int(line.split()[1])
            print(line.decode())
        elif line.startswith(b"ERROR"):
            print(line.decode())
            sys.exit(1)
        else:
            # Unexpected, but continue reading until ACK
            print(line.decode())
    # Consume prompt
    read_until_prompt(ser, echo=True, timeout=10.0)
    print(f"Uploaded {size} bytes to {remote}")


def main():
    ap = argparse.ArgumentParser(description="Mini-FT8 PC terminal")
    ap.add_argument("port", help="COM port (e.g., COM7)")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    ap.add_argument("command", nargs="*", help="Optional one-shot command with args (help/list/info/read/write ...)")
    args = ap.parse_args()

    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.5)
    time.sleep(0.1)
    ser.reset_input_buffer()

    def run_one(cmd_parts):
        if not cmd_parts:
            return
        cmd = cmd_parts[0].lower()
        if cmd in ("help", "list", "info"):
            cmd_simple(ser, cmd.upper())
        elif cmd == "read" and len(cmd_parts) >= 2:
            remote = cmd_parts[1]
            local = os.path.basename(remote)
            if len(cmd_parts) >= 3:
                local = cmd_parts[2]
            cmd_read(ser, remote, local)
        elif cmd == "write" and len(cmd_parts) >= 2:
            local = cmd_parts[1]
            remote = os.path.basename(local)
            if len(cmd_parts) >= 3:
                remote = cmd_parts[2]
            cmd_write(ser, local, remote)
        else:
            print("Unknown/invalid command:", " ".join(cmd_parts))

    # One-shot command if provided
    if args.command:
        run_one(args.command)
        return

    # Interactive loop
    print("Connected. Type commands: help, list, info, read <remote> [local], write <local> [remote], or quit/exit.")
    # Consume initial prompt if present
    ser.write(b"\r\n")
    ser.flush()
    read_until_prompt(ser, echo=False)
    while True:
        try:
            line = input("MINIFT8> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        if line.lower() in ("quit", "exit"):
            break
        parts = line.split()
        run_one(parts)

    ser.close()


if __name__ == "__main__":
    main()
