"""
Chunked WRITEBIN uploader with per-chunk CRC and ACK.

Protocol:
- Send: WRITEBIN <file> <size> <crc_hex>\r\n
- Device replies: OK: send <size> bytes, chunk <N> +4crc
- For each chunk (payload <= N):
    PC sends: payload bytes + 4-byte CRC32 (little-endian) of payload
    Device replies: ACK <bytes_received>
- Device finishes with OK crc <crc> (or ERROR) and prompt.

"""

import binascii
import struct
import sys
import time
from pathlib import Path

import serial


def read_until_token(ser: serial.Serial, token: bytes, timeout_s: float) -> bytes:
    start = time.time()
    buf = b""
    while time.time() - start < timeout_s:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            if token in buf:
                return buf
        else:
            time.sleep(0.01)
    return buf


def read_collect(ser: serial.Serial, token: bytes, timeout_s: float) -> bytes:
    start = time.time()
    buf = b""
    while time.time() - start < timeout_s:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            if token in buf:
                break
        else:
            time.sleep(0.01)
    return buf


def fmt_hex(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def upload(port: str, local_path: str, remote_name: str):
    data = Path(local_path).read_bytes()
    size = len(data)
    # CRC matches firmware crc32_update(0, data)
    crc_total = binascii.crc32(data) & 0xFFFFFFFF
    cmd = f"WRITEBIN {remote_name} {size} {crc_total:08X}\r\n".encode()
    print(f"Using {size} bytes, total CRC {crc_total:08X}")
    print(f"Expect FIRST8: {fmt_hex(data[:8])}")
    print(f"Expect LAST8 : {fmt_hex(data[-8:]) if size >= 8 else fmt_hex(data)}")
    if size >= 1:
        first_chunk = data[:min(512, size)]
        crc_chunk0 = binascii.crc32(first_chunk) & 0xFFFFFFFF
        print(f"Chunk0 CRC: {crc_chunk0:08X}, len {len(first_chunk)}")

    with serial.Serial(port, 115200, timeout=0.2) as ser:
        ser.reset_input_buffer()

        # Sync prompt
        ser.write(b"\r\n")
        ser.flush()
        sync = read_until_token(ser, b"MINIFT8>", timeout_s=2.0)
        print(f"Sync: {sync!r}")

        ser.write(cmd)
        ser.flush()
        resp = read_until_token(ser, b"OK: send", timeout_s=3.0)
        print(f"Cmd resp: {resp!r}")
        if b"OK: send" not in resp:
            raise RuntimeError("Did not get OK: send")

        chunk_size = 512  # match firmware chunking
        sent = 0
        leftover = b""
        while sent < size:
            end = min(sent + chunk_size, size)
            payload = data[sent:end]
            crc_chunk = binascii.crc32(payload) & 0xFFFFFFFF
            ser.write(payload + struct.pack("<I", crc_chunk))  # little-endian CRC per chunk
            ser.flush()
            ack = read_until_token(ser, b"ACK", timeout_s=5.0)
            if b"ACK" not in ack:
                raise RuntimeError(f"Missing ACK after {end} bytes, got {ack!r}")
            leftover += ack  # keep anything the device already sent
            sent = end
            if sent % (16 * 1024) == 0 or sent == size:
                print(f"Sent {sent}/{size}")

        tail = leftover + read_collect(ser, b"MINIFT8>", timeout_s=30.0)
        print(f"Tail: {tail!r}")
        if b"OK crc" not in tail or b"MINIFT8>" not in tail:
            raise RuntimeError("Upload may have failed (missing OK crc or prompt)")

        print("Upload complete.")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <serial_port> <local_file> <remote_name>")
        sys.exit(1)
    upload(sys.argv[1], sys.argv[2], sys.argv[3])
