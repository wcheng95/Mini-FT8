#!/usr/bin/env python3
"""Build-local V2 UAC diagnostic patch.

The managed usb_host_uac component must remain byte-identical to the registry
package. This script copies the pinned source into the build tree and promotes
only the existing bad-isoc debug message to WARN.
"""

import argparse
import hashlib
from pathlib import Path

SOURCE_SHA256 = "2d7549c7e4657744b92079c2c226b558e1934e587ab5030d80974f392eedf75e"

OLD = (
    '                    ESP_LOGD(TAG, "Bad RX Isoc packet %d status %d", '
    'i, in_xfer->isoc_packet_desc[i].status);'
)
NEW = (
    '                    ESP_LOGW(TAG, "V2 RX bad-isoc packet=%d status=%d", '
    'i, in_xfer->isoc_packet_desc[i].status);'
)


def patch_source(source: bytes) -> bytes:
    if hashlib.sha256(source).hexdigest() != SOURCE_SHA256:
        raise ValueError("UAC source differs from pinned registry component")
    text = source.decode()
    if text.count(OLD) != 1:
        raise ValueError("bad-isoc diagnostic patch anchor mismatch")
    return text.replace(OLD, NEW).encode()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    if args.source.resolve() == args.output.resolve():
        parser.error("output must not overwrite managed source")

    try:
        patched = patch_source(args.source.read_bytes())
    except ValueError as error:
        parser.error(str(error))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not args.output.exists() or args.output.read_bytes() != patched:
        args.output.write_bytes(patched)


if __name__ == "__main__":
    main()
