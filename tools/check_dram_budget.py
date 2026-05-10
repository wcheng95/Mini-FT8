#!/usr/bin/env python3
"""
check_dram_budget.py — POST_BUILD DRAM headroom checker for Mini-FT8.

Usage (standalone):
    python tools/check_dram_budget.py <build_dir> [--variant ft8|ft4]

Hooked automatically as a CMake POST_BUILD step via main/CMakeLists.txt.

Exit codes:
    0  — DRAM budget passes (or idf-size data unavailable — non-fatal)
    1  — DRAM budget exceeded; build should fail

Variant thresholds:
    ft8   50 KB free DIRAM  (NimBLE on, tighter headroom)
    ft4   60 KB free DIRAM  (NimBLE off, more headroom expected)

The thresholds exist to catch silent CDC-ACM allocation failures caused by
NimBLE consuming DRAM at runtime. The FT8 variant failed in exactly this way
during development: NimBLE consumed enough DRAM to prevent CDC-ACM from
allocating its transfer buffers, causing QMX TX to silently not work.
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path


THRESHOLDS = {
    "ft8": 50 * 1024,   # 50 KB — BLE on
    "ft4": 60 * 1024,   # 60 KB — BLE off
}

DEFAULT_VARIANT = "ft8"


def run_idf_size(build_dir: Path) -> dict | None:
    """Run `idf.py size --format json` and return parsed JSON, or None on failure."""
    try:
        result = subprocess.run(
            ["idf.py", "-B", str(build_dir), "size", "--format", "json"],
            capture_output=True,
            text=True,
            timeout=120,
        )
        if result.returncode != 0:
            print(f"[check_dram_budget] idf.py size failed (rc={result.returncode}); skipping check.")
            return None
        return json.loads(result.stdout)
    except (FileNotFoundError, subprocess.TimeoutExpired, json.JSONDecodeError) as e:
        print(f"[check_dram_budget] Could not run idf.py size: {e}; skipping check.")
        return None


def get_free_diram(size_data: dict) -> int | None:
    """
    Extract free DIRAM bytes from idf-size JSON output.

    idf-size JSON structure (IDF >= v5):
        {"targets": {"esp32s3": {"memories": {"dram0_0_seg": {"used": N, "total": N}}}}}
    We sum all segments whose key contains "dram" and compute total - used.
    Falls back to legacy flat structure if nested form is absent.
    """
    try:
        # Modern nested form
        for target in size_data.get("targets", {}).values():
            memories = target.get("memories", {})
            total_used = 0
            total_size = 0
            for seg_name, seg in memories.items():
                if "dram" in seg_name.lower():
                    total_used += seg.get("used", 0)
                    total_size += seg.get("total", 0)
            if total_size > 0:
                return total_size - total_used

        # Legacy flat form: {"dram_0": {"used": N, "total": N}, ...}
        total_used = 0
        total_size = 0
        for key, val in size_data.items():
            if "dram" in key.lower() and isinstance(val, dict):
                total_used += val.get("used", 0)
                total_size += val.get("total", 0)
        if total_size > 0:
            return total_size - total_used

    except Exception as e:
        print(f"[check_dram_budget] Error parsing size JSON: {e}")

    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("build_dir", nargs="?", default="build",
                        help="CMake build directory (default: build)")
    parser.add_argument("--variant", choices=list(THRESHOLDS.keys()),
                        default=DEFAULT_VARIANT,
                        help=f"Build variant (default: {DEFAULT_VARIANT})")
    args = parser.parse_args()

    build_dir = Path(args.build_dir).resolve()
    threshold = THRESHOLDS[args.variant]

    print(f"[check_dram_budget] variant={args.variant}  threshold={threshold // 1024} KB free DIRAM")

    size_data = run_idf_size(build_dir)
    if size_data is None:
        print("[check_dram_budget] WARNING: could not determine DRAM usage — check skipped (non-fatal).")
        return 0

    free_diram = get_free_diram(size_data)
    if free_diram is None:
        print("[check_dram_budget] WARNING: could not parse DRAM segment data — check skipped (non-fatal).")
        return 0

    free_kb = free_diram / 1024
    print(f"[check_dram_budget] static free DIRAM: {free_kb:.1f} KB  (threshold: {threshold // 1024} KB)")

    if free_diram < threshold:
        print(
            f"[check_dram_budget] FAIL: only {free_kb:.1f} KB static free DIRAM — "
            f"below {threshold // 1024} KB threshold for {args.variant} variant.\n"
            f"  NimBLE + CDC-ACM at runtime will likely exhaust DIRAM.\n"
            f"  Reduce static allocations or disable unused features."
        )
        return 1

    print(f"[check_dram_budget] OK: {free_kb:.1f} KB >= {threshold // 1024} KB threshold.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
