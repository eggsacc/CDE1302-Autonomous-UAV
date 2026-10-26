#!/usr/bin/env python3
"""Prepare/verify an isolated Drone-2 sdkconfig. Never builds or flashes hardware."""
from pathlib import Path
import argparse
import re

EXPECTED = {"CONFIG_DRONE_ID": "2", "CONFIG_HOST_IPV4_ADDR": '"192.168.1.102"',
            "CONFIG_DRONE_STATIC_IPV4_ADDR": '"192.168.1.202"'}


def prepare(root):
    source = root / "sdkconfig"
    destination = root / "sdkconfig.yaw-drone2"
    text = source.read_text(encoding="utf-8")
    if 'CONFIG_IDF_TARGET="esp32s3"' not in text:
        raise ValueError("Expected ESP32-S3 target; review hardware/configuration")
    for key, value in EXPECTED.items():
        text, count = re.subn(rf"^{key}=.*$", f"{key}={value}", text, flags=re.MULTILINE)
        if count != 1:
            raise ValueError(f"Expected exactly one {key} setting")
    if destination.exists() and destination.read_text(encoding="utf-8") != text:
        raise ValueError("Existing experiment sdkconfig differs; preserve and review it manually")
    destination.write_text(text, encoding="utf-8", newline="\n")
    return destination


def verify_header(path):
    text = Path(path).read_text(encoding="utf-8")
    for key, value in EXPECTED.items():
        if not re.search(rf"^#define {key} {re.escape(value)}\s*$", text, re.MULTILINE):
            raise ValueError(f"Generated build header has wrong/missing {key}")
    if '#define CONFIG_IDF_TARGET "esp32s3"' not in text:
        raise ValueError("Wrong generated hardware target")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--verify-header", help="Verify build-yaw-drone2/config/sdkconfig.h after build")
    args = ap.parse_args()
    if args.verify_header:
        verify_header(args.verify_header)
        print("Generated header verified: drone 2, ESP 192.168.1.202, host 192.168.1.102, ESP32-S3")
    else:
        path = prepare(Path(__file__).resolve().parents[1])
        print(f"Prepared {path}; original sdkconfig unchanged.")
        print("Review the board, flash, PSRAM, UART pins and Wi-Fi settings with the TA.")
        print("Build only: idf.py -B build-yaw-drone2 -D SDKCONFIG=sdkconfig.yaw-drone2 build")


if __name__ == "__main__":
    main()
