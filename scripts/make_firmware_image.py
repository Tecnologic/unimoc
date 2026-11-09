#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Wraps a firmware binary into a UNIMOC update image (lib/cyphal/firmware_image.hpp).

    make_firmware_image.py --board nucleo_h743zi --vcs 0123abcd build/drive/zephyr/zephyr.signed.bin image.bin

The payload of an STM32H7 image is the MCUboot image (`zephyr.signed.bin` of the sysbuild);
the software version is read from its header unless --version is given. The
bootloader swaps it in when its version (major.minor.revision) is higher than the running one.

The header (40 bytes, little endian): magic "UNIM", header version 1, software
major and minor, VCS revision (64 bit), payload size, CRC32 of the payload
(zlib.crc32, IEEE 802.3), board name zero padded to 16 bytes.
"""

from __future__ import annotations

import argparse
import struct
import sys
import zlib
from pathlib import Path

MAGIC = b"UNIM"
HEADER_VERSION = 1
BOARD_NAME_SIZE = 16


MCUBOOT_MAGIC = bytes.fromhex("3db8f396")  # 0x96f3b83d little endian


def mcuboot_version(payload: bytes) -> tuple[int, int] | None:
    """(major, minor) from the header of an MCUboot image (struct image_header: ver at offset 20), or None."""
    if len(payload) < 32 or payload[:4] != MCUBOOT_MAGIC:
        return None
    return payload[20], payload[21]


def make_image(payload: bytes, board: str, major: int, minor: int, vcs: int) -> bytes:
    name = board.encode()
    if len(name) > BOARD_NAME_SIZE:
        raise ValueError(f"board name longer than {BOARD_NAME_SIZE} bytes: {board}")
    header = MAGIC + struct.pack("<HBBQII", HEADER_VERSION, major, minor, vcs, len(payload),
                                 zlib.crc32(payload) & 0xFFFFFFFF)
    header += name.ljust(BOARD_NAME_SIZE, b"\0")
    assert len(header) == 40
    return header + payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--board", required=True, help="unimoc_jacobi, nucleo_h743zi or native_sim")
    parser.add_argument("--version", help="software MAJOR.MINOR (default: from the MCUboot header, else 0.0)")
    parser.add_argument("--vcs", default="0", help="VCS revision in hex")
    parser.add_argument("payload", type=Path)
    parser.add_argument("image", type=Path)
    args = parser.parse_args()
    payload = args.payload.read_bytes()
    if args.version is not None:
        major, minor = (int(part) for part in args.version.split("."))
    else:
        major, minor = mcuboot_version(payload) or (0, 0)
    args.image.write_bytes(make_image(payload, args.board, major, minor, int(args.vcs, 16)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
