#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""SD card in Renode: the SDMMC1 driver and the SD protocol against an emulated card.

  sdcard_test.py build/arm/apps/fcu_sdcard.elf [--exe C:/opt/renode/renode.exe]

Inserts a card image into SDMMC1 (emulation/renode/STM32H5_SDMMC.cs: writes go straight to
the image file) and runs fcu_sdcard, which initialises the card, then writes, reads back and compares
a single block and an eight-block run at the card's end. The test passes if the firmware
reports PASS with the card's capacity and the image file holds the patterns at the blocks
the firmware wrote - the data really went through the controller to the card.

Exit code 0 = pass, 1 = fail.
"""
from __future__ import annotations

import argparse
import re
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_RENODE = Path("C:/opt/renode/renode.exe")
RESULT = re.compile(r"SDCARD RESULT: (PASS|FAIL)[^\r\n]*(?=\r?\n)")
CARD_BYTES = 64 * 1024 * 1024
BLOCK = 512


def pattern(length: int, seed: int) -> bytes:
    """The firmware's fill(): byte i = (i * 31 + seed * 7 + i / 512) & 0xFF."""
    return bytes((i * 31 + seed * 7 + (i >> 9)) & 0xFF for i in range(length))


def run(exe: Path, elf: Path, image: Path, high_capacity: bool, timeout: float) -> str:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    commands = "; ".join([
        f"$bin=@{elf.as_posix()}",
        f"include @{(HERE / 'renode' / 'fcu.resc').as_posix()}",
        f"sysbus.sdmmc1 InsertCard @{image.as_posix()} {'true' if high_capacity else 'false'}",
        f'emulation CreateServerSocketTerminal {port} "console" false',
        "connector Connect sysbus.usart3 console",
        "start",
    ])
    proc = subprocess.Popen([str(exe), "--disable-gui", "--plain", "--port", "-1", "-e", commands],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL)
    console = b""
    try:
        deadline = time.monotonic() + 60
        while True:
            try:
                sock = socket.create_connection(("127.0.0.1", port), timeout=1)
                break
            except OSError:
                if proc.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Renode did not open the console socket")
                time.sleep(0.2)
        sock.settimeout(1)
        deadline = time.monotonic() + timeout
        while not RESULT.search(console.decode("utf-8", "replace")) and time.monotonic() < deadline:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                break
            console += chunk
            sys.stdout.write(chunk.decode("utf-8", "replace").replace("\r", ""))
            sys.stdout.flush()
    finally:
        proc.kill()
        proc.wait()
    return console.decode("utf-8", "replace")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("elf", type=Path)
    ap.add_argument("--exe", type=Path, default=DEFAULT_RENODE)
    ap.add_argument("--timeout", type=float, default=60.0, help="seconds")
    args = ap.parse_args()
    if not args.exe.exists():
        print(f"error: {args.exe} not found", file=sys.stderr)
        return 2

    failures = 0

    def check(ok: bool, what: str) -> None:
        nonlocal failures
        print(f"{'ok  ' if ok else 'FAIL'} {what}")
        failures += 0 if ok else 1

    # an SDHC card (block addresses, CSD 2.0) and an SDSC card (byte addresses, CSD 1.0, CMD16)
    for kind, card_bytes in (("SDHC", CARD_BYTES), ("SDSC", CARD_BYTES // 2)):
        print(f"--- {kind} card, {card_bytes // (1024 * 1024)} MB")
        with tempfile.TemporaryDirectory() as tmp:
            image = Path(tmp) / "card.img"
            image.write_bytes(bytes(card_bytes))
            console = run(args.exe, args.elf.resolve(), image, kind == "SDHC", args.timeout)
            data = image.read_bytes()

        m = RESULT.search(console)
        check(m is not None and m.group(1) == "PASS", f"firmware: {m.group(0) if m else 'no result'}")
        blocks = card_bytes // BLOCK
        check(m is not None and f"({blocks} blocks, {kind})" in m.group(0), f"{blocks} blocks, {kind} reported")
        single = (blocks - 1) * BLOCK
        check(data[single:single + BLOCK] == pattern(BLOCK, 1), "the image holds the single block")
        run_at = (blocks - 1 - 8) * BLOCK
        check(data[run_at:run_at + 8 * BLOCK] == pattern(8 * BLOCK, 2), "the image holds the eight-block run")
        check(data[:run_at] == bytes(run_at), "nothing else was written")

    print("SDCARD TEST", "PASS" if failures == 0 else f"FAIL ({failures})")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
