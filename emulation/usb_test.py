#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""USB device in Renode: the STM32H743 USB2_OTG_FS driver and the USB stack against a scripted host.

  usb_test.py build/arm-h743/apps/stm32h743/h743_usb.elf [--exe C:/opt/renode/renode.exe]
  usb_test.py <firmware.elf> --mode modbus --uid 00340034 30355108 34303537

Runs the firmware on the FCU revision 3 (fcu_rev3.resc); the USB core's model
(emulation/renode/STM32_OTG_FS.cs) carries a host that enumerates the device (device and
configuration descriptors, SET_ADDRESS - the address must be in DCFG before the status stage -,
SET_CONFIGURATION) and then talks to the first CDC data interface: "echo" sends 79 bytes and
expects them back, "modbus" sends a Modbus RTU request and checks the answer. The model writes
"USB RESULT: PASS ..." or "USB RESULT: FAIL <step>" to a file this script reads.

--uid writes the chip UID into the emulated system memory (a product firmware that checks its
provisioning); --monitor adds Renode monitor commands before the start (e.g. an input pin:
"sysbus.gpioPortC OnGPIO 0 true"). Exit code 0 = pass, 1 = fail.
"""
from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_RENODE = Path("C:/opt/renode/renode.exe")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("elf", type=Path)
    ap.add_argument("--exe", type=Path, default=DEFAULT_RENODE)
    ap.add_argument("--mode", choices=["echo", "modbus"], default="echo")
    ap.add_argument("--uid", nargs=3, metavar="WORD", help="chip UID words (hex) at 0x1FF1E800")
    ap.add_argument("--monitor", action="append", default=[], metavar="COMMAND", help="Renode monitor command")
    ap.add_argument("--timeout", type=float, default=90.0, help="seconds")
    args = ap.parse_args()
    if not args.exe.exists():
        print(f"error: {args.exe} not found", file=sys.stderr)
        return 2
    with tempfile.TemporaryDirectory() as tmp:
        result_file = Path(tmp) / "usb_result.txt"
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
        commands = [f"$bin=@{args.elf.resolve().as_posix()}", f"include @{(HERE / 'renode' / 'fcu_rev3.resc').as_posix()}"]
        for i, word in enumerate(args.uid or []):
            commands.append(f"sysbus WriteDoubleWord {0x1FF1E800 + 4 * i:#x} 0x{word}")
        commands += args.monitor
        commands += [
            f'sysbus.usb Mode "{args.mode}"',
            f"sysbus.usb ResultFile @{result_file.as_posix()}",
            f'emulation CreateServerSocketTerminal {port} "console" false',
            "connector Connect sysbus.usart3 console",
            "start",
        ]
        proc = subprocess.Popen([str(args.exe), "--disable-gui", "--plain", "--port", "-1", "-e", "; ".join(commands)],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL)
        console = b""
        result = ""
        try:
            deadline = time.monotonic() + 60
            while True:
                try:
                    sock = socket.create_connection(("127.0.0.1", port), timeout=1)
                    break
                except OSError:
                    if proc.poll() is not None or time.monotonic() > deadline:
                        print("error: Renode did not open the console socket", file=sys.stderr)
                        return 1
                    time.sleep(0.2)
            sock.settimeout(0.5)
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                if result_file.exists() and (result := result_file.read_text().strip()):
                    break
                try:
                    chunk = sock.recv(4096)
                    if chunk:
                        console += chunk
                        sys.stdout.write(chunk.decode("utf-8", "replace").replace("\r", ""))
                        sys.stdout.flush()
                except socket.timeout:
                    pass
        finally:
            proc.kill()
            proc.wait()
    print(result or f"USB RESULT: FAIL no result within {args.timeout:.0f} s")
    return 0 if result.startswith("USB RESULT: PASS") else 1


if __name__ == "__main__":
    sys.exit(main())
