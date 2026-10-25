#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run rtoos firmware for the Factory Control Unit in QEMU or Renode.

  run.py qemu   build/arm/apps/fcu_selftest.elf            # self-test, exit code = result
  run.py renode build/arm/apps/fcu_selftest.elf --allow-unsupported "stack limit"
  run.py qemu   build/arm/apps/fcu_demo.elf --interactive  # console until Ctrl+C

The console (USART3) is streamed to stdout. In test mode the run ends at the line
"SELFTEST RESULT: ..." (or at the timeout). The test passes if the result is PASS
and every UNSUPPORTED test is covered by --allow-unsupported (emulator features
known to be missing, e.g. Renode does not enforce PSPLIM).
"""
from __future__ import annotations

import argparse
import re
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_QEMU = Path("C:/opt/qemu-stm32h5/qemu-system-arm.exe")
DEFAULT_RENODE = Path("C:/opt/renode/renode.exe")
RESULT = re.compile(r"SELFTEST RESULT: (PASS|FAIL)")
UNSUPPORTED = re.compile(r"\[selftest\] UNSUPPORTED (.*)")


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def start_qemu(exe: Path, elf: Path) -> tuple[subprocess.Popen, object]:
    proc = subprocess.Popen(
        # -icount shift=2: one instruction = 4 ns, i.e. 250 MIPS at the firmware's
        # 250 MHz SYSCLK. Virtual time (SysTick, DWT_CYCCNT) then follows the executed
        # code instead of the host clock; sleep=off jumps idle time straight to the
        # next timer. Otherwise the coarse Windows host timer makes SysTick periods
        # merge (lost ticks: 20 ticks took ~32 ms of CYCCNT time).
        [str(exe), "-M", "fcu", "-icount", "shift=2,sleep=off", "-display", "none", "-monitor", "none",
         "-serial", "stdio", "-kernel", str(elf)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    return proc, proc.stdout


def start_renode(exe: Path, elf: Path) -> tuple[subprocess.Popen, object]:
    port = free_port()
    resc = (HERE / "renode" / "fcu.resc").as_posix()
    commands = "; ".join([
        f"$bin=@{elf.as_posix()}",
        f"include @{resc}",
        f'emulation CreateServerSocketTerminal {port} "console" false',
        "connector Connect sysbus.usart3 console",
        "start",
    ])
    proc = subprocess.Popen(
        [str(exe), "--disable-gui", "--plain", "--port", "-1", "-e", commands],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL)
    deadline = time.monotonic() + 60
    while True:
        try:
            sock = socket.create_connection(("127.0.0.1", port), timeout=1)
            break
        except OSError:
            if proc.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("Renode did not open the console socket")
            time.sleep(0.2)
    sock.settimeout(None)
    return proc, sock.makefile("rb")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("emulator", choices=["qemu", "renode"])
    ap.add_argument("elf", type=Path)
    ap.add_argument("--exe", type=Path, help="emulator executable (default: C:/opt/...)")
    ap.add_argument("--timeout", type=float, default=60.0, help="seconds (test mode)")
    ap.add_argument("--allow-unsupported", action="append", default=[], metavar="TEXT",
                    help="accept UNSUPPORTED tests whose name contains TEXT")
    ap.add_argument("--interactive", action="store_true", help="just show the console")
    args = ap.parse_args()

    exe = args.exe or (DEFAULT_QEMU if args.emulator == "qemu" else DEFAULT_RENODE)
    if not exe.exists():
        print(f"error: {exe} not found", file=sys.stderr)
        return 2
    starter = start_qemu if args.emulator == "qemu" else start_renode
    proc, stream = starter(exe, args.elf.resolve())

    result = None
    unsupported: list[str] = []
    deadline = time.monotonic() + args.timeout
    try:
        lines: list[bytes] = []
        done = threading.Event()

        def pump() -> None:
            try:
                for raw in iter(stream.readline, b""):
                    lines.append(raw)
                    text = raw.decode("utf-8", "replace")
                    sys.stdout.write(text.replace("\r", ""))
                    sys.stdout.flush()
                    if RESULT.search(text):
                        break
            except (OSError, ValueError):
                pass  # emulator terminated / console closed
            finally:
                done.set()

        threading.Thread(target=pump, daemon=True).start()
        while not done.is_set():
            if not args.interactive and time.monotonic() > deadline:
                print(f"\nerror: no result within {args.timeout:.0f} s", file=sys.stderr)
                break
            if proc.poll() is not None:
                done.wait(1)
                break
            done.wait(0.2)
        text = b"".join(lines).decode("utf-8", "replace")
        if m := RESULT.search(text):
            result = m.group(1)
        unsupported = [u.strip() for u in UNSUPPORTED.findall(text)]
    except KeyboardInterrupt:
        pass
    finally:
        proc.kill()
        proc.wait()

    if args.interactive:
        return 0
    unexpected = [u for u in unsupported if not any(a in u for a in args.allow_unsupported)]
    for u in unexpected:
        print(f"error: unexpected UNSUPPORTED test: {u}", file=sys.stderr)
    ok = result == "PASS" and not unexpected
    print(f"=> {args.emulator}: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
