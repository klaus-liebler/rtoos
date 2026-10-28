#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Sigmoid stepper ramp, measured on the STEP pin in Renode.

  stepper_test.py build/arm/apps/fcu_stepper.elf [--exe C:/opt/renode/renode.exe]

Runs fcu_stepper in Renode with a probe on the STEP pin (PB4): a Renode Python hook records
every edge with its virtual time. The firmware reports its ramp parameters on the console and
runs two moves. The test then checks the step rate over time - the reciprocal of the time
between consecutive edges:

  long move (2000 steps)
    * every ramp step runs at the sigmoid's rate for the time it starts (within 2 %)
    * the sigmoid explains the ramp far better than a straight line between the same end
      points (RMS error below a fifth of the linear ramp's)
    * the strongest acceleration lies in the middle of the ramp, both ends are gentle
    * the cruise runs at cruise_hz, the deceleration mirrors the ramp
  short move (101 steps)
    * a symmetric triangle that stays below cruise_hz

Exit code 0 = pass, 1 = fail.
"""
from __future__ import annotations

import argparse
import math
import re
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_RENODE = Path("C:/opt/renode/renode.exe")
PARAMS = re.compile(r"stepper: tick_hz (\d+) start_hz (\d+) cruise_hz (\d+) ramp_us (\d+) steepness (\d+) "
                    r"ramp_steps (\d+)")
DONE = "STEPPER RESULT: DONE"

# Renode Python (IronPython), run in the monitor: a probe LED on PB4 and a hook that writes
# "<virtual time in us> <state>" for every change of the pin.
HOOK = r"""
from Antmicro.Renode.Core import EmulationManager
from Antmicro.Renode.Peripherals.Miscellaneous import LED
machine = list(EmulationManager.Instance.CurrentEmulation.Machines)[0]
probe = [l for l in machine.GetPeripheralsOfType[LED]() if machine.GetLocalName(l) == "step_probe"][0]
edges_file = open(r"{path}", "w")
def on_step(led, state):
    edges_file.write("%.3f %d\n" % (machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds, 1 if state else 0))
    edges_file.flush()
probe.StateChanged += on_step
"""


def sigmoid(x: float) -> float:
    return 1.0 / (1.0 + math.exp(-x))


def sigmoid_rate(t_us: float, start: float, cruise: float, ramp_us: float, k: float) -> float:
    x = min(t_us / ramp_us, 1.0) * k - k / 2
    lo, hi = sigmoid(-k / 2), sigmoid(k / 2)
    return start + (cruise - start) * (sigmoid(x) - lo) / (hi - lo)


PROBE = """step_probe: Miscellaneous.LED @ gpioPortB 4

gpioPortB:
    4 -> step_probe@0
"""


def run_renode(exe: Path, elf: Path, tmp: Path, timeout: float) -> str:
    edges_path, hook_path, probe_path = tmp / "edges.txt", tmp / "hook.py", tmp / "probe.repl"
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    hook_path.write_text(HOOK.replace("{path}", edges_path.as_posix()), encoding="utf-8")
    probe_path.write_text(PROBE, encoding="utf-8")
    resc = (HERE / "renode" / "fcu.resc").as_posix()
    commands = "; ".join([
        f"$bin=@{elf.as_posix()}",
        f"include @{resc}",
        f"machine LoadPlatformDescription @{probe_path.as_posix()}",
        f"python \"exec(open(r'{hook_path.as_posix()}').read())\"",
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
        while DONE.encode() not in console and time.monotonic() < deadline:
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


class Check:
    def __init__(self) -> None:
        self.failures = 0

    def __call__(self, ok: bool, what: str) -> None:
        print(f"{'ok  ' if ok else 'FAIL'} {what}")
        self.failures += 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("elf", type=Path)
    ap.add_argument("--exe", type=Path, default=DEFAULT_RENODE)
    ap.add_argument("--timeout", type=float, default=120.0, help="seconds")
    args = ap.parse_args()
    if not args.exe.exists():
        print(f"error: {args.exe} not found", file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory() as tmp:
        console = run_renode(args.exe, args.elf.resolve(), Path(tmp), args.timeout)
        edges = [float(line.split()[0]) for line in (Path(tmp) / "edges.txt").read_text().splitlines() if line.strip()]

    check = Check()
    m = PARAMS.search(console)
    check(m is not None and DONE in console, "firmware reported its ramp and finished both moves")
    if m is None:
        return 1
    tick_hz, start, cruise, ramp_us, k, ramp_steps = (int(g) for g in m.groups())

    # The two moves: separated by the pause between them (> 100 ms without an edge).
    moves: list[list[float]] = [[]]
    for t in edges:
        if moves[-1] and t - moves[-1][-1] > 100_000:
            moves.append([])
        moves[-1].append(t)
    check([len(mv) for mv in moves] == [2000, 101], f"edges per move {[len(mv) for mv in moves]} == [2000, 101]")
    if len(moves) != 2 or len(moves[0]) != 2000:
        return 1

    # ---- long move ----------------------------------------------------------------------
    long_move = moves[0]
    # the first edge ends the first step, which runs at start_hz: the move began one period earlier
    origin = long_move[0] - 1e6 / start
    begins = [origin] + long_move[:-1]                      # start of each step
    rates = [1e6 / (end - begin) for begin, end in zip(begins, long_move)]
    t_rel = [b - origin for b in begins]

    errors = [abs(rates[i] - sigmoid_rate(t_rel[i], start, cruise, ramp_us, k)) / rates[i] for i in range(ramp_steps)]
    worst = max(range(ramp_steps), key=lambda i: errors[i])
    check(errors[worst] < 0.02,
          f"ramp follows the sigmoid: worst step {worst} at {t_rel[worst] / 1000:.1f} ms, "
          f"{rates[worst]:.1f} steps/s vs {sigmoid_rate(t_rel[worst], start, cruise, ramp_us, k):.1f} "
          f"({errors[worst] * 100:.2f} %)")

    def rms(model) -> float:
        return math.sqrt(sum((rates[i] - model(t_rel[i])) ** 2 for i in range(ramp_steps)) / ramp_steps)
    rms_sigmoid = rms(lambda t: sigmoid_rate(t, start, cruise, ramp_us, k))
    rms_linear = rms(lambda t: start + (cruise - start) * min(t / ramp_us, 1.0))
    check(rms_sigmoid < rms_linear / 5,
          f"sigmoid fits far better than a linear ramp: RMS {rms_sigmoid:.1f} vs {rms_linear:.1f} steps/s")

    accel = [(t_rel[i], (rates[i] - rates[i - 1]) / ((t_rel[i] - t_rel[i - 1]) * 1e-6)) for i in range(1, ramp_steps)]
    peak_t, peak_a = max(accel, key=lambda a: a[1])
    check(abs(peak_t - ramp_us / 2) < ramp_us * 0.15,
          f"strongest acceleration {peak_a:.0f} steps/s^2 at {peak_t / 1000:.0f} ms (ramp middle {ramp_us / 2000:.0f} ms)")
    first_a = max(a for t, a in accel if t < ramp_us * 0.1)
    last_a = max(a for t, a in accel if t > ramp_us * 0.9)
    check(first_a < peak_a * 0.25 and last_a < peak_a * 0.25,
          f"gentle ends: {first_a:.0f} and {last_a:.0f} steps/s^2 (< 25 % of the peak)")
    check(all(rates[i] >= rates[i - 1] * 0.99 for i in range(1, ramp_steps)), "the rate never falls during the ramp")

    cruise_rates = rates[ramp_steps + 10:len(rates) - ramp_steps - 10]
    check(all(abs(r - cruise) / cruise < 0.01 for r in cruise_rates),
          f"cruise at {cruise} steps/s: {min(cruise_rates):.1f} .. {max(cruise_rates):.1f}")
    mirror = max(abs(rates[i] - rates[len(rates) - 1 - i]) / rates[i] for i in range(1, ramp_steps))
    check(mirror < 0.02, f"deceleration mirrors the ramp (largest difference {mirror * 100:.2f} %)")

    # ---- short move ---------------------------------------------------------------------
    short = moves[1]
    s_begins = [short[0] - 1e6 / start] + short[:-1]
    s_rates = [1e6 / (e - b) for b, e in zip(s_begins, short)]
    sym = max(abs(s_rates[i] - s_rates[-1 - i]) / s_rates[i] for i in range(len(s_rates)))
    check(sym < 0.02, f"short move is a symmetric triangle (largest difference {sym * 100:.2f} %)")
    peak = max(s_rates)
    check(peak < cruise * 0.99 and s_rates.index(peak) in (49, 50, 51),
          f"its peak {peak:.1f} steps/s lies in the middle, below cruise")

    print("STEPPER TEST", "PASS" if check.failures == 0 else f"FAIL ({check.failures})")
    return 0 if check.failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
