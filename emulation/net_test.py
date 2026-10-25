#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""End-to-end network test of the FCU firmware in Renode (TAP network).

  net_test.py <firmware.elf> <web dir with index.html.br + blob.bin> <ca.crt> [renode.exe]
  net_test.py --browser <firmware.elf> [renode.exe]     # SNTP + Renode until Ctrl+C

Starts a DHCP server (192.168.77.1:67, lease 192.168.77.2 for 20 s), a DNS server
(:53, pool.ntp.org -> 192.168.77.1) and an SNTP server (:12300) on the host side,
boots the firmware in Renode connected to the TAP adapter "rtoos-tap", and checks:
  the DHCP lease and its renewal, the DNS lookup of the SNTP server, ping, mDNS
  (fcu.local), SNTP, HTTPS (TLS 1.3, certificate + host name against the CA in
  certs/ca.crt), the brotli web app byte-exact, 404/406, the 100 KB blob with
  throughput over a keep-alive connection, the WebSocket upgrade, status messages,
  100 echo round trips with random 1..511-byte payloads, the LED command and the
  rejection of a cross-origin WebSocket.
Exit code 0 if every check passed, 77 (CTest: skipped) if the TAP adapter with
192.168.77.1 is missing - run emulation/renode/setup-tap.ps1 once as administrator.
"""
from __future__ import annotations

import base64
import hashlib
import os
import socket
import ssl
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

HOST_IP = "192.168.77.1"
DEVICE_IP = "192.168.77.2"
HERE = Path(__file__).resolve().parent
RENODE = Path("C:/opt/renode/renode.exe")
SKIPPED = 77

results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    results.append((name, ok, detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}{' - ' + detail if detail else ''}", flush=True)
    return ok


# ---- console ----------------------------------------------------------------------------

class Console:
    def __init__(self, sock: socket.socket):
        self.lines: list[str] = []
        self.cv = threading.Condition()
        threading.Thread(target=self._pump, args=(sock.makefile("rb"),), daemon=True).start()

    def _pump(self, stream) -> None:
        try:
            for raw in iter(stream.readline, b""):
                line = raw.decode("utf-8", "replace").rstrip("\r\n")
                print(f"    | {line}", flush=True)
                with self.cv:
                    self.lines.append(line)
                    self.cv.notify_all()
        except (OSError, ValueError):
            pass  # emulator terminated / console closed

    def wait_for(self, text: str, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        with self.cv:
            while True:
                if any(text in line for line in self.lines):
                    return True
                left = deadline - time.monotonic()
                if left <= 0:
                    return False
                self.cv.wait(left)


def tap_present() -> bool:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind((HOST_IP, 0))
        return True
    except OSError:
        return False


def start_renode(renode: Path, elf: Path) -> tuple[subprocess.Popen, Console]:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    resc = (HERE / "renode" / "fcu-net.resc").as_posix()
    commands = "; ".join([
        f"$bin=@{elf.as_posix()}",
        f"include @{resc}",
        f'emulation CreateServerSocketTerminal {port} "console" false',
        "connector Connect sysbus.usart3 console",
        "start",
    ])
    proc = subprocess.Popen([str(renode), "--disable-gui", "--plain", "--port", "-1", "-e", commands],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.monotonic() + 60
    while True:
        try:
            sock = socket.create_connection(("127.0.0.1", port), timeout=1)
            break
        except OSError:
            if proc.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("Renode did not start")
            time.sleep(0.2)
    sock.settimeout(None)
    return proc, Console(sock)


# ---- SNTP server ------------------------------------------------------------------------

def sntp_server(stop: threading.Event) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST_IP, 12300))
    s.settimeout(0.5)
    while not stop.is_set():
        try:
            data, addr = s.recvfrom(512)
        except socket.timeout:
            continue
        t = time.time() + 2208988800
        sec, frac = int(t), int((t - int(t)) * 2**32)
        r = bytearray(48)
        r[0], r[1] = 0x24, 2
        r[24:32] = data[40:48]
        r[32:40] = struct.pack(">II", sec, frac)
        r[40:48] = struct.pack(">II", sec, frac)
        s.sendto(r, addr)


# ---- DHCP server (RFC 2131, just enough for one client) ---------------------------------

DHCP_COOKIE = bytes([99, 130, 83, 99])
LEASE_S = 20


def dhcp_options(data: bytes) -> dict[int, bytes]:
    opts, pos = {}, 240
    while pos < len(data) and data[pos] != 255:
        if data[pos] == 0:
            pos += 1
            continue
        opts[data[pos]] = data[pos + 2:pos + 2 + data[pos + 1]]
        pos += 2 + data[pos + 1]
    return opts


def dhcp_server(stop: threading.Event, log: list[str]) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((HOST_IP, 67))
    s.settimeout(0.5)
    host, device = socket.inet_aton(HOST_IP), socket.inet_aton(DEVICE_IP)
    names = {1: "DISCOVER", 3: "REQUEST"}
    while not stop.is_set():
        try:
            data, _ = s.recvfrom(1500)
        except (socket.timeout, ConnectionResetError):
            continue
        if len(data) < 240 or data[0] != 1 or data[236:240] != DHCP_COOKIE:
            continue
        opts = dhcp_options(data)
        mtype = opts.get(53, b"\0")[0]
        if mtype not in names:
            continue
        ciaddr = data[12:16]
        log.append(names[mtype] + (" (renew)" if ciaddr != bytes(4) else ""))
        if mtype == 1:
            reply_type = 2  # OFFER
        else:
            requested = opts.get(50, ciaddr)
            reply_type = 5 if requested == device else 6  # ACK / NAK
        r = bytearray(240)
        r[0:4] = bytes([2, 1, 6, 0])
        r[4:8] = data[4:8]      # xid
        r[10:12] = data[10:12]  # flags
        r[12:16] = ciaddr
        if reply_type != 6:
            r[16:20] = device   # yiaddr
        r[20:24] = host
        r[28:44] = data[28:44]  # chaddr
        r[236:240] = DHCP_COOKIE
        for code, value in ((53, bytes([reply_type])), (54, host), (51, struct.pack(">I", LEASE_S)),
                            (1, socket.inet_aton("255.255.255.0")), (3, host), (6, host)):
            r += bytes([code, len(value)]) + value
        r += b"\xff"
        to = (socket.inet_ntoa(ciaddr), 68) if ciaddr != bytes(4) else ("255.255.255.255", 68)
        s.sendto(bytes(r), to)


# ---- DNS server -------------------------------------------------------------------------

def dns_server(stop: threading.Event, zone: dict[str, str], log: list[str]) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST_IP, 53))
    s.settimeout(0.5)
    while not stop.is_set():
        try:
            q, addr = s.recvfrom(512)
        except (socket.timeout, ConnectionResetError):
            continue
        if len(q) < 12:
            continue
        labels, pos = [], 12
        while pos < len(q) and q[pos] != 0:
            labels.append(q[pos + 1:pos + 1 + q[pos]].decode("ascii", "replace"))
            pos += 1 + q[pos]
        name, question = ".".join(labels).lower(), q[12:pos + 5]
        qtype = struct.unpack(">H", q[pos + 1:pos + 3])[0] if pos + 3 <= len(q) else 0
        log.append(f"{name} from port {addr[1]}")
        if qtype == 1 and name in zone:
            answer = b"\xc0\x0c" + struct.pack(">HHIH", 1, 1, 60, 4) + socket.inet_aton(zone[name])
            s.sendto(q[:2] + struct.pack(">HHHHH", 0x8180, 1, 1, 0, 0) + question + answer, addr)
        else:
            s.sendto(q[:2] + struct.pack(">HHHHH", 0x8183, 1, 0, 0, 0) + question, addr)  # NXDOMAIN


def start_servers(stop: threading.Event, dhcp_log: list[str], dns_log: list[str]) -> None:
    threading.Thread(target=sntp_server, args=(stop,), daemon=True).start()
    threading.Thread(target=dhcp_server, args=(stop, dhcp_log), daemon=True).start()
    threading.Thread(target=dns_server, args=(stop, {"pool.ntp.org": HOST_IP}, dns_log), daemon=True).start()


# ---- HTTPS ------------------------------------------------------------------------------

def tls_connect(ca: Path) -> ssl.SSLSocket:
    ctx = ssl.create_default_context(cafile=str(ca))
    ctx.minimum_version = ssl.TLSVersion.TLSv1_3
    ctx.verify_flags |= ssl.VERIFY_X509_STRICT  # RFC 5280 profile (Python >= 3.13 default)
    raw = socket.create_connection((DEVICE_IP, 443), timeout=20)
    return ctx.wrap_socket(raw, server_hostname="fcu.local")


def http_request(sock: ssl.SSLSocket, path: str, headers: dict[str, str] | None = None) -> tuple[int, dict[str, str], bytes]:
    h = {"Host": "fcu.local", "Accept-Encoding": "gzip, deflate, br"}
    h.update(headers or {})
    req = f"GET {path} HTTP/1.1\r\n" + "".join(f"{k}: {v}\r\n" for k, v in h.items()) + "\r\n"
    sock.sendall(req.encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("closed during headers")
        buf += chunk
    head, body = buf.split(b"\r\n\r\n", 1)
    lines = head.decode().split("\r\n")
    status = int(lines[0].split()[1])
    fields = {k.strip().lower(): v.strip() for k, v in (l.split(":", 1) for l in lines[1:])}
    length = int(fields.get("content-length", "0"))
    while len(body) < length:
        chunk = sock.recv(65536)
        if not chunk:
            raise ConnectionError("closed during body")
        body += chunk
    return status, fields, body[:length]


# ---- WebSocket --------------------------------------------------------------------------

def ws_open(ca: Path, origin: str) -> tuple[ssl.SSLSocket | None, int]:
    s = tls_connect(ca)
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall((f"GET /ws HTTP/1.1\r\nHost: fcu.local\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
               f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\nOrigin: {origin}\r\n\r\n").encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        chunk = s.recv(1024)
        if not chunk:
            return None, 0
        buf += chunk
    head = buf.split(b"\r\n\r\n", 1)[0].decode()
    status = int(head.split()[1])
    if status != 101:
        s.close()
        return None, status
    expected = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
    if f"Sec-WebSocket-Accept: {expected}" not in head:
        s.close()
        return None, -1
    return s, 101


def ws_send(s: ssl.SSLSocket, payload: bytes, opcode: int = 2) -> None:
    mask = os.urandom(4)
    n = len(payload)
    header = bytes([0x80 | opcode]) + (bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack(">H", n))
    s.sendall(header + mask + bytes(b ^ mask[i & 3] for i, b in enumerate(payload)))


class WsReader:
    def __init__(self, s: ssl.SSLSocket):
        self.s, self.buf = s, b""

    def _need(self, n: int) -> None:
        while len(self.buf) < n:
            chunk = self.s.recv(65536)
            if not chunk:
                raise ConnectionError("websocket closed")
            self.buf += chunk

    def frame(self) -> tuple[int, bytes]:
        self._need(2)
        opcode, n = self.buf[0] & 0x0F, self.buf[1] & 0x7F
        pos = 2
        if n == 126:
            self._need(4)
            n, pos = struct.unpack(">H", self.buf[2:4])[0], 4
        self._need(pos + n)
        payload, self.buf = self.buf[pos:pos + n], self.buf[pos + n:]
        return opcode, payload

    def message(self, type_byte: int, timeout: float = 5) -> bytes:
        self.s.settimeout(timeout)
        while True:
            opcode, payload = self.frame()
            if opcode == 2 and payload and payload[0] == type_byte:
                return payload


# ---- main -------------------------------------------------------------------------------

def browser(elf: Path, renode: Path) -> int:
    stop = threading.Event()
    dhcp_log: list[str] = []
    dns_log: list[str] = []
    start_servers(stop, dhcp_log, dns_log)
    proc, console = start_renode(renode, elf)
    try:
        if console.wait_for("https://fcu.local/", 60):
            print("\n  open https://fcu.local/ (or https://192.168.77.2/) - Ctrl+C to stop\n", flush=True)
        while proc.poll() is None:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        proc.kill()
        proc.wait()
    return 0


def main() -> int:
    if sys.argv[1] == "--browser":
        if not tap_present():
            print(f"error: no network adapter with {HOST_IP} (run emulation/renode/setup-tap.ps1)", file=sys.stderr)
            return 2
        return browser(Path(sys.argv[2]), Path(sys.argv[3]) if len(sys.argv) > 3 else RENODE)
    elf, web, ca = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    renode = Path(sys.argv[4]) if len(sys.argv) > 4 else RENODE
    if not tap_present():
        print(f"skipped: no network adapter with {HOST_IP} (run emulation/renode/setup-tap.ps1)", flush=True)
        return SKIPPED
    stop = threading.Event()
    dhcp_log: list[str] = []
    dns_log: list[str] = []
    start_servers(stop, dhcp_log, dns_log)
    proc, console = start_renode(renode, elf)
    try:
        if not check("boot: link up and HTTPS server", console.wait_for("https://fcu.local/", 60) and console.wait_for("link up", 30)):
            return 1
        bound = console.wait_for("dhcp: bound 192.168.77.2 mask 255.255.255.0, router 192.168.77.1, dns 192.168.77.1", 30)
        check("DHCP lease (DISCOVER, OFFER, REQUEST, ACK)", bound and dhcp_log[:2] == ["DISCOVER", "REQUEST"],
              f"server saw {dhcp_log}")
        if not bound:
            return 1
        resolved = console.wait_for("dns: pool.ntp.org -> 192.168.77.1", 15)
        port = int(dns_log[0].split()[-1]) if dns_log else 0
        check("DNS lookup of the SNTP server (random source port)", resolved and port >= 49152, f"query {dns_log}")

        r = subprocess.run(["ping", "-n", "2", DEVICE_IP], capture_output=True, text=True, encoding="cp850")
        check("ping 192.168.77.2", "TTL=" in r.stdout)

        q = struct.pack(">HHHHHH", 0x4242, 0, 1, 0, 0, 0) + b"\x03fcu\x05local\x00" + struct.pack(">HH", 1, 1)
        m = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        m.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(HOST_IP))
        m.bind((HOST_IP, 0))
        m.settimeout(2)
        answer = None
        for _ in range(3):
            m.sendto(q, ("224.0.0.251", 5353))
            try:
                data, _ = m.recvfrom(512)
                answer = socket.inet_ntoa(data[-4:])
                break
            except socket.timeout:
                pass
        check("mDNS fcu.local", answer == DEVICE_IP, f"answer {answer}")
        check("SNTP synchronized", console.wait_for("sntp synchronized", 15))

        # HTTPS: web app, keep-alive, blob
        t0 = time.monotonic()
        s = tls_connect(ca)
        check("TLS 1.3 handshake (certificate + host name verified)", s.version() == "TLSv1.3",
              f"{s.version()} {s.cipher()[0]} in {time.monotonic() - t0:.2f} s")
        status, fields, body = http_request(s, "/")
        expected = (web / "index.html.br").read_bytes()
        check("GET / (brotli web app, byte-exact)",
              status == 200 and fields.get("content-encoding") == "br" and body == expected,
              f"{status}, {len(body)} bytes")
        t0 = time.monotonic()
        status, fields, body = http_request(s, "/blob")  # same connection: keep-alive
        dt = time.monotonic() - t0
        blob = (web / "blob.bin").read_bytes()
        check("GET /blob 100 KB on the same connection", status == 200 and body == blob,
              f"{len(body) / 1024:.0f} KiB in {dt:.2f} s = {len(body) / 1024 / dt:.0f} KiB/s")
        status, _, _ = http_request(s, "/missing")
        check("404 for unknown path", status == 404)
        s.close()
        s = tls_connect(ca)
        status, _, _ = http_request(s, "/", {"Accept-Encoding": "gzip"})
        check("406 when brotli is not accepted", status == 406)
        s.close()

        # WebSocket
        ws, status = ws_open(ca, "https://fcu.local")
        if check("WebSocket upgrade (101, Sec-WebSocket-Accept)", ws is not None, f"status {status}"):
            reader = WsReader(ws)
            st = reader.message(0x01, 3)
            check("status message (0x01, 47 bytes)", len(st) == 47, f"uptime {struct.unpack('<I', st[1:5])[0]} ms")
            rtts, ok = [], True
            for i in range(100):
                payload = os.urandom(1 + (i * 97) % 511)
                t0 = time.monotonic()
                ws_send(ws, b"\x20" + payload)
                reply = reader.message(0x21)
                rtts.append((time.monotonic() - t0) * 1000)
                if reply[1:] != payload:
                    ok = False
                    break
            rtts.sort()
            check("100 echo round trips, payload 1..511 bytes, byte-exact", ok and len(rtts) == 100,
                  f"median {rtts[len(rtts) // 2]:.1f} ms, max {rtts[-1]:.1f} ms")
            ws_send(ws, b"\x10\x01")
            check("LED command -> firmware", console.wait_for("app: led on", 5))
            st = reader.message(0x01, 3)
            check("status reports LED on", st[7] == 1)
            handshake, seal16, open16, echo, mhz, seal_fixed, open_fixed = struct.unpack("<IHHIHII", st[25:47])
            check("cycle counts reported (DWT_CYCCNT)", handshake > 0 and seal16 > 0 and open16 > 0 and echo > 0 and mhz == 250,
                  f"handshake {handshake} cycles = {handshake / mhz / 1000:.1f} ms, "
                  f"encrypt {seal_fixed} cycles/record + {seal16 / 16:.1f} c/B, "
                  f"decrypt {open_fixed} cycles/record + {open16 / 16:.1f} c/B, "
                  f"echo {echo} cycles = {echo / mhz:.0f} us (at {mhz} MHz)")
            # The cost model must not drift with the record mix: 3 s of 39-byte status
            # records only (a plain cycles/bytes average would climb towards 100+ c/B).
            time.sleep(3)
            later = reader.message(0x01, 3)
            seal16_later, seal_fixed_later = struct.unpack("<H", later[29:31])[0], struct.unpack("<I", later[39:43])[0]
            check("encrypt cost stable under small status records (+-15 %)",
                  abs(seal16_later - seal16) <= 0.15 * seal16 and abs(seal_fixed_later - seal_fixed) <= 0.15 * seal_fixed,
                  f"{seal16 / 16:.1f} -> {seal16_later / 16:.1f} c/B, {seal_fixed} -> {seal_fixed_later} cycles/record")
            ws.close()
        _, status = ws_open(ca, "https://evil.example")
        check("cross-origin WebSocket rejected (403)", status == 403)
        # Browser-like load: idle keep-alive and spare connections beyond the session table.
        idle = []
        for _ in range(7):
            c = tls_connect(ca)
            http_request(c, "/")  # keep-alive: the connection stays open, idle
            idle.append(c)
        t0 = time.monotonic()
        ws, status = ws_open(ca, "https://fcu.local")
        connect_ms = (time.monotonic() - t0) * 1000
        first = WsReader(ws).message(0x01, 3) if ws else b""
        first_ms = (time.monotonic() - t0) * 1000
        check("WebSocket despite 7 idle keep-alive connections (eviction), first data < 300 ms",
              ws is not None and len(first) == 47 and first_ms < 300,
              f"101 after {connect_ms:.0f} ms, first status after {first_ms:.0f} ms")
        for c in idle:
            c.close()
        if ws:
            ws.close()
        renewed = console.wait_for("dhcp: renewed 192.168.77.2", LEASE_S)
        check("DHCP renewal at T1 (unicast REQUEST)", renewed and "REQUEST (renew)" in dhcp_log, f"server saw {dhcp_log}")
    finally:
        stop.set()
        proc.kill()
        proc.wait()

    failed = [n for n, ok, _ in results if not ok]
    print(f"=> network test: {len(results) - len(failed)}/{len(results)} passed", flush=True)
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
