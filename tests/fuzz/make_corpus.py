#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Writes the seed corpus of the AFL++ harnesses (tests/fuzz/corpus/<harness>/), docs/fuzzing.md.

  python tests/fuzz/make_corpus.py

Valid inputs in each harness's input format (see the comment at the top of each harness).
plc: the images of tests/plc_corpus. opcua: the client streams of opcua_test.cpp, written by the
host tests with RTOS_FUZZ_CORPUS=tests/fuzz/corpus/opcua (this script leaves that directory alone).
"""
import shutil
import struct
from pathlib import Path

HERE = Path(__file__).resolve().parent
CORPUS = HERE / "corpus"


def write(harness: str, name: str, data: bytes) -> None:
    d = CORPUS / harness
    d.mkdir(parents=True, exist_ok=True)
    (d / name).write_bytes(data)


# ---- http: segment byte + stream ----------------------------------------------------------------
write("http", "get", b"\x10GET /index.html HTTP/1.1\r\nHost: fcu.local\r\nAccept-Encoding: gzip, br\r\n\r\n")
write("http", "upgrade", b"\x07GET /ws HTTP/1.1\r\nHost: fcu.local\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
      b"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\nOrigin: https://fcu.local\r\n\r\n")
write("http", "pipelined", b"\x3fGET /a HTTP/1.1\r\n\r\nGET /b HTTP/1.0\r\nConnection: close\r\n\r\n")
write("http", "post", b"\x20POST /api/plc HTTP/1.1\r\nHost: fcu.local\r\nContent-Length: 4\r\n\r\nabcd")


# ---- websocket: segment byte + frames -----------------------------------------------------------
def ws(opcode: int, payload: bytes, fin: bool = True, mask: bytes = b"\x37\xfa\x21\x3d") -> bytes:
    n = len(payload)
    head = bytes([(0x80 if fin else 0) | opcode])
    if n < 126:
        head += bytes([0x80 | n])
    elif n < 65536:
        head += bytes([0x80 | 126]) + struct.pack(">H", n)
    else:
        head += bytes([0x80 | 127]) + struct.pack(">Q", n)
    return head + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload))


write("websocket", "text", b"\x05" + ws(1, b"Hello"))
write("websocket", "fragments_ping", b"\x03" + ws(1, b"a", fin=False) + ws(9, b"") + ws(0, b"b"))
write("websocket", "close", b"\x1f" + ws(8, b"\x03\xe8"))
write("websocket", "binary_300", b"\x10" + ws(2, bytes(range(256)) + bytes(44)))

# ---- dns: offset byte + message -----------------------------------------------------------------
name = b"\x03fcu\x05local\x00"
query = struct.pack(">HHHHHH", 0, 0, 1, 0, 0, 0) + name + struct.pack(">HH", 1, 1)
answer = (struct.pack(">HHHHHH", 7, 0x8400, 1, 1, 0, 0) + name + struct.pack(">HH", 1, 1) +
          b"\xc0\x0c" + struct.pack(">HHIH", 1, 1, 300, 4) + bytes([10, 0, 0, 1]))
write("dns", "query", bytes([12]) + query)
write("dns", "answer_pointer", bytes([12 + len(name) + 4]) + answer)
write("dns", "pointer_loop", bytes([12]) + struct.pack(">HHHHHH", 0, 0, 1, 0, 0, 0) + b"\xc0\x0c")

# ---- dhcp: options ------------------------------------------------------------------------------
write("dhcp", "offer", bytes([53, 1, 2, 54, 4, 10, 0, 0, 1, 1, 4, 255, 255, 255, 0, 3, 4, 10, 0, 0, 1,
                              6, 8, 10, 0, 0, 1, 10, 0, 0, 2, 51, 4, 0, 0, 14, 16, 0, 255]))
write("dhcp", "request", bytes([53, 1, 3, 50, 4, 192, 168, 77, 100, 54, 4, 192, 168, 77, 2,
                                12, 3, ord('f'), ord('c'), ord('u'), 55, 3, 1, 3, 6, 255]))
write("dhcp", "overload_pad", bytes([0, 0, 52, 1, 3, 53, 1, 5, 255]))


# ---- modbus: mode byte + PDU, or + RTU stream (0xFE 0xFF = silence) -----------------------------
def crc16(b: bytes) -> int:
    crc = 0xFFFF
    for x in b:
        crc ^= x
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def rtu(body: bytes) -> bytes:
    return body + struct.pack("<H", crc16(body))


write("modbus", "read_holding", b"\x00\x03\x00\x10\x00\x05")
write("modbus", "read_input", b"\x00\x04\x00\x00\x00\x32")
write("modbus", "write_single", b"\x00\x06\x00\x63\xab\xcd")
write("modbus", "write_multiple", b"\x00\x10\x00\x00\x00\x02\x04\x00\x01\x00\x02")
write("modbus", "read_write_multiple", b"\x00\x17\x00\x00\x00\x02\x00\x04\x00\x01\x02\x12\x34")
write("modbus", "rtu_two_frames", b"\x83" + rtu(b"\x01\x03\x00\x00\x00\x0a") + b"\xfe\xff" +
      rtu(b"\x01\x10\x00\x00\x00\x01\x02\x12\x34"))
write("modbus", "rtu_broadcast", b"\x8f" + rtu(b"\x00\x06\x00\x05\x00\x07"))


# ---- ethercat: master operations ---------------------------------------------------------------
def sm(n: int, start: int, length: int, ctrl: int, en: int = 1) -> bytes:
    return bytes([2, n]) + struct.pack("<HHBB", start, length, ctrl, en)


def al(state: int) -> bytes:
    return bytes([1, state])


def mbx(data: bytes, cnt: int = 1, typ: int = 3) -> bytes:
    return bytes([3, typ, cnt, len(data)]) + data


def sdo_upload(index: int, sub: int) -> bytes:
    return b"\x00\x20" + struct.pack("<BHB", 0x40, index, sub) + bytes(4)


def sdo_download(index: int, sub: int, value: int, size: int) -> bytes:
    cmd = 0x23 | ((4 - size) << 2)
    return b"\x00\x20" + struct.pack("<BHB", cmd, index, sub) + value.to_bytes(4, "little")


mailbox = sm(0, 0x1000, 128, 0x26) + sm(1, 0x1080, 128, 0x22)
preop = mailbox + al(2)
write("ethercat", "preop_upload", preop + mbx(sdo_upload(0x1018, 1)) + bytes([4]) + mbx(sdo_upload(0x6000, 2), 2) + bytes([4]))
write("ethercat", "preop_download", preop + mbx(sdo_download(0x8000, 1, 1234, 4)) + bytes([4]) +
      mbx(sdo_download(0x1C12, 0, 0, 1), 2) + bytes([4]) + mbx(sdo_download(0x1C12, 1, 0x1600, 2), 3) + bytes([4]))
write("ethercat", "sdo_info", preop + mbx(b"\x00\x30\x01\x00\x00\x00\x01\x00") + bytes([4, 9, 4]) +
      mbx(b"\x00\x30\x03\x00\x00\x00\x00\x60", 2) + bytes([4]) + mbx(b"\x00\x30\x05\x00\x00\x00\x00\x60\x02\x00", 3) + bytes([4]))
write("ethercat", "to_op", preop + sm(2, 0x1100, 2, 0x64) + sm(3, 0x1400, 6, 0x20) + al(4) + bytes([6]) + al(8) +
      bytes([5, 2, 0x34, 0x12, 6, 7, 1, 0x14]))
write("ethercat", "eeprom", bytes([8, 1]) + struct.pack("<I", 0) + bytes([8, 0x81]) + struct.pack("<I", 8) +
      bytes([8, 2]) + struct.pack("<I", 4))
write("ethercat", "bad_mailbox_length", preop + bytes([0]) + struct.pack("<HB", 0x1000, 2) + b"\xff\xff" + mbx(b"\x00\x20"))

# ---- plc: the compiler's test images -------------------------------------------------------------
for img in sorted((HERE.parent / "plc_corpus").glob("*.img")):
    (CORPUS / "plc").mkdir(parents=True, exist_ok=True)
    shutil.copyfile(img, CORPUS / "plc" / img.name)

print("corpus written to", CORPUS)
