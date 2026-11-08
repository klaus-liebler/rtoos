#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The PLC instruction set (docs/plc.md): the single source of truth for the C++ VM, the TypeScript
VM/compiler and the opcode tables in the documentation.

    python tools/plc_isa.py --hpp include/rtos/plc/isa.hpp --md docs/plc.md
    python tools/plc_isa.py --ts <firmware>/web/src/plc/generated/isa.ts

--md replaces the text between the markers "<!-- plc_isa:begin -->" and "<!-- plc_isa:end -->".

Opcodes are numbered in table order. Any change to the table (an opcode, an operand, a trap, a
native) changes ISA_HASH; images carry the hash and a VM rejects images built for another table.
ISA_VERSION is raised by hand for changes that are not just additions.
"""
import argparse
import re
import zlib
from pathlib import Path

ISA_VERSION = 1

# Operand kinds: name -> (bytes, C++ type, meaning)
OPERANDS = {
    "i8": (1, "std::int8_t", "signed 8-bit immediate"),
    "u16": (2, "std::uint16_t", "unsigned 16-bit"),
    "u32": (4, "std::uint32_t", "unsigned 32-bit"),
    "i32": (4, "std::int32_t", "signed 32-bit"),
    "u64": (8, "std::uint64_t", "unsigned 64-bit"),
    "i64": (8, "std::int64_t", "signed 64-bit"),
}

# (name, operands, pops, pushes, description). Operands are named "kind:role".
OPS = []


def op(name, operands, pops, pushes, text):
    OPS.append((name, operands, pops, pushes, text))


# ---- stack and constants ----------------------------------------------------------------------
op("NOP", [], 0, 0, "nothing")
op("PUSH_I8", ["i8:value"], 0, 1, "push the value sign-extended to i32")
op("PUSH_I32", ["i32:value"], 0, 1, "push an i32 (also the bits of an f32)")
op("PUSH_I64", ["i64:value"], 0, 1, "push an i64 (also the bits of an f64)")
op("DUP", [], 1, 2, "duplicate the top slot")
op("DROP", [], 1, 0, "remove the top slot")
op("SWAP", [], 2, 2, "exchange the two top slots")

# ---- memory -----------------------------------------------------------------------------------
# G: absolute data address (u32), L: offset from the frame pointer (u16), T: offset from THIS (u16),
# I: indirect, the address is popped (loads) or below the value (stores).
LOADS = [("U8", "zero-extended byte"), ("S8", "sign-extended byte"), ("U16", "zero-extended 16 bits"),
         ("S16", "sign-extended 16 bits"), ("32", "32 bits"), ("64", "64 bits")]
STORES = [("8", "low byte"), ("16", "low 16 bits"), ("32", "low 32 bits"), ("64", "64 bits")]
MODES = [("G", "u32:addr", "data[addr]"), ("L", "u16:offset", "data[FP + offset]"),
         ("T", "u16:offset", "data[THIS + offset]")]
for mode, operand, where in MODES:
    for width, text in LOADS:
        op(f"LD{mode}_{width}", [operand], 0, 1, f"push {text} of {where}")
for mode, operand, where in MODES:
    for width, text in STORES:
        op(f"ST{mode}_{width}", [operand], 1, 0, f"pop a value, store its {text} to {where}")
for width, text in LOADS:
    op(f"LDI_{width}", [], 1, 1, f"pop an address, push {text} of data[address]")
for width, text in STORES:
    op(f"STI_{width}", [], 2, 0, f"pop a value and an address below it, store the {text}")
op("ADDR_G", ["u32:addr"], 0, 1, "push the absolute address addr")
op("ADDR_L", ["u16:offset"], 0, 1, "push the absolute address FP + offset")
op("ADDR_T", ["u16:offset"], 0, 1, "push the absolute address THIS + offset")
op("MEMCPY", ["u32:size"], 2, 0, "pop a source address and a destination address below it, copy size bytes")

# ---- arithmetic and logic ---------------------------------------------------------------------
for t in ("I32", "I64"):
    op(f"ADD_{t}", [], 2, 1, "a + b, wraps")
    op(f"SUB_{t}", [], 2, 1, "a - b, wraps")
    op(f"MUL_{t}", [], 2, 1, "a * b, wraps")
    op(f"DIVS_{t}", [], 2, 1, "a / b signed, toward zero; b = 0 traps DIV_BY_ZERO; MIN / -1 = MIN")
    op(f"DIVU_{t}", [], 2, 1, "a / b unsigned; b = 0 traps DIV_BY_ZERO")
    op(f"MODS_{t}", [], 2, 1, "a MOD b signed (sign of a); b = 0 traps DIV_BY_ZERO; MIN MOD -1 = 0")
    op(f"MODU_{t}", [], 2, 1, "a MOD b unsigned; b = 0 traps DIV_BY_ZERO")
    op(f"NEG_{t}", [], 1, 1, "-a, wraps")
    op(f"AND_{t}", [], 2, 1, "bitwise and")
    op(f"OR_{t}", [], 2, 1, "bitwise or")
    op(f"XOR_{t}", [], 2, 1, "bitwise exclusive or")
    op(f"NOT_{t}", [], 1, 1, "bitwise complement")
    op(f"SHL_{t}", [], 2, 1, "a << b (b unsigned; b >= width gives 0)")
    op(f"SHR_{t}", [], 2, 1, "a >> b logical (b unsigned; b >= width gives 0)")
for t in ("F32", "F64"):
    op(f"ADD_{t}", [], 2, 1, "a + b (IEEE 754, NaN canonical)")
    op(f"SUB_{t}", [], 2, 1, "a - b")
    op(f"MUL_{t}", [], 2, 1, "a * b")
    op(f"DIV_{t}", [], 2, 1, "a / b; b = 0 traps DIV_BY_ZERO")
    op(f"NEG_{t}", [], 1, 1, "-a (flips the sign bit)")

# ---- comparisons: push 1 or 0 (i32) -----------------------------------------------------------
for t in ("I32", "I64"):
    op(f"EQ_{t}", [], 2, 1, "a = b")
    op(f"NE_{t}", [], 2, 1, "a <> b")
    for cmp, text in (("LT", "<"), ("LE", "<="), ("GT", ">"), ("GE", ">=")):
        op(f"{cmp}S_{t}", [], 2, 1, f"a {text} b signed")
        op(f"{cmp}U_{t}", [], 2, 1, f"a {text} b unsigned")
for t in ("F32", "F64"):
    for cmp, text in (("EQ", "="), ("NE", "<>"), ("LT", "<"), ("LE", "<="), ("GT", ">"), ("GE", ">=")):
        op(f"{cmp}_{t}", [], 2, 1, f"a {text} b (IEEE: false with NaN, except <>)")

# ---- normalization and conversion -------------------------------------------------------------
op("SEXT8", [], 1, 1, "i32: sign-extend the low 8 bits (SINT)")
op("SEXT16", [], 1, 1, "i32: sign-extend the low 16 bits (INT)")
op("ZEXT8", [], 1, 1, "i32: keep the low 8 bits (USINT, BYTE)")
op("ZEXT16", [], 1, 1, "i32: keep the low 16 bits (UINT, WORD)")
op("I32S_TO_I64", [], 1, 1, "sign-extend")
op("I32U_TO_I64", [], 1, 1, "zero-extend")
op("I64_TO_I32", [], 1, 1, "keep the low 32 bits")
for src in ("I32S", "I32U", "I64S", "I64U"):
    for dst in ("F32", "F64"):
        op(f"{src}_TO_{dst}", [], 1, 1, "convert, rounding to nearest even")
op("F32_TO_F64", [], 1, 1, "exact")
op("F64_TO_F32", [], 1, 1, "round to nearest even")
for src in ("F32", "F64"):
    op(f"{src}_ROUND_I64", [], 1, 1, "to i64, rounding half away from zero; NaN or out of range traps CONVERSION")
    op(f"{src}_TRUNC_I64", [], 1, 1, "to i64, toward zero; NaN or out of range traps CONVERSION")
op("CHK_I32", ["i32:lo", "i32:hi"], 1, 1, "trap RANGE unless lo <= a <= hi (signed); a stays")
op("CHK_U32", ["u32:lo", "u32:hi"], 1, 1, "trap RANGE unless lo <= a <= hi (unsigned); a stays")
op("CHK_I64", ["i64:lo", "i64:hi"], 1, 1, "trap RANGE unless lo <= a <= hi (signed); a stays")
op("CHK_U64", ["u64:lo", "u64:hi"], 1, 1, "trap RANGE unless lo <= a <= hi (unsigned); a stays")

# ---- control ----------------------------------------------------------------------------------
op("JMP", ["i32:rel"], 0, 0, "PC = start of the next instruction + rel")
op("JZ", ["i32:rel"], 1, 0, "pop; jump if the i32 is 0")
op("JNZ", ["i32:rel"], 1, 0, "pop; jump if the i32 is not 0")
op("CALL", ["u32:target"], 0, 0, "call the function at code offset target (an ENTER); THIS unchanged")
op("CALL_FB", ["u32:target"], 1, 0, "pop an instance address, call target with THIS = that address")
op("ENTER", ["u32:frame"], 0, 0, "first instruction of every body: allocate and zero frame bytes, FP = their start")
op("RET", [], 0, 0, "release the frame, return (from the task's top call: the program is done)")
op("SYSCALL", ["u16:native"], -1, -1, "call a native (table below); its signature gives pops and pushes")
op("TRAP", ["u16:code"], 0, 0, "stop with trap USER and the code (e.g. a CASE without a matching branch)")

# ---- traps: why a program stops -------------------------------------------------------------
TRAPS = [
    ("NONE", "no trap"),
    ("DIV_BY_ZERO", "integer or floating-point division by zero"),
    ("RANGE", "a CHK_* failed: array index, subrange, or a narrowing *_TO_* conversion"),
    ("CONVERSION", "floating-point to integer: NaN or out of range"),
    ("MEMORY", "a data access outside the data segment"),
    ("STACK_OVERFLOW", "operand stack full"),
    ("STACK_UNDERFLOW", "operand stack empty"),
    ("CALL_DEPTH", "control stack full (call depth)"),
    ("FRAME_OVERFLOW", "the task's frame area is full"),
    ("BUDGET", "the instruction budget of the cycle is used up"),
    ("NATIVE", "a native reported an error"),
    ("USER", "TRAP instruction"),
]

# ---- natives: (name, argument types, result types, description) ------------------------------
# The types are I32/I64/F32/F64 slots. The signature hash in the image covers name and types.
NATIVES = [
    ("TIME_MS", [], ["I32"], "the cycle's start time in ms (the task's clock, wraps)"),
    ("ABS_I32", ["I32"], ["I32"], "absolute value, wraps (ABS of MIN is MIN)"),
    ("ABS_I64", ["I64"], ["I64"], "absolute value, wraps"),
    ("ABS_F32", ["F32"], ["F32"], "clears the sign bit"),
    ("ABS_F64", ["F64"], ["F64"], "clears the sign bit"),
    ("SQRT_F32", ["F32"], ["F32"], "square root (correctly rounded, NaN canonical)"),
    ("SQRT_F64", ["F64"], ["F64"], "square root (correctly rounded, NaN canonical)"),
    ("ROL_8", ["I32", "I32"], ["I32"], "rotate the low 8 bits left by n mod 8"),
    ("ROL_16", ["I32", "I32"], ["I32"], "rotate the low 16 bits left by n mod 16"),
    ("ROL_32", ["I32", "I32"], ["I32"], "rotate left by n mod 32"),
    ("ROL_64", ["I64", "I32"], ["I64"], "rotate left by n mod 64"),
    ("ROR_8", ["I32", "I32"], ["I32"], "rotate the low 8 bits right by n mod 8"),
    ("ROR_16", ["I32", "I32"], ["I32"], "rotate the low 16 bits right by n mod 16"),
    ("ROR_32", ["I32", "I32"], ["I32"], "rotate right by n mod 32"),
    ("ROR_64", ["I64", "I32"], ["I64"], "rotate right by n mod 64"),
    # Transcendental functions: libm (target) and JavaScript (simulator) may differ in the last
    # bit; they are not part of the byte-identical differential tests.
    ("LN_F64", ["F64"], ["F64"], "natural logarithm"),
    ("LOG_F64", ["F64"], ["F64"], "base-10 logarithm"),
    ("EXP_F64", ["F64"], ["F64"], "e^x"),
    ("EXPT_F64", ["F64", "F64"], ["F64"], "a^b"),
    ("SIN_F64", ["F64"], ["F64"], "sine"),
    ("COS_F64", ["F64"], ["F64"], "cosine"),
    ("TAN_F64", ["F64"], ["F64"], "tangent"),
    ("ASIN_F64", ["F64"], ["F64"], "arc sine"),
    ("ACOS_F64", ["F64"], ["F64"], "arc cosine"),
    ("ATAN_F64", ["F64"], ["F64"], "arc tangent"),
]
TRANSCENDENTAL_FIRST = "LN_F64"


def operand(o):
    kind, role = o.split(":")
    return kind, role


def table_text():
    lines = [f"isa {ISA_VERSION}"]
    lines += [f"op {n} {' '.join(o)} {p} {q}" for n, o, p, q, _ in OPS]
    lines += [f"trap {n}" for n, _ in TRAPS]
    lines += [f"native {n} {' '.join(a)} -> {' '.join(r)}" for n, a, r, _ in NATIVES]
    return "\n".join(lines) + "\n"


def isa_hash():
    return zlib.crc32(table_text().encode())


def native_signature(n, args, results):
    return zlib.crc32(f"{n}({','.join(args)})->{','.join(results)}".encode())


def size_of(ops):
    return 1 + sum(OPERANDS[operand(o)[0]][0] for o in ops)


def gen_hpp():
    assert len(OPS) <= 256
    out = ["// SPDX-License-Identifier: MIT",
           "// GENERATED by tools/plc_isa.py - do not edit. The PLC instruction set (docs/plc.md).",
           "#pragma once", "", "#include <array>", "#include <cstddef>", "#include <cstdint>", "",
           "namespace rtos::plc {", "",
           f"inline constexpr std::uint16_t isa_version = {ISA_VERSION};",
           f"inline constexpr std::uint32_t isa_hash = 0x{isa_hash():08X};", "",
           "enum class Op : std::uint8_t {"]
    out += [f"    {n} = {i}," for i, (n, *_ ) in enumerate(OPS)]
    out += ["};", f"inline constexpr std::size_t op_count = {len(OPS)};", "",
            "/// Instruction length in bytes (opcode + operands); 0: not an opcode.",
            "inline constexpr std::array<std::uint8_t, 256> op_size = [] {",
            "    std::array<std::uint8_t, 256> s{};"]
    out += [f"    s[{i}] = {size_of(o)};" for i, (_, o, *_r) in enumerate(OPS)]
    out += ["    return s;", "}();", "", "enum class Trap : std::uint8_t {"]
    out += [f"    {n} = {i},  ///< {t}" for i, (n, t) in enumerate(TRAPS)]
    out += ["};", "", "enum class Native : std::uint16_t {"]
    out += [f"    {n} = {i},  ///< {t}" for i, (n, _a, _r, t) in enumerate(NATIVES)]
    out += ["};", f"inline constexpr std::size_t native_count = {len(NATIVES)};", "",
            "struct NativeInfo {", "    std::uint8_t args;     ///< slots popped",
            "    std::uint8_t results;  ///< slots pushed", "    std::uint32_t signature;", "};",
            "inline constexpr std::array<NativeInfo, native_count> native_info{{"]
    out += [f"    {{{len(a)}, {len(r)}, 0x{native_signature(n, a, r):08X}}},  // {n}" for n, a, r, _ in NATIVES]
    out += ["}};", "", "}  // namespace rtos::plc", ""]
    return "\n".join(out)


def gen_ts():
    out = ["// SPDX-License-Identifier: MIT",
           "// GENERATED by rtoos/tools/plc_isa.py - do not edit. The PLC instruction set (rtoos/docs/plc.md).",
           "", f"export const ISA_VERSION = {ISA_VERSION};", f"export const ISA_HASH = 0x{isa_hash():08X};", "",
           "export const enum Op {"]
    out += [f"\t{n} = {i}," for i, (n, *_ ) in enumerate(OPS)]
    out += ["}", "", "export type OperandKind = " + " | ".join(f'"{k}"' for k in OPERANDS) + ";", "",
            "export interface OpInfo {", "\treadonly name: string;", "\treadonly operands: readonly OperandKind[];",
            "\treadonly size: number;", "\treadonly pops: number;", "\treadonly pushes: number;", "}", "",
            "export const OPS: readonly OpInfo[] = ["]
    for n, o, p, q, _ in OPS:
        kinds = ", ".join(f'"{operand(x)[0]}"' for x in o)
        out.append(f'\t{{ name: "{n}", operands: [{kinds}], size: {size_of(o)}, pops: {p}, pushes: {q} }},')
    out += ["];", "", "export const enum Trap {"]
    out += [f"\t{n} = {i}," for i, (n, _t) in enumerate(TRAPS)]
    out += ["}", "", "export const TRAP_NAMES: readonly string[] = [" + ", ".join(f'"{n}"' for n, _ in TRAPS) + "];",
            "", "export type Slot = \"I32\" | \"I64\" | \"F32\" | \"F64\";", "",
            "export const enum Native {"]
    out += [f"\t{n} = {i}," for i, (n, *_ ) in enumerate(NATIVES)]
    out += ["}", "", "export interface NativeInfo {", "\treadonly name: string;", "\treadonly args: readonly Slot[];",
            "\treadonly results: readonly Slot[];", "\treadonly signature: number;",
            "\t/** libm and JavaScript may differ in the last bit: not byte-identical across the VMs. */",
            "\treadonly transcendental: boolean;", "}", "", "export const NATIVES: readonly NativeInfo[] = ["]
    transcendental = False
    for n, a, r, _ in NATIVES:
        transcendental = transcendental or n == TRANSCENDENTAL_FIRST
        args = ", ".join(f'"{x}"' for x in a)
        res = ", ".join(f'"{x}"' for x in r)
        out.append(f'\t{{ name: "{n}", args: [{args}], results: [{res}], signature: 0x{native_signature(n, a, r):08X}, '
                   f'transcendental: {"true" if transcendental else "false"} }},')
    out += ["];", ""]
    return "\n".join(out)


def gen_md():
    out = [f"ISA version {ISA_VERSION}, hash `0x{isa_hash():08X}`, {len(OPS)} opcodes.", "",
           "| # | Opcode | Operands | Pops | Pushes | Effect |", "|---|---|---|---|---|---|"]
    for i, (n, o, p, q, t) in enumerate(OPS):
        ops = ", ".join(f"{operand(x)[1]}: {operand(x)[0]}" for x in o)
        pops = "sig." if p < 0 else str(p)
        pushes = "sig." if q < 0 else str(q)
        out.append(f"| {i} | `{n}` | {ops} | {pops} | {pushes} | {t} |")
    out += ["", "| # | Trap | Meaning |", "|---|---|---|"]
    out += [f"| {i} | `{n}` | {t} |" for i, (n, t) in enumerate(TRAPS)]
    out += ["", "| # | Native | Arguments | Results | Meaning |", "|---|---|---|---|---|"]
    out += [f"| {i} | `{n}` | {', '.join(a) or '-'} | {', '.join(r)} | {t} |" for i, (n, a, r, t) in enumerate(NATIVES)]
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hpp")
    ap.add_argument("--ts")
    ap.add_argument("--md")
    a = ap.parse_args()
    if a.hpp:
        Path(a.hpp).write_text(gen_hpp(), encoding="utf-8", newline="\n")
    if a.ts:
        Path(a.ts).parent.mkdir(parents=True, exist_ok=True)
        Path(a.ts).write_text(gen_ts(), encoding="utf-8", newline="\n")
    if a.md:
        p = Path(a.md)
        text = p.read_text(encoding="utf-8")
        new, n = re.subn(r"(<!-- plc_isa:begin -->\n).*?(<!-- plc_isa:end -->)",
                         lambda m: m.group(1) + gen_md() + "\n" + m.group(2), text, flags=re.S)
        if n != 1:
            raise SystemExit(f"{a.md}: markers not found")
        p.write_text(new, encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
