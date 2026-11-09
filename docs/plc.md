# PLC: bytecode VM for IEC 61131-3 programs

rtoos runs IEC 61131-3 programs (ST, FBD, SFC) as **bytecode**. The compiler is not part of rtoos:
it runs in the browser (TypeScript, in the Factory Control Unit's web UI, `web/src/plc/`) and
produces an **image**, which the device stores and runs without a browser. The design and its
reasons are in the FCU specification `docs/plc-iec61131.md` of the firmware repository; this page
is the contract between the compiler and the VM.

| | |
|---|---|
| Instruction set | [tools/plc_isa.py](../tools/plc_isa.py) - the single source: generates [isa.hpp](../include/rtos/plc/isa.hpp), the TypeScript `isa.ts` of the compiler and the tables below |
| VM | [vm.hpp](../include/rtos/plc/vm.hpp), [vm.cpp](../src/plc/vm.cpp) (library `rtos_plc`) |
| Image | [image.hpp](../include/rtos/plc/image.hpp): format, loader, verifier |
| Tests | [plc_vm_test.cpp](../tests/plc_vm_test.cpp) (host), differential tests against the TypeScript VM ([§ Differential tests](#differential-tests)) |

## Machine model

- **Code segment**: the bytecode of all POUs, read-only, copied from the image into RAM.
- **Data segment**: one byte array for everything the program stores: global variables, program
  and function block instances, and per task a **frame area** for the locals of functions and
  `VAR_TEMP`. All addresses are byte offsets into it; every access is checked against its size
  (trap `MEMORY`). Little-endian, natural alignment is not required.
- **Operand stack**: 64 slots of 64 bits per execution context. A slot holds raw bits: an i32 or
  f32 in the low 32 bits (the upper half is ignored), an i64 or f64 in all 64. The instructions are
  typed; nothing at run time knows what a slot holds.
- **Control stack**: 32 entries per execution context (return PC, saved FP and THIS, frame size).
  It is not in the data segment: a program cannot overwrite a return address.
- **Registers** of an execution context: PC, FP (frame pointer, the current frame's start in the
  data segment), THIS (the current instance), the frame area's top.

### Values and types

| IEC type | Slot | In memory | Note |
|---|---|---|---|
| BOOL | i32, 0 or 1 | 1 byte | NOT = XOR 1 |
| SINT, USINT, BYTE | i32 | 1 byte | normalized with `SEXT8`/`ZEXT8` after every arithmetic instruction |
| INT, UINT, WORD | i32 | 2 bytes | `SEXT16`/`ZEXT16` |
| DINT, UDINT, DWORD | i32 | 4 bytes | |
| LINT, ULINT, LWORD | i64 | 8 bytes | |
| REAL | f32 | 4 bytes | IEEE 754 binary32 |
| LREAL | f64 | 8 bytes | IEEE 754 binary64 (software on the Cortex-M33) |
| TIME | i32 | 4 bytes | milliseconds, signed (±24.8 days) |

Integer arithmetic wraps (two's complement). Division and MOD by zero trap (`DIV_BY_ZERO`); so does a
floating-point division by zero. `MIN / -1` gives `MIN` (no trap). Shifts by the type's width or more give 0.

**Floating point is identical on every VM.** `+ - * /`, square root and the conversions are exactly
rounded IEEE 754 operations on the host (x86-64 SSE), on the Cortex-M33 (single precision in the FPU,
double precision in libgcc's soft-float), on the Cortex-M7 and in JavaScript (an f32 operation is done
in binary64 and rounded once to binary32, which gives the same result for these operations because
53 ≥ 2·24 + 2). A NaN result is replaced by the canonical quiet NaN (`0x7FC00000`, `0x7FF8000000000000`),
because the platforms create NaNs with different bits. Only the transcendental natives (`LN_F64` and the
ones after it) may differ in the last bit between libm and JavaScript.

Floating point to integer: `*_ROUND_I64` rounds half away from zero (`REAL_TO_INT(2.5) = 3`,
`(-2.5) → -3`), `*_TRUNC_I64` toward zero (`TRUNC`); NaN or a value outside i64 traps `CONVERSION`.
Narrowing to the IEC type is a range check (`CHK_*`, trap `RANGE`), as is every array index.

### Calls

- Every body (program, function block, function) starts with `ENTER frame`: it pushes nothing, allocates
  `frame` bytes in the task's frame area, zeroes them and sets FP to their start. The compiler
  initializes non-zero locals right after it.
- **Function**: the caller pushes the arguments in declaration order and executes `CALL`. The
  function pops them into its frame (last argument first), computes, pushes its result and executes `RET`.
  THIS stays the caller's.
- **Function block**: the caller stores the inputs into the instance, pushes the instance address and
  executes `CALL_FB`; the body sees the instance as THIS; the caller reads the outputs from the instance
  afterwards. VAR_IN_OUT are addresses in the instance.
- **Program**: like a function block whose instance is fixed; the task calls it (task table).
- `RET` releases the frame and restores PC, FP and THIS from the control stack. A `RET` of the
  task's top call ends that program for this cycle.

### Natives

`SYSCALL n` calls native function n of the table below. A native pops its arguments (last argument
on top) and pushes its results. The image lists the natives it uses with their signature hashes;
the loader rejects an image that needs a native this VM does not have or has with another signature.

### Execution

A task's cycle runs its programs in order. `Vm::run` executes until the programs are done, a trap
occurs, or the **instruction budget** of the call is used up (trap `BUDGET`: a deterministic watchdog,
the same in the emulators and on the hardware). After a trap the program stays stopped where it
was: PC and the trap code are kept for the error message.

## Image format

Little-endian. Sections are 4-byte aligned, the offsets count from the start of the image.

| Offset | Size | Field |
|---|---|---|
| 0 | 8 | magic `"FCUPLC\0\1"` |
| 8 | 2 | ISA version |
| 10 | 2 | header size (80) |
| 12 | 4 | image length |
| 16 | 4 | CRC-32 (IEEE 802.3, as zlib) over bytes 20 … length-1 |
| 20 | 4 | ISA hash |
| 24 | 4 | I/O map hash (the register map the image was built for; 0: none) |
| 28 | 4 | project hash (links the image to the debug information in the IDE) |
| 32 | 4 | compiler version (major << 16 \| minor << 8 \| patch) |
| 36 | 4 + 4 | code: offset, size |
| 44 | 4 | data segment size |
| 48 | 4 + 4 | initial values: offset, size |
| 56 | 4 + 4 | tasks: offset, count |
| 64 | 4 + 4 | I/O bindings: offset, count |
| 72 | 4 + 4 | natives: offset, count |

The image contains no time stamp: the same project gives a byte-identical image.

**Initial values**: records `addr: u32, length: u32, bytes[length]`, each padded to 4 bytes. The rest of
the data segment starts as 0.

**Task** (32 bytes): `interval: u16` (in I/O cycles), `priority: u8`, `max_overruns: u8`, `budget: u32`
(instructions per cycle), `frame_base: u32`, `frame_size: u32` (the task's frame area), `programs: u32`
(offset of the program list), `program_count: u32`, `io_first: u32`, `io_count: u32` (the task's
I/O bindings, a range of the I/O table). Program list entry (8 bytes): `entry: u32` (code offset of an
`ENTER`), `instance: u32` (data address).

**I/O binding** (12 bytes): `register: u16`, `area: u8` (0 input register, 1 holding register),
`kind: u8` (0 input to the program, 1 output, 2 command, 3 parameter), `bits: u8` (1: BOOL, 16, 32),
`flags: u8` (bit 0: signed), `reserved: u16`, `addr: u32` (data address of the variable).

**Native** (8 bytes): `id: u16`, `reserved: u16`, `signature: u32`.

### Verifier

`Image::load` checks the whole image once, so the interpreter does not have to:

- magic, header size, length, CRC, ISA version and hash; every section inside the image and aligned;
- code and data fit into the VM's memory; initial values and frame areas inside the data segment,
  frame areas of different tasks do not overlap;
- **every instruction** decodes: a known opcode, operands inside the code; the code ends on an
  instruction boundary;
- every jump target is the start of an instruction, every `CALL`/`CALL_FB` target and every program
  entry is an `ENTER`;
- every `G` address plus its access width is inside the data segment;
- every `SYSCALL` names a native that the image lists and the VM provides with the same signature.

What only the run time can know is checked when it happens: `L`/`T`/indirect addresses, the stacks,
the frame area, the budget, division by zero, conversions and `CHK_*`.

## Program store

[slot_store.hpp](../include/rtos/plc/slot_store.hpp) keeps the image and a small device configuration
(control source, autostart) in two A/B slots of the MCU's own flash, so the PLC starts without an SD card
and a power loss never leaves half a program:

- `commit(image, config)` erases and writes the **inactive** slot: image first, the header (sequence
  number, length, CRCs, configuration) last. A slot counts only with a valid header and an intact image;
  `mount()` takes the valid slot with the highest sequence number, the other one is the previous program.
- `set_config()` appends a record (one program unit) to the active slot's journal at its end; the newest
  valid record wins. A full journal moves: the image is copied into the other slot with a fresh journal.
- On the FCU both chips use 256 KB at `0x081C'0000` (STM32H563: 2 x 16 sectors of 8 KB, STM32H743: the last
  two 128 KB sectors of bank 2); the image may be up to 124 KB.
- [plc_slot_store_test.cpp](../tests/plc_slot_store_test.cpp) cuts the power after every flash operation of
  a commit and of a journal move, for both geometries: afterwards the old or the new program is complete.
- [apps/plcslots](../apps/plcslots/main.cpp) runs the store in the real flash layout of both chips in Renode
  (`plcslots_renode`, `h743_plcslots_renode`), against the flash interface models (unlock, sector erase).

## Instruction set

<!-- plc_isa:begin -->
ISA version 1, hash `0xE6361F83`, 155 opcodes.

| # | Opcode | Operands | Pops | Pushes | Effect |
|---|---|---|---|---|---|
| 0 | `NOP` |  | 0 | 0 | nothing |
| 1 | `PUSH_I8` | value: i8 | 0 | 1 | push the value sign-extended to i32 |
| 2 | `PUSH_I32` | value: i32 | 0 | 1 | push an i32 (also the bits of an f32) |
| 3 | `PUSH_I64` | value: i64 | 0 | 1 | push an i64 (also the bits of an f64) |
| 4 | `DUP` |  | 1 | 2 | duplicate the top slot |
| 5 | `DROP` |  | 1 | 0 | remove the top slot |
| 6 | `SWAP` |  | 2 | 2 | exchange the two top slots |
| 7 | `LDG_U8` | addr: u32 | 0 | 1 | push zero-extended byte of data[addr] |
| 8 | `LDG_S8` | addr: u32 | 0 | 1 | push sign-extended byte of data[addr] |
| 9 | `LDG_U16` | addr: u32 | 0 | 1 | push zero-extended 16 bits of data[addr] |
| 10 | `LDG_S16` | addr: u32 | 0 | 1 | push sign-extended 16 bits of data[addr] |
| 11 | `LDG_32` | addr: u32 | 0 | 1 | push 32 bits of data[addr] |
| 12 | `LDG_64` | addr: u32 | 0 | 1 | push 64 bits of data[addr] |
| 13 | `LDL_U8` | offset: u16 | 0 | 1 | push zero-extended byte of data[FP + offset] |
| 14 | `LDL_S8` | offset: u16 | 0 | 1 | push sign-extended byte of data[FP + offset] |
| 15 | `LDL_U16` | offset: u16 | 0 | 1 | push zero-extended 16 bits of data[FP + offset] |
| 16 | `LDL_S16` | offset: u16 | 0 | 1 | push sign-extended 16 bits of data[FP + offset] |
| 17 | `LDL_32` | offset: u16 | 0 | 1 | push 32 bits of data[FP + offset] |
| 18 | `LDL_64` | offset: u16 | 0 | 1 | push 64 bits of data[FP + offset] |
| 19 | `LDT_U8` | offset: u16 | 0 | 1 | push zero-extended byte of data[THIS + offset] |
| 20 | `LDT_S8` | offset: u16 | 0 | 1 | push sign-extended byte of data[THIS + offset] |
| 21 | `LDT_U16` | offset: u16 | 0 | 1 | push zero-extended 16 bits of data[THIS + offset] |
| 22 | `LDT_S16` | offset: u16 | 0 | 1 | push sign-extended 16 bits of data[THIS + offset] |
| 23 | `LDT_32` | offset: u16 | 0 | 1 | push 32 bits of data[THIS + offset] |
| 24 | `LDT_64` | offset: u16 | 0 | 1 | push 64 bits of data[THIS + offset] |
| 25 | `STG_8` | addr: u32 | 1 | 0 | pop a value, store its low byte to data[addr] |
| 26 | `STG_16` | addr: u32 | 1 | 0 | pop a value, store its low 16 bits to data[addr] |
| 27 | `STG_32` | addr: u32 | 1 | 0 | pop a value, store its low 32 bits to data[addr] |
| 28 | `STG_64` | addr: u32 | 1 | 0 | pop a value, store its 64 bits to data[addr] |
| 29 | `STL_8` | offset: u16 | 1 | 0 | pop a value, store its low byte to data[FP + offset] |
| 30 | `STL_16` | offset: u16 | 1 | 0 | pop a value, store its low 16 bits to data[FP + offset] |
| 31 | `STL_32` | offset: u16 | 1 | 0 | pop a value, store its low 32 bits to data[FP + offset] |
| 32 | `STL_64` | offset: u16 | 1 | 0 | pop a value, store its 64 bits to data[FP + offset] |
| 33 | `STT_8` | offset: u16 | 1 | 0 | pop a value, store its low byte to data[THIS + offset] |
| 34 | `STT_16` | offset: u16 | 1 | 0 | pop a value, store its low 16 bits to data[THIS + offset] |
| 35 | `STT_32` | offset: u16 | 1 | 0 | pop a value, store its low 32 bits to data[THIS + offset] |
| 36 | `STT_64` | offset: u16 | 1 | 0 | pop a value, store its 64 bits to data[THIS + offset] |
| 37 | `LDI_U8` |  | 1 | 1 | pop an address, push zero-extended byte of data[address] |
| 38 | `LDI_S8` |  | 1 | 1 | pop an address, push sign-extended byte of data[address] |
| 39 | `LDI_U16` |  | 1 | 1 | pop an address, push zero-extended 16 bits of data[address] |
| 40 | `LDI_S16` |  | 1 | 1 | pop an address, push sign-extended 16 bits of data[address] |
| 41 | `LDI_32` |  | 1 | 1 | pop an address, push 32 bits of data[address] |
| 42 | `LDI_64` |  | 1 | 1 | pop an address, push 64 bits of data[address] |
| 43 | `STI_8` |  | 2 | 0 | pop a value and an address below it, store the low byte |
| 44 | `STI_16` |  | 2 | 0 | pop a value and an address below it, store the low 16 bits |
| 45 | `STI_32` |  | 2 | 0 | pop a value and an address below it, store the low 32 bits |
| 46 | `STI_64` |  | 2 | 0 | pop a value and an address below it, store the 64 bits |
| 47 | `ADDR_G` | addr: u32 | 0 | 1 | push the absolute address addr |
| 48 | `ADDR_L` | offset: u16 | 0 | 1 | push the absolute address FP + offset |
| 49 | `ADDR_T` | offset: u16 | 0 | 1 | push the absolute address THIS + offset |
| 50 | `MEMCPY` | size: u32 | 2 | 0 | pop a source address and a destination address below it, copy size bytes |
| 51 | `ADD_I32` |  | 2 | 1 | a + b, wraps |
| 52 | `SUB_I32` |  | 2 | 1 | a - b, wraps |
| 53 | `MUL_I32` |  | 2 | 1 | a * b, wraps |
| 54 | `DIVS_I32` |  | 2 | 1 | a / b signed, toward zero; b = 0 traps DIV_BY_ZERO; MIN / -1 = MIN |
| 55 | `DIVU_I32` |  | 2 | 1 | a / b unsigned; b = 0 traps DIV_BY_ZERO |
| 56 | `MODS_I32` |  | 2 | 1 | a MOD b signed (sign of a); b = 0 traps DIV_BY_ZERO; MIN MOD -1 = 0 |
| 57 | `MODU_I32` |  | 2 | 1 | a MOD b unsigned; b = 0 traps DIV_BY_ZERO |
| 58 | `NEG_I32` |  | 1 | 1 | -a, wraps |
| 59 | `AND_I32` |  | 2 | 1 | bitwise and |
| 60 | `OR_I32` |  | 2 | 1 | bitwise or |
| 61 | `XOR_I32` |  | 2 | 1 | bitwise exclusive or |
| 62 | `NOT_I32` |  | 1 | 1 | bitwise complement |
| 63 | `SHL_I32` |  | 2 | 1 | a << b (b unsigned; b >= width gives 0) |
| 64 | `SHR_I32` |  | 2 | 1 | a >> b logical (b unsigned; b >= width gives 0) |
| 65 | `ADD_I64` |  | 2 | 1 | a + b, wraps |
| 66 | `SUB_I64` |  | 2 | 1 | a - b, wraps |
| 67 | `MUL_I64` |  | 2 | 1 | a * b, wraps |
| 68 | `DIVS_I64` |  | 2 | 1 | a / b signed, toward zero; b = 0 traps DIV_BY_ZERO; MIN / -1 = MIN |
| 69 | `DIVU_I64` |  | 2 | 1 | a / b unsigned; b = 0 traps DIV_BY_ZERO |
| 70 | `MODS_I64` |  | 2 | 1 | a MOD b signed (sign of a); b = 0 traps DIV_BY_ZERO; MIN MOD -1 = 0 |
| 71 | `MODU_I64` |  | 2 | 1 | a MOD b unsigned; b = 0 traps DIV_BY_ZERO |
| 72 | `NEG_I64` |  | 1 | 1 | -a, wraps |
| 73 | `AND_I64` |  | 2 | 1 | bitwise and |
| 74 | `OR_I64` |  | 2 | 1 | bitwise or |
| 75 | `XOR_I64` |  | 2 | 1 | bitwise exclusive or |
| 76 | `NOT_I64` |  | 1 | 1 | bitwise complement |
| 77 | `SHL_I64` |  | 2 | 1 | a << b (b unsigned; b >= width gives 0) |
| 78 | `SHR_I64` |  | 2 | 1 | a >> b logical (b unsigned; b >= width gives 0) |
| 79 | `ADD_F32` |  | 2 | 1 | a + b (IEEE 754, NaN canonical) |
| 80 | `SUB_F32` |  | 2 | 1 | a - b |
| 81 | `MUL_F32` |  | 2 | 1 | a * b |
| 82 | `DIV_F32` |  | 2 | 1 | a / b; b = 0 traps DIV_BY_ZERO |
| 83 | `NEG_F32` |  | 1 | 1 | -a (flips the sign bit) |
| 84 | `ADD_F64` |  | 2 | 1 | a + b (IEEE 754, NaN canonical) |
| 85 | `SUB_F64` |  | 2 | 1 | a - b |
| 86 | `MUL_F64` |  | 2 | 1 | a * b |
| 87 | `DIV_F64` |  | 2 | 1 | a / b; b = 0 traps DIV_BY_ZERO |
| 88 | `NEG_F64` |  | 1 | 1 | -a (flips the sign bit) |
| 89 | `EQ_I32` |  | 2 | 1 | a = b |
| 90 | `NE_I32` |  | 2 | 1 | a <> b |
| 91 | `LTS_I32` |  | 2 | 1 | a < b signed |
| 92 | `LTU_I32` |  | 2 | 1 | a < b unsigned |
| 93 | `LES_I32` |  | 2 | 1 | a <= b signed |
| 94 | `LEU_I32` |  | 2 | 1 | a <= b unsigned |
| 95 | `GTS_I32` |  | 2 | 1 | a > b signed |
| 96 | `GTU_I32` |  | 2 | 1 | a > b unsigned |
| 97 | `GES_I32` |  | 2 | 1 | a >= b signed |
| 98 | `GEU_I32` |  | 2 | 1 | a >= b unsigned |
| 99 | `EQ_I64` |  | 2 | 1 | a = b |
| 100 | `NE_I64` |  | 2 | 1 | a <> b |
| 101 | `LTS_I64` |  | 2 | 1 | a < b signed |
| 102 | `LTU_I64` |  | 2 | 1 | a < b unsigned |
| 103 | `LES_I64` |  | 2 | 1 | a <= b signed |
| 104 | `LEU_I64` |  | 2 | 1 | a <= b unsigned |
| 105 | `GTS_I64` |  | 2 | 1 | a > b signed |
| 106 | `GTU_I64` |  | 2 | 1 | a > b unsigned |
| 107 | `GES_I64` |  | 2 | 1 | a >= b signed |
| 108 | `GEU_I64` |  | 2 | 1 | a >= b unsigned |
| 109 | `EQ_F32` |  | 2 | 1 | a = b (IEEE: false with NaN, except <>) |
| 110 | `NE_F32` |  | 2 | 1 | a <> b (IEEE: false with NaN, except <>) |
| 111 | `LT_F32` |  | 2 | 1 | a < b (IEEE: false with NaN, except <>) |
| 112 | `LE_F32` |  | 2 | 1 | a <= b (IEEE: false with NaN, except <>) |
| 113 | `GT_F32` |  | 2 | 1 | a > b (IEEE: false with NaN, except <>) |
| 114 | `GE_F32` |  | 2 | 1 | a >= b (IEEE: false with NaN, except <>) |
| 115 | `EQ_F64` |  | 2 | 1 | a = b (IEEE: false with NaN, except <>) |
| 116 | `NE_F64` |  | 2 | 1 | a <> b (IEEE: false with NaN, except <>) |
| 117 | `LT_F64` |  | 2 | 1 | a < b (IEEE: false with NaN, except <>) |
| 118 | `LE_F64` |  | 2 | 1 | a <= b (IEEE: false with NaN, except <>) |
| 119 | `GT_F64` |  | 2 | 1 | a > b (IEEE: false with NaN, except <>) |
| 120 | `GE_F64` |  | 2 | 1 | a >= b (IEEE: false with NaN, except <>) |
| 121 | `SEXT8` |  | 1 | 1 | i32: sign-extend the low 8 bits (SINT) |
| 122 | `SEXT16` |  | 1 | 1 | i32: sign-extend the low 16 bits (INT) |
| 123 | `ZEXT8` |  | 1 | 1 | i32: keep the low 8 bits (USINT, BYTE) |
| 124 | `ZEXT16` |  | 1 | 1 | i32: keep the low 16 bits (UINT, WORD) |
| 125 | `I32S_TO_I64` |  | 1 | 1 | sign-extend |
| 126 | `I32U_TO_I64` |  | 1 | 1 | zero-extend |
| 127 | `I64_TO_I32` |  | 1 | 1 | keep the low 32 bits |
| 128 | `I32S_TO_F32` |  | 1 | 1 | convert, rounding to nearest even |
| 129 | `I32S_TO_F64` |  | 1 | 1 | convert, rounding to nearest even |
| 130 | `I32U_TO_F32` |  | 1 | 1 | convert, rounding to nearest even |
| 131 | `I32U_TO_F64` |  | 1 | 1 | convert, rounding to nearest even |
| 132 | `I64S_TO_F32` |  | 1 | 1 | convert, rounding to nearest even |
| 133 | `I64S_TO_F64` |  | 1 | 1 | convert, rounding to nearest even |
| 134 | `I64U_TO_F32` |  | 1 | 1 | convert, rounding to nearest even |
| 135 | `I64U_TO_F64` |  | 1 | 1 | convert, rounding to nearest even |
| 136 | `F32_TO_F64` |  | 1 | 1 | exact |
| 137 | `F64_TO_F32` |  | 1 | 1 | round to nearest even |
| 138 | `F32_ROUND_I64` |  | 1 | 1 | to i64, rounding half away from zero; NaN or out of range traps CONVERSION |
| 139 | `F32_TRUNC_I64` |  | 1 | 1 | to i64, toward zero; NaN or out of range traps CONVERSION |
| 140 | `F64_ROUND_I64` |  | 1 | 1 | to i64, rounding half away from zero; NaN or out of range traps CONVERSION |
| 141 | `F64_TRUNC_I64` |  | 1 | 1 | to i64, toward zero; NaN or out of range traps CONVERSION |
| 142 | `CHK_I32` | lo: i32, hi: i32 | 1 | 1 | trap RANGE unless lo <= a <= hi (signed); a stays |
| 143 | `CHK_U32` | lo: u32, hi: u32 | 1 | 1 | trap RANGE unless lo <= a <= hi (unsigned); a stays |
| 144 | `CHK_I64` | lo: i64, hi: i64 | 1 | 1 | trap RANGE unless lo <= a <= hi (signed); a stays |
| 145 | `CHK_U64` | lo: u64, hi: u64 | 1 | 1 | trap RANGE unless lo <= a <= hi (unsigned); a stays |
| 146 | `JMP` | rel: i32 | 0 | 0 | PC = start of the next instruction + rel |
| 147 | `JZ` | rel: i32 | 1 | 0 | pop; jump if the i32 is 0 |
| 148 | `JNZ` | rel: i32 | 1 | 0 | pop; jump if the i32 is not 0 |
| 149 | `CALL` | target: u32 | 0 | 0 | call the function at code offset target (an ENTER); THIS unchanged |
| 150 | `CALL_FB` | target: u32 | 1 | 0 | pop an instance address, call target with THIS = that address |
| 151 | `ENTER` | frame: u32 | 0 | 0 | first instruction of every body: allocate and zero frame bytes, FP = their start |
| 152 | `RET` |  | 0 | 0 | release the frame, return (from the task's top call: the program is done) |
| 153 | `SYSCALL` | native: u16 | sig. | sig. | call a native (table below); its signature gives pops and pushes |
| 154 | `TRAP` | code: u16 | 0 | 0 | stop with trap USER and the code (e.g. a CASE without a matching branch) |

| # | Trap | Meaning |
|---|---|---|
| 0 | `NONE` | no trap |
| 1 | `DIV_BY_ZERO` | integer or floating-point division by zero |
| 2 | `RANGE` | a CHK_* failed: array index, subrange, or a narrowing *_TO_* conversion |
| 3 | `CONVERSION` | floating-point to integer: NaN or out of range |
| 4 | `MEMORY` | a data access outside the data segment |
| 5 | `STACK_OVERFLOW` | operand stack full |
| 6 | `STACK_UNDERFLOW` | operand stack empty |
| 7 | `CALL_DEPTH` | control stack full (call depth) |
| 8 | `FRAME_OVERFLOW` | the task's frame area is full |
| 9 | `BUDGET` | the instruction budget of the cycle is used up |
| 10 | `NATIVE` | a native reported an error |
| 11 | `USER` | TRAP instruction |

| # | Native | Arguments | Results | Meaning |
|---|---|---|---|---|
| 0 | `TIME_MS` | - | I32 | the cycle's start time in ms (the task's clock, wraps) |
| 1 | `ABS_I32` | I32 | I32 | absolute value, wraps (ABS of MIN is MIN) |
| 2 | `ABS_I64` | I64 | I64 | absolute value, wraps |
| 3 | `ABS_F32` | F32 | F32 | clears the sign bit |
| 4 | `ABS_F64` | F64 | F64 | clears the sign bit |
| 5 | `SQRT_F32` | F32 | F32 | square root (correctly rounded, NaN canonical) |
| 6 | `SQRT_F64` | F64 | F64 | square root (correctly rounded, NaN canonical) |
| 7 | `ROL_8` | I32, I32 | I32 | rotate the low 8 bits left by n mod 8 |
| 8 | `ROL_16` | I32, I32 | I32 | rotate the low 16 bits left by n mod 16 |
| 9 | `ROL_32` | I32, I32 | I32 | rotate left by n mod 32 |
| 10 | `ROL_64` | I64, I32 | I64 | rotate left by n mod 64 |
| 11 | `ROR_8` | I32, I32 | I32 | rotate the low 8 bits right by n mod 8 |
| 12 | `ROR_16` | I32, I32 | I32 | rotate the low 16 bits right by n mod 16 |
| 13 | `ROR_32` | I32, I32 | I32 | rotate right by n mod 32 |
| 14 | `ROR_64` | I64, I32 | I64 | rotate right by n mod 64 |
| 15 | `LN_F64` | F64 | F64 | natural logarithm |
| 16 | `LOG_F64` | F64 | F64 | base-10 logarithm |
| 17 | `EXP_F64` | F64 | F64 | e^x |
| 18 | `EXPT_F64` | F64, F64 | F64 | a^b |
| 19 | `SIN_F64` | F64 | F64 | sine |
| 20 | `COS_F64` | F64 | F64 | cosine |
| 21 | `TAN_F64` | F64 | F64 | tangent |
| 22 | `ASIN_F64` | F64 | F64 | arc sine |
| 23 | `ACOS_F64` | F64 | F64 | arc cosine |
| 24 | `ATAN_F64` | F64 | F64 | arc tangent |
<!-- plc_isa:end -->

## Differential tests

The firmware repository's corpus (`web/src/plc/corpus/*.st`) is compiled by the TypeScript compiler
and run in the TypeScript VM; `npm run plc:corpus` writes each image and the trace of its variables per
cycle to `tests/plc_corpus/`. [plc_corpus_test.cpp](../tests/plc_corpus_test.cpp) runs the same images
in the C++ VM and requires byte-identical traces. A vitest test fails when the files in `tests/plc_corpus/`
are not what the current compiler produces.
