# EtherCAT

An EtherCAT slave (SubDevice) stack for an ESC on a host interface (PDI): `include/rtos/ethercat/`, header-only,
no heap, testable on the host against a simulated ESC. The ESC hardware is behind the `EscPort` concept; the
first driver is the Microchip LAN9253 on SPI (`include/rtos/devices/lan9253.hpp`).

Status: EEPROM emulation, AL state machine with checks, diagnostics (step E1); mailbox with CoE - SDO,
SDO information, PDO assignment - over a constexpr object dictionary (E2); process data with the
SyncManager watchdog (E3). Not offered: EoE, FoE, distributed clocks (SYNC0/1), complete access,
segmented SDO.

## Parts

| Part | File | What it does |
|---|---|---|
| ESC access | `ethercat/esc.hpp` | `EscPort` concept (read/write a run of bytes at an ESC address), the ESC registers of ETG.1000.4/.6 (as the LAN9253 implements them, DS00003421B 11.16), AL states and status codes, event bits, SyncManager/FMMU readers, PHY access through the MII management |
| SII | `ethercat/sii.hpp` | `make_sii<Words>(SiiConfig)`: the EEPROM image at compile time (ETG.2010) - configuration area with CRC, identity, mailbox, categories strings/general/FMMU/SyncManager |
| Object dictionary | `ethercat/object_dictionary.hpp` | `OdObject`/`OdEntry` tables: constants, strings, words of the application's `modbus::RegisterMap`, the PDO assignment; `standard_objects()` (1000h-1018h, 1C00h, 1C12h/1C13h), `join_objects()`, `valid_dictionary()` |
| CoE | `ethercat/coe.hpp` | `CoeServer`: SDO upload/download (expedited, normal), abort codes, SDO information (OD list, object/entry description, fragments), PDO assignment writable in PRE-OP; pure (bytes in, bytes out) |
| Slave | `ethercat/slave.hpp` | `Slave<Esc, App, Words>::poll(ready)`: EEPROM emulation (also before the ESC is ready; the master's writes kept in RAM), AL state machine, mailbox (SM0/SM1, repeat request, duplicates, mailbox errors), `read_diagnostics()` |
| LAN9253 | `devices/lan9253.hpp` | `Lan9253<Bus>`: SPI "EtherCAT Direct Mapped" (PDI 0x82) with the data sheet's worst-case timing, `probe()`, `ready()`, IRQ setup, the internal PHYs (100BASE-TX FD autonegotiation with Auto-MDIX, state) |

## Driving it

One thread calls `poll()` - after the ESC's IRQ (open drain, active low; EXTI) or periodically where the line
is missing: every event is in the AL event request register (0220h), polling only adds latency. Before the
ESC is ready (LAN9253 HW_CFG READY: the configuration area is loaded from the emulated EEPROM) only the EEPROM
emulation may touch the ESC: `poll(false)`.

The application (`SlaveApp`) is told about AL changes, refused requests and EEPROM commands - it logs, drives
LEDs and puts its outputs into the safe state; rtoos itself does not log.

## AL state machine (ETG.1000.6 6.4)

- Allowed: one step up (INIT -> PRE-OP -> SAFE-OP -> OP), any step down. Anything else: `0x0011` invalid
  state change; an unknown state `0x0012`; BOOT `0x0013` (no bootstrap).
- INIT -> PRE-OP: the mailbox SyncManagers (SM0 master -> slave, SM1 slave -> master) enabled, in mailbox
  mode, at the layout's address and size, else `0x0016`.
- PRE-OP -> SAFE-OP: SM2 (outputs) and SM3 (inputs) buffered, at the layout's address, with the
  sizes of the assigned PDOs - or disabled when there is no data; else `0x001D` / `0x001E`.
- A refused request sets the error flag (AL status bit 4) and the code; the slave ignores further requests
  until the master acknowledges (AL control bit 4).

## Mailbox and CoE

- SM0 (master -> slave) is read when its status says full - the whole buffer, since reading its last byte
  frees it; the response goes into SM1 when that is empty, again the whole buffer (writing its last byte
  marks it full). A response the master has not collected yet waits; a newer one replaces it (counted).
- The mailbox counter (1..7) of a request equal to the previous one is a retry: dropped. A repeat request
  (SM1 activate bit 1 toggled) gets the last response again and the repeat acknowledge.
- Any protocol but CoE gets a mailbox error (unsupported protocol); a length beyond the buffer one too.
- CoE: SDO expedited (up to 4 bytes) and normal transfers within one mailbox. Segmented transfers and
  complete access are not offered (aborted): the dictionary's largest value fits one mailbox. An abort is
  sent with the SDO request service (ETG.1000.6 5.6.2.7).
- Register entries go through the `RegisterMap` - the same values as Modbus and OPC UA, the application's
  write filter (refusal: abort 0800'0021h "local control") and write hook included. A 32-bit value is
  one UDINT/DINT (the map holds the high word first); an octet string the registers in order.
- The PDO assignment (1C12h/1C13h) is the default (every PDO of the dictionary) after INIT; in PRE-OP a
  master writes subindex 0 = 0, the PDO indexes, then the count. Its sizes are what PRE-OP -> SAFE-OP
  checks SM2/SM3 against.

## Process data

- Entering SAFE-OP the sizes come from the PDO assignment; the TxPDO image (inputs) goes into SM3 at
  once and then whenever the master has read the last one (AL event of SM3) - the master's cycle paces
  it. The entries are packed as over SDO (little endian, a 32-bit register value as one DINT/UDINT).
- In OP a new SM2 image (AL event of SM2) is applied entry by entry - only the entries whose bytes
  changed since the last image (the first image in OP: all). Outputs that another protocol (Modbus,
  OPC UA, a web UI) wrote meanwhile stay until the master changes its value. In SAFE-OP the images are
  read but not applied (ETG.1000.6: outputs in the safe state).
- Leaving OP - a master request, or the SyncManager watchdog (no SM2 image within the watchdog time
  0420h; the slave then goes to SAFE-OP with error 0x001B) - writes 0 into every mapped output entry.
- Writes go through the `RegisterMap`: a refused one (the application's write filter) is counted, not
  retried.
- Struct registers (octet strings) are SDO-only: no PDO entry.

## Tests

`tests/ethercat_test.cpp` against `tests/ethercat_sim.hpp` (the ESC as the PDI sees it, with the side effects
the slave relies on: AL control event set by the master's write and cleared by the PDI's read, the EEPROM
command handshake, the mailbox full/empty semantics of SM0/SM1 and the repeat request, the 3-buffer
events of SM2/SM3 and the watchdog), the CoE server alone (SDO, aborts, information, assignment), process
data (inputs paced by the master, outputs only in OP and only when changed, the safe state when leaving
OP and on the watchdog), and the LAN9253 SPI framing against a recording bus.
