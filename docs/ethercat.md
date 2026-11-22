# EtherCAT

An EtherCAT slave (SubDevice) stack for an ESC on a host interface (PDI): `include/rtos/ethercat/`, header-only,
no heap, testable on the host against a simulated ESC. The ESC hardware is behind the `EscPort` concept; the
first driver is the Microchip LAN9253 on SPI (`include/rtos/devices/lan9253.hpp`).

Status: EEPROM emulation, AL state machine with checks, diagnostics (step E1). Mailbox with CoE (E2) and
process data (E3) follow.

## Parts

| Part | File | What it does |
|---|---|---|
| ESC access | `ethercat/esc.hpp` | `EscPort` concept (read/write a run of bytes at an ESC address), the ESC registers of ETG.1000.4/.6 (as the LAN9253 implements them, DS00003421B 11.16), AL states and status codes, event bits, SyncManager/FMMU readers, PHY access through the MII management |
| SII | `ethercat/sii.hpp` | `make_sii<Words>(SiiConfig)`: the EEPROM image at compile time (ETG.2010) - configuration area with CRC, identity, mailbox, categories strings/general/FMMU/SyncManager |
| Slave | `ethercat/slave.hpp` | `Slave<Esc, App, Words>::poll(ready)`: EEPROM emulation (also before the ESC is ready; the master's writes kept in RAM), AL state machine, `read_diagnostics()` |
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
  application's PDO sizes - or disabled when there is no data; else `0x001D` / `0x001E`.
- A refused request sets the error flag (AL status bit 4) and the code; the slave ignores further requests
  until the master acknowledges (AL control bit 4).

## Tests

`tests/ethercat_test.cpp` against `tests/ethercat_sim.hpp` (the ESC as the PDI sees it, with the side effects
the slave relies on: AL control event set by the master's write and cleared by the PDI's read, the EEPROM
command handshake), and the LAN9253 SPI framing against a recording bus.
