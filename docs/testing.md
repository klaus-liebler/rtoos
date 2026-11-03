# Testing

rtoos is tested on three levels. Each level covers what the level below it cannot.

| Level | Where | Covers | Command |
|---|---|---|---|
| 1: host unit tests | GoogleTest on the PC | all logic: kernel (on a mock CPU port), IP stack (over simulated links), TLS, HTTPS, WebSocket, Modbus, OPC UA, USB (against a simulated host); fuzzers | `ctest --preset host` |
| 2: QEMU | own machine `fcu` | the kernel on the real CPU mechanisms: PendSV, lazy FPU, interrupt priorities and BASEPRI, SysTick, PSPLIM | `ctest --preset arm` (`selftest_qemu`) |
| 3: Renode | FCU platform with Ethernet, timers, SPI with an HX711 and an SD card, on a TAP network | everything that involves the CPU's peripherals: the same self-test, the end-to-end network test of the real firmware, the stepper ramp measured on the STEP pin, the SD card driver against a card image, the HX711 behind a bit-timed SPI, interrupt-driven I2C against an INA226, GPIO edge interrupts | `ctest --preset arm` (`selftest_renode`, `net_renode`, `stepper_renode`, `sdcard_renode`, `flashfs_renode`, `hx711_renode`, `i2c_renode`, `exti_renode`) |

Every change to the kernel, a port, the chip or the board must pass `ctest --preset host` and `ctest --preset arm`. `net_renode` reports *skipped* without the TAP adapter, and a skip does not count as a pass for network changes.

## Level 1: host unit tests

```
cmake --preset host && cmake --build --preset host && ctest --preset host
```

Always build through the preset: it puts the GCC DLLs on the PATH, and `cmake --build build/host` fails silently without them.

| Area | Tests |
|---|---|
| Kernel | [kernel_test.cpp](../tests/kernel_test.cpp), [sync_test.cpp](../tests/sync_test.cpp), [block_pool_test.cpp](../tests/block_pool_test.cpp) on `MockPort` ([mock_port.hpp](../tests/mock_port.hpp)); `KernelWrapTest` starts just below the 32-bit tick wrap |
| IP stack | [net_test.cpp](../tests/net_test.cpp), [net_services_test.cpp](../tests/net_services_test.cpp), [dhcp_dns_test.cpp](../tests/dhcp_dns_test.cpp), [dhcp_server_test.cpp](../tests/dhcp_server_test.cpp), [tcp_server_test.cpp](../tests/tcp_server_test.cpp): malformed headers, sequence number wrap, RFC 5961, zero-window probes, SYN flood, ARP spoofing, address conflicts; ideas from the lwIP, NetX Duo and FreeRTOS-Plus-TCP test suites |
| TLS, HTTPS, WebSocket | [tls_test.cpp](../tests/tls_test.cpp), [http_ws_test.cpp](../tests/http_ws_test.cpp), [https_fairness_test.cpp](../tests/https_fairness_test.cpp) (order on the wire under load) |
| Modbus | [modbus_test.cpp](../tests/modbus_test.cpp) |
| OPC UA | [opcua_test.cpp](../tests/opcua_test.cpp), `opcua_o6_*_test.cpp`, [opcua_codec_test.cpp](../tests/opcua_codec_test.cpp); 309 test cases ported from open62541 ([OPC UA](opcua.md#tests-from-open62541)) |
| USB | [usb_device_test.cpp](../tests/usb_device_test.cpp), [usb_cdc_test.cpp](../tests/usb_cdc_test.cpp), [usb_ncm_test.cpp](../tests/usb_ncm_test.cpp); cases from TinyUSB and the USBX regression tests |

Fixtures: [net_fixture.hpp](../tests/net_fixture.hpp) connects stacks and simulated peers, and checks after every network test that all packet buffers went back to the pool. [usb_fixture.hpp](../tests/usb_fixture.hpp) is a simulated USB host, [opcua_service_harness.hpp](../tests/opcua_service_harness.hpp) calls OPC UA services without TCP.

### Fuzzing

[fuzz_test.cpp](../tests/fuzz_test.cpp) and [usb_fuzz_test.cpp](../tests/usb_fuzz_test.cpp) are mutation fuzzers for the receive path with all services, and for the parsers (HTTP, WebSocket, DNS names, DHCP options, Modbus, OPC UA, USB SETUP, NTB). They run deterministically with a fixed seed: GCC has no libFuzzer, and MinGW has no AddressSanitizer. They find contract violations, crashes and UBSan errors, hangs, lost packet buffers, and a stack that no longer answers afterwards. They do not find reads past the end of a buffer.

A longer run:

```
$env:RTOS_FUZZ_ITERATIONS=2000000; $env:RTOS_FUZZ_SEED=7; build/host/tests/rtos_tests.exe --gtest_filter=*Fuzz*
```

## Levels 2 and 3: emulation

Both emulators model the FCU: Cortex-M33 with FPU, MPU and DWT cycle counter, flash, SRAM, RCC, PWR (voltage scaling), FLASH_ACR, GPIO and USART1–3 (the console is USART3). Cache registers read 0 ("not busy"), so the real drivers run unchanged. Renode also models the Ethernet controller with the PHY.

Both run at 250 MIPS, one instruction per clock at 250 MHz: Renode through `PerformanceInMips`, QEMU through `-icount shift=2,sleep=off` (set by `emulation/run.py`; without it, QEMU on Windows loses SysTick periods because the host clock is too coarse). SysTick and `DWT_CYCCNT` agree, but the values count instructions, not real clock cycles with flash wait states. Never quote emulator cycle counts as hardware performance.

### QEMU

QEMU with the machine `fcu` is built from source ([emulation/qemu](../emulation/qemu): sources, patch and build script) into `C:\opt\qemu-stm32h5`:

```
set MSYSTEM=UCRT64& C:\opt\msys64\usr\bin\bash -l emulation/qemu/build-qemu.sh
python emulation/run.py qemu build/arm/apps/fcu_selftest.elf
C:\opt\qemu-stm32h5\qemu-system-arm.exe -M fcu -icount shift=2,sleep=off -nographic -kernel build\arm\apps\fcu_demo.elf -trace "led_*"
```

Model changes go into `emulation/qemu/{hw,include}` and the build patch; rebuild incrementally in `build/qemu/src/build` with `make && make install`.

### Renode

The platform is [emulation/renode/fcu.repl](../emulation/renode/fcu.repl) (on top of `stm32h563.repl`), installed in `C:\opt\renode`:

```
python emulation/run.py renode build/arm/apps/fcu_selftest.elf --allow-unsupported "stack limit"
python emulation/run.py renode build/arm/apps/fcu_demo.elf --interactive
```

With the GUI: start `renode`, then `$bin=@build/arm/apps/fcu_demo.elf; include @emulation/renode/fcu.resc; showAnalyzer usart3; start`.

### Known emulator gaps

An emulator gap is fixed in the emulator model where possible. Otherwise the self-test reports the feature as `UNSUPPORTED`, and the runner allows exactly that feature for that emulator (`--allow-unsupported`). Tests are never weakened.

- Renode does not enforce PSPLIM. A thread stack overflow there corrupts memory silently, and stack painting misses frames that are not written densely. Size stacks from the frames.
- Neither emulator models the `USB_DRD_FS` controller: the driver is tested on the host against a register simulation ([usb_drd_fs_test.cpp](../tests/usb_drd_fs_test.cpp)).
- The flash interface is an own model, [STM32H5_FLASH.cs](../emulation/renode/STM32H5_FLASH.cs) (unlock keys, sector erase). The CPU programs by writing to flash memory, and Renode accepts such writes even without PG and more than once per erase. The "program once" rule is checked by the host tests' NOR simulation instead.
- Renode's own STM32 SDMMC model moves data only through the FIFO (no IDMA), and its SD card answers CMD8 with the card status instead of R7. The platform therefore uses an own model, [STM32H5_SDMMC.cs](../emulation/renode/STM32H5_SDMMC.cs) (compiled by `fcu.resc`): SDMMC v2 with IDMA and an SD card that follows the Simplified Specification (states, R7 echo, SDHC/SDSC addressing, CSD 2.0/1.0, a programming phase after writes). QEMU has no SDMMC.
- Renode's SPI models move whole frames without bus time. The platform uses an own model, [STM32H5_SPI.cs](../emulation/renode/STM32H5_SPI.cs): master, TSIZE/CSTART, FIFOs of 16 (SPI1-3) or 8 (SPI4) frames, each bit shifted in virtual time with MOSI, SCK and MISO as GPIOs, and a clock that stops while the TxFIFO is empty mid-transfer. RM0481 does not document the MOSI level while the clock stands or the SPI is disabled; the model keeps the last bit (ST community reports), and its MOSI starts high (the worst case for a device clocked on MOSI). QEMU has no SPI.
- Renode's STM32 EXTI models take the port selection from SYSCFG and have no separate rising/falling pending registers. The platform uses an own model, [STM32H5_EXTI.cs](../emulation/renode/STM32H5_EXTI.cs) (EXTICR port selection, RTSR/FTSR, RPR/FPR, SWIER, IMR; lines 0-15); every GPIO pin is wired to it. A test signal source, [PulseGenerator.cs](../emulation/renode/PulseGenerator.cs), passes an object through light barrier 1 (PC7) every 7 ms for 1 ms.
- Renode's I2C controllers move bytes without bus time. The platform uses an own model, [STM32H5_I2C.cs](../emulation/renode/STM32H5_I2C.cs): master, nine SCL periods per byte (SCL from TIMINGR), clock stretching until TXDR is written or RXDR read, NACK with the automatic STOP, the event and error interrupts; devices are Renode I2C peripherals, e.g. [INA226.cs](../emulation/renode/INA226.cs) (registers, calibration, conversion times, alert functions on the ALERT pin). No slave mode, RELOAD, SMBus or DMA.
- Renode's timer model has no compare outputs, DMA requests or one-pulse mode, and its period is ARR ticks instead of ARR + 1. The platform uses own models: [STM32H5_TIM.cs](../emulation/renode/STM32H5_TIM.cs) (up-counting, preload, one-pulse mode, compare outputs as GPIOs, update/compare DMA requests, exact ARR + 1) and [STM32H5_GPDMA.cs](../emulation/renode/STM32H5_GPDMA.cs) (linear blocks, hardware and software requests; no linked lists). QEMU has no timers.

### Network test (Renode over TAP)

One-time setup, as administrator:

```
powershell -ExecutionPolicy Bypass -File emulation\renode\setup-tap.ps1
```

The script installs the TAP driver (tap-windows6), creates the adapter `rtoos-tap` with 192.168.77.1/24, and adds a firewall rule for this adapter only. The test script serves DHCP, DNS and SNTP; the device gets 192.168.77.2.

```
ctest --preset arm -R net_renode -V
```

[emulation/net_test.py](../emulation/net_test.py) checks `fcu_net` in this order:

- DHCP lease (DISCOVER/OFFER/REQUEST/ACK, then the ARP check) and its renewal at T1
- DNS lookup of the SNTP server with a random source port
- ping (including a 1472-byte payload), mDNS and SNTP
- TLS handshake with certificate and host name checked against `certs/ca.crt`, strictly per RFC 5280 (`VERIFY_X509_STRICT`)
- the web app byte for byte, plus 404 and 406
- `/blob` with throughput over a keep-alive connection
- WebSocket upgrade, status, 100 echoes (1–511 bytes), LED command, rejection of foreign origins (403)
- a WebSocket despite 7 idle keep-alive connections (eviction), and closing a silent WebSocket client after ping interval plus pong timeout
- Modbus TCP read and write
- OPC UA with asyncua: None read-only, Sign and SignAndEncrypt with the client certificate, rejection of a self-signed client
- 5 parallel 100 KB downloads plus WebSocket echo, Modbus and OPC UA response times (buffer budget and fairness)
- no failed allocation in the wolfSSL pool under all of this load

### SD card test (Renode)

```
ctest --preset arm -R sdcard_renode -V
```

[emulation/sdcard_test.py](../emulation/sdcard_test.py) inserts an SDHC image (64 MB) and an SDSC image (32 MB) into SDMMC1 and runs [`fcu_sdcard`](../apps/sdcard/main.cpp): initialisation, one block and an eight-block run written, read back and compared, then a FAT file system with two files (one in a directory) written, remounted and read back. The test then checks the image file itself: the patterns at the blocks the firmware wrote, a valid FAT boot sector, and both files' contents.

### Stepper test (Renode)

```
ctest --preset arm -R stepper_renode -V
```

[emulation/stepper_test.py](../emulation/stepper_test.py) loads a probe LED on the STEP pin (TIM3_CH1 on PB4: the timer makes the edges, the GPDMA feeds ARR) and hooks a Renode Python handler to its state changes, which records the virtual time of every edge. From the edges of `fcu_stepper`'s four moves, under an interrupt load, it checks the counts, that every edge sits on a timer tick, each move's exact duration, and the sigmoid ramp: each step's rate, the fit against a linear ramp, where the acceleration peaks, cruise, the mirrored deceleration and the short triangle ([Motion](motion.md#tests)).

### HX711 test (Renode)

```
ctest --preset arm -R hx711_renode -V
```

[`fcu_hx711`](../apps/hx711/main.cpp) reads the HX711 model ([HX711.cs](../emulation/renode/HX711.cs): PD_SCK on SPI2_MOSI, DOUT on SPI2_MISO and on PC2, 10 Hz, settling time, gain selection by 25/26/27 pulses, power-down after 60 µs with PD_SCK high) through the driver with gain 64. The first reading is the gain-128 code (the chip's reset state), every later one half of it, so a chip that was reset in between shows. Phases: quiet readings, then 65 readings each with a 100 µs interrupt armed 1..1500 CPU cycles into the transfer, then quiet readings again. Without the TxFIFO fill before `CSTART` the reading with the interrupt 93 cycles in comes back as `0xFFEFFFFF`: the chip powered down after the first bits. A fourth phase reads by interrupts alone: DOUT's falling edge (EXTI2) starts the transfer, the SPI's end-of-transfer interrupt decodes it; one second must bring 10 right readings while the thread sleeps.

The model's counters can be read in the Renode monitor: `sysbus.gpioPortC.hx711 PowerDowns`, `TimingViolations`, `Readings`; `sysbus.spi2 Stalls`, `FramesTransferred`.

### I2C test (Renode)

```
ctest --preset arm -R i2c_renode -V
```

[`fcu_i2c`](../apps/i2c/main.cpp) runs the interrupt-driven `I2cMaster` (an `I2cWait` policy on a semaphore) against the INA226 model on I2C4: probe and NACK on an empty address, the INA226 driver's IDs, configuration and measurement (24000 mV, about 500 mA), a register written and read back. Then 20 register reads with the polled driver and 20 with the interrupt-driven one while a less urgent thread counts: busy-waiting holds it off completely (0), sleeping on the interrupt lets it run.

### EXTI test (Renode)

```
ctest --preset arm -R exti_renode -V
```

[`fcu_exti`](../apps/exti/main.cpp): light barrier 1 interrupted every 7 ms for 1 ms: EXTI on both edges counts every pulse (142 in one second) where sampling the pin every 50 ms - the I/O cycle - sees two. Then the INA226's conversion-ready ALERT on PC9 drives the reads: the thread sleeps until the interrupt, reads and acknowledges; one second brings 112 to 115 conversions (8.8 ms each), and without an acknowledge ALERT stays low (one interrupt only).

### Testing with a browser

1. Trust the CA once ([Security](security.md#trusting-the-ca-in-a-browser)).
2. Start the firmware in Renode together with the DHCP, DNS and SNTP servers: `python emulation/net_test.py --browser build/arm/apps/fcu_net.elf`
3. Open `https://fcu.local/`. The page shows the uptime, the SNTP time, an LED switch, the echo test and a live chart.

## On the board

Build with the `board` preset and flash `build/board/apps/<name>.elf` with the ST-LINK. The console is USART3 at 115200 8N1 on the STDC14 connector (ST-LINK virtual COM port). Run `fcu_selftest` first: on the hardware the stack limit test must pass as well.
