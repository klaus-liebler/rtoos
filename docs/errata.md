# Silicon errata

ST's errata sheets of the supported chips, checked against the drivers this repository has. An
erratum matters only where a driver uses the affected feature; peripherals without a driver here
(OCTOSPI/QUADSPI, FMC, VREFBUF, OPAMP, COMP, HRTIM, LTDC, CRYP/SAES/HASH, LPTIM, IWDG/WWDG,
RTC/TAMP, I3C, LPUART, SAI, CEC, DAC) are left out. Re-check this list when a driver is added or a
sheet gets a new revision.

| Chip | Sheet | Revision checked |
|---|---|---|
| STM32H562/563/573 | ES0565 | Rev 9 (November 2026) |
| STM32H742/743/750/753 | ES0392 | Rev 16 (October 2026) |
| STM32G431/441 | ES0431 | Rev 10 |
| STM32F101/102/103 x8/xB (medium density) | ES096 | Rev 15 (March 2022) |

The PDFs are not in the repository (ST's download pages).

## Worked around in the code

| Erratum | Chips | Where | What the driver does |
|---|---|---|---|
| USB: buffer description table update completes after CTR | H563 (ES0565 2.20.1) | `port/stm32/.../usb_device.hpp` | COUNT_RX holds 0x3FF while the RX buffer has no unread packet; the interrupt waits for the controller to overwrite it (polls that word only when it is still pending, bounded). No fixed delay per packet. Host test `UsbDrdFsTest.*TheControllerWroteIt` |
| OTG: unexpected transfer instead of a zero-length packet | H743 (ES0392 2.25.7) | `port/stm32/.../usb_otg_device.hpp` | ZLP: SNAK, 20 AHB cycles, EPENA, 15 AHB cycles, CNAK (the waits are reads of DIEPCTLn). Host test `UsbOtgFsTest.ZeroLengthPacketsFollowTheErratumSequence` |
| I2C: spurious bus error in controller mode | H563 2.14.2, H743 2.19.4, G431 2.11.x | `port/stm32/.../i2c.hpp` | BERR is cleared and the transfer goes on (polled and interrupt-driven); it ends by NACK, STOP, lost arbitration or timeout |
| I2C: wrong data sampling when tSU;DAT < one kernel clock | H563 2.14.1, H743 2.19.3, G431 2.11.1 | `port/stm32/.../i2c.hpp` | `static_assert`: kernel clock >= 4/10/20 MHz for standard/fast/fast-mode plus (`i2c_kernel_clock_ok`, host test) |
| SPI: truncation of the output signals after EOT | H563 2.18.2, H743 2.22.6 | `port/stm32/.../spi.hpp` | The SPI stays enabled after EOT and is disabled when the next transfer starts, at the earliest one SCK period after EOT (`spi_eot_guard_cycles`, host test) |
| SPI: master stall with CSTART within one SCK after EOT | H743 2.22.2 | `port/stm32/.../spi.hpp` | The same: SPE off/on between transfers (ST's first workaround), and never within one SCK period after EOT |
| ETH: frames dropped at RMII 10 Mbit/s (fake dribble + CRC error) | H563 2.22.10 | `port/stm32/.../ethernet.hpp` | At 10 Mbit/s MTLRXQOMR.FEP on; frames whose only errors are dribble + CRC are taken (`EthernetHw::rmii_10m_fake_dribble`, H563 only) |
| AXI SRAM read data corruption | H743 rev. Y (2.2.10) | `port/stm32h743/.../chip.hpp` `apply_errata()` | On REV_ID 0x1003: AXI_TARG7_FN_MOD.READ_ISS_OVERRIDE (as ST's SystemInit); called first in `board::init()` |
| SRAM write error after a reset at a critical instant | G431 (ES0431 2.2.6) | `port/cortex_m/src/startup.cpp`, `port/stm32g431/src/errata.cpp` | The reset vector reads SRAM1, SRAM2 and CCM SRAM before the first write (naked entry, no stack) |
| DMA: disable failure when GIF is cleared in the cycle of a transfer error | G431 (ES0431 2.3.1) | `port/stm32g431/.../dma.hpp` | The ISR clears TCIF/HTIF/TEIF only; GIF only with the channel disabled |
| I2C: analog filter locks BUSY, master mode cannot start | F103 (ES096 2.8.7) | `port/stm32f103/.../i2c.hpp` `recover_bus()` | ST's sequence: PE off, SCL/SDA as GPIO open-drain - both high, SDA low, SCL low, SCL high, SDA high, every level read back - alternate function again, SWRST, PE on. Before it up to nine SCL clocks while a slave holds SDA low. Run when a transfer finds BUSY set and after every failed transfer. Host test `F1I2c.*` (timing); the sequence itself runs in Renode only as far as the STM32F1_I2C model goes |
| I2C: misplaced STOP | F103 (ES096 2.8.4) | `port/stm32f103/.../i2c.hpp` | SB not set within the timeout after START, or BERR while START is set: the transfer fails and the recovery (with SWRST) runs |
| I2C: events managed before the current byte / wrong data read | F103 (ES096 2.8.1, 2.8.2) | `port/stm32f103/.../i2c.hpp` `receive()` | ST's workaround 2: PRIMASK masks interrupts in the critical windows of the 1-byte (EV6_3), 2-byte (EV6_1, BTF) and N-byte (BTF, N-2, STOP, N-1) closing sequences - a few microseconds |
| I2C: tSU;STA of a repeated start violated at 88-100 kHz | F103 (ES096 2.8.5) | `port/stm32f103/.../i2c.hpp` | `static_assert`: standard mode at most 88 kHz (write_read uses a repeated start); fast mode is unaffected |
| Flash read after WFI with prefetch and 2 wait states | F103 (ES096 2.2.1) | `port/stm32f103/.../clocks.hpp` | `configure_clocks()` keeps RCC_AHBENR.FLITFEN set: the kernel's idle thread sleeps with WFI |
| DMA: disable failure when GIF is cleared in the cycle of a transfer error | F103 (ES096 2.4.1) | `port/stm32f103/.../dma.hpp` | As on the G4: the ISR clears TCIF/HTIF/TEIF only; GIF only with the channel disabled. 2.4.2: only 32-bit register accesses |
| IWDG RVU/PVU not cleared at low APB clock | F103 (ES096 2.7.1, 2.7.2) | `port/stm32f103/.../iwdg.hpp` | `static_assert`: PCLK1 at least twice the fastest LSI (120 kHz) |
| Cortex-M4 776924: VDIV/VSQRT with very short ISRs | G431 (ES0431 2.1.2) | `port/cortex_m/.../core.hpp`, `port/cortex_m4` | No lazy FP stacking on the Cortex-M4 (LSPEN = 0): rtoos cannot know that every application ISR has more than two instructions. Cost: 17 words more stacked when an FP thread is interrupted |

## Checked: not applicable, or covered as the code is

- **STM32F103 (ES096)**: *bxCAN time-triggered mode* (2.11.1): TTCM stays 0. *USART1_RTS with CAN
  unclocked* (2.3.1), *USART1_TX with TIM1_CH2 PWM* (2.3.10): no RTS, the TIM1 driver does not
  touch CH2 unless a board configures it - a board with USART1 on PA9 must not enable a TIM1_CH2
  output without remapping TIM1. *I2C2 SMBA vs. PB12* (2.3.6): only with PB12 as alternate
  function output while I2C2 is clocked; boards use PB12 as GPIO. *I2C1 with SPI1/TIM3_CH2
  remapped* (2.3.7, 2.3.8): PB5 as alternate output with I2C1 clocked - no driver combination here.
  *SMBus NACK*, *NOSTRETCH target mode* (2.8.3, 2.8.6): master mode with clock stretching only.
  *Timer* errata (2.6.x: break/OCREF_CLR, TRGO, toggle at ARR, overcapture): features not used.
  *Debug registers read 0 by software* (2.2.2): the port never reads DBGMCU. *Compatibility issue
  with latest compiler releases* (2.2.9): revisions Z and B (2008) only - newer silicon (Y, X, 1,
  2, 3) runs GCC 16 code; a board with a Z/B part is not supported. Stop/Standby/WFE, RTC, LSE,
  ADC injection, USB, SPI/I2S errata: no driver uses those.

- **Cortex-M4 752770, interrupted loads to SP** (G431 2.1.1): only `LDR SP, ...`; the M4 port sets
  SP by `msr psp/msp`, the compiler never emits it.
- **Cortex-M4 838869, store immediate overlapping exception return** (G431 2.1.3, category B
  rare): needs a store with wait states still buffered at the exception return *and* a third
  interrupt in that window; accepted.
- **Cortex-M7 1259864, data corruption with a write-through data cache** (H743 2.1.1) and **store
  after cache invalidate without barrier** (2.1.5): the only write-through region the CPU writes is
  the flash data area (stm32h7/mpu.hpp R0); `flash_program.hpp` invalidates the D-cache lines of
  every written range and the I-cache afterwards, with DSB/ISB around (`cm7::cache::barrier()`).
  RAM regions are write-back or non-cacheable.
- **ARM Cortex-M7 837070, a BASEPRI raise takes effect one instruction late** (affects r0p0/r0p1
  only): the STM32H743 is r1p1 (ST's `stm32h743xx.h`, `__CM7_REV` 0x0101) - the kernel lock needs
  no `CPSID` around `MSR BASEPRI`, documented in `port/cortex_m7/.../port.hpp`
- **480 MHz not available / VOS0 on revision Y** (H743 2.2.21): the port clocks the H743 at VOS1,
  400 MHz, on every revision.
- **I2C: transmission stalled after the first byte** (H743 2.19.9, G431 2.11.5): only with an APB
  to I2C kernel clock ratio between 1.5 and 3; the boards clock every I2C from its own PCLK.
- **I2C target-mode errata** (SMBus timeout, own address match, OVR underrun): controller mode only.
- **SPI BSY may stay high when SPI is disabled** (G431 2.14.1): the G4 driver waits for BSY with the
  SPI still enabled (after the last RXNE) and disables it only to flush after a timeout.
- **USB_FS: CRC16 in the packet buffer** (G431 2.16.2): the driver never reads past the count.
  ESOF after resume signaling (2.16.1): no remote wake-up.
- **OTG_HS/FS other items** (H743 2.25.1-2.25.6, 2.25.8-2.25.9): analog drift (no workaround),
  revision-Y FIFO write interrupted by register accesses (the FIFO is written in the interrupt
  handler only), host mode / ULPI, buffer DMA mode (the driver is slave mode), OTGINT/CIDSCHG
  (never unmasked).
- **FDCAN** (H563 2.19.x, H743 2.24.x, G431 2.15.x): Tx FIFO inversion needs dedicated Tx buffers
  (TXBC = 0 here); edge filtering needs CCCR.EFBI (never set); DAR mode (H743 2.24.5) only with
  `auto_retransmission = false`, which no board sets - a board that does needs the TXBRP/TXBCF
  handling first.
- **ADC** injected / dual-mode errata: the driver uses neither.
- **USART** DMA, auto-baud, smartcard, ONEBIT: not used. *Data corruption due to a noisy receive
  line* (H563 2.16.1, H743 2.20.3): a glitch in the second half of the stop bit - hardware (filter
  on the line), no software workaround.
- **UCPD TXHRST on a write underflow** (H563 2.21.1, G431 2.17.1): the UCPD driver transmits by DMA
  (ST's workaround).
- **ETH receive watchdog** (H563 2.22.8): RWT/RWTU never written. **Rx/Tx DMA not recovering from a
  bus error** (H563 2.22.2/2.22.3: timestamping/TSO only; H743 2.26.x: revision Y only): a bus error
  needs a DMA access outside the DMA regions - a placement bug that RTOS_DMA_BUFFER and the MPU
  layout rule out; the driver deliberately does not hide one behind a silent controller reset.
- **SDMMC** (H743 2.8.1-2.8.5: revision Y only; 2.8.6 / H563 2.6.1: end bits not checked, no
  workaround, data still correct).
- **PLL1P unduly disabled by software** (H563 2.2.27): clocks are configured once at start-up.
- **RTC / backup domain** errata: no RTC driver yet - re-check when one is added (H743 2.2.32/2.2.33
  need a first-boot sequence).

## Accepted, with the reason

- **H563 2.2.33, CPU freeze on the first flash erase/program after power-on** (about 120 us): the
  FCU firmware's only flash writer is the PLC slot store (a program download); the steppers, USB and
  Ethernet run by DMA through such a pause. ST's workaround (vector table, handlers and the first
  flash operation in SRAM) is not worth it until a board needs less than 120 us of latency then.
