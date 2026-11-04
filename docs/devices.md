# External chips

[include/rtos/devices](../include/rtos/devices) has drivers for the chips around the FCU, ported from its firmware. They are templates over a bus: an I2C bus with a static `write_read()` (`stm32h5::I2cMaster<Id, Config>`, polled, or `I2cMaster<Id, Config, Wait>` interrupt-driven: the calling thread sleeps while the bytes move), an SPI master, or a byte channel (`BufferedUart`) with a clock. Configuration values (shunt, current range, integration time, gain) are template arguments and become register values at compile time. Every driver is tested on the host against a simulated bus ([devices_test.cpp](../tests/devices_test.cpp)) and instantiated with the real buses in [drivers_check.cpp](../apps/drivers_check.cpp).

| Chip | Header | Bus | What it does |
|---|---|---|---|
| TI INA226 | [ina226.hpp](../include/rtos/devices/ina226.hpp) | I2C | bus voltage, shunt voltage, current, power; calibration computed and range-checked from shunt and maximum current |
| ST VL53L0X | [vl53l0x.hpp](../include/rtos/devices/vl53l0x.hpp) | I2C | single-shot distance, non-blocking (`start()`, `poll()`) |
| ams TCS34725 | [tcs34725.hpp](../include/rtos/devices/tcs34725.hpp) | I2C | clear/red/green/blue in one block read; ATIME from the integration time |
| Avia HX711 | [hx711.hpp](../include/rtos/devices/hx711.hpp) | SPI (MOSI clocks, MISO samples) | 24-bit load cell readings, moving average; the 8 clock bytes run from the TxFIFO without a pause (see below) |
| Trinamic TMC2209 | [tmc2209.hpp](../include/rtos/devices/tmc2209.hpp) | single-wire UART (with echo) | init (micro steps, StealthChop), currents, internal step generator, sensorless homing with StallGuard4 (StealthChop, CoolStep off, as the datasheet requires): SG_RESULT polled over UART and/or the DIAG output by edge interrupt (stop ~1 ms after the stall; DIAG from a driver error, reset or charge pump undervoltage fails the homing); `velocity()` in full steps/s at the set microstep resolution |
| Feetech STS servos | [feetech_sts.hpp](../include/rtos/devices/feetech_sts.hpp) | UART (1 Mbit/s) | sync write of positions, feedback, torque |
| WS2812 LEDs | [ws2812.hpp](../include/rtos/devices/ws2812.hpp) | timer + DMA | timing from the timer clock, bit slots for the compare register |

Drivers that wait for an answer (TMC2209, Feetech) block the calling thread for a few milliseconds: they belong in an I/O thread, never in the network thread.

## HX711 on an SPI master

The HX711 has no SPI. Its clock input PD_SCK hangs on MOSI, and every byte `0x55` gives four clock pulses. PD_SCK high for more than 60 µs powers the chip down; a reading in progress is lost and the chip comes back at channel A, gain 128, after 400 ms of settling. Two things keep PD_SCK low when it must be:

- between transfers: the last byte of every transfer is `0x00`, `init()` drives MOSI low once, and the SPI keeps its pins driven while disabled (`AFCNTR`), at the level of the last bit;
- during a transfer: `SpiMaster` fills the TxFIFO before it sets `CSTART`. A master whose TxFIFO runs empty mid-transfer suspends SCK with MOSI at the last bit, and every `0x55` ends with a 1: an interrupt of more than 60 µs between two TXDR writes used to power the chip down. All 8 bytes now sit in the FIFO before the first bit goes out (`Hx711` checks `Spi::fifo_frames >= 8` at compile time), so the transfer runs in hardware alone.

Without polling at all: the DOUT falling edge (conversion ready) raises an EXTI interrupt (`stm32h5::ExtiLine`), whose handler calls `start_read()` - the 8 bytes into the TxFIFO, `CSTART`, return. The SPI's end-of-transfer interrupt calls `complete()`: the RxFIFO holds the 64 sampled bits, they are decoded and stored, the average is published (an atomic: `average()` is safe from any thread). Mask the DOUT line between the two, the data bits toggle DOUT.

The Renode test `hx711_renode` ([apps/hx711](../apps/hx711/main.cpp)) checks all of this against a bit-timed SPI model and an HX711 model ([Testing](testing.md#hx711-test-renode)).

## INA226: conversion ready on ALERT

`enable_conversion_ready_alert()` sets MASK/ENABLE.CNVR: ALERT (open drain, active low) goes low when a measurement is done and stays low until `acknowledge()` reads MASK/ENABLE. An EXTI interrupt on the ALERT pin tells the thread when `read()` has new values instead of reading on a fixed period. The default configuration (0x4327: 4 averages, 1.1 ms per conversion) gives a new measurement every 8.8 ms.
