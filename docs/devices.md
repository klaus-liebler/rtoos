# External chips

[include/rtos/devices](../include/rtos/devices) has drivers for the chips around the FCU, ported from its firmware. They are templates over a bus: an I2C bus with a static `write_read()` (`stm32h5::I2cMaster<Id, Config>`), an SPI master, or a byte channel (`BufferedUart`) with a clock. Configuration values (shunt, current range, integration time, gain) are template arguments and become register values at compile time. Every driver is tested on the host against a simulated bus ([devices_test.cpp](../tests/devices_test.cpp)) and instantiated with the real buses in [drivers_check.cpp](../apps/drivers_check.cpp).

| Chip | Header | Bus | What it does |
|---|---|---|---|
| TI INA226 | [ina226.hpp](../include/rtos/devices/ina226.hpp) | I2C | bus voltage, shunt voltage, current, power; calibration computed and range-checked from shunt and maximum current |
| ST VL53L0X | [vl53l0x.hpp](../include/rtos/devices/vl53l0x.hpp) | I2C | single-shot distance, non-blocking (`start()`, `poll()`) |
| ams TCS34725 | [tcs34725.hpp](../include/rtos/devices/tcs34725.hpp) | I2C | clear/red/green/blue in one block read; ATIME from the integration time |
| Avia HX711 | [hx711.hpp](../include/rtos/devices/hx711.hpp) | SPI (MOSI clocks, MISO samples) | 24-bit load cell readings, moving average; the 8 clock bytes run from the TxFIFO without a pause (see below) |
| Trinamic TMC2209 | [tmc2209.hpp](../include/rtos/devices/tmc2209.hpp) | single-wire UART (with echo) | init (micro steps, StealthChop), currents, internal step generator, sensorless homing with StallGuard |
| Feetech STS servos | [feetech_sts.hpp](../include/rtos/devices/feetech_sts.hpp) | UART (1 Mbit/s) | sync write of positions, feedback, torque |
| WS2812 LEDs | [ws2812.hpp](../include/rtos/devices/ws2812.hpp) | timer + DMA | timing from the timer clock, bit slots for the compare register |

Drivers that wait for an answer (TMC2209, Feetech) block the calling thread for a few milliseconds: they belong in an I/O thread, never in the network thread.

## HX711 on an SPI master

The HX711 has no SPI. Its clock input PD_SCK hangs on MOSI, and every byte `0x55` gives four clock pulses. PD_SCK high for more than 60 µs powers the chip down; a reading in progress is lost and the chip comes back at channel A, gain 128, after 400 ms of settling. Two things keep PD_SCK low when it must be:

- between transfers: the last byte of every transfer is `0x00`, `init()` drives MOSI low once, and the SPI keeps its pins driven while disabled (`AFCNTR`), at the level of the last bit;
- during a transfer: `SpiMaster` fills the TxFIFO before it sets `CSTART`. A master whose TxFIFO runs empty mid-transfer suspends SCK with MOSI at the last bit, and every `0x55` ends with a 1: an interrupt of more than 60 µs between two TXDR writes used to power the chip down. All 8 bytes now sit in the FIFO before the first bit goes out (`Hx711` checks `Spi::fifo_frames >= 8` at compile time), so the transfer runs in hardware alone.

The Renode test `hx711_renode` ([apps/hx711](../apps/hx711/main.cpp)) checks both against a bit-timed SPI model and an HX711 model ([Testing](testing.md#hx711-test-renode)).
