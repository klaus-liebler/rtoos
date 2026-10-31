# External chips

[include/rtos/devices](../include/rtos/devices) has drivers for the chips around the FCU, ported from its firmware. They are templates over a bus: an I2C bus with a static `write_read()` (`stm32h5::I2cMaster<Id, Config>`), an SPI master, or a byte channel (`BufferedUart`) with a clock. Configuration values (shunt, current range, integration time, gain) are template arguments and become register values at compile time. Every driver is tested on the host against a simulated bus ([devices_test.cpp](../tests/devices_test.cpp)) and instantiated with the real buses in [drivers_check.cpp](../apps/drivers_check.cpp).

| Chip | Header | Bus | What it does |
|---|---|---|---|
| TI INA226 | [ina226.hpp](../include/rtos/devices/ina226.hpp) | I2C | bus voltage, shunt voltage, current, power; calibration computed and range-checked from shunt and maximum current |
| ST VL53L0X | [vl53l0x.hpp](../include/rtos/devices/vl53l0x.hpp) | I2C | single-shot distance, non-blocking (`start()`, `poll()`) |
| ams TCS34725 | [tcs34725.hpp](../include/rtos/devices/tcs34725.hpp) | I2C | clear/red/green/blue in one block read; ATIME from the integration time |
| Avia HX711 | [hx711.hpp](../include/rtos/devices/hx711.hpp) | SPI (MOSI clocks, MISO samples) | 24-bit load cell readings, moving average |
| Trinamic TMC2209 | [tmc2209.hpp](../include/rtos/devices/tmc2209.hpp) | single-wire UART (with echo) | init (micro steps, StealthChop), currents, internal step generator, sensorless homing with StallGuard |
| Feetech STS servos | [feetech_sts.hpp](../include/rtos/devices/feetech_sts.hpp) | UART (1 Mbit/s) | sync write of positions, feedback, torque |
| WS2812 LEDs | [ws2812.hpp](../include/rtos/devices/ws2812.hpp) | timer + DMA | timing from the timer clock, bit slots for the compare register |

Drivers that wait for an answer (TMC2209, Feetech) block the calling thread for a few milliseconds: they belong in an I/O thread, never in the network thread.
