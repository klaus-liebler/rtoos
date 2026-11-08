// SPDX-License-Identifier: MIT
//
// Board support: Factory Control Unit revision 3 (factory_in_a_box/pcb_factory_control_unit,
// MCU U17), the same interface as the revision 1/2 package (boards/fcu, STM32H563).
//
//   MCU      STM32H743VIT6 (LQFP100, 2 MB flash, 1 MB RAM), HSE crystal Y3 25 MHz
//   Clock    HSE 25 MHz -> PLL1 -> CPU 400 MHz (VOS1), HCLK 200 MHz, PCLK1-4 100 MHz,
//            timers 200 MHz, PLL1 Q 100 MHz (SPI1-3, SDMMC, FDCAN); HSI48 for USB and RNG
//   Console  USART3, PD8 = TX, PD9 = RX -> STDC14 connector J2, ST-LINK VCP
//   LED      LED_INFO (D1), active low (cathode on the pin): PE1
//   USB-PD   a CH224Q (U1) negotiates; the H743 has no UCPD
//   Ethernet LAN8742A (U3), RMII, PHY address 0, REFCLKO mode (50 MHz on PA1):
//              PA1 REF_CLK  PA2 MDIO  PC1 MDC  PA7 CRS_DV  PC4 RXD0  PC5 RXD1
//              PB11 TX_EN   PB12 TXD0 PB13 TXD1           PA0 ETH_RST (GPIO, low active)
//   SD card  SDMMC1 1-bit: D0 PC8, CK PC12, CMD PD2 (10 k pull-ups on the board), DET PC15
// Pins from the revision 3 netlist (kicad-cli, 2026-11-08), functions checked against ST's pin
// database (stm32h7/pin_functions.hpp).
#pragma once

#include <cstdint>
#include <string_view>

#include "rtos/net/address.hpp"
#include "rtos/port/stm32h7/clocks.hpp"
#include "rtos/port/stm32h7/gpio.hpp"

namespace board {

/// HSE 25 MHz / 5 = 5 MHz reference, x 160 = 800 MHz VCO; P / 2 = 400 MHz CPU, Q / 8 = 100 MHz;
/// HCLK = CPU / 2, every APB / 2 (the clocks.hpp defaults). Checked at compile time.
inline constexpr rtos::port::stm32h7::rcc::ClockTree clocks{
    .pll1 = {.source = rtos::port::stm32h7::rcc::PllSource::Hse, .input_hz = 25'000'000, .m = 5, .n = 160, .p = 2,
             .q = 8},
    .hsi48 = true};
inline constexpr std::uint32_t sysclk_hz = clocks.cpu_hz();  ///< the core clock (SysTick, DWT)
inline constexpr std::uint32_t pll1_q_hz = clocks.pll1_q_hz();
static_assert(sysclk_hz == 400'000'000 && pll1_q_hz == 100'000'000 && clocks.hclk_hz() == 200'000'000);
inline constexpr std::uint32_t console_baud = 115'200;

/// Hardware revision: this package is revision 3 only (RTOS_FCU_REVISION).
inline constexpr unsigned revision = RTOS_FCU_REVISION;
static_assert(revision == 3, "boards/fcu_rev3 is the revision 3 board (STM32H743); revisions 1/2: boards/fcu");
/// The MCU's own USB-PD sink: none on the H743.
inline constexpr bool has_ucpd = false;

namespace gpio = rtos::port::stm32h7::gpio;
inline constexpr gpio::Pin led_info_pin{gpio::GpioPort::E, 1};
inline constexpr gpio::Pin console_tx_pin{gpio::GpioPort::D, 8};
inline constexpr gpio::Pin console_rx_pin{gpio::GpioPort::D, 9};
/// SD card detect switch (J3 DET, no external pull-up); init_sdcard() pulls it up.
inline constexpr gpio::Pin sdcard_detect_pin{gpio::GpioPort::C, 15};

/// MPU (DMA region non-cacheable), caches, clock tree, LED and console. Call first in main().
void init() noexcept;

/// RMII pins and a PHY reset pulse. Call before Ethernet::init().
void init_ethernet() noexcept;

/// SD card slot: SDMMC1 1-bit bus and the card detect input (pulled up).
void init_sdcard() noexcept;

/// Unique, locally administered MAC address derived from the 96-bit device UID.
[[nodiscard]] rtos::net::MacAddress mac_address() noexcept;

inline constexpr unsigned phy_address = 0;

void led_info(bool on) noexcept;
void led_info_toggle() noexcept;

/// Polled console output (safe from any context, including fault handlers).
void print(std::string_view text) noexcept;
void print_dec(std::uint32_t value) noexcept;
void print_hex(std::uint32_t value) noexcept;
void flush() noexcept;

}  // namespace board
