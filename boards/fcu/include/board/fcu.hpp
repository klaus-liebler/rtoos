// SPDX-License-Identifier: MIT
//
// Board support: Factory Control Unit (factory_in_a_box/pcb_factory_control_unit).
//
//   MCU      STM32H563VIT6 (LQFP100, 2 MB flash, 640 KB SRAM), no HSE crystal
//   Clock    HSI 64 MHz -> PLL1 -> SYSCLK = HCLK = PCLKx = 250 MHz (VOS0, 5 wait states)
//   Console  USART3, PD8 = TX, PD9 = RX (AF7) -> STDC14 connector J2, ST-LINK VCP
//   LED      LED_INFO (D1) on PH1, active low (anode via 1 k to 3V3)
//   Ethernet LAN8742A, RMII, PHY address 0 (PHYAD0 open -> internal pull-down),
//            REFCLKO mode (nINTSEL pulled low by R24) -> 50 MHz on PA1;
//            pins (AF11, ST open pin data STM32H563VITx):
//              PA1 REF_CLK  PA2 MDIO  PC1 MDC  PA7 CRS_DV  PC4 RXD0  PC5 RXD1
//              PA5 TX_EN    PC10 TXD0 PB15 TXD1        PA0 ETH_RST (GPIO, low active)
#pragma once

#include <cstdint>
#include <string_view>

#include "rtos/net/address.hpp"
#include "rtos/port/stm32h5/gpio.hpp"

namespace board {

inline constexpr std::uint32_t sysclk_hz = 250'000'000;
inline constexpr std::uint32_t console_baud = 115'200;

inline constexpr rtos::port::stm32h5::gpio::Pin led_info_pin{rtos::port::stm32h5::gpio::GpioPort::H, 1};
inline constexpr rtos::port::stm32h5::gpio::Pin console_tx_pin{rtos::port::stm32h5::gpio::GpioPort::D, 8};
inline constexpr rtos::port::stm32h5::gpio::Pin console_rx_pin{rtos::port::stm32h5::gpio::GpioPort::D, 9};

/// Clock tree, MPU, ICACHE, LED and console. Call first in main().
void init() noexcept;

/// RMII pins (AF11) and a PHY reset pulse. Call before Ethernet::init().
void init_ethernet() noexcept;

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
