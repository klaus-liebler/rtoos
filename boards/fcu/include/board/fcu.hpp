// SPDX-License-Identifier: MIT
//
// Board support: Factory Control Unit (factory_in_a_box/pcb_factory_control_unit).
//
//   MCU      STM32H563VIT6 (LQFP100, 2 MB flash, 640 KB SRAM), no HSE crystal
//   Clock    HSI 64 MHz -> PLL1 -> SYSCLK = HCLK = PCLK2/3 = 250 MHz (VOS0, 5 wait states),
//            PCLK1 = 125 MHz (timers 250 MHz), PLL1 Q = 100 MHz
//   Console  USART3, PD8 = TX, PD9 = RX (AF7) -> STDC14 connector J2, ST-LINK VCP
//   LED      LED_INFO (D1), active low (anode via 1 k to 3V3): PH0 (revision 2), PE15 (revision 1)
//   USB-PD   revision 1: UCPD1 sink on PB13/PB14 (the product starts it); revision 2: a CH224K
//            negotiates alone, UCPD1 unused - board_early_init() (from Reset_Handler) switches the
//            dead-battery pull-downs off, the CC/DB pins carry other signals there
//   Ethernet LAN8742A, RMII, PHY address 0 (PHYAD0 open -> internal pull-down),
//            REFCLKO mode (nINTSEL pulled low by R24) -> 50 MHz on PA1;
//            pins (AF11, ST open pin data STM32H563VITx):
//              PA1 REF_CLK  PA2 MDIO  PC1 MDC  PA7 CRS_DV  PC4 RXD0  PC5 RXD1
//              PA5 TX_EN    PC10 TXD0 PB15 TXD1        PA0 ETH_RST (GPIO, low active)
#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "rtos/net/address.hpp"
#include "rtos/port/stm32h5/clocks.hpp"
#include "rtos/port/stm32h5/gpio.hpp"
#include "rtos/port/stm32h5/usart.hpp"

namespace board {

/// HSI 64 MHz / 32 = 2 MHz reference, x 250 = 500 MHz VCO; P / 2 = 250 MHz SYSCLK = HCLK =
/// PCLK2/3; PCLK1 = 125 MHz (APB1 / 2: the DTS calibration needs PCLK1 below 127 MHz), the
/// APB1 timers still run at 250 MHz (2 x PCLK1); Q / 5 = 100 MHz, the kernel clock of SPI1-3
/// (reset selection) and FDCAN (RCC_CCIPR5 = pll1_q). Checked at compile time (clocks.hpp).
inline constexpr rtos::port::stm32h5::rcc::ClockTree clocks{.pll1 = {.m = 32, .n = 250, .p = 2, .q = 5},
                                                            .apb1_div = 2};
inline constexpr std::uint32_t sysclk_hz = clocks.sysclk_hz();
inline constexpr std::uint32_t pll1_q_hz = clocks.pll1_q_hz();
static_assert(sysclk_hz == 250'000'000 && pll1_q_hz == 100'000'000);
inline constexpr std::uint32_t console_baud = 115'200;

/// Hardware revision (CMake cache variable RTOS_FCU_REVISION): 1 = first boards, 2 = current PCB.
inline constexpr unsigned revision = RTOS_FCU_REVISION;
static_assert(revision == 1 || revision == 2);
/// The MCU's own USB-PD sink (UCPD1 on the USB-C connector's CC lines); false: a CH224K does it.
inline constexpr bool has_ucpd = revision == 1;

inline constexpr rtos::port::stm32h5::gpio::Pin led_info_pin =
    revision == 1 ? rtos::port::stm32h5::gpio::Pin{rtos::port::stm32h5::gpio::GpioPort::E, 15}
                  : rtos::port::stm32h5::gpio::Pin{rtos::port::stm32h5::gpio::GpioPort::H, 0};
inline constexpr rtos::port::stm32h5::gpio::Pin console_tx_pin{rtos::port::stm32h5::gpio::GpioPort::D, 8};
inline constexpr rtos::port::stm32h5::gpio::Pin console_rx_pin{rtos::port::stm32h5::gpio::GpioPort::D, 9};
/// SD card detect switch (J3 DET), revision 2 only; init_sdcard() pulls it up.
inline constexpr rtos::port::stm32h5::gpio::Pin sdcard_detect_pin{rtos::port::stm32h5::gpio::GpioPort::D, 4};

/// The board package's pins, public for the product's compile-time resource checks
/// (gpio::pins_distinct, rtos/distinct.hpp): every pin it configures, by function.
namespace pins {
namespace gpio = rtos::port::stm32h5::gpio;
/// LED off (active low), USART3 console (AF7): init()
inline constexpr std::array base{
    gpio::output(led_info_pin, /*initial_high*/ true),
    gpio::alternate(console_tx_pin, gpio::Signal::USART3_TX),
    gpio::alternate(console_rx_pin, gpio::Signal::USART3_RX, gpio::Speed::Low, gpio::Pull::Up),
};
/// RMII and the PHY reset, held low: init_ethernet()
inline constexpr gpio::Pin phy_reset{gpio::GpioPort::A, 0};
inline constexpr std::array ethernet{
    gpio::alternate({gpio::GpioPort::A, 1}, gpio::Signal::ETH_RMII_REF_CLK, gpio::Speed::VeryHigh),
    gpio::alternate({gpio::GpioPort::A, 2}, gpio::Signal::ETH_MDIO, gpio::Speed::VeryHigh),
    gpio::alternate({gpio::GpioPort::C, 1}, gpio::Signal::ETH_MDC, gpio::Speed::VeryHigh),
    gpio::alternate({gpio::GpioPort::A, 7}, gpio::Signal::ETH_RMII_CRS_DV, gpio::Speed::VeryHigh),
    gpio::alternate({gpio::GpioPort::C, 4}, gpio::Signal::ETH_RMII_RXD0, gpio::Speed::VeryHigh),
    gpio::alternate({gpio::GpioPort::C, 5}, gpio::Signal::ETH_RMII_RXD1, gpio::Speed::VeryHigh),
    gpio::alternate({gpio::GpioPort::A, 5}, gpio::Signal::ETH_RMII_TX_EN, gpio::Speed::VeryHigh),
    gpio::alternate({gpio::GpioPort::C, 10}, gpio::Signal::ETH_RMII_TXD0, gpio::Speed::VeryHigh),
    gpio::alternate({gpio::GpioPort::B, 15}, gpio::Signal::ETH_RMII_TXD1, gpio::Speed::VeryHigh),
    gpio::output(phy_reset, /*initial_high*/ false),
};
/// SD card: SDMMC1, 1-bit bus; CMD and D0 pulled up. Revision 2: card detect switch (J3 DET, no
/// external pull-up) on PD4, pulled up here - on revision 1 PD4 is the INA226 ALERT. init_sdcard()
inline constexpr auto sdcard = [] {
    constexpr std::array bus{
        gpio::alternate({gpio::GpioPort::A, 10}, gpio::Signal::SDMMC1_D0, gpio::Speed::High, gpio::Pull::Up),
        gpio::alternate({gpio::GpioPort::C, 12}, gpio::Signal::SDMMC1_CK, gpio::Speed::High),
        gpio::alternate({gpio::GpioPort::D, 2}, gpio::Signal::SDMMC1_CMD, gpio::Speed::High, gpio::Pull::Up),
    };
    if constexpr (revision == 1) {
        return bus;
    } else {
        return std::array{bus[0], bus[1], bus[2], gpio::input(sdcard_detect_pin, gpio::Pull::Up)};
    }
}();
static_assert(gpio::pins_distinct(base, ethernet, sdcard));
}  // namespace pins

/// The console UART (USART3): a product that lists its U(S)ARTs includes it.
using Console = rtos::port::stm32h5::Usart<rtos::port::stm32h5::UsartId::Usart3>;

/// Clock tree, MPU, ICACHE, LED and console. Call first in main(). (Without UCPD, revision 2,
/// the dead-battery pull-downs are already off: board_early_init(), called by Reset_Handler.)
void init() noexcept;

/// RMII pins (AF11) and a PHY reset pulse. Call before Ethernet::init().
void init_ethernet() noexcept;

/// SD card slot: SDMMC1 1-bit bus (D0 PA10, CK PC12, CMD PD2); revision 2 also the card detect
/// input (PD4, pull-up), revision 1 has none.
void init_sdcard() noexcept;

/// USB device: USB_DRD_FS (PA11/PA12 are its additional function: no pin setup); VDDUSB, HSI48 +
/// CRS and clock (board::UsbDevice). Call before UsbDevice::start().
void init_usb() noexcept;

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
