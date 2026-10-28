// SPDX-License-Identifier: MIT
#include "board/fcu.hpp"

#include <array>

#include "rtos/port/stm32h5/cache.hpp"
#include "rtos/port/stm32h5/flash.hpp"
#include "rtos/port/stm32h5/mpu.hpp"
#include "rtos/port/stm32h5/pwr.hpp"
#include "rtos/port/stm32h5/rcc.hpp"
#include "rtos/port/stm32h5/usart.hpp"

extern "C" std::uint32_t __dma_start;
extern "C" std::uint32_t __flash_used_end;

namespace board {

namespace h5 = rtos::port::stm32h5;
using Console = h5::Usart<h5::UsartId::Usart3>;


// LED off (active low), USART3 console (AF7)
constexpr std::array base_pins{
    h5::gpio::output(led_info_pin, /*initial_high*/ true),
    h5::gpio::alternate(console_tx_pin, 7),
    h5::gpio::alternate(console_rx_pin, 7, h5::gpio::Speed::Low, h5::gpio::Pull::Up),
};

// RMII (AF11, ST open pin data STM32H563VITx) and the PHY reset, held low
constexpr h5::gpio::Pin phy_reset{h5::gpio::GpioPort::A, 0};
constexpr std::array ethernet_pins{
    h5::gpio::alternate({h5::gpio::GpioPort::A, 1}, 11, h5::gpio::Speed::VeryHigh),   // REF_CLK
    h5::gpio::alternate({h5::gpio::GpioPort::A, 2}, 11, h5::gpio::Speed::VeryHigh),   // MDIO
    h5::gpio::alternate({h5::gpio::GpioPort::C, 1}, 11, h5::gpio::Speed::VeryHigh),   // MDC
    h5::gpio::alternate({h5::gpio::GpioPort::A, 7}, 11, h5::gpio::Speed::VeryHigh),   // CRS_DV
    h5::gpio::alternate({h5::gpio::GpioPort::C, 4}, 11, h5::gpio::Speed::VeryHigh),   // RXD0
    h5::gpio::alternate({h5::gpio::GpioPort::C, 5}, 11, h5::gpio::Speed::VeryHigh),   // RXD1
    h5::gpio::alternate({h5::gpio::GpioPort::A, 5}, 11, h5::gpio::Speed::VeryHigh),   // TX_EN
    h5::gpio::alternate({h5::gpio::GpioPort::C, 10}, 11, h5::gpio::Speed::VeryHigh),  // TXD0
    h5::gpio::alternate({h5::gpio::GpioPort::B, 15}, 11, h5::gpio::Speed::VeryHigh),  // TXD1
    h5::gpio::output(phy_reset, /*initial_high*/ false),
};

void init() noexcept {
    h5::rcc::configure_clocks<clocks>();

    h5::configure_mpu({.flash_start = h5::flash_base,
                       .flash_end = reinterpret_cast<std::uint32_t>(&__flash_used_end) - 1u,
                       .ram_start = h5::sram_base,
                       .ram_end = reinterpret_cast<std::uint32_t>(&__dma_start) - 1u,
                       .dma_start = reinterpret_cast<std::uint32_t>(&__dma_start),
                       .dma_end = h5::sram_base + h5::sram_bytes - 1u});
    h5::ICache::enable(h5::ICache::Ways::TwoWay);

    h5::gpio::configure<base_pins>();
    Console::configure<h5::UartConfig{.kernel_hz = clocks.pclk1_hz(), .baud = console_baud, .fifo = false}>();
}

namespace {
/// Busy wait of at least `cycles` CPU cycles (>= 1 cycle per iteration).
void spin(std::uint32_t cycles) noexcept {
    for (std::uint32_t i = 0; i < cycles; ++i) {
        __asm volatile("" ::: "memory");
    }
}
}  // namespace

void init_ethernet() noexcept {
    h5::gpio::configure<ethernet_pins>();
    spin(sysclk_hz / 10'000);  // >= 100 us reset pulse (LAN8742A)
    h5::gpio::write(phy_reset, true);
    spin(sysclk_hz / 500);  // >= 2 ms until the PHY answers on MDIO
}

rtos::net::MacAddress mac_address() noexcept {
    // FNV-1a over the 96-bit UID (RM0481: UID_BASE 0x08FFF800, stm32h563xx.h)
    const auto* uid = reinterpret_cast<const volatile std::uint8_t*>(0x08FF'F800);
    std::uint64_t hash = 0xCBF2'9CE4'8422'2325ull;
    for (int i = 0; i < 12; ++i) {
        hash = (hash ^ uid[i]) * 0x0000'0100'0000'01B3ull;
    }
    return rtos::net::MacAddress::from(0x02,  // locally administered, unicast
                                       static_cast<std::uint8_t>(hash >> 32), static_cast<std::uint8_t>(hash >> 24),
                                       static_cast<std::uint8_t>(hash >> 16), static_cast<std::uint8_t>(hash >> 8),
                                       static_cast<std::uint8_t>(hash));
}

void led_info(bool on) noexcept { h5::gpio::write(led_info_pin, !on); }  // active low

void led_info_toggle() noexcept { h5::gpio::toggle(led_info_pin); }

void print(std::string_view text) noexcept { Console::write(text); }

void print_dec(std::uint32_t value) noexcept {
    std::array<char, 10> digits{};
    std::size_t n = 0;
    do {
        digits[n++] = static_cast<char>('0' + value % 10);
        value /= 10;
    } while (value != 0);
    while (n != 0) {
        Console::put(digits[--n]);
    }
}

void print_hex(std::uint32_t value) noexcept {
    Console::write("0x");
    for (int shift = 28; shift >= 0; shift -= 4) {
        Console::put("0123456789ABCDEF"[(value >> shift) & 0xFu]);
    }
}

void flush() noexcept { Console::flush(); }

}  // namespace board
