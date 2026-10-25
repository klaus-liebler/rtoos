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

// HSI 64 MHz / 32 = 2 MHz reference (PLL1RGE 2-4 MHz), x 250 = 500 MHz VCO, / 2.
constexpr h5::rcc::Pll1Config pll1{.m = 32, .n = 250, .p = 2};
static_assert(pll1.output_hz() == sysclk_hz);

void init() noexcept {
    // Raise the clock in the order RM0481 requires: voltage scale, flash wait states,
    // then the PLL switch-over.
    h5::rcc::use_hsi(h5::rcc::HsiDiv::Div1);
    h5::pwr::set_voltage_scale(h5::pwr::VoltageScale::Vos0);
    h5::flash::set_latency(h5::flash::vos0_wait_states(sysclk_hz), h5::flash::programming_delay(sysclk_hz));
    h5::rcc::use_pll1<pll1>();

    h5::configure_mpu({.flash_start = h5::flash_base,
                       .flash_end = reinterpret_cast<std::uint32_t>(&__flash_used_end) - 1u,
                       .ram_start = h5::sram_base,
                       .ram_end = reinterpret_cast<std::uint32_t>(&__dma_start) - 1u,
                       .dma_start = reinterpret_cast<std::uint32_t>(&__dma_start),
                       .dma_end = h5::sram_base + h5::sram_bytes - 1u});
    h5::ICache::enable(h5::ICache::Ways::TwoWay);

    led_info(false);
    h5::gpio::configure(led_info_pin, h5::gpio::Mode::Output);

    h5::gpio::configure_alternate(console_tx_pin, 7);
    h5::gpio::configure_alternate(console_rx_pin, 7, h5::gpio::Speed::Low, h5::gpio::Pull::Up);
    Console::init(sysclk_hz, console_baud);
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
    using h5::gpio::GpioPort;
    using h5::gpio::Pin;
    constexpr std::array<Pin, 9> rmii{{{GpioPort::A, 1}, {GpioPort::A, 2}, {GpioPort::C, 1},
                                       {GpioPort::A, 7}, {GpioPort::C, 4}, {GpioPort::C, 5},
                                       {GpioPort::A, 5}, {GpioPort::C, 10}, {GpioPort::B, 15}}};
    for (const Pin& pin : rmii) {
        h5::gpio::configure_alternate(pin, 11, h5::gpio::Speed::VeryHigh);
    }
    constexpr Pin phy_reset{GpioPort::A, 0};
    h5::gpio::write(phy_reset, false);
    h5::gpio::configure(phy_reset, h5::gpio::Mode::Output);
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
