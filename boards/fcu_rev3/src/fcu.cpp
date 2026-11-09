// SPDX-License-Identifier: MIT
#include "board/fcu.hpp"
#include "rtos/port/stm32h7/usb_device.hpp"

#include <array>

#include "rtos/port/cm7/cache.hpp"
#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/mpu.hpp"
#include "rtos/port/stm32h7/usart.hpp"

namespace board {

namespace h7 = rtos::port::stm32h7;
namespace cache = rtos::port::cm7::cache;

namespace {
/// Busy wait of at least `cycles` CPU cycles (>= 1 cycle per iteration).
void spin(std::uint32_t cycles) noexcept {
    for (std::uint32_t i = 0; i < cycles; ++i) {
        __asm volatile("" ::: "memory");
    }
}
}  // namespace

void init() noexcept {
    // MPU first (it makes the DMA region non-cacheable), then the caches - the D-cache enable
    // invalidates every line; nothing written through it yet - then the clocks.
    h7::configure_mpu();
    cache::enable_icache();
    cache::enable_dcache();
    h7::rcc::configure_clocks<clocks>();
    h7::gpio::configure<pins::base>();
    Console::configure<h7::UartConfig{.kernel_hz = clocks.pclk1_hz(), .baud = console_baud, .fifo = false}>();
}

void init_ethernet() noexcept {
    h7::gpio::configure<pins::ethernet>();
    spin(sysclk_hz / 10'000);  // >= 100 us reset pulse (LAN8742A)
    h7::gpio::write(pins::phy_reset, true);
    spin(sysclk_hz / 500);  // >= 2 ms until the PHY answers on MDIO
}

void init_sdcard() noexcept { h7::gpio::configure<pins::sdcard>(); }

void init_usb() noexcept {
    h7::gpio::configure<pins::usb>();
    h7::UsbOtgHardware<clocks.hclk_hz(), clocks.cpu_hz()>::enable_clocks();
}

rtos::net::MacAddress mac_address() noexcept {
    // FNV-1a over the 96-bit UID (stm32h743xx.h UID_BASE 0x1FF1'E800)
    const auto* uid = reinterpret_cast<const volatile std::uint8_t*>(h7::uid_base);
    std::uint64_t hash = 0xCBF2'9CE4'8422'2325ull;
    for (int i = 0; i < 12; ++i) {
        hash = (hash ^ uid[i]) * 0x0000'0100'0000'01B3ull;
    }
    return rtos::net::MacAddress::from(0x02,  // locally administered, unicast
                                       static_cast<std::uint8_t>(hash >> 32), static_cast<std::uint8_t>(hash >> 24),
                                       static_cast<std::uint8_t>(hash >> 16), static_cast<std::uint8_t>(hash >> 8),
                                       static_cast<std::uint8_t>(hash));
}

void led_info(bool on) noexcept { h7::gpio::write(led_info_pin, !on); }  // active low

void led_info_toggle() noexcept { h7::gpio::toggle(led_info_pin); }

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
