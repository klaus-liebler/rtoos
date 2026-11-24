// SPDX-License-Identifier: MIT
//
// STM32F103 independent watchdog (RM0008 rev 21 chapter 19): once started it cannot be stopped;
// it resets the chip unless kick() comes in time. It runs from the LSI, which the datasheet
// allows anywhere between 30 and 60 kHz - so the timeout is a range:
//
//   using Watchdog = Iwdg<{.pclk1_hz = clocks.pclk1_hz(), .timeout_ms = 2000}>;
//   Watchdog::start();                         // the LSI starts with it
//   ... Watchdog::kick();                       // at least every timeout_ms
//
// The configuration is computed at compile time: prescaler and reload such that the watchdog
// fires no earlier than timeout_ms even with the fastest LSI (60 kHz); with the slowest (30 kHz)
// it fires after at most twice that (latest_ms).
//
// Errata ES096 2.7.1/2.7.2 "RVU/PVU flag not cleared at low APB clock frequency": PCLK1 must be
// at least twice the IWDG clock - checked at compile time. 2.7.3 (Stop mode): not used.
#pragma once

#include <cstdint>

#include "rtos/port/stm32f1/chip.hpp"

namespace rtos::port::stm32f1 {

struct IwdgConfig {
    std::uint32_t pclk1_hz;
    std::uint32_t timeout_ms;  ///< the earliest the watchdog may fire
};

struct IwdgRegisters {
    std::uint32_t pr = 0, rlr = 0;
    bool valid = false;
    /// Reset after this long without kick() at the slowest LSI.
    [[nodiscard]] constexpr std::uint32_t latest_ms() const noexcept {
        return static_cast<std::uint32_t>((4ull << pr) * (rlr + 1) * 1000 / lsi_min_hz);
    }
    [[nodiscard]] constexpr std::uint32_t earliest_ms() const noexcept {
        return static_cast<std::uint32_t>((4ull << pr) * (rlr + 1) * 1000 / lsi_max_hz);
    }
};

/// The smallest prescaler (/4 .. /256) whose 12-bit reload reaches `timeout_ms` at 60 kHz.
[[nodiscard]] constexpr IwdgRegisters iwdg_registers(std::uint32_t timeout_ms) noexcept {
    for (std::uint32_t pr = 0; pr <= 6; ++pr) {
        const std::uint64_t div = 4ull << pr;
        const std::uint64_t ticks = (static_cast<std::uint64_t>(timeout_ms) * lsi_max_hz + div * 1000 - 1) / (div * 1000);
        if (ticks >= 1 && ticks <= 4096) {
            return {pr, static_cast<std::uint32_t>(ticks - 1), true};
        }
    }
    return {};
}

template <IwdgConfig Config>
class Iwdg {
    static constexpr IwdgRegisters regs = iwdg_registers(Config.timeout_ms);
    static_assert(regs.valid, "IWDG timeout outside 0.07 ms .. 17 s (prescaler 256, reload 4095, LSI 60 kHz)");
    static_assert(Config.pclk1_hz >= 2 * lsi_max_hz, "PCLK1 below twice the LSI (ES096 2.7.1/2.7.2)");

    using KR = PReg<0x0000'3000 + 0x00>;
    using PR = PReg<0x0000'3000 + 0x04>;
    using RLR = PReg<0x0000'3000 + 0x08>;
    using SR = PReg<0x0000'3000 + 0x0C>;

public:
    static constexpr IwdgRegisters registers = regs;

    static void start() noexcept {
        KR::write(0xCCCC);  // start (and with it the LSI)
        KR::write(0x5555);  // PR and RLR writable
        PR::write(regs.pr);
        RLR::write(regs.rlr);
        for (std::uint32_t i = 0; i < 100'000 && SR::read() != 0; ++i) {  // PVU/RVU: values taken over
        }
        KR::write(0xAAAA);
    }

    static void kick() noexcept { KR::write(0xAAAA); }
};

}  // namespace rtos::port::stm32f1
