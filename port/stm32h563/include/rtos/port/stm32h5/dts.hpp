// SPDX-License-Identifier: MIT
//
// STM32H563 digital temperature sensor DTS (RM0481 chapter DTS; registers, bits: SVD v1.8).
//
//   using Temperature = Dts<{.pclk_hz = board::clocks.pclk1_hz()}>;
//   Temperature::init();
//   Temperature::start();                         // a measurement takes a few hundred us
//   if (auto t = Temperature::centi_celsius()) …  // once ready()
//
// The sensor oscillates at a frequency FM(T) proportional to the temperature; in PCLK mode
// (REFCLK_SEL = 0) the DTS counts PCLK cycles during TS1_SMP_TIME periods of it:
//   FM(T) = PCLK * smp_time / TS1_MFREQ
//   T     = T0 + (FM(T) - FMT0) / RAMP_COEFF       (factory values per chip: T0VALR1, RAMPVALR;
//                                                    FMT0 in 0.1 kHz, RAMP_COEFF in Hz/degC)
// The calibration before each measurement needs PCLK / HSREF_CLK_DIV below 1 MHz (SVD field
// description) with HSREF_CLK_DIV at most 127: the configuration checks this at compile time
// - PCLK1 must stay below 127 MHz.
#pragma once

#include <cstdint>
#include <optional>

#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

struct DtsConfig {
    std::uint32_t pclk_hz;         ///< PCLK1 (the DTS's APB clock and reference clock)
    std::uint32_t smp_time = 7;    ///< periods of FM(T) per measurement, 1..15 (more = finer)
};

/// HSREF_CLK_DIV: the smallest divider that brings `pclk_hz` below 1 MHz.
[[nodiscard]] constexpr std::uint32_t dts_hsref_div(std::uint32_t pclk_hz) noexcept {
    return pclk_hz / 1'000'000 + 1;
}

/// Temperature in 1/100 degC from the counter and the factory values (registers as read).
[[nodiscard]] constexpr std::optional<std::int32_t> dts_centi_celsius(std::uint32_t pclk_hz, std::uint32_t smp_time,
                                                                      std::uint32_t mfreq, std::uint32_t t0valr1,
                                                                      std::uint32_t rampvalr) noexcept {
    const std::uint32_t ramp = rampvalr & 0xFFFFu;      // Hz per degC
    const std::uint32_t fmt0_hz = (t0valr1 & 0xFFFFu) * 100u;
    const std::uint32_t t0_code = (t0valr1 >> 16) & 3u;
    if (mfreq == 0 || ramp == 0 || t0_code > 1) {
        return std::nullopt;  // no measurement, or unprogrammed factory values
    }
    const std::int64_t fm_centi_hz = static_cast<std::int64_t>(pclk_hz) * (smp_time == 0 ? 1 : smp_time) * 100 / mfreq;
    const std::int64_t t0_centi = t0_code == 1 ? 13'000 : 3'000;  // TS1_T0: 0 = 30 degC, 1 = 130 degC
    return static_cast<std::int32_t>(t0_centi + (fm_centi_hz - static_cast<std::int64_t>(fmt0_hz) * 100) / ramp);
}

template <DtsConfig Config>
class Dts {
    static constexpr std::uint32_t div = dts_hsref_div(Config.pclk_hz);
    static_assert(div <= 127, "PCLK1 too fast for the DTS calibration (PCLK / 127 must stay below 1 MHz)");
    static_assert(Config.smp_time >= 1 && Config.smp_time <= 15, "smp_time 1..15");
    static constexpr std::uint32_t cfgr1 = (div << 24) | (Config.smp_time << 16);  // PCLK, software trigger
    static constexpr std::uint32_t cfgr1_ts1_en = 1u << 0, cfgr1_ts1_start = 1u << 4;
    static constexpr std::uint32_t sr_ts1_rdy = 1u << 15;

public:
    static void init() noexcept {
        rcc::enable(rcc::gate::dts);
        CFGR1::write(cfgr1);
        CFGR1::write(cfgr1 | cfgr1_ts1_en);
    }

    /// No measurement or calibration in progress.
    [[nodiscard]] static bool ready() noexcept { return (SR::read() & sr_ts1_rdy) != 0; }

    /// Start a measurement (when ready()).
    static void start() noexcept { CFGR1::write(cfgr1 | cfgr1_ts1_en | cfgr1_ts1_start); }

    /// The temperature of the last measurement in 1/100 degC; nothing while one runs.
    [[nodiscard]] static std::optional<std::int32_t> centi_celsius() noexcept {
        if (!ready()) {
            return std::nullopt;
        }
        return dts_centi_celsius(Config.pclk_hz, Config.smp_time, DR::read() & 0xFFFFu, T0VALR1::read(),
                                 RAMPVALR::read());
    }

private:
    static constexpr std::uintptr_t offset = 0x0000'8C00;
    using CFGR1 = PReg<offset + 0x00>;
    using T0VALR1 = PReg<offset + 0x08>;
    using RAMPVALR = PReg<offset + 0x10>;
    using DR = PReg<offset + 0x1C>;
    using SR = PReg<offset + 0x20>;
};

}  // namespace rtos::port::stm32h5
