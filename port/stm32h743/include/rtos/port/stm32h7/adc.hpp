// SPDX-License-Identifier: MIT
//
// STM32H743 ADC (16-bit SAR, ADC1/2/3), one regular channel, and the CPU temperature sensor
// on it (RM0433 chapter ADC; registers, bits: SVD v2.4; sequence, delays, encodings and the
// temperature formula: ST's stm32h7xx_ll_adc.h / stm32h7xx_hal_adc.c - LL_ADC_RESOLUTION_16B,
// LL_ADC_SAMPLINGTIME_*, LL_ADC_DELAY_INTERNAL_REGUL_STAB_US, LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES,
// LL_ADC_DELAY_TEMPSENSOR_STAB_US, ADC_ConfigureBoostMode, __LL_ADC_CALC_TEMPERATURE).
//
//   using Pressure = Adc<AdcId::Adc1, {.cpu_hz = clocks.cpu_hz(), .channel = 18}>;
//   Pressure::init();             // power up, calibrate (offset + linearity), enable
//   Pressure::start_continuous(); // converts on and on; read() returns the latest result
//   std::uint16_t raw = Pressure::read();   // 0..65535
//
// The H7's ADC is not the H5/G4 IP (stm32/adc.hpp): 16 bit, channel preselection (PCSEL),
// boost mode, other sampling times. Kernel clock: per_ck = HSI 64 MHz (RCC_D3CCIPR.ADCSEL = 10;
// the reset selection pll2_p is not running), asynchronous mode, ADC_CCR.PRESC chosen at
// compile time for at most max_adc_hz. The default 6.25 MHz keeps the boost setting
// revision-independent: ST sets BOOST = 0 up to 20 MHz on silicon revision Y and, on revision V
// (which halves the clock internally), up to 6.25 MHz - at or below it BOOST = 0 is right on both.
// Resolution 16 bit only (its RES encoding, 000, is the same on both revisions).
#pragma once

#include <cstdint>
#include <optional>

#include "rtos/port/cortex_m/cycle_counter.hpp"
#include "rtos/port/stm32/rcc.hpp"
#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/rcc.hpp"

namespace rtos::port::stm32h7 {

enum class AdcId : std::uint8_t { Adc1, Adc2, Adc3 };

/// Sampling time in ADC clock cycles (ADC_SMPRx.SMPn, LL_ADC_SAMPLINGTIME_*).
enum class AdcSample : std::uint8_t {
    Cycles1_5 = 0, Cycles2_5, Cycles8_5, Cycles16_5, Cycles32_5, Cycles64_5, Cycles387_5, Cycles810_5
};

struct AdcConfig {
    std::uint32_t kernel_hz = hsi_hz;       ///< per_ck (the HSI)
    std::uint32_t cpu_hz;                   ///< core clock (timeouts, regulator start-up)
    std::uint8_t channel;                   ///< ADC_INPn, 0..19
    AdcSample sample = AdcSample::Cycles16_5;
    std::uint32_t max_adc_hz = 6'250'000;   ///< ADC clock limit (BOOST = 0 on every revision)
};

namespace adc_detail {
inline constexpr std::uint32_t presc_div[12] = {1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256};
inline constexpr std::uint32_t isr_adrdy = 1u << 0, isr_eoc = 1u << 2, isr_ovr = 1u << 4;
inline constexpr std::uint32_t cr_aden = 1u << 0, cr_adstart = 1u << 2, cr_adstp = 1u << 4, cr_adcallin = 1u << 16,
                               cr_advregen = 1u << 28, cr_adcal = 1u << 31;
inline constexpr std::uint32_t cfgr_ovrmod = 1u << 12, cfgr_cont = 1u << 13;  // RES = 000: 16 bit
inline constexpr std::uint32_t ccr_presc_pos = 18, ccr_vsenseen = 1u << 23;   // CKMODE = 00: asynchronous
/// RCC_D3CCIPR.ADCSEL (17:16) = 10: per_ck (ST's LL_RCC_ADC_CLKSOURCE_CLKP).
inline constexpr stm32::rcc::ClockSelection kernel_clock{{rcc::base + 0x058, 16, 2}, 2};

struct Hw {
    std::uintptr_t base, common;
    stm32::rcc::Gate gate;
    std::uint8_t irq;
};
[[nodiscard]] constexpr Hw hw(AdcId id) noexcept {
    switch (id) {
        case AdcId::Adc1: return {0x4002'2000, 0x4002'2300, rcc::gate::adc12, 18};
        case AdcId::Adc2: return {0x4002'2100, 0x4002'2300, rcc::gate::adc12, 18};
        case AdcId::Adc3: return {0x5802'6000, 0x5802'6300, rcc::gate::adc3, 127};
    }
    return {};
}
}  // namespace adc_detail

/// ADC_CCR.PRESC for `c`: smallest divider with kernel / divider <= max_adc_hz; 12 if none.
[[nodiscard]] constexpr std::uint32_t adc_presc(const AdcConfig& c) noexcept {
    for (std::uint32_t i = 0; i < 12; ++i) {
        if (c.kernel_hz / adc_detail::presc_div[i] <= c.max_adc_hz) {
            return i;
        }
    }
    return 12;
}

[[nodiscard]] constexpr std::uint32_t adc_clock_hz(const AdcConfig& c) noexcept {
    return c.kernel_hz / adc_detail::presc_div[adc_presc(c)];
}

template <AdcId Id, AdcConfig Config>
class Adc {
    static constexpr adc_detail::Hw hw = adc_detail::hw(Id);
    static_assert(Config.channel <= 19, "the ADC has channels 0..19");
    static_assert(adc_presc(Config) < 12, "kernel clock too fast for max_adc_hz even at / 256");
    static_assert(Config.max_adc_hz <= 6'250'000, "above 6.25 MHz the boost setting depends on the silicon revision");
    static constexpr std::uint32_t smp_shift = 3u * (Config.channel % 10u);
    static constexpr std::uint32_t cycles_per_us = Config.cpu_hz / 1'000'000;

public:
    static constexpr std::uint8_t irq = hw.irq;
    static constexpr std::uint32_t adc_hz = adc_clock_hz(Config);

    /// Power up (leave deep power-down, regulator), calibrate offset and linearity, enable.
    /// False on a timeout.
    static bool init() noexcept {
        using namespace adc_detail;
        stm32::rcc::select(kernel_clock);
        stm32::rcc::enable(hw.gate);
        CCR::write((CCR::read() & ~(0xFu << ccr_presc_pos | 3u << 16)) | (adc_presc(Config) << ccr_presc_pos));
        CR::write(0);  // DEEPPWD = 0, BOOST = 0
        CR::write(cr_advregen);
        spin_us(10);  // regulator start-up
        CR::write(cr_advregen | cr_adcallin | cr_adcal);  // single-ended, offset + linearity
        if (!wait([] { return (CR::read() & cr_adcal) == 0; }, 100'000)) {
            return false;
        }
        spin_us(4 * 1'000'000 / adc_hz + 1);  // 4 ADC cycles before ADEN
        ISR::write(isr_adrdy);
        CR::write(cr_advregen | cr_aden);
        if (!wait([] { return (ISR::read() & isr_adrdy) != 0; }, 10'000)) {
            return false;
        }
        constexpr std::uint32_t smpr = static_cast<std::uint32_t>(Config.sample) << smp_shift;
        if constexpr (Config.channel < 10) {
            SMPR1::write((SMPR1::read() & ~(7u << smp_shift)) | smpr);
        } else {
            SMPR2::write((SMPR2::read() & ~(7u << smp_shift)) | smpr);
        }
        PCSEL::write(PCSEL::read() | (1u << Config.channel));
        SQR1::write(static_cast<std::uint32_t>(Config.channel) << 6);  // L = 0: one conversion
        CFGR::write(cfgr_ovrmod);
        return true;
    }

    /// Convert again and again; read() returns the latest result.
    static void start_continuous() noexcept {
        using namespace adc_detail;
        CFGR::write(cfgr_ovrmod | cfgr_cont);
        CR::write(cr_advregen | cr_aden | cr_adstart);
    }

    [[nodiscard]] static bool running() noexcept { return (CR::read() & adc_detail::cr_adstart) != 0; }
    [[nodiscard]] static bool enabled() noexcept {
        return (CR::read() & adc_detail::cr_aden) != 0 && (ISR::read() & adc_detail::isr_adrdy) != 0;
    }

    static void stop() noexcept {
        using namespace adc_detail;
        if (running()) {
            CR::write(cr_advregen | cr_aden | cr_adstp);
            (void)wait([] { return (CR::read() & cr_adstart) == 0; }, 1'000);
        }
    }

    /// The latest conversion result (16 bit, right aligned).
    [[nodiscard]] static std::uint16_t read() noexcept { return static_cast<std::uint16_t>(DR::read()); }

    /// One conversion, polled (not while continuous); nothing on a timeout.
    [[nodiscard]] static bool convert(std::uint16_t& out, std::uint32_t timeout_us = 1'000) noexcept {
        using namespace adc_detail;
        ISR::write(isr_eoc | isr_ovr);
        CR::write(cr_advregen | cr_aden | cr_adstart);
        if (!wait([] { return (ISR::read() & isr_eoc) != 0; }, timeout_us)) {
            return false;
        }
        out = read();
        return true;
    }

    /// Temperature sensor and VREFINT switches in the common block (ADC3: the sensor).
    static void enable_temperature_sensor() noexcept { CCR::write(CCR::read() | adc_detail::ccr_vsenseen); }

    static void spin_us(std::uint32_t us) noexcept {
        const std::uint32_t start = cortex_m::CycleCounter::now();
        while (cortex_m::CycleCounter::now() - start < us * cycles_per_us) {
        }
    }

private:
    template <class Done>
    static bool wait(Done done, std::uint32_t timeout_us) noexcept {
        const std::uint32_t start = cortex_m::CycleCounter::now();
        while (!done()) {
            if (cortex_m::CycleCounter::now() - start > timeout_us * cycles_per_us) {
                return false;
            }
        }
        return true;
    }

    using ISR = Reg<hw.base + 0x00>;
    using CR = Reg<hw.base + 0x08>;
    using CFGR = Reg<hw.base + 0x0C>;
    using SMPR1 = Reg<hw.base + 0x14>;
    using SMPR2 = Reg<hw.base + 0x18>;
    using PCSEL = Reg<hw.base + 0x1C>;
    using SQR1 = Reg<hw.base + 0x30>;
    using DR = Reg<hw.base + 0x40>;
    using CCR = Reg<hw.common + 0x08>;
};

// ---- CPU temperature ------------------------------------------------------------------------

/// Factory calibration of the temperature sensor (stm32h7xx_ll_adc.h, the H743's branch): raw
/// 16-bit values at VREF+ = 3.3 V, TS_CAL1 at 30 degC, TS_CAL2 at 110 degC on silicon revision Y
/// (DBGMCU_IDCODE.REV_ID <= 0x1003), 130 degC on revision V.
inline constexpr std::uintptr_t ts_cal1_address = 0x1FF1'E820, ts_cal2_address = 0x1FF1'E840;
inline constexpr std::uintptr_t dbgmcu_idcode = 0x5C00'1000;
inline constexpr std::int32_t ts_cal1_celsius = 30;

/// Temperature in 1/100 degC from a raw 16-bit sample, the calibration values and TS_CAL2's
/// temperature (VREF+ = 3.3 V, as at calibration).
[[nodiscard]] constexpr std::optional<std::int32_t> temperature_centi_celsius(std::uint16_t raw, std::uint16_t cal1,
                                                                              std::uint16_t cal2,
                                                                              std::int32_t cal2_celsius) noexcept {
    if (cal2 <= cal1) {
        return std::nullopt;  // no calibration (e.g. an emulator's zero-filled system memory)
    }
    return ts_cal1_celsius * 100 +
           (static_cast<std::int32_t>(raw) - cal1) * (cal2_celsius - ts_cal1_celsius) * 100 / (cal2 - cal1);
}

/// The CPU temperature on ADC3 channel 18 (LL_ADC_CHANNEL_TEMPSENSOR), the interface of the
/// STM32H5's DTS driver:
///
///   using Temperature = CpuTemperature<{.cpu_hz = clocks.cpu_hz()}>;
///   Temperature::init();
///   if (Temperature::ready()) Temperature::start();  // measures continuously from then on
///   if (auto t = Temperature::centi_celsius()) …     // the latest measurement
struct TemperatureConfig {
    std::uint32_t cpu_hz;
};

template <TemperatureConfig Config>
class CpuTemperature {
    // 387.5 cycles at 5.3 MHz: 72 us sampling, far above the sensor's minimum
    using Sensor = Adc<AdcId::Adc3, {.cpu_hz = Config.cpu_hz, .channel = 18, .sample = AdcSample::Cycles387_5}>;

public:
    static void init() noexcept {
        if (Sensor::init()) {
            Sensor::enable_temperature_sensor();
            Sensor::spin_us(26);  // LL_ADC_DELAY_TEMPSENSOR_STAB_US
        }
    }
    [[nodiscard]] static bool ready() noexcept { return Sensor::enabled(); }
    /// Start continuous conversion (again: harmless).
    static void start() noexcept {
        if (!Sensor::running()) {
            Sensor::start_continuous();
        }
    }
    [[nodiscard]] static std::optional<std::int32_t> centi_celsius() noexcept {
        if (!Sensor::running()) {
            return std::nullopt;
        }
        const auto cal1 = *reinterpret_cast<const volatile std::uint16_t*>(ts_cal1_address);
        const auto cal2 = *reinterpret_cast<const volatile std::uint16_t*>(ts_cal2_address);
        const std::uint32_t rev_id = *reinterpret_cast<const volatile std::uint32_t*>(dbgmcu_idcode) >> 16;
        return temperature_centi_celsius(Sensor::read(), cal1, cal2, rev_id <= 0x1003 ? 110 : 130);
    }
};

}  // namespace rtos::port::stm32h7
