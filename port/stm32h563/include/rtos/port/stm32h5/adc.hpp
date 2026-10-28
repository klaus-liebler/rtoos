// SPDX-License-Identifier: MIT
//
// STM32H563 ADC1/ADC2, one regular channel (RM0481 chapter ADC; registers, bits, IRQs: SVD
// v1.8; regulator start-up 20 us and the 4 ADC cycles after calibration: ST's
// stm32h5xx_ll_adc.h, LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES).
//
//   constexpr AdcConfig pressure{.kernel_hz = board::clocks.hclk_hz(), .cpu_hz = board::sysclk_hz,
//                                .channel = 18, .sample = AdcSample::Cycles2_5};
//   using Pressure = Adc<AdcId::Adc1, pressure>;
//   Pressure::init();             // power up, calibrate, enable
//   Pressure::start_continuous(); // converts on and on; read() returns the latest result
//   std::uint16_t raw = Pressure::read();
//
// The configuration is a template argument: the clock prescaler (ADC_CCR.PRESC, asynchronous
// clock mode from the RCC kernel clock - reset selection: HCLK) is the smallest that keeps the
// ADC clock at or below max_adc_hz, chosen and checked at compile time. In continuous mode an
// unread result is overwritten (OVRMOD = 1): read() always sees the latest conversion.
#pragma once

#include <cstdint>

#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

enum class AdcId : std::uint8_t { Adc1, Adc2 };

/// Sampling time in ADC clock cycles (ADC_SMPRx.SMPn).
enum class AdcSample : std::uint8_t {
    Cycles2_5 = 0, Cycles6_5, Cycles12_5, Cycles24_5, Cycles47_5, Cycles92_5, Cycles247_5, Cycles640_5
};

struct AdcConfig {
    std::uint32_t kernel_hz;                  ///< ADC kernel clock (RCC_CCIPR5.ADCDACSEL; reset: HCLK)
    std::uint32_t cpu_hz;                     ///< core clock (timeouts, regulator start-up)
    std::uint8_t channel;                     ///< ADC_INPn, 0..19
    AdcSample sample = AdcSample::Cycles12_5;
    std::uint8_t resolution_bits = 12;        ///< 12, 10, 8 or 6
    std::uint32_t max_adc_hz = 40'000'000;    ///< ADC clock limit (the FCU's proven value)
};

namespace adc_detail {
inline constexpr std::uint32_t presc_div[12] = {1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256};
}

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

namespace adc_detail {
inline constexpr std::uintptr_t common = 0x0202'8300;  // ADCC
inline constexpr std::uint32_t isr_adrdy = 1u << 0, isr_eoc = 1u << 2, isr_ovr = 1u << 4;
inline constexpr std::uint32_t cr_aden = 1u << 0, cr_adstart = 1u << 2, cr_adstp = 1u << 4,
                               cr_advregen = 1u << 28, cr_adcal = 1u << 31;
inline constexpr std::uint32_t cfgr_ovrmod = 1u << 12, cfgr_cont = 1u << 13, cfgr_jqdis = 1u << 31;
inline constexpr std::uint32_t ccr_presc_pos = 18;  // CKMODE (17:16) = 0: asynchronous kernel clock
}  // namespace adc_detail

template <AdcId Id, AdcConfig Config>
class Adc {
    static constexpr std::uintptr_t offset = Id == AdcId::Adc1 ? 0x0202'8000 : 0x0202'8100;
    static_assert(Config.channel <= 19, "ADC channels 0..19");
    static_assert(Config.resolution_bits == 12 || Config.resolution_bits == 10 || Config.resolution_bits == 8 ||
                      Config.resolution_bits == 6,
                  "ADC resolution 12, 10, 8 or 6 bits");
    static_assert(adc_presc(Config) < 12, "kernel clock too fast for max_adc_hz even at / 256");
    static constexpr std::uint32_t res = (12u - Config.resolution_bits) / 2u;  // RES: 0 = 12 bit
    static constexpr std::uint32_t smp_shift = 3u * (Config.channel % 10u);
    static constexpr std::uint32_t cycles_per_us = Config.cpu_hz / 1'000'000;

public:
    static constexpr std::uint8_t irq = Id == AdcId::Adc1 ? 37 : 69;
    static constexpr std::uint32_t adc_hz = adc_clock_hz(Config);

    /// Power up (leave deep power-down, regulator), calibrate, enable. False on a timeout.
    static bool init() noexcept {
        using namespace adc_detail;
        rcc::enable(rcc::gate::adc12);
        CCR::write((CCR::read() & ~(0xFu << ccr_presc_pos | 3u << 16)) | (adc_presc(Config) << ccr_presc_pos));
        CR::write(0);  // DEEPPWD = 0
        CR::write(cr_advregen);
        spin_us(20);  // regulator start-up
        CR::write(cr_advregen | cr_adcal);
        if (!wait([] { return (CR::read() & cr_adcal) == 0; }, 10'000)) {
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
        SQR1::write(static_cast<std::uint32_t>(Config.channel) << 6);  // L = 0: one conversion
        CFGR::write(cfgr_jqdis | cfgr_ovrmod | (res << 3));
        return true;
    }

    /// Convert again and again; read() returns the latest result.
    static void start_continuous() noexcept {
        using namespace adc_detail;
        CFGR::write(cfgr_jqdis | cfgr_ovrmod | cfgr_cont | (res << 3));
        CR::write(cr_advregen | cr_aden | cr_adstart);
    }

    static void stop() noexcept {
        using namespace adc_detail;
        if ((CR::read() & cr_adstart) != 0) {
            CR::write(cr_advregen | cr_aden | cr_adstp);
            (void)wait([] { return (CR::read() & cr_adstart) == 0; }, 1'000);
        }
    }

    /// The latest conversion result (right aligned).
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

private:
    static void spin_us(std::uint32_t us) noexcept {
        const std::uint32_t start = cm33::CycleCounter::now();
        while (cm33::CycleCounter::now() - start < us * cycles_per_us) {
        }
    }
    template <class Done>
    static bool wait(Done done, std::uint32_t timeout_us) noexcept {
        const std::uint32_t start = cm33::CycleCounter::now();
        while (!done()) {
            if (cm33::CycleCounter::now() - start > timeout_us * cycles_per_us) {
                return false;
            }
        }
        return true;
    }

    using ISR = PReg<offset + 0x00>;
    using CR = PReg<offset + 0x08>;
    using CFGR = PReg<offset + 0x0C>;
    using SMPR1 = PReg<offset + 0x14>;
    using SMPR2 = PReg<offset + 0x18>;
    using SQR1 = PReg<offset + 0x30>;
    using DR = PReg<offset + 0x40>;
    using CCR = PReg<adc_detail::common + 0x08>;
};

}  // namespace rtos::port::stm32h5
