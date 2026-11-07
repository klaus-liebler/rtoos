// SPDX-License-Identifier: MIT
//
// STM32 general-purpose and advanced timers, the timer IP of the STM32H5 and STM32G4 (RM0481/
// RM0440 chapters TIM; the registers the driver uses are the same in both SVDs v1.8, except
// TIMx_DCR.DBSS, which only the H5 has: TimerHw::burst_source).
//
// The configuration is a template argument; prescaler (PSC) and auto-reload (ARR) are
// computed and checked at compile time:
//
//   // PWM: 20 kHz, at least 1000 steps of duty resolution
//   using Fan = stm32h5::Timer<TimerId::Tim4, {.kernel_hz = board::clocks.timer1_hz(), .update_hz = 20'000,
//                                             .min_steps = 1000}>;
//   Fan::init();  Fan::configure_pwm<3>();  Fan::start();  Fan::set_duty_permille<3>(400);
//
//   // Time base: 1 MHz ticks, the period set at run time (step pulses, timeouts)
//   using Steps = stm32h5::Timer<TimerId::Tim16, {.kernel_hz = board::clocks.timer2_hz(), .tick_hz = 1'000'000}>;
//   Steps::init();  Steps::set_period(250);  Steps::enable_update_irq();  Steps::start();
//
// Kernel clock: the chip's ClockTree::timer1_hz() for the APB1 timers, timer2_hz() for the APB2
// timers. Which timers count 32 bit: TimerHw::counter32 (H5: TIM2, TIM5; G4: TIM2).
#pragma once

#include <contracts>
#include <cstdint>

#include "rtos/port/cortex_m/registers.hpp"
#include "rtos/port/stm32/rcc.hpp"

namespace rtos::port::stm32 {

struct TimerConfig {
    std::uint32_t kernel_hz;      ///< timer kernel clock (ClockTree::timer1_hz() / timer2_hz())
    std::uint32_t tick_hz = 0;    ///< counter frequency (exact divisor of kernel_hz), or 0: from update_hz
    std::uint32_t update_hz = 0;  ///< period frequency (PWM frequency), or 0: period set at run time
    std::uint32_t min_steps = 0;  ///< with update_hz: at least this many ticks per period
    bool preload = true;          ///< ARR/CCR changes take effect at the next update (ARPE, OCxPE)
};

struct TimerRegisters {
    std::uint32_t psc = 0, arr = 0;
    bool valid = false;
};

/// One timer of a chip.
struct TimerHw {
    std::uintptr_t base;
    rcc::Gate gate;
    std::uint8_t irq;        ///< update (or global) interrupt
    std::uint8_t channels;   ///< capture/compare channels
    bool counter32;
    bool advanced;           ///< has BDTR (main output enable)
    bool burst_source;       ///< TIMx_DCR.DBSS selects the DMA burst event (STM32H5; not on the G4)
};

namespace timer_detail {
inline constexpr std::uint32_t cr1_cen = 1u << 0, cr1_opm = 1u << 3, cr1_arpe = 1u << 7;
inline constexpr std::uint32_t dier_uie = 1u << 0, dier_ude = 1u << 8;
inline constexpr std::uint32_t sr_uif = 1u << 0;
inline constexpr std::uint32_t egr_ug = 1u << 0;
inline constexpr std::uint32_t bdtr_moe = 1u << 15;
inline constexpr std::uint32_t ocm_pwm1 = 0b0110;  // OCxM[2:0]; OCxM[3] stays 0
inline constexpr std::uint32_t ocm_toggle = 0b0011;
}  // namespace timer_detail

/// PSC/ARR for `c` on a counter of `max_arr` (0xFFFF or 0xFFFF'FFFF).
[[nodiscard]] constexpr TimerRegisters timer_registers(const TimerConfig& c, std::uint32_t max_arr) noexcept {
    if (c.tick_hz != 0) {
        if (c.kernel_hz % c.tick_hz != 0 || c.kernel_hz / c.tick_hz > 0x10000) {
            return {};
        }
        const std::uint32_t psc = c.kernel_hz / c.tick_hz - 1;
        if (c.update_hz == 0) {
            return {psc, max_arr, true};
        }
        const std::uint64_t ticks = c.tick_hz / c.update_hz;
        if (ticks < 1 || ticks - 1 > max_arr || ticks < c.min_steps) {
            return {};
        }
        return {psc, static_cast<std::uint32_t>(ticks - 1), true};
    }
    if (c.update_hz == 0) {
        return {};
    }
    // smallest prescaler whose period fits: best resolution
    const std::uint64_t total = (static_cast<std::uint64_t>(c.kernel_hz) + c.update_hz / 2) / c.update_hz;
    for (std::uint64_t div = 1; div <= 0x10000; ++div) {
        const std::uint64_t ticks = (total + div / 2) / div;
        if (ticks >= 1 && ticks - 1 <= max_arr) {
            if (ticks < c.min_steps) {
                return {};
            }
            return {static_cast<std::uint32_t>(div - 1), static_cast<std::uint32_t>(ticks - 1), true};
        }
    }
    return {};
}

/// Update frequency that `r` produces at `kernel_hz` (in mHz, for the error check).
[[nodiscard]] constexpr std::uint64_t timer_update_mhz(const TimerRegisters& r, std::uint32_t kernel_hz) noexcept {
    return static_cast<std::uint64_t>(kernel_hz) * 1000 / ((r.psc + 1ull) * (r.arr + 1ull));
}

/// The update frequency of `r` is within 1 % of `c.update_hz` (or none was asked for).
[[nodiscard]] constexpr bool timer_update_within_1_percent(const TimerConfig& c, const TimerRegisters& r) noexcept {
    if (c.update_hz == 0) {
        return true;
    }
    const std::uint64_t actual = timer_update_mhz(r, c.kernel_hz) * 100;
    const std::uint64_t wanted = c.update_hz * 1000ull;
    return actual >= wanted * 99 && actual <= wanted * 101;
}

template <TimerHw Hw, TimerConfig Config>
class Timer {
    static constexpr std::uint32_t max_arr = Hw.counter32 ? 0xFFFF'FFFFu : 0xFFFFu;
    static constexpr TimerRegisters regs = timer_registers(Config, max_arr);
    static_assert(Config.tick_hz != 0 || Config.update_hz != 0, "give tick_hz, update_hz or both");
    static_assert(Config.tick_hz == 0 || Config.kernel_hz % Config.tick_hz == 0,
                  "tick_hz must divide the timer kernel clock exactly");
    static_assert(regs.valid, "no prescaler/auto-reload for this timer (range or min_steps)");
    static_assert(timer_update_within_1_percent(Config, regs), "update frequency more than 1 % off");

public:
    static constexpr std::uint8_t irq = Hw.irq;
    static constexpr std::uint32_t tick_hz = Config.kernel_hz / (regs.psc + 1);
    static constexpr std::uint32_t period_ticks = regs.arr + 1;  ///< as configured (update_hz)
    static constexpr std::uint32_t max_period = max_arr;          ///< ticks; set_period() limit

    /// Clock on, prescaler and period loaded (update event), counter stopped.
    static void init() noexcept {
        rcc::enable(Hw.gate);
        CR1::write(0);
        PSC::write(regs.psc);
        ARR::write(regs.arr);
        EGR::write(timer_detail::egr_ug);  // load PSC/ARR now
        SR::write(0);
        CR1::write(Config.preload ? timer_detail::cr1_arpe : 0u);
    }

    static void start() noexcept { CR1::write(CR1::read() | timer_detail::cr1_cen); }
    /// One-pulse mode: the counter stops at the next update event (CEN clears itself).
    static void set_one_pulse(bool on) noexcept {
        CR1::write(on ? CR1::read() | timer_detail::cr1_opm : CR1::read() & ~timer_detail::cr1_opm);
    }
    /// Update event by software (EGR.UG): CNT = 0, preloaded ARR/PSC/CCR active, UIF set (and a
    /// DMA request if enabled).
    static void generate_update() noexcept { EGR::write(timer_detail::egr_ug); }
    static void stop() noexcept { CR1::write(CR1::read() & ~timer_detail::cr1_cen); }
    [[nodiscard]] static bool running() noexcept { return (CR1::read() & timer_detail::cr1_cen) != 0; }

    /// Period in ticks (1 .. max_period + 1); with preload from the next update on.
    static void set_period(std::uint32_t ticks) noexcept {
        contract_assert(ticks >= 1 && ticks - 1 <= max_arr);
        ARR::write(ticks - 1);
    }
    [[nodiscard]] static std::uint32_t period() noexcept { return ARR::read() + 1; }
    [[nodiscard]] static std::uint32_t counter() noexcept { return CNT::read(); }
    static void set_counter(std::uint32_t value) noexcept { CNT::write(value); }

    // ---- update interrupt / DMA ----------------------------------------------------------
    static void enable_update_irq() noexcept { DIER::write(DIER::read() | timer_detail::dier_uie); }
    static void disable_update_irq() noexcept { DIER::write(DIER::read() & ~timer_detail::dier_uie); }
    /// In the ISR: true (and acknowledged) if an update caused it.
    [[nodiscard]] static bool take_update() noexcept {
        if ((SR::read() & timer_detail::sr_uif) == 0) {
            return false;
        }
        SR::write(~timer_detail::sr_uif);  // rc_w0: only UIF cleared
        return true;
    }
    static void enable_update_dma() noexcept { DIER::write(DIER::read() | timer_detail::dier_ude); }
    static void disable_update_dma() noexcept { DIER::write(DIER::read() & ~timer_detail::dier_ude); }

    // ---- PWM -------------------------------------------------------------------------------

    /// Channel `Ch` (1..4) as PWM mode 1 output: high while CNT < CCR (inverted if active_low).
    template <unsigned Ch>
    static void configure_pwm(bool active_low = false) noexcept {
        static_assert(Ch >= 1 && Ch <= Hw.channels, "timer has no such channel");
        constexpr unsigned shift = ((Ch - 1) % 2) * 8;
        constexpr std::uint32_t mode = (timer_detail::ocm_pwm1 << (shift + 4)) | (Config.preload ? 1u << (shift + 3) : 0u);
        constexpr std::uint32_t mask = (0xFFu << shift) | (1u << (16 + shift));
        if constexpr (Ch <= 2) {
            CCMR1::write((CCMR1::read() & ~mask) | mode);
        } else {
            CCMR2::write((CCMR2::read() & ~mask) | mode);
        }
        constexpr unsigned e = 4 * (Ch - 1);
        CCER::write((CCER::read() & ~(3u << e)) | (1u << e) | (active_low ? 1u << (e + 1) : 0u));
        if constexpr (Hw.advanced) {
            BDTR::write(BDTR::read() | timer_detail::bdtr_moe);
        }
    }

    template <unsigned Ch>
    static void set_compare(std::uint32_t value) noexcept {
        static_assert(Ch >= 1 && Ch <= Hw.channels, "timer has no such channel");
        CCR<Ch>::write(value);
    }

    /// Duty 0..1000 per mille of the current period. 1000 = always on: CCR = period would not
    /// fit a 16-bit CCR for a full 65536-tick period, so the top is clamped to ARR (one tick off).
    template <unsigned Ch>
    static void set_duty_permille(std::uint32_t permille) noexcept {
        const std::uint32_t arr = ARR::read();
        const std::uint64_t ccr = (static_cast<std::uint64_t>(arr) + 1) * (permille > 1000 ? 1000 : permille) / 1000;
        set_compare<Ch>(ccr > arr && arr == max_arr ? arr : static_cast<std::uint32_t>(ccr));
    }

    /// Channel `Ch` as output compare in toggle mode: the output changes at every CNT == CCR
    /// (one edge per match), CCR without preload.
    template <unsigned Ch>
    static void configure_toggle() noexcept {
        static_assert(Ch >= 1 && Ch <= Hw.channels, "timer has no such channel");
        constexpr unsigned shift = ((Ch - 1) % 2) * 8;
        constexpr std::uint32_t mode = timer_detail::ocm_toggle << (shift + 4);
        constexpr std::uint32_t mask = (0xFFu << shift) | (1u << (16 + shift));
        if constexpr (Ch <= 2) {
            CCMR1::write((CCMR1::read() & ~mask) | mode);
        } else {
            CCMR2::write((CCMR2::read() & ~mask) | mode);
        }
        constexpr unsigned e = 4 * (Ch - 1);
        CCER::write((CCER::read() & ~(3u << e)) | (1u << e));
        if constexpr (Hw.advanced) {
            BDTR::write(BDTR::read() | timer_detail::bdtr_moe);
        }
    }

    /// Channel `Ch` raises a DMA request at its compare event.
    template <unsigned Ch>
    static void enable_compare_dma() noexcept {
        static_assert(Ch >= 1 && Ch <= Hw.channels, "timer has no such channel");
        DIER::write(DIER::read() | (1u << (8 + Ch)));  // CCxDE
    }

    // ---- compare interrupt (deadlines on a free-running counter) ---------------------------
    // With the channel in its reset mode (output compare, frozen), CCxIF is set when CNT
    // reaches CCRx; set_compare<Ch>() moves the deadline.

    template <unsigned Ch>
    static void enable_compare_irq() noexcept {
        static_assert(Ch >= 1 && Ch <= Hw.channels, "timer has no such channel");
        DIER::write(DIER::read() | (1u << Ch));  // CCxIE
    }
    template <unsigned Ch>
    static void disable_compare_irq() noexcept {
        static_assert(Ch >= 1 && Ch <= Hw.channels, "timer has no such channel");
        DIER::write(DIER::read() & ~(1u << Ch));
    }
    /// In the ISR: true (and acknowledged) if channel `Ch` matched.
    template <unsigned Ch>
    [[nodiscard]] static bool take_compare() noexcept {
        static_assert(Ch >= 1 && Ch <= Hw.channels, "timer has no such channel");
        if ((SR::read() & (1u << Ch)) == 0) {
            return false;
        }
        SR::write(~(1u << Ch));  // rc_w0: only CCxIF cleared
        return true;
    }
    /// Raise the compare event of channel `Ch` by software (EGR.CCxG).
    template <unsigned Ch>
    static void trigger_compare() noexcept {
        static_assert(Ch >= 1 && Ch <= Hw.channels, "timer has no such channel");
        EGR::write(1u << Ch);
    }

    /// DMA burst: `length` registers from offset `base_offset` (bytes) through DMAR at the event
    /// `source` (1 update, 2..5 CC1..CC4; RM0481 TIMx_DCR.DBSS).
    static void configure_dma_burst(std::uint32_t base_offset, std::uint32_t length, std::uint32_t source) noexcept
        requires(Hw.burst_source)
    {
        contract_assert(length >= 1 && length <= 18 && base_offset % 4 == 0);
        DCR::write((base_offset / 4) | ((length - 1) << 8) | (source << 16));
    }

    /// DMA burst: `length` registers from offset `base_offset` (bytes) through DMAR, one burst
    /// per DMA request of the channel that serves DMAR (RM0440 TIMx_DCR: no DBSS on the G4).
    static void configure_dma_burst(std::uint32_t base_offset, std::uint32_t length) noexcept
        requires(!Hw.burst_source)
    {
        contract_assert(length >= 1 && length <= 18 && base_offset % 4 == 0);
        DCR::write((base_offset / 4) | ((length - 1) << 8));
    }

    /// Bus addresses for DMA transfers.
    template <unsigned Ch>
    [[nodiscard]] static constexpr std::uintptr_t compare_address() noexcept {
        return Hw.base + 0x34 + 4 * (Ch - 1);
    }
    [[nodiscard]] static constexpr std::uintptr_t dmar_address() noexcept { return Hw.base + 0x3E0; }
    /// ARR for DMA: with preload (ARPE) a write sets the period after the next update event.
    [[nodiscard]] static constexpr std::uintptr_t arr_address() noexcept { return Hw.base + 0x2C; }
    static constexpr bool preload = Config.preload;

private:
    using CR1 = cortex_m::Reg<Hw.base + 0x00>;
    using DIER = cortex_m::Reg<Hw.base + 0x0C>;
    using SR = cortex_m::Reg<Hw.base + 0x10>;
    using EGR = cortex_m::Reg<Hw.base + 0x14>;
    using CCMR1 = cortex_m::Reg<Hw.base + 0x18>;
    using CCMR2 = cortex_m::Reg<Hw.base + 0x1C>;
    using CCER = cortex_m::Reg<Hw.base + 0x20>;
    using CNT = cortex_m::Reg<Hw.base + 0x24>;
    using PSC = cortex_m::Reg<Hw.base + 0x28>;
    using ARR = cortex_m::Reg<Hw.base + 0x2C>;
    template <unsigned Ch>
    using CCR = cortex_m::Reg<Hw.base + 0x34 + 4 * (Ch - 1)>;
    using BDTR = cortex_m::Reg<Hw.base + 0x44>;
    using DCR = cortex_m::Reg<Hw.base + 0x3DC>;
};

/// A free-running 1 MHz clock on a 32-bit timer with one deadline (compare channel `Ch`):
/// arm(at) raises the timer interrupt at `at`, at once if `at` already passed. It satisfies
/// usb_pd::DeadlineTimer.
///
///   using Clock = stm32h5::MicrosecondTimer<TimerId::Tim2, board::clocks.timer1_hz()>;
///   Clock::init();
///   TIM2 IRQ: if (Clock::take()) { ... }
template <TimerHw Hw, std::uint32_t KernelHz, unsigned Ch = 1>
class MicrosecondTimer {
    using Tim = Timer<Hw, TimerConfig{.kernel_hz = KernelHz, .tick_hz = 1'000'000, .preload = false}>;
    static_assert(Tim::max_period == 0xFFFF'FFFFu, "needs a 32-bit timer (H5: TIM2, TIM5; G4: TIM2)");

public:
    static constexpr std::uint8_t irq = Tim::irq;

    static void init() noexcept {
        Tim::init();
        Tim::start();
    }
    [[nodiscard]] std::uint32_t now() const noexcept { return Tim::counter(); }
    void arm(std::uint32_t at) noexcept {
        Tim::template set_compare<Ch>(at);
        (void)Tim::template take_compare<Ch>();  // a stale match from an earlier wrap
        Tim::template enable_compare_irq<Ch>();
        if (static_cast<std::int32_t>(at - Tim::counter()) <= 0) {
            Tim::template trigger_compare<Ch>();  // passed while arming
        }
    }
    void disarm() noexcept { Tim::template disable_compare_irq<Ch>(); }
    /// In the ISR: true (and acknowledged) if the deadline caused it.
    [[nodiscard]] static bool take() noexcept { return Tim::template take_compare<Ch>(); }
};

}  // namespace rtos::port::stm32
