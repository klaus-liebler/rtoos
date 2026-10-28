// SPDX-License-Identifier: MIT
//
// STM32H563 I2C master (RM0481 chapter I2C; registers, bits, IRQ numbers: SVD v1.8).
//
//   using Bus = I2cMaster<I2cId::I2c4, {.kernel_hz = board::sysclk_hz, .cpu_hz = board::sysclk_hz,
//                                       .bus_hz = 100'000}>;
//   Bus::init();
//   std::array<std::byte, 2> v;
//   if (Bus::read_register(0x40, 0x02, v) == I2cResult::Ok) { ... }   // INA226 bus voltage
//
// Polled transfers with a timeout (DWT cycle counter, CycleCounter::enable() must have run):
// the calling thread busy-waits while the bytes move - a few hundred microseconds for a
// register read at 100 kHz. Transfers are at most 255 bytes per direction (no RELOAD).
//
// The configuration is a template argument: TIMINGR is computed at compile time from the
// kernel clock and the bus speed (i2c_timing(), the I2C-bus specification UM10204 limits) and
// checked there - prescaler and counters in range, SCL not faster than asked, not slower than
// 90 % of it. At run time init() writes constants.
// After a timeout or bus error the peripheral is reset; recover() also resets it in RCC.
#pragma once

#include <contracts>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

// ---- timing ---------------------------------------------------------------------------------

struct I2cTiming {
    std::uint32_t presc = 0, scldel = 0, sdadel = 0, sclh = 0, scll = 0;
    bool valid = false;

    [[nodiscard]] constexpr std::uint32_t timingr() const noexcept {
        return (presc << 28) | (scldel << 20) | (sdadel << 16) | (sclh << 8) | scll;
    }
};

namespace i2c_detail {
struct Mode {
    std::uint32_t low_ns, high_ns, setup_ns;  // tLOW, tHIGH, tSU;DAT minimum (UM10204 table 10)
};
[[nodiscard]] constexpr Mode mode(std::uint32_t bus_hz) noexcept {
    return bus_hz <= 100'000 ? Mode{4700, 4000, 250} : (bus_hz <= 400'000 ? Mode{1300, 600, 100} : Mode{500, 260, 50});
}
inline constexpr std::uint64_t rise_ps = 100'000;  // assumed SCL/SDA rise time
inline constexpr std::uint64_t filter_ps = 50'000;  // analog filter delay, minimum (datasheet)
}  // namespace i2c_detail

/// TIMINGR for `bus_hz` (<= 1 MHz) at kernel clock `kernel_hz`: the smallest prescaler whose
/// counters fit; SCL never faster than `bus_hz`, tLOW/tHIGH/tSU;DAT at least the minimum.
[[nodiscard]] constexpr I2cTiming i2c_timing(std::uint32_t kernel_hz, std::uint32_t bus_hz) noexcept {
    using namespace i2c_detail;
    const Mode m = mode(bus_hz);
    const std::uint64_t kernel_ps = 1'000'000'000'000ull / kernel_hz;
    const std::uint64_t period_ps = (1'000'000'000'000ull + bus_hz - 1) / bus_hz;
    // SCL edge synchronisation per half period: analog filter + 3 kernel clocks + rise time
    const std::uint64_t sync_ps = filter_ps + 3 * kernel_ps + rise_ps;
    for (std::uint32_t presc = 0; presc < 16; ++presc) {
        const std::uint64_t tp = kernel_ps * (presc + 1);
        const auto ticks = [tp](std::uint64_t ps) { return (ps + tp - 1) / tp; };
        const std::uint64_t total = ticks(period_ps > 2 * sync_ps ? period_ps - 2 * sync_ps : 0);
        std::uint64_t low = ticks(m.low_ns * 1000ull);
        std::uint64_t high = ticks(m.high_ns * 1000ull);
        if (low + high < total) {  // spread the rest: low gets the larger share
            const std::uint64_t rest = total - low - high;
            low += rest - rest / 2;
            high += rest / 2;
        }
        const std::uint64_t scldel = ticks(m.setup_ns * 1000ull + rise_ps);
        if (low > 256 || high > 256 || scldel > 16) {
            continue;
        }
        return {presc, static_cast<std::uint32_t>(scldel - 1), 0, static_cast<std::uint32_t>(high - 1),
                static_cast<std::uint32_t>(low - 1), true};
    }
    return {};
}

/// SCL frequency that `t` produces at `kernel_hz` (with the synchronisation delays above).
[[nodiscard]] constexpr std::uint32_t i2c_frequency(const I2cTiming& t, std::uint32_t kernel_hz) noexcept {
    using namespace i2c_detail;
    const std::uint64_t kernel_ps = 1'000'000'000'000ull / kernel_hz;
    const std::uint64_t tp = kernel_ps * (t.presc + 1);
    const std::uint64_t period = (t.scll + 1 + t.sclh + 1) * tp + 2 * (filter_ps + 3 * kernel_ps + rise_ps);
    return static_cast<std::uint32_t>(1'000'000'000'000ull / period);
}

// ---- master -----------------------------------------------------------------------------------

enum class I2cId : std::uint8_t { I2c1, I2c2, I2c3, I2c4 };

enum class I2cResult : std::uint8_t { Ok, Nack, BusError, ArbitrationLost, Timeout };

struct I2cConfig {
    std::uint32_t kernel_hz;          ///< kernel clock of the I2C (normally PCLK)
    std::uint32_t cpu_hz;             ///< core clock: scales the timeouts (DWT cycles)
    std::uint32_t bus_hz = 100'000;   ///< SCL: 100 kHz, 400 kHz or 1 MHz
};

template <I2cConfig C>
consteval I2cTiming checked_i2c_timing() {
    static_assert(C.bus_hz >= 10'000 && C.bus_hz <= 1'000'000, "I2C bus speed outside 10 kHz .. 1 MHz");
    constexpr I2cTiming t = i2c_timing(C.kernel_hz, C.bus_hz);
    static_assert(t.valid, "no I2C timing for this kernel clock and bus speed (prescaler or counters out of range)");
    static_assert(i2c_frequency(t, C.kernel_hz) <= C.bus_hz, "I2C timing faster than the bus speed");
    static_assert(i2c_frequency(t, C.kernel_hz) >= C.bus_hz / 10 * 9, "I2C timing below 90 % of the bus speed");
    return t;
}

namespace i2c_detail {
struct Info {
    std::uintptr_t offset;
    rcc::Gate gate;
    std::uint8_t irq_event, irq_error;
};
[[nodiscard]] constexpr Info info(I2cId id) noexcept {
    switch (id) {
        case I2cId::I2c1: return {0x0000'5400, rcc::gate::i2c1, 51, 52};
        case I2cId::I2c2: return {0x0000'5800, rcc::gate::i2c2, 53, 54};
        case I2cId::I2c3: return {0x0400'2800, rcc::gate::i2c3, 80, 81};
        case I2cId::I2c4: return {0x0400'2C00, rcc::gate::i2c4, 125, 126};
    }
    return {};
}
inline constexpr std::uint32_t cr1_pe = 1u << 0;
inline constexpr std::uint32_t cr2_rd_wrn = 1u << 10, cr2_start = 1u << 13, cr2_stop = 1u << 14,
                               cr2_autoend = 1u << 25;
inline constexpr std::uint32_t cr2_nbytes_pos = 16;
inline constexpr std::uint32_t isr_txe = 1u << 0, isr_txis = 1u << 1, isr_rxne = 1u << 2, isr_nackf = 1u << 4,
                               isr_stopf = 1u << 5, isr_tc = 1u << 6, isr_berr = 1u << 8, isr_arlo = 1u << 9,
                               isr_busy = 1u << 15;
inline constexpr std::uint32_t icr_all = (1u << 3) | (1u << 4) | (1u << 5) | (1u << 8) | (1u << 9) | (1u << 10) |
                                         (1u << 11) | (1u << 12) | (1u << 13);
}  // namespace i2c_detail

template <I2cId Id, I2cConfig Config>
class I2cMaster {
    static constexpr i2c_detail::Info info = i2c_detail::info(Id);
    static constexpr I2cTiming timing = checked_i2c_timing<Config>();
    static constexpr std::uint32_t cycles_per_us = Config.cpu_hz / 1'000'000;

public:
    static constexpr std::uint8_t irq_event = info.irq_event, irq_error = info.irq_error;

    /// Clock on, timing, analog filter on.
    static void init() noexcept {
        rcc::enable(info.gate);
        CR1::write(0);
        TIMINGR::write(timing.timingr());
        ICR::write(i2c_detail::icr_all);
        CR1::write(i2c_detail::cr1_pe);
    }

    /// Reset the peripheral in RCC and set it up again (a bus that hangs after an error).
    /// A slave holding SDA low needs nine SCL pulses on the pins - that is the board's job.
    static void recover() noexcept {
        rcc::reset(info.gate);
        init();
    }

    /// Write `tx`, then (repeated start) read `rx`; either may be empty. `address`: 7 bit.
    static I2cResult write_read(std::uint8_t address, std::span<const std::byte> tx, std::span<std::byte> rx,
                                std::uint32_t timeout_us = 10'000) noexcept {
        contract_assert(tx.size() <= 255 && rx.size() <= 255 && address < 0x80);
        using namespace i2c_detail;
        const Deadline deadline{timeout_us * cycles_per_us};
        if (!wait(deadline, [] { return (ISR::read() & isr_busy) == 0; })) {
            return fail(I2cResult::Timeout);
        }
        ICR::write(icr_all);
        const std::uint32_t sadd = static_cast<std::uint32_t>(address) << 1;
        if (!tx.empty() || rx.empty()) {  // rx and tx empty: address only (probe)
            CR2::write(sadd | (static_cast<std::uint32_t>(tx.size()) << cr2_nbytes_pos) |
                       (rx.empty() ? cr2_autoend : 0u) | cr2_start);
            for (const std::byte b : tx) {
                if (const I2cResult r = wait_flag(deadline, isr_txis); r != I2cResult::Ok) {
                    return r;
                }
                TXDR::write(static_cast<std::uint32_t>(b));
            }
            if (const I2cResult r = wait_flag(deadline, rx.empty() ? isr_stopf : isr_tc); r != I2cResult::Ok) {
                return r;
            }
        }
        if (!rx.empty()) {
            CR2::write(sadd | cr2_rd_wrn | (static_cast<std::uint32_t>(rx.size()) << cr2_nbytes_pos) | cr2_autoend |
                       cr2_start);
            for (std::byte& b : rx) {
                if (const I2cResult r = wait_flag(deadline, isr_rxne); r != I2cResult::Ok) {
                    return r;
                }
                b = static_cast<std::byte>(RXDR::read());
            }
            if (const I2cResult r = wait_flag(deadline, isr_stopf); r != I2cResult::Ok) {
                return r;
            }
        }
        ICR::write(icr_all);
        return I2cResult::Ok;
    }

    static I2cResult write(std::uint8_t address, std::span<const std::byte> tx,
                           std::uint32_t timeout_us = 10'000) noexcept {
        return write_read(address, tx, {}, timeout_us);
    }
    static I2cResult read(std::uint8_t address, std::span<std::byte> rx, std::uint32_t timeout_us = 10'000) noexcept {
        return write_read(address, {}, rx, timeout_us);
    }

    /// Does a device answer at `address`? (address + write, no data)
    static bool probe(std::uint8_t address, std::uint32_t timeout_us = 2'000) noexcept {
        return write_read(address, {}, {}, timeout_us) == I2cResult::Ok;
    }

    /// Register read: write the 8-bit register number, repeated start, read.
    static I2cResult read_register(std::uint8_t address, std::uint8_t reg, std::span<std::byte> rx,
                                   std::uint32_t timeout_us = 10'000) noexcept {
        const std::byte r{reg};
        return write_read(address, std::span(&r, 1), rx, timeout_us);
    }

    /// Register write: the 8-bit register number followed by `data` (at most 32 bytes).
    static I2cResult write_register(std::uint8_t address, std::uint8_t reg, std::span<const std::byte> data,
                                    std::uint32_t timeout_us = 10'000) noexcept {
        contract_assert(data.size() <= 32);
        std::byte frame[33];
        frame[0] = std::byte{reg};
        for (std::size_t i = 0; i < data.size(); ++i) {
            frame[1 + i] = data[i];
        }
        return write(address, std::span<const std::byte>(frame, data.size() + 1), timeout_us);
    }

    [[nodiscard]] static bool busy() noexcept { return (ISR::read() & i2c_detail::isr_busy) != 0; }

private:
    struct Deadline {
        std::uint32_t cycles;
        std::uint32_t start = cm33::CycleCounter::now();
        [[nodiscard]] bool expired() const noexcept { return cm33::CycleCounter::now() - start > cycles; }
    };

    template <class Done>
    static bool wait(const Deadline& d, Done done) noexcept {
        while (!done()) {
            if (d.expired()) {
                return false;
            }
        }
        return true;
    }

    /// Wait for `flag`; a NACK, bus error or lost arbitration ends the transfer.
    static I2cResult wait_flag(const Deadline& d, std::uint32_t flag) noexcept {
        using namespace i2c_detail;
        for (;;) {
            const std::uint32_t isr = ISR::read();
            if ((isr & isr_nackf) != 0) {
                // A NACK ends the transfer with an automatic STOP (AUTOEND) - or a software
                // STOP after a write that was to continue with a read.
                if ((isr & isr_stopf) == 0 && (CR2::read() & cr2_autoend) == 0) {
                    CR2::write(CR2::read() | cr2_stop);
                }
                (void)wait(d, [] { return (ISR::read() & isr_stopf) != 0; });
                ICR::write(icr_all);
                ISR::write(isr_txe);  // flush TXDR
                return I2cResult::Nack;
            }
            if ((isr & isr_berr) != 0) {
                return fail(I2cResult::BusError);
            }
            if ((isr & isr_arlo) != 0) {
                return fail(I2cResult::ArbitrationLost);
            }
            if ((isr & flag) != 0) {
                return I2cResult::Ok;
            }
            if (d.expired()) {
                return fail(I2cResult::Timeout);
            }
        }
    }

    /// Software reset (PE off/on): releases the lines and clears the state machine.
    static I2cResult fail(I2cResult r) noexcept {
        CR1::write(CR1::read() & ~i2c_detail::cr1_pe);
        while ((CR1::read() & i2c_detail::cr1_pe) != 0) {
        }
        ICR::write(i2c_detail::icr_all);
        CR1::write(CR1::read() | i2c_detail::cr1_pe);
        return r;
    }

    using CR1 = PReg<info.offset + 0x00>;
    using CR2 = PReg<info.offset + 0x04>;
    using TIMINGR = PReg<info.offset + 0x10>;
    using ISR = PReg<info.offset + 0x18>;
    using ICR = PReg<info.offset + 0x1C>;
    using RXDR = PReg<info.offset + 0x24>;
    using TXDR = PReg<info.offset + 0x28>;

};

}  // namespace rtos::port::stm32h5
