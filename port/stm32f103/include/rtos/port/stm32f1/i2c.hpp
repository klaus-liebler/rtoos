// SPDX-License-Identifier: MIT
//
// STM32F103 I2C master (RM0008 rev 21 chapter 26). The F1 I2C is the first-generation IP: no
// TIMINGR and no automatic transfer length - software steps through every event (SB, ADDR,
// TXE/BTF, RXNE) - and a list of silicon errata. Same interface as the shared driver of the
// newer chips (port/stm32/i2c.hpp):
//
//   using Bus = I2cMaster<I2cId::I2c2, {.pclk1_hz = clocks.pclk1_hz(), .cpu_hz = clocks.sysclk_hz(),
//                                       .bus_hz = 50'000, .scl = {GpioPort::B, 10}, .sda = {GpioPort::B, 11}}>;
//   Bus::init();
//   if (Bus::write_register(0x40, 0x06, data) != I2cResult::Ok) { Bus::recover(); }
//
// Polled: the calling thread busy-waits while the bytes move (DWT cycle counter for the
// timeouts, CycleCounter::enable() must have run). One thread owns a bus.
//
// The configuration is a template argument: FREQ, CCR and TRISE are computed at compile time
// (RM0008 26.6.8/26.6.9: standard mode CCR = PCLK1 / (2 f_SCL) >= 4, fast mode duty 2:1
// CCR = PCLK1 / (3 f_SCL) >= 1; TRISE = 1000 ns resp. 300 ns maximum rise time + 1) and checked
// there - PCLK1 a whole number of MHz in range, SCL not faster than asked, not slower than 90 %
// of it. The SCL/SDA pins are part of the configuration and must carry this instance's signals
// (pin database), because the bus recovery drives them as GPIOs.
//
// Silicon errata (ES096 rev 15, STM32F103x8/B; the same items in ES0340 for the xC/D/E):
//   * 2.8.7 "I2C analog filter may provide wrong value, locking BUSY flag and preventing master
//     mode entry": after power-on or an ESD/noise event BUSY stays set although SCL and SDA are
//     high; neither SWRST nor a reset clears it. recover_bus() runs ST's 15-step sequence (the
//     pins as GPIO open-drain: both high, SDA low, SCL low, SCL high, SDA high - every level
//     read back - then alternate function, SWRST, PE) and, before it, clocks SCL up to nine
//     times while a slave holds SDA low (a transfer cut off mid-byte). Every transfer that finds
//     BUSY set, or ends with an error, calls it. Long, noisy cables to the slaves make this the
//     normal recovery path, not an exception.
//   * 2.8.4 "Wrong behavior of I2C peripheral in master mode after a misplaced Stop": a START
//     that does not raise SB within the timeout, or BERR while START is set, needs SWRST - the
//     transfer fails, the bus is recovered.
//   * 2.8.1 / 2.8.2 "Some software events must be managed before the current byte is being
//     transferred" / "Wrong data read into data register": in master receive the events EV6_1,
//     EV6_3 and the BTF sequence of the last bytes must not be delayed. ST's workaround 2: the
//     driver masks interrupts (PRIMASK) for these few instructions - also zero-latency ISRs,
//     for a few microseconds.
//   * 2.8.5 "Setup time for a repeated Start condition": violated in standard mode between 88
//     and 100 kHz. write_read() uses a repeated start: standard mode is limited to 88 kHz at
//     compile time (fast mode above 100 kHz is not affected).
//   * 2.8.3 / 2.8.6 (SMBus NACK, target mode NOSTRETCH): master mode only, clock stretching on.
#pragma once

#include <contracts>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/port/cortex_m/cycle_counter.hpp"
#include "rtos/port/cortex_m/registers.hpp"
#include "rtos/port/stm32/i2c.hpp"
#include "rtos/port/stm32f1/chip.hpp"
#include "rtos/port/stm32f1/gpio.hpp"
#include "rtos/port/stm32f1/rcc.hpp"

namespace rtos::port::stm32f1 {

using stm32::I2cResult;

enum class I2cId : std::uint8_t { I2c1, I2c2 };

struct I2cConfig {
    std::uint32_t pclk1_hz;           ///< the I2C's clock (PCLK1), a whole number of MHz
    std::uint32_t cpu_hz;             ///< core clock: scales the timeouts (DWT cycles)
    std::uint32_t bus_hz = 88'000;    ///< SCL; standard mode up to 88 kHz (erratum 2.8.5), fast up to 400 kHz
    gpio::Pin scl, sda;               ///< the pins (bus recovery drives them as GPIOs)
};

/// Register values of an I2cConfig.
struct I2cTiming {
    std::uint32_t freq = 0, ccr = 0, trise = 0;
    bool valid = false;

    /// SCL frequency these values produce (without rise times and clock stretching).
    [[nodiscard]] constexpr std::uint32_t scl_hz(std::uint32_t pclk1_hz) const noexcept {
        if (ccr == 0) return 0;
        const bool fast = (ccr & (1u << 15)) != 0;
        return pclk1_hz / ((fast ? 3u : 2u) * (ccr & 0xFFFu));
    }
};

[[nodiscard]] constexpr I2cTiming i2c_timing(std::uint32_t pclk1_hz, std::uint32_t bus_hz) noexcept {
    if (pclk1_hz % 1'000'000 != 0 || bus_hz == 0) {
        return {};
    }
    const std::uint32_t freq = pclk1_hz / 1'000'000;
    if (bus_hz <= 100'000) {
        if (freq < 2 || freq > 36) return {};
        const std::uint32_t ccr = (pclk1_hz + 2 * bus_hz - 1) / (2 * bus_hz);
        if (ccr < 4 || ccr > 0xFFF) return {};
        return {freq, ccr, freq + 1, true};
    }
    if (bus_hz > 400'000 || freq < 4 || freq > 36) return {};
    const std::uint32_t ccr = (pclk1_hz + 3 * bus_hz - 1) / (3 * bus_hz);
    if (ccr < 1 || ccr > 0xFFF) return {};
    return {freq, (1u << 15) | ccr, freq * 300 / 1000 + 1, true};
}

template <I2cConfig C>
consteval I2cTiming checked_i2c_timing() {
    static_assert(C.pclk1_hz % 1'000'000 == 0, "PCLK1 must be a whole number of MHz (I2C_CR2.FREQ)");
    static_assert(C.bus_hz <= 88'000 || C.bus_hz > 100'000,
                  "standard mode between 88 and 100 kHz violates tSU;STA of a repeated start (ES096 2.8.5)");
    constexpr I2cTiming t = i2c_timing(C.pclk1_hz, C.bus_hz);
    static_assert(t.valid, "no I2C timing for this PCLK1 and bus speed (FREQ 2..36 MHz, CCR range)");
    static_assert(t.scl_hz(C.pclk1_hz) <= C.bus_hz, "I2C timing faster than the bus speed");
    static_assert(t.scl_hz(C.pclk1_hz) >= C.bus_hz / 10 * 9, "I2C timing below 90 % of the bus speed");
    return t;
}

struct I2cHw {
    std::uintptr_t base;
    stm32::rcc::Gate gate;
    std::uint8_t irq_event, irq_error;
    gpio::Signal scl, sda;
};

[[nodiscard]] consteval I2cHw i2c_hw(I2cId id) {
    switch (id) {
        case I2cId::I2c1: return {periph_base + 0x0000'5400, rcc::gate::i2c1, 31, 32, gpio::Signal::I2C1_SCL, gpio::Signal::I2C1_SDA};
        case I2cId::I2c2: return {periph_base + 0x0000'5800, rcc::gate::i2c2, 33, 34, gpio::Signal::I2C2_SCL, gpio::Signal::I2C2_SDA};
    }
    return {};
}

/// What went wrong on a bus so far (counts; for diagnostics).
struct I2cStats {
    std::uint32_t nacks = 0;
    std::uint32_t errors = 0;          ///< bus error, lost arbitration, timeout
    std::uint32_t recoveries = 0;      ///< recover_bus() runs
    std::uint32_t failed_recoveries = 0;  ///< lines still not both high afterwards
};

namespace i2c_f1_detail {
inline constexpr std::uint32_t cr1_pe = 1u << 0, cr1_start = 1u << 8, cr1_stop = 1u << 9, cr1_ack = 1u << 10,
                               cr1_pos = 1u << 11, cr1_swrst = 1u << 15;
inline constexpr std::uint32_t sr1_sb = 1u << 0, sr1_addr = 1u << 1, sr1_btf = 1u << 2, sr1_rxne = 1u << 6,
                               sr1_txe = 1u << 7, sr1_berr = 1u << 8, sr1_arlo = 1u << 9, sr1_af = 1u << 10,
                               sr1_ovr = 1u << 11;
inline constexpr std::uint32_t sr2_busy = 1u << 1;
/// CRL/CRH nibble: GPIO output open-drain 2 MHz (CNF 01, MODE 10).
inline constexpr std::uint32_t nibble_gpio_od = 0b0110;

inline std::uint32_t mask_irq() noexcept {
    std::uint32_t primask;
    __asm volatile("mrs %0, primask \n cpsid i" : "=r"(primask)::"memory");
    return primask;
}
inline void restore_irq(std::uint32_t primask) noexcept { __asm volatile("msr primask, %0" ::"r"(primask) : "memory"); }
}  // namespace i2c_f1_detail

template <I2cId Id, I2cConfig Config>
class I2cMaster {
    static constexpr I2cHw hw = i2c_hw(Id);
    static constexpr I2cTiming timing = checked_i2c_timing<Config>();
    static constexpr std::uint32_t cycles_per_us = Config.cpu_hz / 1'000'000;
    static_assert(gpio::detail::find(Config.scl, hw.scl) != nullptr, "the SCL pin does not carry this I2C's SCL");
    static_assert(gpio::detail::find(Config.sda, hw.sda) != nullptr, "the SDA pin does not carry this I2C's SDA");
    static_assert(cycles_per_us != 0, "cpu_hz below 1 MHz");

public:
    static constexpr std::uintptr_t base = hw.base;  ///< the instance (resource checks: rtos/distinct.hpp)
    static constexpr std::uint8_t irq_event = hw.irq_event, irq_error = hw.irq_error;
    static constexpr I2cTiming register_values = timing;

    /// Clock on, timing, enable. The pins must be configured (alternate, open drain) before.
    static void init() noexcept {
        rcc::enable(hw.gate);
        CR1::write(i2c_f1_detail::cr1_swrst);
        CR1::write(0);
        load_registers();
    }

    /// Reset the peripheral in RCC and recover the bus (a bus that hangs after an error).
    static void recover() noexcept {
        rcc::reset(hw.gate);
        (void)recover_bus();
    }

    /// ES096 2.8.7: free the lines and the analog filters, reset the state machine. True if SCL
    /// and SDA are both high afterwards (a slave or the cable can still hold a line low).
    static bool recover_bus() noexcept {
        using namespace i2c_f1_detail;
        ++stats_.recoveries;
        CR1::write(CR1::read() & ~cr1_pe);  // 1. PE off
        const std::uint32_t scl_af = nibble_of(Config.scl), sda_af = nibble_of(Config.sda);
        gpio::write(Config.scl, true);  // 2. both GPIO open-drain, high
        gpio::write(Config.sda, true);
        gpio::set_nibble(Config.scl, nibble_gpio_od);
        gpio::set_nibble(Config.sda, nibble_gpio_od);
        bool ok = settle(Config.scl, true) && settle(Config.sda, true);  // 3.
        // A slave cut off mid-byte holds SDA low: up to nine clocks let it finish (UM10204 3.1.16).
        for (int i = 0; i < 9 && !gpio::read(Config.sda); ++i) {
            gpio::write(Config.scl, false);
            delay_us(half_period_us);
            gpio::write(Config.scl, true);
            (void)settle(Config.scl, true);
            delay_us(half_period_us);
            ok = gpio::read(Config.sda);
        }
        // 4.-11. SDA low (START), SCL low, SCL high, SDA high (STOP): every analog filter sees edges
        gpio::write(Config.sda, false);
        ok = settle(Config.sda, false) && ok;
        gpio::write(Config.scl, false);
        ok = settle(Config.scl, false) && ok;
        gpio::write(Config.scl, true);
        ok = settle(Config.scl, true) && ok;
        gpio::write(Config.sda, true);
        ok = settle(Config.sda, true) && ok;
        gpio::set_nibble(Config.scl, scl_af);  // 12. alternate function open-drain again
        gpio::set_nibble(Config.sda, sda_af);
        CR1::write(cr1_swrst);  // 13., 14. SWRST: clears BUSY now that the filters follow the lines
        CR1::write(0);
        load_registers();  // 15. PE on (SWRST cleared every register)
        if (!ok) {
            ++stats_.failed_recoveries;
        }
        return ok;
    }

    /// Write `tx`, then (repeated start) read `rx`; either may be empty, both empty = probe.
    /// `address`: 7 bit. A failed transfer recovers the bus before it returns.
    static I2cResult write_read(std::uint8_t address, std::span<const std::byte> tx, std::span<std::byte> rx,
                                std::uint32_t timeout_us = 10'000) noexcept {
        contract_assert(address < 0x80);
        using namespace i2c_f1_detail;
        const Deadline d{timeout_us * cycles_per_us};
        if ((SR2::read() & sr2_busy) != 0 && !recover_bus()) {  // erratum 2.8.7, or a stuck line
            ++stats_.errors;
            return I2cResult::BusError;
        }
        if (!tx.empty() || rx.empty()) {
            if (const I2cResult r = start(d, static_cast<std::uint8_t>(address << 1)); r != I2cResult::Ok) {
                return r;
            }
            (void)SR1::read();
            (void)SR2::read();  // EV6: ADDR cleared
            for (const std::byte b : tx) {
                if (const I2cResult r = wait_sr1(d, sr1_txe); r != I2cResult::Ok) {
                    return r;
                }
                DR::write(static_cast<std::uint32_t>(b));
            }
            if (!tx.empty()) {
                if (const I2cResult r = wait_sr1(d, sr1_btf); r != I2cResult::Ok) {
                    return r;
                }
            }
            if (rx.empty()) {
                return stop(d);
            }
        }
        return receive(d, address, rx);
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
    static I2cResult read_register(std::uint8_t address, std::uint8_t reg, std::span<std::byte> rx,
                                   std::uint32_t timeout_us = 10'000) noexcept {
        const std::byte r{reg};
        return write_read(address, std::span(&r, 1), rx, timeout_us);
    }
    /// Register write: the 8-bit register number followed by `data` (at most 64 bytes).
    static I2cResult write_register(std::uint8_t address, std::uint8_t reg, std::span<const std::byte> data,
                                    std::uint32_t timeout_us = 10'000) noexcept {
        contract_assert(data.size() <= 64);
        std::byte frame[65];
        frame[0] = std::byte{reg};
        for (std::size_t i = 0; i < data.size(); ++i) {
            frame[1 + i] = data[i];
        }
        return write(address, std::span<const std::byte>(frame, data.size() + 1), timeout_us);
    }

    [[nodiscard]] static bool busy() noexcept { return (SR2::read() & i2c_f1_detail::sr2_busy) != 0; }
    [[nodiscard]] static const I2cStats& stats() noexcept { return stats_; }

private:
    static constexpr std::uint32_t half_period_us = 10;  // bit-banged SCL during recovery: 50 kHz
    static inline I2cStats stats_{};

    struct Deadline {
        std::uint32_t cycles;
        std::uint32_t start = cortex_m::CycleCounter::now();
        [[nodiscard]] bool expired() const noexcept { return cortex_m::CycleCounter::now() - start > cycles; }
    };

    static void delay_us(std::uint32_t us) noexcept {
        const Deadline d{us * cycles_per_us};
        while (!d.expired()) {
        }
    }

    /// Wait (at most 1 ms: rise time of a long cable) until `pin` reads `level`.
    static bool settle(gpio::Pin pin, bool level) noexcept {
        const Deadline d{1000 * cycles_per_us};
        delay_us(half_period_us);
        while (gpio::read(pin) != level) {
            if (d.expired()) {
                return false;
            }
        }
        return true;
    }

    static std::uint32_t nibble_of(gpio::Pin pin) noexcept {
        const auto& r = *reinterpret_cast<volatile std::uint32_t*>(gpio::gpio_base + 0x400u * static_cast<unsigned>(pin.port) +
                                                                     (pin.number < 8 ? 0x00 : 0x04));
        return (r >> (4 * (pin.number % 8))) & 0xFu;
    }

    static void load_registers() noexcept {
        CR2::write(timing.freq);
        CCR::write(timing.ccr);
        TRISE::write(timing.trise);
        CR1::write(i2c_f1_detail::cr1_pe);
    }

    /// START (or repeated START) and the address byte; Ok once ADDR is set (not yet cleared).
    static I2cResult start(const Deadline& d, std::uint8_t address_byte) noexcept {
        using namespace i2c_f1_detail;
        CR1::write((CR1::read() & ~(cr1_stop | cr1_pos)) | cr1_start | cr1_ack);
        while ((SR1::read() & sr1_sb) == 0) {  // erratum 2.8.4: no SB -> SWRST (in fail)
            if ((SR1::read() & sr1_berr) != 0 || d.expired()) {
                return fail(I2cResult::BusError);
            }
        }
        DR::write(address_byte);  // EV5 cleared by SR1 read + DR write
        return wait_sr1(d, sr1_addr);
    }

    /// Wait for `flag` in SR1; NACK, bus error, lost arbitration and timeout end the transfer.
    static I2cResult wait_sr1(const Deadline& d, std::uint32_t flag) noexcept {
        using namespace i2c_f1_detail;
        for (;;) {
            const std::uint32_t sr1 = SR1::read();
            if ((sr1 & sr1_af) != 0) {
                SR1::write(~sr1_af);
                CR1::write(CR1::read() | cr1_stop);
                ++stats_.nacks;
                (void)wait_stop(d);
                return I2cResult::Nack;
            }
            if ((sr1 & sr1_arlo) != 0) {
                return fail(I2cResult::ArbitrationLost);
            }
            if ((sr1 & sr1_berr) != 0) {
                return fail(I2cResult::BusError);
            }
            if ((sr1 & flag) != 0) {
                return I2cResult::Ok;
            }
            if (d.expired()) {
                return fail(I2cResult::Timeout);
            }
        }
    }

    static bool wait_stop(const Deadline& d) noexcept {
        while ((CR1::read() & i2c_f1_detail::cr1_stop) != 0) {  // cleared by hardware when sent
            if (d.expired()) {
                return false;
            }
        }
        return true;
    }

    static I2cResult stop(const Deadline& d) noexcept {
        CR1::write(CR1::read() | i2c_f1_detail::cr1_stop);
        return wait_stop(d) ? I2cResult::Ok : fail(I2cResult::Timeout);
    }

    /// Master receiver, RM0008 26.3.3 with the closing sequences of AN2824 for 1, 2 and > 2 bytes
    /// and ES096 2.8.1/2.8.2 workaround 2 (interrupts masked in the critical windows).
    static I2cResult receive(const Deadline& d, std::uint8_t address, std::span<std::byte> rx) noexcept {
        using namespace i2c_f1_detail;
        const std::size_t n = rx.size();
        if (const I2cResult r = start(d, static_cast<std::uint8_t>((address << 1) | 1u)); r != I2cResult::Ok) {
            return r;
        }
        if (n == 1) {
            CR1::write(CR1::read() & ~cr1_ack);  // EV6_3: NACK, clear ADDR, STOP - uninterrupted
            const std::uint32_t m = mask_irq();
            (void)SR1::read();
            (void)SR2::read();
            CR1::write(CR1::read() | cr1_stop);
            restore_irq(m);
            if (const I2cResult r = wait_sr1(d, sr1_rxne); r != I2cResult::Ok) {
                return r;
            }
            rx[0] = static_cast<std::byte>(DR::read());
            return wait_stop(d) ? I2cResult::Ok : fail(I2cResult::Timeout);
        }
        if (n == 2) {
            // EV6_1 (RM0008 fig. "master receiver N=2", ST HAL): POS on while ADDR still stretches
            // SCL, clear ADDR, then ACK off - uninterrupted, before the first byte ends. Clearing ACK
            // together with POS before ADDR (RM0008 26.3.3 text) NACKs the FIRST byte on the real
            // chip: the slave lets go of SDA and the second byte reads 0xFF (seen on a PCA9685).
            CR1::write(CR1::read() | cr1_pos);
            const std::uint32_t m = mask_irq();
            (void)SR1::read();
            (void)SR2::read();
            CR1::write(CR1::read() & ~cr1_ack);
            restore_irq(m);
            if (const I2cResult r = wait_sr1(d, sr1_btf); r != I2cResult::Ok) {  // data 1 in DR, 2 in shift
                return r;
            }
            const std::uint32_t m2 = mask_irq();
            CR1::write(CR1::read() | cr1_stop);
            rx[0] = static_cast<std::byte>(DR::read());
            rx[1] = static_cast<std::byte>(DR::read());
            restore_irq(m2);
            CR1::write(CR1::read() & ~cr1_pos);
            return wait_stop(d) ? I2cResult::Ok : fail(I2cResult::Timeout);
        }
        (void)SR1::read();
        (void)SR2::read();  // ADDR cleared, ACK on
        std::size_t i = 0;
        for (; i + 3 < n; ++i) {
            if (const I2cResult r = wait_sr1(d, sr1_rxne); r != I2cResult::Ok) {
                return r;
            }
            rx[i] = static_cast<std::byte>(DR::read());
        }
        if (const I2cResult r = wait_sr1(d, sr1_btf); r != I2cResult::Ok) {  // N-2 in DR, N-1 in shift
            return r;
        }
        CR1::write(CR1::read() & ~cr1_ack);
        const std::uint32_t m = mask_irq();
        rx[i++] = static_cast<std::byte>(DR::read());  // N-2
        CR1::write(CR1::read() | cr1_stop);
        rx[i++] = static_cast<std::byte>(DR::read());  // N-1
        restore_irq(m);
        if (const I2cResult r = wait_sr1(d, sr1_rxne); r != I2cResult::Ok) {
            return r;
        }
        rx[i] = static_cast<std::byte>(DR::read());  // N
        return wait_stop(d) ? I2cResult::Ok : fail(I2cResult::Timeout);
    }

    /// An error ends the transfer: STOP, then the erratum recovery (SWRST and the line sequence).
    static I2cResult fail(I2cResult r) noexcept {
        ++stats_.errors;
        CR1::write(CR1::read() | i2c_f1_detail::cr1_stop);
        (void)recover_bus();
        return r;
    }

    using CR1 = cortex_m::Reg<hw.base + 0x00>;
    using CR2 = cortex_m::Reg<hw.base + 0x04>;
    using DR = cortex_m::Reg<hw.base + 0x10>;
    using SR1 = cortex_m::Reg<hw.base + 0x14>;
    using SR2 = cortex_m::Reg<hw.base + 0x18>;
    using CCR = cortex_m::Reg<hw.base + 0x1C>;
    using TRISE = cortex_m::Reg<hw.base + 0x20>;
};

}  // namespace rtos::port::stm32f1
