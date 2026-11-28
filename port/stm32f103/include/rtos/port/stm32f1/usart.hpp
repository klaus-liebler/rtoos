// SPDX-License-Identifier: MIT
//
// STM32F103 USART (RM0008 rev 21 chapter 27): the F1 USART has no FIFO and the older register
// layout (SR, DR, BRR, CR1-3); errors clear by reading SR, then DR. Same interface as the shared
// driver of the newer chips (port/stm32/usart.hpp):
//
//   Usart<UsartId::Usart1>           polled put/write/flush (console, fault handlers)
//   BufferedUart<UsartId::Usart1, C> interrupt driven: receive and transmit rings
//
//   constexpr UartConfig console{.kernel_hz = clocks.pclk2_hz(), .baud = 115'200};
//   Usart<UsartId::Usart1>::configure<console>();
//
// The configuration is a template argument: BRR is computed and checked at compile time (16x
// oversampling, BRR = f_PCLK / baud with 4 fraction bits - RM0008 27.3.4; error within 2 %).
// Kernel clock: PCLK2 for USART1, PCLK1 for USART2/3.
//
// Errata (ES096): 2.3.1 USART1_RTS with CAN not clocked and 2.3.10 USART1_TX with TIM1_CH2 PWM -
// the driver uses neither RTS nor touches TIM1; 2.9.x (parity error flag, idle frame detection)
// - no parity checking beyond counting, no idle detection.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "rtos/port/cortex_m/registers.hpp"
#include "rtos/port/stm32/rcc.hpp"
#include "rtos/port/stm32/usart.hpp"
#include "rtos/port/stm32f1/chip.hpp"
#include "rtos/port/stm32f1/rcc.hpp"
#include "rtos/sync/spsc_ring.hpp"

namespace rtos::port::stm32f1 {

using stm32::Parity, stm32::UartStats, stm32::uart_brr;

struct UartConfig {
    std::uint32_t kernel_hz;       ///< PCLK2 (USART1) or PCLK1 (USART2/3)
    std::uint32_t baud = 115'200;
    Parity parity = Parity::None;  ///< with parity the frame has 9 bits (8 data + parity)
    bool two_stop_bits = false;
    bool half_duplex = false;      ///< single wire on TX (HDSEL)
};

enum class UsartId : std::uint8_t { Usart1, Usart2, Usart3 };

struct UsartHw {
    std::uintptr_t base;
    stm32::rcc::Gate gate;
    std::uint8_t irq;
};

[[nodiscard]] consteval UsartHw usart_hw(UsartId id) {
    switch (id) {
        case UsartId::Usart1: return {periph_base + 0x0001'3800, rcc::gate::usart1, 37};
        case UsartId::Usart2: return {periph_base + 0x0000'4400, rcc::gate::usart2, 38};
        case UsartId::Usart3: return {periph_base + 0x0000'4800, rcc::gate::usart3, 39};
    }
    return {};
}

namespace usart_detail {
inline constexpr std::uint32_t sr_pe = 1u << 0, sr_fe = 1u << 1, sr_ne = 1u << 2, sr_ore = 1u << 3,
                               sr_rxne = 1u << 5, sr_tc = 1u << 6, sr_txe = 1u << 7;
inline constexpr std::uint32_t sr_errors = sr_pe | sr_fe | sr_ne | sr_ore;
inline constexpr std::uint32_t cr1_re = 1u << 2, cr1_te = 1u << 3, cr1_rxneie = 1u << 5, cr1_txeie = 1u << 7,
                               cr1_ps = 1u << 9, cr1_pce = 1u << 10, cr1_m = 1u << 12, cr1_ue = 1u << 13;
inline constexpr std::uint32_t cr2_stop_2 = 2u << 12;
inline constexpr std::uint32_t cr3_eie = 1u << 0, cr3_hdsel = 1u << 3;
}  // namespace usart_detail

struct UartRegisters {
    std::uint32_t brr, cr1, cr2, cr3;
};

/// Baud rate error of `c` in 1/10000 (0.01 %).
[[nodiscard]] constexpr std::uint32_t uart_baud_error(const UartConfig& c) noexcept {
    const std::uint64_t actual = c.kernel_hz / uart_brr(c.kernel_hz, c.baud);
    const std::uint64_t diff = actual > c.baud ? actual - c.baud : c.baud - actual;
    return static_cast<std::uint32_t>(diff * 10'000 / c.baud);
}

[[nodiscard]] constexpr UartRegisters uart_registers(const UartConfig& c) noexcept {
    using namespace usart_detail;
    const std::uint32_t parity =
        c.parity == Parity::None ? 0u : (cr1_pce | cr1_m | (c.parity == Parity::Odd ? cr1_ps : 0u));
    return {uart_brr(c.kernel_hz, c.baud), parity | cr1_te | cr1_re | cr1_ue, c.two_stop_bits ? cr2_stop_2 : 0u,
            c.half_duplex ? cr3_hdsel : 0u};
}

template <UartConfig C>
consteval UartRegisters checked_uart_registers() {
    static_assert(C.baud > 0 && C.kernel_hz >= 16 * C.baud, "baud rate too high for the kernel clock (BRR < 16)");
    static_assert(uart_brr(C.kernel_hz, C.baud) <= 0xFFFF, "baud rate too low for the kernel clock (BRR > 0xFFFF)");
    static_assert(uart_baud_error(C) <= 200, "baud rate error above 2 %");
    return uart_registers(C);
}

template <UsartId Id>
class Usart {
    static constexpr UsartHw hw = usart_hw(Id);

public:
    static constexpr std::uintptr_t base = hw.base;  ///< the instance (resource checks: rtos/distinct.hpp)
    static constexpr std::uint8_t irq = hw.irq;

    /// Clock on, frame format, baud rate; transmitter and receiver enabled, no interrupts.
    template <UartConfig C>
    static void configure() noexcept {
        constexpr UartRegisters r = checked_uart_registers<C>();
        rcc::enable(hw.gate);
        CR1::write(0);
        BRR::write(r.brr);
        CR2::write(r.cr2);
        CR3::write(r.cr3);
        CR1::write(r.cr1);
    }

    static void put(char c) noexcept {
        while ((SR::read() & usart_detail::sr_txe) == 0) {
        }
        DR::write(static_cast<std::uint8_t>(c));
    }

    static void write(std::string_view text) noexcept {
        for (char c : text) {
            if (c == '\n') {
                put('\r');
            }
            put(c);
        }
    }

    /// Block until the last character has left the shift register.
    static void flush() noexcept {
        while ((SR::read() & usart_detail::sr_tc) == 0) {
        }
    }

private:
    template <UsartId, UartConfig, std::size_t, std::size_t>
    friend class BufferedUart;

    using SR = cortex_m::Reg<hw.base + 0x00>;
    using DR = cortex_m::Reg<hw.base + 0x04>;
    using BRR = cortex_m::Reg<hw.base + 0x08>;
    using CR1 = cortex_m::Reg<hw.base + 0x0C>;
    using CR2 = cortex_m::Reg<hw.base + 0x10>;
    using CR3 = cortex_m::Reg<hw.base + 0x14>;
};

/// Interrupt-driven UART. Call on_irq() from the USARTx handler; one thread uses the read/write
/// side.
template <UsartId Id, UartConfig Config, std::size_t RxBytes = 128, std::size_t TxBytes = 256>
class BufferedUart {
    using U = Usart<Id>;

public:
    static constexpr std::uintptr_t base = U::base;
    static constexpr std::uint8_t irq = U::irq;

    /// Configure and enable the receive interrupt. The caller sets the NVIC priority and enables
    /// `irq` (kernel-aware priority: on_irq() is lock-free).
    void start() noexcept {
        using namespace usart_detail;
        constexpr UartRegisters r = checked_uart_registers<Config>();
        U::template configure<Config>();
        U::CR3::write(r.cr3 | cr3_eie);
        U::CR1::write(r.cr1 | cr1_rxneie);
    }

    // ---- thread side -------------------------------------------------------------------

    /// Queue `data` for sending; returns how much fit (never waits).
    std::size_t write(std::span<const std::byte> data) noexcept {
        const std::size_t n = tx_.write(data);
        if (n != 0) {
            // races with the ISR clearing TXEIE only harmlessly: at worst one more interrupt
            // that finds the ring empty
            U::CR1::write(U::CR1::read() | usart_detail::cr1_txeie);
        }
        return n;
    }

    /// Free bytes in the transmit ring: a write of at most this much goes in whole (from the
    /// one writing thread; the ISR only frees more).
    [[nodiscard]] std::size_t tx_space() const noexcept { return tx_.space(); }

    std::size_t read(std::span<std::byte> out) noexcept { return rx_.read(out); }
    [[nodiscard]] std::size_t available() const noexcept { return rx_.size(); }

    /// Everything queued has left the shift register.
    [[nodiscard]] bool tx_idle() const noexcept {
        return tx_.size() == 0 && (U::SR::read() & usart_detail::sr_tc) != 0;
    }

    [[nodiscard]] const UartStats& stats() const noexcept { return stats_; }

    // ---- interrupt side ----------------------------------------------------------------

    void on_irq() noexcept {
        using namespace usart_detail;
        const std::uint32_t sr = U::SR::read();
        if ((sr & (sr_rxne | sr_errors)) != 0) {
            // reading DR after SR clears RXNE and the error flags (RM0008 27.6.1)
            const auto b = static_cast<std::byte>(U::DR::read());
            stats_.overruns += (sr & sr_ore) != 0;
            stats_.frame_errors += (sr & (sr_pe | sr_fe | sr_ne)) != 0;
            if ((sr & sr_rxne) != 0 && rx_.write(std::span(&b, 1)) == 0) {
                ++stats_.rx_dropped;
            }
        }
        if ((U::CR1::read() & cr1_txeie) != 0 && (sr & sr_txe) != 0) {
            std::byte b;
            if (tx_.read(std::span(&b, 1)) == 0) {
                U::CR1::write(U::CR1::read() & ~cr1_txeie);
            } else {
                U::DR::write(static_cast<std::uint32_t>(b));
            }
        }
    }

private:
    ByteRing<RxBytes> rx_;
    ByteRing<TxBytes> tx_;
    UartStats stats_{};
};

}  // namespace rtos::port::stm32f1
