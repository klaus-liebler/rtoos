// SPDX-License-Identifier: MIT
//
// STM32H563 USART/UART (RM0481 chapter USART; registers, bits, IRQ numbers: SVD v1.8).
//
// The whole configuration is a template argument: the compiler computes every register
// value (BRR, CR1-3) and checks it - baud rate divider in range, baud rate error within
// 2 % - and the run-time code only writes constants.
//
//   constexpr UartConfig console{.kernel_hz = board::sysclk_hz, .baud = 115'200};
//   Usart<UsartId::Usart3>::configure<console>();
//
//   Usart<Id>                    the registers: configure, polled put/write/flush (console,
//                                fault handlers - works from any context)
//   BufferedUart<Id, Config>     interrupt driven: a receive ring the ISR fills (no byte is
//                                lost while the thread transmits - the echo of a single-wire
//                                bus included) and a transmit ring the ISR empties. The thread
//                                side never blocks; device drivers above it wait with their
//                                own deadline.
//
// Kernel clock: the RCC_CCIPRx reset selection, PCLK1 (USART2/3, UART4/5/7/8/9/12) or
// PCLK2 (USART1); oversampling 16, so BRR = f_kernel / baud.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"
#include "rtos/sync/spsc_ring.hpp"

namespace rtos::port::stm32h5 {

enum class UsartId : std::uint8_t { Usart1, Usart2, Usart3, Uart4, Uart5, Uart7, Uart8, Uart9, Uart12 };

enum class Parity : std::uint8_t { None, Even, Odd };

struct UartConfig {
    std::uint32_t kernel_hz;       ///< kernel clock of the USART (normally PCLK)
    std::uint32_t baud = 115'200;
    Parity parity = Parity::None;  ///< with parity the frame has 9 bits (8 data + parity)
    bool two_stop_bits = false;
    bool half_duplex = false;      ///< single wire on TX (HDSEL); the receiver sees the own bytes
    bool fifo = true;              ///< 8-byte FIFOs in both directions
};

namespace usart_detail {

struct Info {
    std::uintptr_t offset;  ///< from periph_alias
    rcc::Gate gate;
    std::uint8_t irq;
};

[[nodiscard]] consteval Info info(UsartId id) {
    switch (id) {
        case UsartId::Usart1: return {0x0001'3800, rcc::gate::usart1, 58};
        case UsartId::Usart2: return {0x0000'4400, rcc::gate::usart2, 59};
        case UsartId::Usart3: return {0x0000'4800, rcc::gate::usart3, 60};
        case UsartId::Uart4: return {0x0000'4C00, rcc::gate::uart4, 61};
        case UsartId::Uart5: return {0x0000'5000, rcc::gate::uart5, 62};
        case UsartId::Uart7: return {0x0000'7800, rcc::gate::uart7, 98};
        case UsartId::Uart8: return {0x0000'7C00, rcc::gate::uart8, 99};
        case UsartId::Uart9: return {0x0000'8000, rcc::gate::uart9, 100};
        case UsartId::Uart12: return {0x0000'8400, rcc::gate::uart12, 101};
    }
    return {};
}

// CR1
inline constexpr std::uint32_t cr1_ue = 1u << 0, cr1_re = 1u << 2, cr1_te = 1u << 3, cr1_rxfneie = 1u << 5,
                               cr1_txfnfie = 1u << 7, cr1_ps = 1u << 9, cr1_pce = 1u << 10, cr1_m0 = 1u << 12,
                               cr1_fifoen = 1u << 29;
// CR2
inline constexpr std::uint32_t cr2_stop_2 = 2u << 12;
// CR3
inline constexpr std::uint32_t cr3_eie = 1u << 0, cr3_hdsel = 1u << 3;
// ISR / ICR
inline constexpr std::uint32_t isr_pe = 1u << 0, isr_fe = 1u << 1, isr_ne = 1u << 2, isr_ore = 1u << 3,
                               isr_rxfne = 1u << 5, isr_tc = 1u << 6, isr_txfnf = 1u << 7;
inline constexpr std::uint32_t isr_errors = isr_pe | isr_fe | isr_ne | isr_ore;

}  // namespace usart_detail

/// Register values of a UartConfig (computed at compile time).
struct UartRegisters {
    std::uint32_t brr, cr1, cr2, cr3;
};

[[nodiscard]] constexpr std::uint32_t uart_brr(std::uint32_t kernel_hz, std::uint32_t baud) noexcept {
    return (kernel_hz + baud / 2) / baud;
}

/// Baud rate error of `c` in 1/10000 (0.01 %).
[[nodiscard]] constexpr std::uint32_t uart_baud_error(const UartConfig& c) noexcept {
    const std::uint64_t actual = c.kernel_hz / uart_brr(c.kernel_hz, c.baud);
    const std::uint64_t diff = actual > c.baud ? actual - c.baud : c.baud - actual;
    return static_cast<std::uint32_t>(diff * 10'000 / c.baud);
}

[[nodiscard]] constexpr UartRegisters uart_registers(const UartConfig& c) noexcept {
    using namespace usart_detail;
    const std::uint32_t parity =
        c.parity == Parity::None ? 0u : (cr1_pce | cr1_m0 | (c.parity == Parity::Odd ? cr1_ps : 0u));
    return {uart_brr(c.kernel_hz, c.baud), parity | (c.fifo ? cr1_fifoen : 0u) | cr1_te | cr1_re | cr1_ue,
            c.two_stop_bits ? cr2_stop_2 : 0u, c.half_duplex ? cr3_hdsel : 0u};
}

/// Compile-time check of a configuration (used by every configure()).
template <UartConfig C>
consteval UartRegisters checked_uart_registers() {
    static_assert(C.baud > 0 && C.kernel_hz >= 16 * C.baud, "baud rate too high for the kernel clock (BRR < 16)");
    static_assert(uart_brr(C.kernel_hz, C.baud) <= 0xFFFF, "baud rate too low for the kernel clock (BRR > 0xFFFF)");
    static_assert(uart_baud_error(C) <= 200, "baud rate error above 2 %");
    return uart_registers(C);
}

template <UsartId Id>
class Usart {
    static constexpr usart_detail::Info info = usart_detail::info(Id);

public:
    static constexpr std::uint8_t irq = info.irq;

    /// Clock on, frame format, baud rate; transmitter and receiver enabled, no interrupts.
    template <UartConfig C>
    static void configure() noexcept {
        constexpr UartRegisters r = checked_uart_registers<C>();
        rcc::enable(info.gate);
        CR1::write(0);
        BRR::write(r.brr);
        CR2::write(r.cr2);
        CR3::write(r.cr3);
        CR1::write(r.cr1);
    }

    static void put(char c) noexcept {
        while ((ISR::read() & usart_detail::isr_txfnf) == 0) {
        }
        TDR::write(static_cast<std::uint8_t>(c));
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
        while ((ISR::read() & usart_detail::isr_tc) == 0) {
        }
    }

private:
    template <UsartId, UartConfig, std::size_t, std::size_t>
    friend class BufferedUart;

    using CR1 = PReg<info.offset + 0x00>;
    using CR2 = PReg<info.offset + 0x04>;
    using CR3 = PReg<info.offset + 0x08>;
    using BRR = PReg<info.offset + 0x0C>;
    using ISR = PReg<info.offset + 0x1C>;
    using ICR = PReg<info.offset + 0x20>;
    using RDR = PReg<info.offset + 0x24>;
    using TDR = PReg<info.offset + 0x28>;
};

struct UartStats {
    std::uint32_t overruns = 0;       ///< hardware overrun (ORE): the ISR came too late
    std::uint32_t frame_errors = 0;   ///< framing, noise or parity error
    std::uint32_t rx_dropped = 0;     ///< receive ring full: the thread reads too slowly
};

/// Interrupt-driven UART. Call on_irq() from the USARTx/UARTx handler; one thread uses the
/// read/write side.
template <UsartId Id, UartConfig Config, std::size_t RxBytes = 256, std::size_t TxBytes = 256>
class BufferedUart {
    using U = Usart<Id>;

public:
    static constexpr std::uint8_t irq = U::irq;

    /// Configure and enable the receive interrupt. The caller sets the NVIC priority and
    /// enables `irq` (kernel-aware priority: on_irq() is lock-free, the application may
    /// notify a thread from it).
    void start() noexcept {
        using namespace usart_detail;
        constexpr UartRegisters r = checked_uart_registers<Config>();
        U::template configure<Config>();
        U::CR3::write(r.cr3 | cr3_eie);
        U::CR1::write(r.cr1 | cr1_rxfneie);
    }

    /// Switch to another baud rate (checked like the configuration; the line must be idle).
    template <std::uint32_t Baud>
    void set_baud() noexcept {
        constexpr UartConfig c = [] {
            UartConfig n = Config;
            n.baud = Baud;
            return n;
        }();
        constexpr UartRegisters r = checked_uart_registers<c>();
        const std::uint32_t cr1 = U::CR1::read();
        U::CR1::write(cr1 & ~usart_detail::cr1_ue);
        U::BRR::write(r.brr);
        U::CR1::write(cr1);
    }

    // ---- thread side -------------------------------------------------------------------

    /// Queue `data` for sending; returns how much fit (never waits).
    std::size_t write(std::span<const std::byte> data) noexcept {
        const std::size_t n = tx_.write(data);
        if (n != 0) {
            // Read-modify-write races with the ISR clearing TXFNFIE only harmlessly: at worst
            // one more interrupt that finds the ring empty.
            U::CR1::write(U::CR1::read() | usart_detail::cr1_txfnfie);
        }
        return n;
    }

    /// Received bytes, up to out.size() (never waits).
    std::size_t read(std::span<std::byte> out) noexcept { return rx_.read(out); }

    [[nodiscard]] std::size_t available() const noexcept { return rx_.size(); }

    /// Forget everything received so far.
    void discard_input() noexcept {
        std::byte sink[32];
        while (rx_.read(sink) != 0) {
        }
    }

    /// Everything queued has left the shift register.
    [[nodiscard]] bool tx_idle() const noexcept {
        return tx_.size() == 0 && (U::ISR::read() & usart_detail::isr_tc) != 0;
    }

    [[nodiscard]] const UartStats& stats() const noexcept { return stats_; }

    // ---- interrupt side ----------------------------------------------------------------

    void on_irq() noexcept {
        using namespace usart_detail;
        std::uint32_t isr = U::ISR::read();
        if ((isr & isr_errors) != 0) {
            stats_.overruns += (isr & isr_ore) != 0;
            stats_.frame_errors += (isr & (isr_pe | isr_fe | isr_ne)) != 0;
            U::ICR::write(isr & isr_errors);
        }
        while ((isr & isr_rxfne) != 0) {
            const auto b = static_cast<std::byte>(U::RDR::read());
            if (rx_.write(std::span(&b, 1)) == 0) {
                ++stats_.rx_dropped;
            }
            isr = U::ISR::read();
        }
        if ((U::CR1::read() & cr1_txfnfie) != 0) {
            while ((U::ISR::read() & isr_txfnf) != 0) {
                std::byte b;
                if (tx_.read(std::span(&b, 1)) == 0) {
                    U::CR1::write(U::CR1::read() & ~cr1_txfnfie);
                    break;
                }
                U::TDR::write(static_cast<std::uint32_t>(b));
            }
        }
    }

private:
    ByteRing<RxBytes> rx_;
    ByteRing<TxBytes> tx_;
    UartStats stats_{};
};

}  // namespace rtos::port::stm32h5
