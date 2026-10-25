// SPDX-License-Identifier: MIT
//
// STM32H563 USART, polled transmit (console / diagnostics). 8N1, oversampling 16,
// FIFO disabled, kernel clock = PCLK (RCC_CCIPR1 reset default).
#pragma once

#include <cstdint>
#include <string_view>

#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

enum class UsartId : std::uint8_t { Usart1, Usart2, Usart3 };

template <UsartId Id>
class Usart {
public:
    /// Enable the peripheral clock and start the transmitter (and receiver).
    static void init(std::uint32_t kernel_clock_hz, std::uint32_t baud) noexcept {
        if constexpr (Id == UsartId::Usart1) {
            rcc::APB2ENR::set(rcc::apb2_usart1);
            (void)rcc::APB2ENR::read();  // wait 2 cycles after enabling a clock
        } else {
            rcc::APB1LENR::set(Id == UsartId::Usart2 ? rcc::apb1l_usart2 : rcc::apb1l_usart3);
            (void)rcc::APB1LENR::read();
        }
        CR1::write(0);
        BRR::write((kernel_clock_hz + baud / 2) / baud);
        CR1::write(cr1_te | cr1_re | cr1_ue);
    }

    static void put(char c) noexcept {
        while ((ISR::read() & isr_txe) == 0) {
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
        while ((ISR::read() & isr_tc) == 0) {
        }
    }

private:
    static constexpr std::uintptr_t offset = Id == UsartId::Usart1   ? 0x0001'3800
                                             : Id == UsartId::Usart2 ? 0x0000'4400
                                                                     : 0x0000'4800;
    using CR1 = PReg<offset + 0x00>;
    using BRR = PReg<offset + 0x0C>;
    using ISR = PReg<offset + 0x1C>;
    using TDR = PReg<offset + 0x28>;
    static constexpr std::uint32_t cr1_ue = 1u << 0;
    static constexpr std::uint32_t cr1_re = 1u << 2;
    static constexpr std::uint32_t cr1_te = 1u << 3;
    static constexpr std::uint32_t isr_tc = 1u << 6;
    static constexpr std::uint32_t isr_txe = 1u << 7;  // TXFNF when FIFO enabled
};

}  // namespace rtos::port::stm32h5
