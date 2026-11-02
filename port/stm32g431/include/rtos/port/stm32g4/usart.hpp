// SPDX-License-Identifier: MIT
//
// STM32G431 USART/UART: the shared STM32 driver (stm32/usart.hpp) with the G431's instances
// (RM0440 chapter USART; addresses, clock gates: SVD v1.8; which instances exist and their IRQ
// numbers: ST's stm32g431xx.h - USART1-3 and UART4, no UART5).
//
//   constexpr UartConfig console{.kernel_hz = clocks.pclk2_hz(), .baud = 115'200};
//   Usart<UsartId::Usart1>::configure<console>();
//
// Kernel clock: the RCC_CCIPR reset selection, PCLK2 (USART1) or PCLK1 (USART2/3, UART4).
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/stm32/usart.hpp"
#include "rtos/port/stm32g4/chip.hpp"
#include "rtos/port/stm32g4/rcc.hpp"

namespace rtos::port::stm32g4 {

using stm32::checked_uart_registers, stm32::uart_baud_error, stm32::uart_brr, stm32::uart_registers;
using stm32::Parity, stm32::UartConfig, stm32::UartRegisters, stm32::UartStats;

enum class UsartId : std::uint8_t { Usart1, Usart2, Usart3, Uart4 };

[[nodiscard]] consteval stm32::UsartHw usart_hw(UsartId id) {
    switch (id) {
        case UsartId::Usart1: return {periph_base + 0x0001'3800, rcc::gate::usart1, 37};
        case UsartId::Usart2: return {periph_base + 0x0000'4400, rcc::gate::usart2, 38};
        case UsartId::Usart3: return {periph_base + 0x0000'4800, rcc::gate::usart3, 39};
        case UsartId::Uart4: return {periph_base + 0x0000'4C00, rcc::gate::uart4, 52};
    }
    return {};
}

template <UsartId Id>
using Usart = stm32::Usart<usart_hw(Id)>;

template <UsartId Id, UartConfig Config, std::size_t RxBytes = 256, std::size_t TxBytes = 256>
using BufferedUart = stm32::BufferedUart<usart_hw(Id), Config, RxBytes, TxBytes>;

}  // namespace rtos::port::stm32g4
