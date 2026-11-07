// SPDX-License-Identifier: MIT
//
// STM32H743 USART/UART: the shared STM32 driver (stm32/usart.hpp) with the H743's instances
// (RM0433 chapter USART; addresses, clock gates: SVD v2.4; IRQ numbers: ST's stm32h743xx.h).
//
//   constexpr UartConfig console{.kernel_hz = clocks.pclk1_hz(), .baud = 115'200};
//   Usart<UsartId::Usart3>::configure<console>();
//
// Kernel clock: the RCC_D2CCIP2R reset selection, PCLK2 (USART1/6) or PCLK1 (USART2/3,
// UART4/5/7/8).
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/stm32/usart.hpp"
#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/rcc.hpp"

namespace rtos::port::stm32h7 {

using stm32::checked_uart_registers, stm32::uart_baud_error, stm32::uart_brr, stm32::uart_registers;
using stm32::Parity, stm32::UartConfig, stm32::UartRegisters, stm32::UartStats;

enum class UsartId : std::uint8_t { Usart1, Usart2, Usart3, Uart4, Uart5, Usart6, Uart7, Uart8 };

[[nodiscard]] consteval stm32::UsartHw usart_hw(UsartId id) {
    switch (id) {
        case UsartId::Usart1: return {0x4001'1000, rcc::gate::usart1, 37};
        case UsartId::Usart2: return {0x4000'4400, rcc::gate::usart2, 38};
        case UsartId::Usart3: return {0x4000'4800, rcc::gate::usart3, 39};
        case UsartId::Uart4: return {0x4000'4C00, rcc::gate::uart4, 52};
        case UsartId::Uart5: return {0x4000'5000, rcc::gate::uart5, 53};
        case UsartId::Usart6: return {0x4001'1400, rcc::gate::usart6, 71};
        case UsartId::Uart7: return {0x4000'7800, rcc::gate::uart7, 82};
        case UsartId::Uart8: return {0x4000'7C00, rcc::gate::uart8, 83};
    }
    return {};
}

template <UsartId Id>
using Usart = stm32::Usart<usart_hw(Id)>;

template <UsartId Id, UartConfig Config, std::size_t RxBytes = 256, std::size_t TxBytes = 256>
using BufferedUart = stm32::BufferedUart<usart_hw(Id), Config, RxBytes, TxBytes>;

}  // namespace rtos::port::stm32h7
