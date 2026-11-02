// SPDX-License-Identifier: MIT
//
// STM32H563 USART/UART: the shared STM32 driver (stm32/usart.hpp) with the H563's instances
// (RM0481 chapter USART; addresses, clock gates, IRQ numbers: SVD v1.8).
//
//   constexpr UartConfig console{.kernel_hz = board::clocks.pclk1_hz(), .baud = 115'200};
//   Usart<UsartId::Usart3>::configure<console>();
//   BufferedUart<UsartId::Uart5, {.kernel_hz = board::clocks.pclk1_hz(), .half_duplex = true}> bus;
//
// Kernel clock: the RCC_CCIPRx reset selection, PCLK1 (USART2/3, UART4/5/7/8/9/12) or
// PCLK2 (USART1).
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/stm32/usart.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

using stm32::checked_uart_registers, stm32::uart_baud_error, stm32::uart_brr, stm32::uart_registers;
using stm32::Parity, stm32::UartConfig, stm32::UartRegisters, stm32::UartStats;

enum class UsartId : std::uint8_t { Usart1, Usart2, Usart3, Uart4, Uart5, Uart7, Uart8, Uart9, Uart12 };

[[nodiscard]] consteval stm32::UsartHw usart_hw(UsartId id) {
    switch (id) {
        case UsartId::Usart1: return {periph_alias + 0x0001'3800, rcc::gate::usart1, 58};
        case UsartId::Usart2: return {periph_alias + 0x0000'4400, rcc::gate::usart2, 59};
        case UsartId::Usart3: return {periph_alias + 0x0000'4800, rcc::gate::usart3, 60};
        case UsartId::Uart4: return {periph_alias + 0x0000'4C00, rcc::gate::uart4, 61};
        case UsartId::Uart5: return {periph_alias + 0x0000'5000, rcc::gate::uart5, 62};
        case UsartId::Uart7: return {periph_alias + 0x0000'7800, rcc::gate::uart7, 98};
        case UsartId::Uart8: return {periph_alias + 0x0000'7C00, rcc::gate::uart8, 99};
        case UsartId::Uart9: return {periph_alias + 0x0000'8000, rcc::gate::uart9, 100};
        case UsartId::Uart12: return {periph_alias + 0x0000'8400, rcc::gate::uart12, 101};
    }
    return {};
}

template <UsartId Id>
using Usart = stm32::Usart<usart_hw(Id)>;

template <UsartId Id, UartConfig Config, std::size_t RxBytes = 256, std::size_t TxBytes = 256>
using BufferedUart = stm32::BufferedUart<usart_hw(Id), Config, RxBytes, TxBytes>;

}  // namespace rtos::port::stm32h5
