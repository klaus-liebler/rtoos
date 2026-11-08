// SPDX-License-Identifier: MIT
//
// What a product needs of the board's chip without naming it (FCU revisions 1/2: STM32H563,
// Cortex-M33). The revision 3 package (boards/fcu_rev3) has the same header for the STM32H743,
// so code written against board::Port, board::Ethernet, board::Rng ... builds for every revision.
//
//   Kernel kernel;
//   RTOS_BOARD_BIND_KERNEL(kernel)
//   using Kernel = rtos::Kernel<board::Port, Config>;
//   RTOS_DMA_BUFFER board::Ethernet<Pool, 8> eth{pool, board::mac_address(), board::phy_address};
//   eth.init(board::hclk_hz);
#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "board/fcu.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/cortex_m/cycle_counter.hpp"
#include "rtos/port/cortex_m/registers.hpp"
#include "rtos/port/stm32h5/cache.hpp"  // RTOS_DMA_BUFFER
#include "rtos/port/stm32h5/ethernet.hpp"
#include "rtos/port/stm32h5/rng.hpp"
#include "rtos/port/stm32h5/usb_device.hpp"

/// Connects one kernel instance to the CPU port's exception handlers (once, at namespace scope).
#define RTOS_BOARD_BIND_KERNEL(kernel_object) RTOS_CM33_BIND_KERNEL(kernel_object)
/// The USB device controller's interrupt handler (board::UsbDevice::on_irq()).
#define RTOS_BOARD_USB_IRQ_HANDLER USB_DRD_FS_IRQHandler

namespace board {

namespace chip = rtos::port::stm32h5;
using Port = rtos::port::cm33::Cm33Port;
using CycleCounter = rtos::port::cortex_m::CycleCounter;
namespace nvic = rtos::port::cortex_m::nvic;
/// NVIC priority bits (an IRQ priority p goes into nvic::ipr as p << (8 - irq_priority_bits)).
inline constexpr unsigned irq_priority_bits = RTOS_CM33_PRIO_BITS;
inline constexpr std::string_view mcu_name = "STM32H563";
/// AHB clock: the Ethernet's MDIO divider and 1 us tick (= SYSCLK on the H563).
inline constexpr std::uint32_t hclk_hz = clocks.hclk_hz();

template <rtos::net::PacketAllocator Pool, std::size_t RxCount = 8, std::size_t TxCount = 8>
using Ethernet = chip::Ethernet<Pool, RxCount, TxCount>;
using Rng = chip::Rng;
/// USB device controller (rtos::usb::DeviceController): USB_DRD_FS; board::init_usb() first.
using UsbDevice = chip::UsbDrdFs<>;

}  // namespace board
