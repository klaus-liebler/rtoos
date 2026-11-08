// SPDX-License-Identifier: MIT
//
// What a product needs of the board's chip without naming it (FCU revision 3: STM32H743,
// Cortex-M7). The revision 1/2 package (boards/fcu) has the same header for the STM32H563,
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
#include "rtos/port/cm7/port.hpp"
#include "rtos/port/cortex_m/cycle_counter.hpp"
#include "rtos/port/cortex_m/registers.hpp"
#include "rtos/port/stm32h7/ethernet.hpp"
#include "rtos/port/stm32h7/mpu.hpp"  // RTOS_DMA_BUFFER
#include "rtos/port/stm32h7/rng.hpp"
#include "rtos/port/stm32h7/usb_device.hpp"

/// Connects one kernel instance to the CPU port's exception handlers (once, at namespace scope).
#define RTOS_BOARD_BIND_KERNEL(kernel_object) RTOS_CM7_BIND_KERNEL(kernel_object)
/// The USB device controller's interrupt handler (board::UsbDevice::on_irq()).
#define RTOS_BOARD_USB_IRQ_HANDLER OTG_FS_IRQHandler

namespace board {

namespace chip = rtos::port::stm32h7;
using Port = rtos::port::cm7::Cm7Port;
using CycleCounter = rtos::port::cortex_m::CycleCounter;
namespace nvic = rtos::port::cortex_m::nvic;
/// NVIC priority bits (an IRQ priority p goes into nvic::ipr as p << (8 - irq_priority_bits)).
inline constexpr unsigned irq_priority_bits = RTOS_CM4_PRIO_BITS;
inline constexpr std::string_view mcu_name = "STM32H743";
/// AHB clock: the Ethernet's MDIO divider and 1 us tick (200 MHz; the CPU runs at 400 MHz).
inline constexpr std::uint32_t hclk_hz = clocks.hclk_hz();

template <rtos::net::PacketAllocator Pool, std::size_t RxCount = 8, std::size_t TxCount = 8>
using Ethernet = chip::Ethernet<Pool, RxCount, TxCount>;
using Rng = chip::Rng;
/// USB device controller (rtos::usb::DeviceController): USB2_OTG_FS; board::init_usb() first.
using UsbDevice = chip::UsbOtgFs<hclk_hz, clocks.cpu_hz()>;

}  // namespace board
