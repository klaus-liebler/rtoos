// SPDX-License-Identifier: MIT
//
// STM32G431 USB full-speed device: the shared STM32 driver (stm32/usb_device.hpp) on the G431's
// registers (USB at 0x40005C00, 16-bit registers; PMA 1 KB at 0x40006000 in 16-bit half-words,
// buffer table at 0 (BTABLE reset value); IRQ USB_LP = 20. RM0440 chapter USB, SVD v1.8; the BCDR
// at 0x58, the PMA address and access scheme: ST's stm32g431xx.h, stm32g4xx_ll_usb.h PMA_ACCESS 1).
//
//   UsbFs<> usb;                        // UsbHardware::enable_clocks() first (board)
//   rtos::usb::Device<UsbFs<>> device{usb, descriptors, functions};
//   extern "C" void USB_LP_IRQHandler() { usb.on_irq(); wake the stack's thread }
//
// The driver sees the PMA as 32-bit words (the H5 layout); pma_read/pma_write split them into
// the two half-words at offset and offset + 2. The buffer descriptors ADDRn_TX/COUNTn_TX and
// ADDRn_RX/COUNTn_RX are exactly those halves.
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/stm32/usb_device.hpp"
#include "rtos/port/stm32g4/chip.hpp"
#include "rtos/port/stm32g4/rcc.hpp"

namespace rtos::port::stm32g4 {

using stm32::UsbDeviceStats;

/// The real registers (USB at 0x40005C00, PMA at 0x40006000).
struct UsbHardware {
    static constexpr std::uint8_t irq = 20;  // USB_LP: every event of the device
    static constexpr std::size_t pma_bytes = 1024;

    /// HSI48 as the kernel clock (RCC_CCIPR.CLK48SEL reset value), trimmed by the CRS against
    /// the host's start-of-frame packets (CRS_CFGR reset: SYNCSRC = USB SOF, RELOAD 47999,
    /// FELIM 0x22 - ST's LL_CRS_*_DEFAULT), the controller's clock.
    static void enable_clocks() noexcept {
        rcc::enable_hsi48();
        rcc::enable(rcc::gate::crs);
        using CRS_CR = PReg<0x0000'2000>;
        CRS_CR::write(CRS_CR::read() | (1u << 5) | (1u << 6));  // CEN, AUTOTRIMEN
        rcc::enable(rcc::gate::usb);
    }

    /// Interrupts off while the thread changes a register the ISR also writes.
    struct Masked {
        std::uint32_t primask;
        Masked() noexcept { __asm volatile("mrs %0, primask\n cpsid i" : "=r"(primask)::"memory"); }
        ~Masked() { __asm volatile("msr primask, %0" ::"r"(primask) : "memory"); }
    };

    static constexpr std::uintptr_t base = periph_base + 0x0000'5C00, pma = periph_base + 0x0000'6000;
    static volatile std::uint16_t& reg(std::uintptr_t offset) noexcept {
        return *reinterpret_cast<volatile std::uint16_t*>(base + offset);
    }
    static volatile std::uint16_t& half(std::size_t offset) noexcept {
        return *reinterpret_cast<volatile std::uint16_t*>(pma + offset);
    }
    std::uint32_t chep(unsigned n) noexcept { return reg(4 * n); }
    void set_chep(unsigned n, std::uint32_t v) noexcept { reg(4 * n) = static_cast<std::uint16_t>(v); }
    std::uint32_t cntr() noexcept { return reg(0x40); }
    void set_cntr(std::uint32_t v) noexcept { reg(0x40) = static_cast<std::uint16_t>(v); }
    std::uint32_t istr() noexcept { return reg(0x44); }
    void set_istr(std::uint32_t v) noexcept { reg(0x44) = static_cast<std::uint16_t>(v); }
    void set_daddr(std::uint32_t v) noexcept { reg(0x4C) = static_cast<std::uint16_t>(v); }
    std::uint32_t bcdr() noexcept { return reg(0x58); }
    void set_bcdr(std::uint32_t v) noexcept { reg(0x58) = static_cast<std::uint16_t>(v); }
    std::uint32_t pma_read(std::size_t offset) noexcept {
        return half(offset) | (std::uint32_t{half(offset + 2)} << 16);
    }
    void pma_write(std::size_t offset, std::uint32_t v) noexcept {
        half(offset) = static_cast<std::uint16_t>(v);
        half(offset + 2) = static_cast<std::uint16_t>(v >> 16);
    }
};

template <class Regs = UsbHardware>
using UsbFs = stm32::UsbDevice<Regs>;

}  // namespace rtos::port::stm32g4
