// SPDX-License-Identifier: MIT
//
// STM32H563 USB_DRD_FS in device mode: the shared STM32 driver (stm32/usb_device.hpp) on the
// H563's registers (USB_DRD_FS at 0x40016000, PMA 2 KB at 0x40016400 in 32-bit words, IRQ 74;
// RM0481 chapter USB, SVD v1.8, ST's stm32h563xx.h / stm32h5xx_ll_usb.h).
//
//   UsbDrdFs<> usb;                     // UsbHardware::enable_clocks() first (board)
//   rtos::usb::Device<UsbDrdFs<>> device{usb, descriptors, functions};
//   extern "C" void USB_DRD_FS_IRQHandler() { usb.on_irq(); wake the network thread }
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/stm32/usb_device.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

using stm32::UsbDeviceStats;

/// The real registers (USB_DRD_FS at 0x40016000, PMA at 0x40016400).
struct UsbHardware {
    static constexpr std::uint8_t irq = 74;  // USB_FS
    static constexpr std::size_t pma_bytes = 2048;

    /// VDDUSB valid (PWR_USBSCR), HSI48 as the kernel clock (RCC_CCIPR4.USBFSSEL), trimmed by the
    /// CRS against the host's start-of-frame packets (CRS_CFGR reset: SYNCSRC = USB SOF, RELOAD
    /// 47999, FELIM 0x22 - ST's LL_CRS_*_DEFAULT), the controller's clock.
    static void enable_clocks() noexcept {
        using USBSCR = PReg<0x0402'0800 + 0x38>;
        USBSCR::write(USBSCR::read() | (1u << 24) | (1u << 25));  // USB33DEN, USB33SV
        rcc::enable_hsi48();
        rcc::select(rcc::kernel_clock::usbfs, rcc::kernel_clock::usbfs_hsi48);
        rcc::enable(rcc::gate::crs);
        using CRS_CR = PReg<0x0000'6000>;
        CRS_CR::write(CRS_CR::read() | (1u << 5) | (1u << 6));  // CEN, AUTOTRIMEN
        rcc::enable(rcc::gate::usbfs);
    }

    /// Interrupts off while the thread changes a register the ISR also writes.
    struct Masked {
        std::uint32_t primask;
        Masked() noexcept { __asm volatile("mrs %0, primask\n cpsid i" : "=r"(primask)::"memory"); }
        ~Masked() { __asm volatile("msr primask, %0" ::"r"(primask) : "memory"); }
    };

    static constexpr std::uintptr_t base = 0x4001'6000, pma = 0x4001'6400;
    static volatile std::uint32_t& reg(std::uintptr_t offset) noexcept {
        return *reinterpret_cast<volatile std::uint32_t*>(base + offset);
    }
    std::uint32_t chep(unsigned n) noexcept { return reg(4 * n); }
    void set_chep(unsigned n, std::uint32_t v) noexcept { reg(4 * n) = v; }
    std::uint32_t cntr() noexcept { return reg(0x40); }
    void set_cntr(std::uint32_t v) noexcept { reg(0x40) = v; }
    std::uint32_t istr() noexcept { return reg(0x44); }
    void set_istr(std::uint32_t v) noexcept { reg(0x44) = v; }
    void set_daddr(std::uint32_t v) noexcept { reg(0x4C) = v; }
    std::uint32_t bcdr() noexcept { return reg(0x58); }
    void set_bcdr(std::uint32_t v) noexcept { reg(0x58) = v; }
    std::uint32_t pma_read(std::size_t offset) noexcept {
        return *reinterpret_cast<volatile std::uint32_t*>(pma + offset);
    }
    void pma_write(std::size_t offset, std::uint32_t v) noexcept {
        *reinterpret_cast<volatile std::uint32_t*>(pma + offset) = v;
    }
};

template <class Regs = UsbHardware>
using UsbDrdFs = stm32::UsbDevice<Regs>;

}  // namespace rtos::port::stm32h5
