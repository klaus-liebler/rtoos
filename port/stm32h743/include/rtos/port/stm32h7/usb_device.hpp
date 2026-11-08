// SPDX-License-Identifier: MIT
//
// STM32H743 USB2_OTG_FS in device mode (full speed, embedded PHY on PA11 = DM, PA12 = DP, AF10):
// the shared OTG driver (stm32/usb_otg_device.hpp) on the H743's registers. Addresses, IRQ, bits:
// ST's stm32h743xx.h (USB2_OTG_FS_PERIPH_BASE 0x40080000, OTG_FS_IRQn 101, RCC/PWR/CRS bits);
// clock and supply after ST's HAL (stm32h7xx_hal_rcc_ex.h, stm32h7xx_hal_pwr_ex.c).
//
//   using Usb = UsbOtgFs<board::hclk_hz, board::sysclk_hz>;
//   Usb usb;                             // UsbOtgHardware<...>::enable_clocks() first
//   rtos::usb::Device<Usb> device{usb, descriptors, functions};
//   extern "C" void OTG_FS_IRQHandler() { usb.on_irq(); wake the network thread }
//
// Kernel clock: HSI48 (RCC_D2CCIP2R.USBSEL = 3, RCC_USBCLKSOURCE_HSI48), trimmed by the CRS
// against the host's start-of-frame packets of USB2 (CRS_CFGR.SYNCSRC = 3,
// RCC_CRS_SYNC_SOURCE_USB2 - the reset value 2 is USB1, the OTG_HS). Supply: the USB transceiver
// runs from VDD33USB; PWR_CR3.USB33DEN switches its level detector on, USB33RDY tells it is
// valid (HAL_PWREx_EnableUSBVoltageDetector). Sleep: the FS core has no ULPI PHY, so its ULPI
// clock is not requested in sleep mode (RCC_AHB1LPENR.USB2OTGFSULPILPEN = 0, ST's
// __HAL_RCC_USB2_OTG_FS_ULPI_CLK_SLEEP_DISABLE) - the kernel idles in WFI.
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/cortex_m/cycle_counter.hpp"
#include "rtos/port/stm32/usb_otg_device.hpp"
#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/pwr.hpp"
#include "rtos/port/stm32h7/rcc.hpp"

namespace rtos::port::stm32h7 {

using stm32::UsbDeviceStats;

/// The real registers of USB2_OTG_FS. `HclkHz`: the AHB clock - GUSBCFG.TRDT (ST's
/// USB_SetTurnaroundTime: 6 from 32 MHz on); `CpuHz`: the core clock (delays, DWT).
template <std::uint32_t HclkHz, std::uint32_t CpuHz>
struct UsbOtgHardware {
    static_assert(HclkHz >= 32'000'000, "AHB clock below 32 MHz: GUSBCFG.TRDT would need ST's table");
    static constexpr std::uint8_t irq = 101;  // OTG_FS_IRQn
    static constexpr std::uint32_t trdt = 6;
    static constexpr std::uint32_t tx_fifo_words = 16;  ///< one 64-byte packet per IN endpoint
    static constexpr std::uintptr_t base = 0x4008'0000;

    /// VDD33USB detector, HSI48 + CRS (USB2 SOF), USBSEL = HSI48, the controller's clock.
    static void enable_clocks() noexcept {
        constexpr std::uint32_t cr3_usb33den = 1u << 24, cr3_usb33rdy = 1u << 26;
        pwr::CR3::write(pwr::CR3::read() | cr3_usb33den);
        for (int i = 0; i < 1'000'000 && (pwr::CR3::read() & cr3_usb33rdy) == 0; ++i) {
        }
        rcc::enable_hsi48();
        using D2CCIP2R = Reg<rcc::base + 0x054>;
        D2CCIP2R::write(D2CCIP2R::read() | (3u << 20));  // USBSEL = HSI48
        rcc::enable(rcc::gate::crs);
        using CRS_CR = Reg<0x4000'8400>;
        using CRS_CFGR = Reg<0x4000'8404>;
        CRS_CR::write(CRS_CR::read() & ~(1u << 5));                          // CEN off: CFGR writable
        CRS_CFGR::write((CRS_CFGR::read() & ~(3u << 28)) | (3u << 28));      // SYNCSRC = USB2 SOF
        CRS_CR::write(CRS_CR::read() | (1u << 5) | (1u << 6));               // CEN, AUTOTRIMEN
        using AHB1LPENR = Reg<rcc::base + 0x100>;
        AHB1LPENR::write(AHB1LPENR::read() & ~(1u << 28));  // USB2OTGFSULPILPEN
        rcc::enable(rcc::gate::usb2otgfs);
    }

    /// Interrupts off while the thread changes a register the ISR also writes.
    struct Masked {
        std::uint32_t primask;
        Masked() noexcept { __asm volatile("mrs %0, primask\n cpsid i" : "=r"(primask)::"memory"); }
        ~Masked() { __asm volatile("msr primask, %0" ::"r"(primask) : "memory"); }
    };

    static volatile std::uint32_t& reg(std::uint32_t offset) noexcept {
        return *reinterpret_cast<volatile std::uint32_t*>(base + offset);
    }
    std::uint32_t read(std::uint32_t offset) noexcept { return reg(offset); }
    void write(std::uint32_t offset, std::uint32_t v) noexcept { reg(offset) = v; }
    /// A word of the RX FIFO (any address of FIFO 0 pops).
    std::uint32_t rx_pop() noexcept { return reg(0x1000); }
    /// A word into the TX FIFO of IN endpoint n.
    void tx_push(unsigned n, std::uint32_t w) noexcept { reg(0x1000 * (n + 1)) = w; }
    void delay_ms(std::uint32_t ms) noexcept {
        const std::uint32_t start = cortex_m::CycleCounter::now();
        while (cortex_m::CycleCounter::now() - start < ms * (CpuHz / 1000)) {
        }
    }
};

template <std::uint32_t HclkHz, std::uint32_t CpuHz, class Regs = UsbOtgHardware<HclkHz, CpuHz>>
using UsbOtgFs = stm32::UsbOtgDevice<Regs>;

}  // namespace rtos::port::stm32h7
