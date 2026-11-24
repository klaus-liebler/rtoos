// SPDX-License-Identifier: MIT
//
// STM32F103 DMA1: single transfers between memory and a peripheral, paced by the peripheral's
// request (RM0008 rev 21 chapter 13; registers as on the G4's DMA - ISR, IFCR, per channel CCR,
// CNDTR, CPAR, CMAR at 0x08 + 0x14 (channel - 1) - but without DMAMUX: every request is wired to
// one fixed channel, RM0008 table 78). The configuration names the request; its channel follows
// at compile time.
//
//   // WS2812 bits as compare values into TIM1 CCR1, one per TIM1 update
//   using Leds = DmaChannel<{.request = DmaRequest::Tim1Up, .direction = DmaDirection::MemoryToPeripheral,
//                            .memory_bytes = 2, .peripheral_bytes = 2}>;            // channel 5
//   Leds::init();
//   Leds::start(Tim1::compare_address<1>(), slots.data(), slots.size());
//
// The same interface as the G4 DMA and the H5 GPDMA drivers (init, start, done, failed, busy,
// remaining, clear, stop). Two drivers whose requests share a channel cannot run together: the
// channel is one resource (RM0008: "only one request may be activated at a time").
//
// Silicon errata (ES096 rev 15):
//   * 2.4.1 "DMA disable failure and error flag omission upon simultaneous transfer error and
//     global flag clear": the driver clears TCIF/HTIF/TEIF only, GIF only with the channel off.
//   * 2.4.2 "Byte and half-word accesses not supported": every register access is 32 bit.
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/cortex_m/registers.hpp"
#include "rtos/port/stm32f1/chip.hpp"
#include "rtos/port/stm32f1/rcc.hpp"

namespace rtos::port::stm32f1 {

/// DMA1 requests of the STM32F103x8/xB (RM0008 table 78), each with its channel in the value's
/// upper nibble.
enum class DmaRequest : std::uint8_t {
    Adc1 = 0x10, Tim2Ch3 = 0x11, Tim4Ch1 = 0x12,
    Spi1Rx = 0x20, Usart3Tx = 0x21, Tim1Ch1 = 0x22, Tim2Up = 0x23, Tim3Ch3 = 0x24,
    Spi1Tx = 0x30, Usart3Rx = 0x31, Tim1Ch2 = 0x32, Tim3Ch4 = 0x33, Tim3Up = 0x34,
    Spi2Rx = 0x40, Usart1Tx = 0x41, I2c2Tx = 0x42, Tim1Ch4 = 0x43, Tim1Trig = 0x44, Tim4Ch2 = 0x45,
    Spi2Tx = 0x50, Usart1Rx = 0x51, I2c2Rx = 0x52, Tim1Up = 0x53, Tim2Ch1 = 0x54, Tim4Ch3 = 0x55,
    Usart2Rx = 0x60, I2c1Tx = 0x61, Tim1Ch3 = 0x62, Tim3Ch1 = 0x63, Tim3Trig = 0x64,
    Usart2Tx = 0x70, I2c1Rx = 0x71, Tim2Ch2 = 0x72, Tim2Ch4 = 0x73, Tim4Up = 0x74,
};

/// The DMA1 channel (1..7) a request is wired to.
[[nodiscard]] constexpr unsigned dma_channel(DmaRequest r) noexcept { return static_cast<unsigned>(r) >> 4; }

enum class DmaDirection : std::uint8_t { MemoryToPeripheral, PeripheralToMemory };

struct DmaConfig {
    DmaRequest request;
    DmaDirection direction;
    std::uint8_t memory_bytes = 1;     ///< 1, 2 or 4
    std::uint8_t peripheral_bytes = 1; ///< 1, 2 or 4
    bool memory_increment = true;
    std::uint8_t priority = 0;         ///< 0 (low) .. 3 (very high)
};

namespace dma_detail {
[[nodiscard]] constexpr std::uint32_t log2(std::uint8_t bytes) noexcept { return bytes == 4 ? 2 : (bytes == 2 ? 1 : 0); }
inline constexpr std::uint32_t ccr_en = 1u << 0, ccr_tcie = 1u << 1, ccr_teie = 1u << 3, ccr_dir = 1u << 4,
                               ccr_minc = 1u << 7;
inline constexpr std::uint32_t ccr_psize_pos = 8, ccr_msize_pos = 10, ccr_pl_pos = 12;
// ISR / IFCR: 4 bits per channel (GIF, TCIF, HTIF, TEIF) at 4 (channel - 1)
inline constexpr std::uint32_t isr_tcif = 1u << 1, isr_teif = 1u << 3, flags_all = 0xFu;
/// TCIF, HTIF, TEIF without GIF (ES096 2.4.1)
inline constexpr std::uint32_t flags_events = 0xEu;
inline constexpr std::uintptr_t ahbenr = periph_base + 0x0002'1014;  // RCC_AHBENR (DMA1EN = bit 0)
}  // namespace dma_detail

/// CCR for `c` without EN and the interrupt enables.
[[nodiscard]] constexpr std::uint32_t dma_ccr(const DmaConfig& c) noexcept {
    using namespace dma_detail;
    return (c.direction == DmaDirection::MemoryToPeripheral ? ccr_dir : 0u) | (c.memory_increment ? ccr_minc : 0u) |
           (log2(c.peripheral_bytes) << ccr_psize_pos) | (log2(c.memory_bytes) << ccr_msize_pos) |
           (static_cast<std::uint32_t>(c.priority) << ccr_pl_pos);
}

template <DmaConfig Config>
class DmaChannel {
    static constexpr unsigned channel = dma_channel(Config.request);
    static_assert(channel >= 1 && channel <= 7, "DMA1 has channels 1..7");
    static_assert((Config.memory_bytes == 1 || Config.memory_bytes == 2 || Config.memory_bytes == 4) &&
                      (Config.peripheral_bytes == 1 || Config.peripheral_bytes == 2 ||
                       Config.peripheral_bytes == 4),
                  "data widths 1, 2 or 4 bytes");
    static_assert(Config.priority <= 3, "priority 0..3");
    static constexpr std::uintptr_t dma = 0x0002'0000;  // DMA1
    static constexpr std::uintptr_t offset = dma + 0x08 + 0x14 * (channel - 1);
    static constexpr std::uint32_t ccr = dma_ccr(Config);
    static constexpr unsigned flag_shift = 4 * (channel - 1);

public:
    static constexpr unsigned channel_number = channel;
    /// DMA1_Channel1..7_IRQn = 11..17
    static constexpr std::uint8_t irq = static_cast<std::uint8_t>(10 + channel);

    static void init() noexcept {
        auto& enr = *reinterpret_cast<volatile std::uint32_t*>(dma_detail::ahbenr);
        enr = enr | 1u;  // DMA1EN (AHB: no reset register on the medium density parts)
        [[maybe_unused]] const std::uint32_t settle = enr;  // RM0008: wait for the clock
        stop();
    }

    /// Move `items` data between `memory` and the register at `peripheral_address`; false if
    /// the channel is still busy or `items` is 0 or above 65535.
    static bool start(std::uintptr_t peripheral_address, const void* memory, std::size_t items,
                      bool interrupt = false) noexcept {
        using namespace dma_detail;
        if (items == 0 || items > 0xFFFF || busy()) {
            return false;
        }
        CCR::write(0);
        IFCR::write(flags_all << flag_shift);  // the channel is off: GIF may go too (ES096 2.4.1)
        CPAR::write(static_cast<std::uint32_t>(peripheral_address));
        CMAR::write(static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(memory)));
        CNDTR::write(static_cast<std::uint32_t>(items));
        CCR::write(ccr | (interrupt ? ccr_tcie | ccr_teie : 0u) | ccr_en);
        return true;
    }

    /// The last transfer completed (TCIF).
    [[nodiscard]] static bool done() noexcept { return ((ISR::read() >> flag_shift) & dma_detail::isr_tcif) != 0; }
    /// A bus error stopped the channel (TEIF; the hardware cleared EN).
    [[nodiscard]] static bool failed() noexcept { return ((ISR::read() >> flag_shift) & dma_detail::isr_teif) != 0; }
    /// Enabled and data left to move.
    [[nodiscard]] static bool busy() noexcept {
        return (CCR::read() & dma_detail::ccr_en) != 0 && (CNDTR::read() & 0xFFFFu) != 0;
    }
    [[nodiscard]] static std::size_t remaining() noexcept { return CNDTR::read() & 0xFFFFu; }
    /// In the ISR: acknowledge the events (never GIF while the channel may run, ES096 2.4.1).
    static void clear() noexcept { IFCR::write(dma_detail::flags_events << flag_shift); }
    static void stop() noexcept {
        CCR::write(0);
        IFCR::write(dma_detail::flags_all << flag_shift);
    }

private:
    using ISR = cortex_m::Reg<periph_base + dma + 0x00>;
    using IFCR = cortex_m::Reg<periph_base + dma + 0x04>;
    using CCR = cortex_m::Reg<periph_base + offset + 0x00>;
    using CNDTR = cortex_m::Reg<periph_base + offset + 0x04>;
    using CPAR = cortex_m::Reg<periph_base + offset + 0x08>;
    using CMAR = cortex_m::Reg<periph_base + offset + 0x0C>;
};

}  // namespace rtos::port::stm32f1
