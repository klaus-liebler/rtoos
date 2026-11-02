// SPDX-License-Identifier: MIT
//
// STM32G431 DMA1/DMA2 with the DMAMUX request router: single transfers between memory and a
// peripheral, paced by the peripheral's request (RM0440 chapters DMA, DMAMUX; registers, bits:
// SVD STM32G431 v1.8; DMAMUX channel addresses: ST's stm32g431xx.h - DMA1 channel k uses DMAMUX
// channel k-1 at 0x40020800 + 4 (k-1), DMA2 channel k DMAMUX channel 5+k at 0x40020820 + 4 (k-1);
// request numbers: ST's stm32g4xx_ll_dmamux.h, LL_DMAMUX_REQ_*; IRQ numbers: stm32g431xx.h).
//
//   // WS2812 bits as compare values into TIM16 CCR1, one per TIM16_CH1 request
//   using Leds = DmaChannel<{.controller = 1, .channel = 3, .request = DmaRequest::Tim16Ch1,
//                            .direction = DmaDirection::MemoryToPeripheral,
//                            .memory_bytes = 2, .peripheral_bytes = 2}>;
//   Leds::init();
//   Leds::start(Tim16::compare_address<1>(), bits.data(), bits.size());
//
// The same interface as the STM32H5 GPDMA driver (init, start, done, failed, busy, remaining,
// clear, stop), so the shared drivers (stm32::Ucpd) take either. The configuration is a template
// argument: CCR (widths, increments, direction, priority) and the DMAMUX request are computed
// at compile time. A transfer moves `items` data (CNDTR counts data items, at most 65535).
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/stm32g4/chip.hpp"
#include "rtos/port/stm32g4/rcc.hpp"

namespace rtos::port::stm32g4 {

enum class DmaRequest : std::uint8_t {
    Adc1 = 5,
    Spi1Rx = 10, Spi1Tx = 11, Spi2Rx = 12, Spi2Tx = 13, Spi3Rx = 14, Spi3Tx = 15,
    I2c1Rx = 16, I2c1Tx = 17, I2c2Rx = 18, I2c2Tx = 19, I2c3Rx = 20, I2c3Tx = 21,
    Usart1Rx = 24, Usart1Tx = 25, Usart2Rx = 26, Usart2Tx = 27, Usart3Rx = 28, Usart3Tx = 29,
    Uart4Rx = 30, Uart4Tx = 31, Lpuart1Rx = 34, Lpuart1Tx = 35, Adc2 = 36,
    Tim1Ch1 = 42, Tim1Ch2 = 43, Tim1Ch3 = 44, Tim1Ch4 = 45, Tim1Up = 46,
    Tim8Ch1 = 49, Tim8Ch2 = 50, Tim8Ch3 = 51, Tim8Ch4 = 52, Tim8Up = 53,
    Tim2Ch1 = 56, Tim2Ch2 = 57, Tim2Ch3 = 58, Tim2Ch4 = 59, Tim2Up = 60,
    Tim3Ch1 = 61, Tim3Ch2 = 62, Tim3Ch3 = 63, Tim3Ch4 = 64, Tim3Up = 65,
    Tim4Ch1 = 67, Tim4Ch2 = 68, Tim4Ch3 = 69, Tim4Ch4 = 70, Tim4Up = 71,
    Tim15Ch1 = 78, Tim15Up = 79,
    Tim16Ch1 = 82, Tim16Up = 83, Tim17Ch1 = 84, Tim17Up = 85,
    Ucpd1Rx = 114, Ucpd1Tx = 115,
};

enum class DmaDirection : std::uint8_t { MemoryToPeripheral, PeripheralToMemory };

struct DmaConfig {
    unsigned controller = 1;           ///< DMA1 or DMA2
    unsigned channel;                  ///< 1..6 (one user each)
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
}  // namespace dma_detail

/// CCR for `c` without EN and the interrupt enables.
[[nodiscard]] constexpr std::uint32_t dma_ccr(const DmaConfig& c) noexcept {
    using namespace dma_detail;
    return (c.direction == DmaDirection::MemoryToPeripheral ? ccr_dir : 0u) | (c.memory_increment ? ccr_minc : 0u) |
           (log2(c.peripheral_bytes) << ccr_psize_pos) | (log2(c.memory_bytes) << ccr_msize_pos) |
           (static_cast<std::uint32_t>(c.priority) << ccr_pl_pos);
}

/// DMAMUX channel register of DMA `controller` channel `channel` (stm32g431xx.h).
[[nodiscard]] constexpr std::uintptr_t dmamux_channel_offset(unsigned controller, unsigned channel) noexcept {
    return 0x0002'0800 + (controller == 1 ? 0x00u : 0x20u) + 4 * (channel - 1);
}

template <DmaConfig Config>
class DmaChannel {
    static_assert(Config.controller == 1 || Config.controller == 2, "the STM32G431 has DMA1 and DMA2");
    static_assert(Config.channel >= 1 && Config.channel <= 6, "the STM32G431's DMAs have channels 1..6");
    static_assert((Config.memory_bytes == 1 || Config.memory_bytes == 2 || Config.memory_bytes == 4) &&
                      (Config.peripheral_bytes == 1 || Config.peripheral_bytes == 2 ||
                       Config.peripheral_bytes == 4),
                  "data widths 1, 2 or 4 bytes");
    static_assert(Config.priority <= 3, "priority 0..3");
    static constexpr std::uintptr_t dma = Config.controller == 1 ? 0x0002'0000 : 0x0002'0400;
    static constexpr std::uintptr_t offset = dma + 0x08 + 0x14 * (Config.channel - 1);
    static constexpr std::uint32_t ccr = dma_ccr(Config);
    static constexpr unsigned flag_shift = 4 * (Config.channel - 1);

public:
    static constexpr std::uint8_t irq = Config.controller == 1 ? static_cast<std::uint8_t>(10 + Config.channel)
                                                              : (Config.channel == 6 ? std::uint8_t{97}
                                                                                     : static_cast<std::uint8_t>(55 + Config.channel));

    static void init() noexcept {
        rcc::enable(rcc::gate::dmamux1);
        rcc::enable(Config.controller == 1 ? rcc::gate::dma1 : rcc::gate::dma2);
        stop();
        MUX::write(static_cast<std::uint32_t>(Config.request));
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
        IFCR::write(flags_all << flag_shift);
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
    /// Data items still to move.
    [[nodiscard]] static std::uint32_t remaining() noexcept { return CNDTR::read() & 0xFFFFu; }
    /// In the ISR: acknowledge the flags.
    static void clear() noexcept { IFCR::write(dma_detail::flags_all << flag_shift); }

    /// Abort: disable the channel (it becomes idle, the registers writable again).
    static void stop() noexcept {
        CCR::write(0);
        IFCR::write(dma_detail::flags_all << flag_shift);
    }

private:
    using ISR = PReg<dma + 0x00>;
    using IFCR = PReg<dma + 0x04>;
    using CCR = PReg<offset + 0x00>;
    using CNDTR = PReg<offset + 0x04>;
    using CPAR = PReg<offset + 0x08>;
    using CMAR = PReg<offset + 0x0C>;
    using MUX = PReg<dmamux_channel_offset(Config.controller, Config.channel)>;
};

}  // namespace rtos::port::stm32g4
