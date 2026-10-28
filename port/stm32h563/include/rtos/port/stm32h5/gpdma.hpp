// SPDX-License-Identifier: MIT
//
// STM32H563 GPDMA1: linear single-block transfers between memory and a peripheral, paced by
// the peripheral's hardware request (RM0481 chapter GPDMA; registers, bits, IRQs: SVD v1.8;
// request numbers: ST's stm32h5xx_hal_dma.h, GPDMA1_REQUEST_*).
//
//   // WS2812 bits as compare values into TIM15 CCR1, one per TIM15_CH1 request
//   using Leds = DmaChannel<{.channel = 2, .request = DmaRequest::Tim15Ch1,
//                            .direction = DmaDirection::MemoryToPeripheral,
//                            .memory_bytes = 2, .peripheral_bytes = 2}>;
//   Leds::init();
//   Leds::start(Tim15::compare_address<1>(), bits.data(), bits.size());
//   ... Leds::done() ...
//
// The configuration is a template argument: CxTR1 (widths, increments) and CxTR2 (request,
// which side the request paces) are computed and checked at compile time. A transfer moves
// `items` data of the memory width; the GPDMA counts bytes of the source (CxBR1.BNDT, at most
// 65535). The memory must be reachable by the DMA and, with caches, coherent: rtoos keeps DMA
// buffers in the non-cacheable region (RTOS_DMA_BUFFER).
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

enum class DmaRequest : std::uint8_t {
    Spi2Rx = 8, Spi2Tx = 9,
    Usart3Rx = 25, Usart3Tx = 26, Uart5Rx = 29, Uart5Tx = 30,
    Tim2Ch1 = 72, Tim2Ch2 = 73, Tim2Ch3 = 74, Tim2Ch4 = 75, Tim2Up = 76,
    Tim3Ch1 = 77, Tim3Ch2 = 78, Tim3Ch3 = 79, Tim3Ch4 = 80, Tim3Up = 81,
    Tim4Ch1 = 83, Tim4Ch2 = 84, Tim4Ch3 = 85, Tim4Ch4 = 86, Tim4Up = 87,
    Tim15Ch1 = 94, Tim15Up = 95,
    Tim16Ch1 = 98, Tim16Up = 99, Tim17Ch1 = 100, Tim17Up = 101,
    Ucpd1Rx = 112, Ucpd1Tx = 113,
};

enum class DmaDirection : std::uint8_t { MemoryToPeripheral, PeripheralToMemory };

struct DmaConfig {
    unsigned channel;              ///< GPDMA1 channel 0..7 (one user each)
    DmaRequest request;
    DmaDirection direction;
    std::uint8_t memory_bytes = 1;     ///< 1, 2 or 4
    std::uint8_t peripheral_bytes = 1; ///< 1, 2 or 4
    bool memory_increment = true;
    std::uint8_t priority = 0;         ///< 0 (low, weighted) .. 3 (high)
};

namespace gpdma_detail {
[[nodiscard]] constexpr std::uint32_t log2(std::uint8_t bytes) noexcept { return bytes == 4 ? 2 : (bytes == 2 ? 1 : 0); }
inline constexpr std::uint32_t tr1_sinc = 1u << 3, tr1_dinc = 1u << 19;
inline constexpr std::uint32_t tr2_dreq = 1u << 10;
inline constexpr std::uint32_t cr_en = 1u << 0, cr_reset = 1u << 1, cr_tcie = 1u << 8, cr_dteie = 1u << 10;
inline constexpr std::uint32_t sr_idlef = 1u << 0, sr_tcf = 1u << 8, sr_dtef = 1u << 10, sr_usef = 1u << 12;
inline constexpr std::uint32_t fcr_all = 0x7F00u;  // TCF .. TOF
}  // namespace gpdma_detail

/// CxTR1 for `c`: source = memory or peripheral, with its width and increment.
[[nodiscard]] constexpr std::uint32_t dma_tr1(const DmaConfig& c) noexcept {
    using namespace gpdma_detail;
    const bool to_peripheral = c.direction == DmaDirection::MemoryToPeripheral;
    const std::uint32_t src_width = log2(to_peripheral ? c.memory_bytes : c.peripheral_bytes);
    const std::uint32_t dst_width = log2(to_peripheral ? c.peripheral_bytes : c.memory_bytes);
    const std::uint32_t src_inc = to_peripheral && c.memory_increment ? tr1_sinc : 0u;
    const std::uint32_t dst_inc = !to_peripheral && c.memory_increment ? tr1_dinc : 0u;
    return src_width | src_inc | (dst_width << 16) | dst_inc;  // PAM 0: right-aligned, zero-padded
}

/// CxTR2 for `c`: the request; DREQ when the destination (the peripheral) paces the transfer.
[[nodiscard]] constexpr std::uint32_t dma_tr2(const DmaConfig& c) noexcept {
    return static_cast<std::uint32_t>(c.request) |
           (c.direction == DmaDirection::MemoryToPeripheral ? gpdma_detail::tr2_dreq : 0u);
}

template <DmaConfig Config>
class DmaChannel {
    static_assert(Config.channel < 8, "GPDMA1 has channels 0..7");
    static_assert((Config.memory_bytes == 1 || Config.memory_bytes == 2 || Config.memory_bytes == 4) &&
                      (Config.peripheral_bytes == 1 || Config.peripheral_bytes == 2 ||
                       Config.peripheral_bytes == 4),
                  "data widths 1, 2 or 4 bytes");
    static_assert(Config.priority <= 3, "priority 0..3");
    static constexpr std::uintptr_t offset = 0x0002'0000 + 0x80 * Config.channel;
    static constexpr std::uint32_t tr1 = dma_tr1(Config), tr2 = dma_tr2(Config);
    static constexpr std::uint32_t source_bytes =
        Config.direction == DmaDirection::MemoryToPeripheral ? Config.memory_bytes : Config.peripheral_bytes;

public:
    static constexpr std::uint8_t irq = static_cast<std::uint8_t>(27 + Config.channel);

    static void init() noexcept {
        rcc::enable(rcc::gate::gpdma1);
        stop();
    }

    /// Move `items` data of the memory width between `memory` and the register at
    /// `peripheral_address`; false if the channel is still busy or `items` is too many.
    static bool start(std::uintptr_t peripheral_address, const void* memory, std::size_t items,
                      bool interrupt = false) noexcept {
        using namespace gpdma_detail;
        const std::size_t bytes = items * source_bytes;  // BNDT counts source bytes
        if (bytes == 0 || bytes > 0xFFFF || busy()) {
            return false;
        }
        const auto mem = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(memory));
        const auto per = static_cast<std::uint32_t>(peripheral_address);
        const bool to_peripheral = Config.direction == DmaDirection::MemoryToPeripheral;
        FCR::write(fcr_all);
        TR1::write(tr1);
        TR2::write(tr2);
        BR1::write(static_cast<std::uint32_t>(bytes));
        SAR::write(to_peripheral ? mem : per);
        DAR::write(to_peripheral ? per : mem);
        LLR::write(0);
        CR::write((static_cast<std::uint32_t>(Config.priority) << 22) | (interrupt ? cr_tcie | cr_dteie : 0u) | cr_en);
        return true;
    }

    /// The last transfer completed (TCF).
    [[nodiscard]] static bool done() noexcept { return (SR::read() & gpdma_detail::sr_tcf) != 0; }
    /// A bus or setting error stopped the channel.
    [[nodiscard]] static bool failed() noexcept {
        return (SR::read() & (gpdma_detail::sr_dtef | gpdma_detail::sr_usef)) != 0;
    }
    [[nodiscard]] static bool busy() noexcept { return (SR::read() & gpdma_detail::sr_idlef) == 0; }
    /// Source bytes still to move.
    [[nodiscard]] static std::uint32_t remaining() noexcept { return BR1::read() & 0xFFFFu; }
    /// In the ISR: acknowledge the flags.
    static void clear() noexcept { FCR::write(gpdma_detail::fcr_all); }

    /// Abort: reset the channel (it becomes idle).
    static void stop() noexcept {
        using namespace gpdma_detail;
        if (busy()) {
            CR::write(cr_reset);
        }
        CR::write(0);
        FCR::write(fcr_all);
    }

private:
    using FCR = PReg<offset + 0x5C>;
    using SR = PReg<offset + 0x60>;
    using CR = PReg<offset + 0x64>;
    using TR1 = PReg<offset + 0x90>;
    using TR2 = PReg<offset + 0x94>;
    using BR1 = PReg<offset + 0x98>;
    using SAR = PReg<offset + 0x9C>;
    using DAR = PReg<offset + 0xA0>;
    using LLR = PReg<offset + 0xCC>;
};

}  // namespace rtos::port::stm32h5
