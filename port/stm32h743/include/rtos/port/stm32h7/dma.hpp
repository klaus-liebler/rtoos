// SPDX-License-Identifier: MIT
//
// STM32H743 DMA1/DMA2 streams with the DMAMUX1 request router: single transfers between memory
// and a peripheral, paced by the peripheral's request (RM0433 chapters DMA, DMAMUX; registers,
// bits: SVD STM32H743 v2.4; DMAMUX1 channels: ST's stm32h743xx.h - DMA1 stream s uses DMAMUX1
// channel s, DMA2 stream s channel 8 + s; request numbers: ST's stm32h7xx_ll_dmamux.h,
// LL_DMAMUX1_REQ_*; IRQ numbers: stm32h743xx.h).
//
//   // WS2812 bits as compare values into TIM15 CCR1, one per TIM15_CH1 request
//   using Leds = DmaChannel<{.controller = 1, .stream = 2, .request = DmaRequest::Tim15Ch1,
//                            .direction = DmaDirection::MemoryToPeripheral,
//                            .memory_bytes = 2, .peripheral_bytes = 2}>;
//   Leds::init();
//   Leds::start(Tim15::compare_address<1>(), bits.data(), bits.size());
//
// The same interface as the STM32H5 GPDMA and STM32G4 DMA drivers (init, start, done, failed,
// busy, remaining, clear, stop). The streams run in direct mode (no FIFO): memory and peripheral
// have the same data width - unlike the H5's GPDMA, this DMA does not widen data (its FIFO
// packs two half-words into one word instead), so different widths do not compile. A transfer
// moves `items` data (SxNDTR, at most 65535).
//
// Memory: DMA1/DMA2 reach the AXI SRAM and SRAM1-4, not the DTCM (RM0433 bus matrix). With the
// data cache on, the buffers must be non-cacheable (RTOS_DMA_BUFFER, mpu.hpp) or the caller
// cleans/invalidates them (cm7/cache.hpp).
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/rcc.hpp"

namespace rtos::port::stm32h7 {

enum class DmaRequest : std::uint8_t {
    Adc1 = 9, Adc2 = 10,
    Tim1Ch1 = 11, Tim1Ch2 = 12, Tim1Ch3 = 13, Tim1Ch4 = 14, Tim1Up = 15,
    Tim2Ch1 = 18, Tim2Ch2 = 19, Tim2Ch3 = 20, Tim2Ch4 = 21, Tim2Up = 22,
    Tim3Ch1 = 23, Tim3Ch2 = 24, Tim3Ch3 = 25, Tim3Ch4 = 26, Tim3Up = 27,
    Tim4Ch1 = 29, Tim4Ch2 = 30, Tim4Ch3 = 31, Tim4Up = 32,
    I2c1Rx = 33, I2c1Tx = 34, I2c2Rx = 35, I2c2Tx = 36,
    Spi1Rx = 37, Spi1Tx = 38, Spi2Rx = 39, Spi2Tx = 40,
    Usart1Rx = 41, Usart1Tx = 42, Usart2Rx = 43, Usart2Tx = 44, Usart3Rx = 45, Usart3Tx = 46,
    Tim8Ch1 = 47, Tim8Ch2 = 48, Tim8Ch3 = 49, Tim8Ch4 = 50, Tim8Up = 51,
    Tim5Ch1 = 55, Tim5Ch2 = 56, Tim5Ch3 = 57, Tim5Ch4 = 58, Tim5Up = 59,
    Spi3Rx = 61, Spi3Tx = 62, Uart4Rx = 63, Uart4Tx = 64, Uart5Rx = 65, Uart5Tx = 66,
    Usart6Rx = 71, Usart6Tx = 72, I2c3Rx = 73, I2c3Tx = 74,
    Uart7Rx = 79, Uart7Tx = 80, Uart8Rx = 81, Uart8Tx = 82,
    Spi4Rx = 83, Spi4Tx = 84, Spi5Rx = 85, Spi5Tx = 86,
    Tim15Ch1 = 105, Tim15Up = 106, Tim16Ch1 = 109, Tim16Up = 110, Tim17Ch1 = 111, Tim17Up = 112,
    Adc3 = 115,
};

enum class DmaDirection : std::uint8_t { MemoryToPeripheral, PeripheralToMemory };

struct DmaConfig {
    unsigned controller = 1;           ///< DMA1 or DMA2
    unsigned stream;                   ///< 0..7 (one user each)
    DmaRequest request;
    DmaDirection direction;
    std::uint8_t memory_bytes = 1;     ///< 1, 2 or 4; = peripheral_bytes (direct mode)
    std::uint8_t peripheral_bytes = 1; ///< 1, 2 or 4
    bool memory_increment = true;
    std::uint8_t priority = 0;         ///< 0 (low) .. 3 (very high)
};

namespace dma_detail {
[[nodiscard]] constexpr std::uint32_t log2(std::uint8_t bytes) noexcept { return bytes == 4 ? 2 : (bytes == 2 ? 1 : 0); }
inline constexpr std::uint32_t cr_en = 1u << 0, cr_teie = 1u << 2, cr_tcie = 1u << 4, cr_dir_m2p = 1u << 6,
                               cr_minc = 1u << 10;
inline constexpr std::uint32_t cr_psize_pos = 11, cr_msize_pos = 13, cr_pl_pos = 16;
// LISR/HISR, LIFCR/HIFCR: 6 bits per stream (FEIF, -, DMEIF, TEIF, HTIF, TCIF) at 0, 6, 16, 22
inline constexpr std::uint32_t flag_teif = 1u << 3, flag_tcif = 1u << 5, flags_all = 0x3Du;
[[nodiscard]] constexpr unsigned flag_shift(unsigned stream) noexcept {
    constexpr unsigned shifts[4] = {0, 6, 16, 22};
    return shifts[stream % 4];
}
}  // namespace dma_detail

/// SxCR for `c` without EN and the interrupt enables.
[[nodiscard]] constexpr std::uint32_t dma_cr(const DmaConfig& c) noexcept {
    using namespace dma_detail;
    return (c.direction == DmaDirection::MemoryToPeripheral ? cr_dir_m2p : 0u) | (c.memory_increment ? cr_minc : 0u) |
           (log2(c.peripheral_bytes) << cr_psize_pos) | (log2(c.memory_bytes) << cr_msize_pos) |
           (static_cast<std::uint32_t>(c.priority) << cr_pl_pos);
}

/// DMAMUX1 channel register of DMA `controller` stream `stream` (stm32h743xx.h).
[[nodiscard]] constexpr std::uintptr_t dmamux_channel_address(unsigned controller, unsigned stream) noexcept {
    return 0x4002'0800 + 4 * ((controller == 1 ? 0u : 8u) + stream);
}

/// NVIC line of DMA `controller` stream `stream` (stm32h743xx.h DMAx_Streamy_IRQn).
[[nodiscard]] constexpr std::uint8_t dma_irq(unsigned controller, unsigned stream) noexcept {
    if (controller == 1) {
        return stream == 7 ? std::uint8_t{47} : static_cast<std::uint8_t>(11 + stream);
    }
    return stream <= 4 ? static_cast<std::uint8_t>(56 + stream) : static_cast<std::uint8_t>(68 + stream - 5);
}

template <DmaConfig Config>
class DmaChannel {
    static_assert(Config.controller == 1 || Config.controller == 2, "the STM32H743 has DMA1 and DMA2 (MDMA, BDMA: not here)");
    static_assert(Config.stream <= 7, "the streams are 0..7");
    static_assert((Config.memory_bytes == 1 || Config.memory_bytes == 2 || Config.memory_bytes == 4) &&
                      Config.memory_bytes == Config.peripheral_bytes,
                  "direct mode: memory and peripheral data widths 1, 2 or 4 bytes, the same (the H7's DMA packs, it does not widen)");
    static_assert(Config.priority <= 3, "priority 0..3");
    static constexpr std::uintptr_t dma = Config.controller == 1 ? 0x4002'0000 : 0x4002'0400;
    static constexpr std::uintptr_t stream = dma + 0x10 + 0x18 * Config.stream;
    static constexpr std::uint32_t cr = dma_cr(Config);
    static constexpr unsigned shift = dma_detail::flag_shift(Config.stream);
    static constexpr std::uintptr_t isr = dma + (Config.stream < 4 ? 0x00 : 0x04);
    static constexpr std::uintptr_t ifcr = dma + (Config.stream < 4 ? 0x08 : 0x0C);

public:
    static constexpr DmaConfig config = Config;  ///< the configuration (rtos::motion::TimerStepper reads it)
    /// The stream, DMA1 0-7 and DMA2 8-15 (resource checks, rtos/distinct.hpp: no stream twice).
    static constexpr unsigned id = (Config.controller - 1) * 8 + Config.stream;
    static constexpr std::uint8_t irq = dma_irq(Config.controller, Config.stream);

    static void init() noexcept {
        rcc::enable(Config.controller == 1 ? rcc::gate::dma1 : rcc::gate::dma2);
        stop();
        MUX::write(static_cast<std::uint32_t>(Config.request));
    }

    /// Move `items` data between `memory` and the register at `peripheral_address`; false if
    /// the stream is still busy or `items` is 0 or above 65535.
    static bool start(std::uintptr_t peripheral_address, const void* memory, std::size_t items,
                      bool interrupt = false) noexcept {
        using namespace dma_detail;
        if (items == 0 || items > 0xFFFF || busy()) {
            return false;
        }
        disable();
        IFCR::write(flags_all << shift);
        PAR::write(static_cast<std::uint32_t>(peripheral_address));
        M0AR::write(static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(memory)));
        NDTR::write(static_cast<std::uint32_t>(items));
        FCR::write(0);  // direct mode
        CR::write(cr | (interrupt ? cr_tcie | cr_teie : 0u) | cr_en);
        return true;
    }

    /// The last transfer completed (TCIF).
    [[nodiscard]] static bool done() noexcept { return ((ISR::read() >> shift) & dma_detail::flag_tcif) != 0; }
    /// A bus error stopped the stream (TEIF; the hardware cleared EN).
    [[nodiscard]] static bool failed() noexcept { return ((ISR::read() >> shift) & dma_detail::flag_teif) != 0; }
    /// Enabled and data left to move.
    [[nodiscard]] static bool busy() noexcept {
        return (CR::read() & dma_detail::cr_en) != 0 && (NDTR::read() & 0xFFFFu) != 0;
    }
    /// Data items still to move.
    [[nodiscard]] static std::uint32_t remaining() noexcept { return NDTR::read() & 0xFFFFu; }
    /// In the ISR: acknowledge the flags.
    static void clear() noexcept { IFCR::write(dma_detail::flags_all << shift); }

    /// Abort: disable the stream (it becomes idle, the registers writable again).
    static void stop() noexcept {
        disable();
        IFCR::write(dma_detail::flags_all << shift);
    }

private:
    /// EN = 0 takes effect after the current data item: wait until the stream reads disabled.
    static void disable() noexcept {
        CR::write(CR::read() & ~dma_detail::cr_en);
        while ((CR::read() & dma_detail::cr_en) != 0) {
        }
    }

    using ISR = Reg<isr>;
    using IFCR = Reg<ifcr>;
    using CR = Reg<stream + 0x00>;
    using NDTR = Reg<stream + 0x04>;
    using PAR = Reg<stream + 0x08>;
    using M0AR = Reg<stream + 0x0C>;
    using FCR = Reg<stream + 0x14>;
    using MUX = Reg<dmamux_channel_address(Config.controller, Config.stream)>;
};

}  // namespace rtos::port::stm32h7
