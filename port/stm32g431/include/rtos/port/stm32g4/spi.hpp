// SPDX-License-Identifier: MIT
//
// STM32G431 SPI master (RM0440 chapter SPI; registers, bits: SVD STM32G431 v1.8; IRQ numbers:
// ST's stm32g431xx.h). The G4's SPI is the older IP generation (CR1/CR2/SR/DR with a 4-frame
// FIFO), not the STM32H5's (CFG1/CFG2/TSIZE) - a driver of its own, with the same interface
// as stm32h5::SpiMaster:
//
//   constexpr SpiConfig hx711{.kernel_hz = clocks.pclk1_hz(), .max_bus_hz = 1'000'000,
//                             .cpu_hz = clocks.sysclk_hz()};
//   using Scale = SpiMaster<SpiId::Spi2, hx711>;
//   Scale::init();
//   Scale::transfer(tx, rx);              // full duplex, tx.size() == rx.size()
//
// The configuration is a template argument: the baud rate prescaler (CR1.BR, /2../256) is the
// smallest one whose SCK does not exceed max_bus_hz, chosen and checked at compile time;
// init() writes constants. Frames of 4..8 bits (CR2.DS, FRXTH: RXNE per frame), software slave
// select (SSM/SSI; the device driver drives its chip select as a GPIO). Polled transfers, one
// frame in flight, with a timeout on the DWT cycle counter.
//
// Kernel clock: the bus clock - PCLK2 for SPI1, PCLK1 for SPI2/3.
#pragma once

#include <contracts>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/port/cortex_m/cycle_counter.hpp"
#include "rtos/port/stm32g4/chip.hpp"
#include "rtos/port/stm32g4/rcc.hpp"

namespace rtos::port::stm32g4 {

enum class SpiId : std::uint8_t { Spi1, Spi2, Spi3 };

struct SpiConfig {
    std::uint32_t kernel_hz;       ///< bus clock of the SPI (PCLK2 / PCLK1)
    std::uint32_t max_bus_hz;      ///< SCK must not exceed this
    std::uint32_t cpu_hz = 0;      ///< core clock for the timeouts; 0: = kernel_hz
    std::uint8_t mode = 0;         ///< SPI mode 0..3 (CPOL = bit 1, CPHA = bit 0)
    std::uint8_t data_bits = 8;    ///< 4..8
    bool lsb_first = false;
};

/// CR1.BR for `c`: smallest prescaler 2^(BR+1) with kernel / prescaler <= max_bus_hz; 8 if none.
[[nodiscard]] constexpr std::uint32_t spi_br(const SpiConfig& c) noexcept {
    for (std::uint32_t br = 0; br < 8; ++br) {
        if (c.kernel_hz / (2u << br) <= c.max_bus_hz) {
            return br;
        }
    }
    return 8;
}

[[nodiscard]] constexpr std::uint32_t spi_bus_hz(const SpiConfig& c) noexcept { return c.kernel_hz / (2u << spi_br(c)); }

namespace spi_detail {
struct Info {
    std::uintptr_t offset;
    rcc::Gate gate;
    std::uint8_t irq;
};
[[nodiscard]] consteval Info info(SpiId id) {
    switch (id) {
        case SpiId::Spi1: return {0x0001'3000, rcc::gate::spi1, 35};
        case SpiId::Spi2: return {0x0000'3800, rcc::gate::spi2, 36};
        case SpiId::Spi3: return {0x0000'3C00, rcc::gate::spi3, 51};
    }
    return {};
}
inline constexpr std::uint32_t cr1_cpha = 1u << 0, cr1_cpol = 1u << 1, cr1_mstr = 1u << 2, cr1_br_pos = 3,
                               cr1_spe = 1u << 6, cr1_lsbfirst = 1u << 7, cr1_ssi = 1u << 8, cr1_ssm = 1u << 9;
inline constexpr std::uint32_t cr2_ds_pos = 8, cr2_frxth = 1u << 12;
inline constexpr std::uint32_t sr_rxne = 1u << 0, sr_txe = 1u << 1, sr_bsy = 1u << 7, sr_frlvl = 3u << 9;
}  // namespace spi_detail

/// CR1 of `c` without SPE.
[[nodiscard]] constexpr std::uint32_t spi_cr1(const SpiConfig& c) noexcept {
    using namespace spi_detail;
    return cr1_mstr | cr1_ssm | cr1_ssi | (spi_br(c) << cr1_br_pos) | ((c.mode & 1u) != 0 ? cr1_cpha : 0u) |
           ((c.mode & 2u) != 0 ? cr1_cpol : 0u) | (c.lsb_first ? cr1_lsbfirst : 0u);
}

/// CR2 of `c`: frame size DS = bits - 1, RXNE at one byte in the FIFO.
[[nodiscard]] constexpr std::uint32_t spi_cr2(const SpiConfig& c) noexcept {
    return ((static_cast<std::uint32_t>(c.data_bits) - 1u) << spi_detail::cr2_ds_pos) | spi_detail::cr2_frxth;
}

template <SpiId Id, SpiConfig Config>
class SpiMaster {
    static constexpr spi_detail::Info info = spi_detail::info(Id);
    static_assert(Config.mode <= 3, "SPI mode must be 0..3");
    static_assert(Config.data_bits >= 4 && Config.data_bits <= 8, "SPI frames of 4..8 bits");
    static_assert(spi_br(Config) < 8, "max_bus_hz below kernel clock / 256");
    static constexpr std::uint32_t cr1 = spi_cr1(Config), cr2 = spi_cr2(Config);
    static constexpr std::uint32_t cycles_per_us = (Config.cpu_hz != 0 ? Config.cpu_hz : Config.kernel_hz) / 1'000'000;

public:
    static constexpr std::uint8_t irq = info.irq;
    static constexpr std::uint32_t bus_hz = spi_bus_hz(Config);

    /// Clock on, frame format, prescaler; enabled (the pins stay driven while idle).
    static void init() noexcept {
        rcc::enable(info.gate);
        CR1::write(cr1);
        CR2::write(cr2);
        CR1::write(cr1 | spi_detail::cr1_spe);
    }

    /// Full duplex: send `tx`, receive as many frames into `rx` (same size). False on timeout.
    static bool transfer(std::span<const std::byte> tx, std::span<std::byte> rx,
                         std::uint32_t timeout_us = 10'000) noexcept {
        contract_assert(tx.size() == rx.size());
        return run(tx.size(), tx.data(), rx.data(), timeout_us);
    }

    /// Send only (received frames are discarded).
    static bool write(std::span<const std::byte> tx, std::uint32_t timeout_us = 10'000) noexcept {
        return run(tx.size(), tx.data(), nullptr, timeout_us);
    }

    /// Receive only (sends `fill`).
    static bool read(std::span<std::byte> rx, std::byte fill = std::byte{0xFF},
                     std::uint32_t timeout_us = 10'000) noexcept {
        fill_ = fill;
        return run(rx.size(), nullptr, rx.data(), timeout_us);
    }

private:
    static bool run(std::size_t n, const std::byte* tx, std::byte* rx, std::uint32_t timeout_us) noexcept {
        using namespace spi_detail;
        const std::uint32_t start = cortex_m::CycleCounter::now();
        const std::uint32_t limit = timeout_us * cycles_per_us;
        const auto expired = [&] { return cortex_m::CycleCounter::now() - start > limit; };
        bool ok = true;
        for (std::size_t i = 0; i < n && ok; ++i) {
            while ((SR::read() & sr_txe) == 0) {
                if (expired()) {
                    ok = false;
                    break;
                }
            }
            if (!ok) {
                break;
            }
            dr8() = static_cast<std::uint8_t>(tx != nullptr ? tx[i] : fill_);
            while ((SR::read() & sr_rxne) == 0) {
                if (expired()) {
                    ok = false;
                    break;
                }
            }
            if (ok) {
                const auto b = static_cast<std::byte>(dr8());
                if (rx != nullptr) {
                    rx[i] = b;
                }
            }
        }
        while ((SR::read() & sr_bsy) != 0 && !expired()) {
        }
        if (!ok) {  // SPE off and on: flushes the FIFOs (RM0440 "Disabling the SPI")
            CR1::write(cr1);
            while ((SR::read() & sr_frlvl) != 0) {
                [[maybe_unused]] const std::uint8_t drop = dr8();
            }
            CR1::write(cr1 | cr1_spe);
        }
        return ok;
    }

    // Byte accesses: a 16-bit access to DR would move two frames (data packing).
    static volatile std::uint8_t& dr8() noexcept {
        return *reinterpret_cast<volatile std::uint8_t*>(periph_base + info.offset + 0x0C);
    }

    using CR1 = PReg<info.offset + 0x00>;
    using CR2 = PReg<info.offset + 0x04>;
    using SR = PReg<info.offset + 0x08>;

    static inline std::byte fill_{0xFF};
};

}  // namespace rtos::port::stm32g4
