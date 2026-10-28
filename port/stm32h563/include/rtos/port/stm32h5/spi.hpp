// SPDX-License-Identifier: MIT
//
// STM32H563 SPI master (RM0481 chapter SPI; registers, bits, IRQ numbers: SVD v1.8).
//
//   constexpr SpiConfig hx711{.kernel_hz = board::pll1_q_hz, .max_bus_hz = 1'000'000};
//   using Scale = SpiMaster<SpiId::Spi2, hx711>;
//   Scale::init();
//   Scale::transfer(tx, rx);              // full duplex, tx.size() == rx.size()
//
// The configuration is a template argument: the baud rate prescaler (MBR, /2../256) is the
// smallest one whose SCK does not exceed max_bus_hz, chosen and checked at compile time;
// init() writes constants. Frames of 4..8 bits, software slave select (the device driver
// drives its chip select as a GPIO). Polled transfers of at most 65535 frames with a timeout
// on the DWT cycle counter.
//
// Kernel clock: RCC_CCIPR3 reset selection - pll1_q for SPI1-3, PCLK2 for SPI4.
#pragma once

#include <contracts>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

enum class SpiId : std::uint8_t { Spi1, Spi2, Spi3, Spi4 };

struct SpiConfig {
    std::uint32_t kernel_hz;       ///< kernel clock of the SPI
    std::uint32_t max_bus_hz;      ///< SCK must not exceed this
    std::uint32_t cpu_hz = 0;      ///< core clock for the timeouts; 0: = kernel_hz
    std::uint8_t mode = 0;         ///< SPI mode 0..3 (CPOL = bit 1, CPHA = bit 0)
    std::uint8_t data_bits = 8;    ///< 4..8
    bool lsb_first = false;
};

/// MBR field for `c`: smallest prescaler 2^(MBR+1) with kernel / prescaler <= max_bus_hz; 8 if none.
[[nodiscard]] constexpr std::uint32_t spi_mbr(const SpiConfig& c) noexcept {
    for (std::uint32_t mbr = 0; mbr < 8; ++mbr) {
        if (c.kernel_hz / (2u << mbr) <= c.max_bus_hz) {
            return mbr;
        }
    }
    return 8;
}

[[nodiscard]] constexpr std::uint32_t spi_bus_hz(const SpiConfig& c) noexcept {
    return c.kernel_hz / (2u << spi_mbr(c));
}

namespace spi_detail {
struct Info {
    std::uintptr_t offset;
    rcc::Gate gate;
    std::uint8_t irq;
};
[[nodiscard]] consteval Info info(SpiId id) {
    switch (id) {
        case SpiId::Spi1: return {0x0001'3000, rcc::gate::spi1, 55};
        case SpiId::Spi2: return {0x0000'3800, rcc::gate::spi2, 56};
        case SpiId::Spi3: return {0x0000'3C00, rcc::gate::spi3, 57};
        case SpiId::Spi4: return {0x0001'4C00, rcc::gate::spi4, 82};
    }
    return {};
}
inline constexpr std::uint32_t cr1_spe = 1u << 0, cr1_cstart = 1u << 9, cr1_ssi = 1u << 12;
inline constexpr std::uint32_t cfg2_master = 1u << 22, cfg2_lsbfrst = 1u << 23, cfg2_cpha = 1u << 24,
                               cfg2_cpol = 1u << 25, cfg2_ssm = 1u << 26, cfg2_afcntr = 1u << 31;
inline constexpr std::uint32_t sr_rxp = 1u << 0, sr_txp = 1u << 1, sr_eot = 1u << 3;
inline constexpr std::uint32_t ifcr_all = 0xBF8u;  // EOTC..MODFC (3..9), SUSPC (11)
}  // namespace spi_detail

template <SpiId Id, SpiConfig Config>
class SpiMaster {
    static constexpr spi_detail::Info info = spi_detail::info(Id);
    static_assert(Config.mode <= 3, "SPI mode must be 0..3");
    static_assert(Config.data_bits >= 4 && Config.data_bits <= 8, "SPI frames of 4..8 bits");
    static_assert(spi_mbr(Config) < 8, "max_bus_hz below kernel clock / 256");
    static constexpr std::uint32_t cfg1 = (static_cast<std::uint32_t>(Config.data_bits) - 1u) | (spi_mbr(Config) << 28);
    static constexpr std::uint32_t cfg2 = spi_detail::cfg2_master | spi_detail::cfg2_ssm | spi_detail::cfg2_afcntr |
                                          ((Config.mode & 1u) != 0 ? spi_detail::cfg2_cpha : 0u) |
                                          ((Config.mode & 2u) != 0 ? spi_detail::cfg2_cpol : 0u) |
                                          (Config.lsb_first ? spi_detail::cfg2_lsbfrst : 0u);
    static constexpr std::uint32_t cycles_per_us = (Config.cpu_hz != 0 ? Config.cpu_hz : Config.kernel_hz) / 1'000'000;

public:
    static constexpr std::uint8_t irq = info.irq;
    static constexpr std::uint32_t bus_hz = spi_bus_hz(Config);

    /// Clock on, frame format, prescaler; the pins stay driven while idle (AFCNTR).
    static void init() noexcept {
        rcc::enable(info.gate);
        CR1::write(spi_detail::cr1_ssi);
        CFG1::write(cfg1);
        CFG2::write(cfg2);
    }

    /// Full duplex: send `tx`, receive as many frames into `rx` (same size). False on timeout.
    static bool transfer(std::span<const std::byte> tx, std::span<std::byte> rx,
                         std::uint32_t timeout_us = 10'000) noexcept {
        contract_assert(tx.size() == rx.size() && tx.size() <= 0xFFFF);
        return run(tx.size(), tx.data(), rx.data(), timeout_us);
    }

    /// Send only (received frames are discarded).
    static bool write(std::span<const std::byte> tx, std::uint32_t timeout_us = 10'000) noexcept {
        contract_assert(tx.size() <= 0xFFFF);
        return run(tx.size(), tx.data(), nullptr, timeout_us);
    }

    /// Receive only (sends `fill`).
    static bool read(std::span<std::byte> rx, std::byte fill = std::byte{0xFF},
                     std::uint32_t timeout_us = 10'000) noexcept {
        contract_assert(rx.size() <= 0xFFFF);
        fill_ = fill;
        return run(rx.size(), nullptr, rx.data(), timeout_us);
    }

private:
    static bool run(std::size_t n, const std::byte* tx, std::byte* rx, std::uint32_t timeout_us) noexcept {
        using namespace spi_detail;
        if (n == 0) {
            return true;
        }
        const std::uint32_t start = cm33::CycleCounter::now();
        const std::uint32_t limit = timeout_us * cycles_per_us;
        CR2::write(static_cast<std::uint32_t>(n));  // TSIZE
        CR1::write(cr1_ssi | cr1_spe);
        CR1::write(cr1_ssi | cr1_spe | cr1_cstart);
        std::size_t sent = 0, received = 0;
        bool ok = true;
        while (received < n) {
            const std::uint32_t sr = SR::read();
            if (sent < n && (sr & sr_txp) != 0) {
                txdr8() = static_cast<std::uint8_t>(tx != nullptr ? tx[sent] : fill_);
                ++sent;
            }
            if ((sr & sr_rxp) != 0) {
                const auto b = static_cast<std::byte>(rxdr8());
                if (rx != nullptr) {
                    rx[received] = b;
                }
                ++received;
            }
            if (cm33::CycleCounter::now() - start > limit) {
                ok = false;
                break;
            }
        }
        while (ok && (SR::read() & sr_eot) == 0) {
            if (cm33::CycleCounter::now() - start > limit) {
                ok = false;
            }
        }
        IFCR::write(ifcr_all);
        CR1::write(cr1_ssi);  // SPE = 0: also flushes the FIFOs after a timeout
        return ok;
    }

    // Byte accesses: a 32-bit access to TXDR/RXDR would move four frames (data packing).
    static volatile std::uint8_t& txdr8() noexcept {
        return *reinterpret_cast<volatile std::uint8_t*>(periph_alias + info.offset + 0x20);
    }
    static std::uint8_t rxdr8() noexcept {
        return *reinterpret_cast<volatile std::uint8_t*>(periph_alias + info.offset + 0x30);
    }

    using CR1 = PReg<info.offset + 0x00>;
    using CR2 = PReg<info.offset + 0x04>;
    using CFG1 = PReg<info.offset + 0x08>;
    using CFG2 = PReg<info.offset + 0x0C>;
    using SR = PReg<info.offset + 0x14>;
    using IFCR = PReg<info.offset + 0x18>;

    static inline std::byte fill_{0xFF};
};

}  // namespace rtos::port::stm32h5
