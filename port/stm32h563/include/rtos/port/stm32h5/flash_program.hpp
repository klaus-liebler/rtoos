// SPDX-License-Identifier: MIT
//
// STM32H563 flash programming: sector erase and quad-word programming of the main flash
// (RM0481 chapter FLASH; registers, bits: SVD v1.8; keys, sector and bank size: ST's
// stm32h5xx_hal_flash.h / stm32h563xx.h).
//
//   {
//       flash::Writer<board::sysclk_hz> w;              // unlocks; locks again at scope end
//       w.erase(region + 0x2000);                        // the 8 KB sector at that address
//       w.program(region + 0x2000, data);                // 16-byte aligned, multiple of 16
//   }
//
// The flash has two banks of 1 MB (128 sectors of 8 KB). Code keeps running from one bank
// while the other is erased or programmed (read-while-write); erasing or programming the bank
// the CPU executes from stalls it until the operation ends. Each quad-word (128 bits, with ECC)
// can be programmed once after an erase. Writes of a quad-word go to the write buffer word by
// word with interrupts masked (PRIMASK), as ST's driver does: an interrupt in between that
// itself touched the flash would break the sequence.
//
// A bank swap (option SWAP_BANK) exchanges the banks' addresses; the driver refuses to work
// then instead of erasing the wrong bank. After every operation the ICACHE is invalidated: it
// may hold the old contents.
#pragma once

#include <cstddef>
#include <cstring>
#include <cstdint>
#include <span>

#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/stm32h5/cache.hpp"
#include "rtos/port/stm32h5/chip.hpp"

namespace rtos::port::stm32h5::flash {

inline constexpr std::size_t sector_bytes = 0x2000;  // FLASH_SECTOR_SIZE
inline constexpr std::size_t bank_bytes = flash_bytes / 2;
inline constexpr std::size_t sectors_per_bank = bank_bytes / sector_bytes;  // 128 (FLASH_SECTOR_NB)
inline constexpr std::size_t quad_word = 16;

enum class Status : std::uint8_t {
    Ok,
    WriteProtected,     ///< WRPERR
    SequenceError,      ///< PGSERR: e.g. a quad-word programmed twice without erase
    StrobeError,        ///< STRBERR
    Inconsistency,      ///< INCERR
    Timeout,
    BankSwap,           ///< SWAP_BANK set: refused
    OutOfRange,         ///< not in the main flash, or misaligned
};

namespace detail {
using NSKEYR = PReg<0x0002'2000 + 0x04>;
using NSSR = PReg<0x0002'2000 + 0x20>;
using NSCR = PReg<0x0002'2000 + 0x28>;
using NSCCR = PReg<0x0002'2000 + 0x30>;
using OPTSR_CUR = PReg<0x0002'2000 + 0x50>;
inline constexpr std::uint32_t key1 = 0x4567'0123, key2 = 0xCDEF'89AB;
inline constexpr std::uint32_t sr_bsy = 1u << 0, sr_wbne = 1u << 1, sr_dbne = 1u << 3, sr_wrperr = 1u << 17,
                               sr_pgserr = 1u << 18, sr_strberr = 1u << 19, sr_incerr = 1u << 20;
inline constexpr std::uint32_t sr_errors = sr_wrperr | sr_pgserr | sr_strberr | sr_incerr;
inline constexpr std::uint32_t cr_lock = 1u << 0, cr_pg = 1u << 1, cr_ser = 1u << 2, cr_strt = 1u << 5,
                               cr_bksel = 1u << 31;
inline constexpr std::uint32_t cr_snb_pos = 6;
inline constexpr std::uint32_t ccr_all = 0xFF'0000u;  // CLR_EOP .. CLR_OPTCHANGEERR
inline constexpr std::uint32_t optsr_swap_bank = 1u << 31;
}  // namespace detail

template <std::uint32_t CpuHz>
class Writer {
public:
    Writer() noexcept {
        using namespace detail;
        if ((NSCR::read() & cr_lock) != 0) {
            NSKEYR::write(key1);
            NSKEYR::write(key2);
        }
    }
    ~Writer() {
        detail::NSCR::write(detail::cr_lock);
        ICache::invalidate();
    }
    Writer(const Writer&) = delete;
    Writer& operator=(const Writer&) = delete;

    /// Erase the 8 KB sector that contains `address` (to 0xFF).
    Status erase(std::uintptr_t address) noexcept {
        using namespace detail;
        if (address < flash_base || address >= flash_base + flash_bytes) {
            return Status::OutOfRange;
        }
        if (const Status s = prepare(); s != Status::Ok) {
            return s;
        }
        const std::size_t offset = address - flash_base;
        const std::uint32_t sector = static_cast<std::uint32_t>((offset % bank_bytes) / sector_bytes);
        const std::uint32_t bank2 = offset >= bank_bytes ? cr_bksel : 0u;
        NSCR::write(cr_ser | bank2 | (sector << cr_snb_pos));
        NSCR::write(cr_ser | bank2 | (sector << cr_snb_pos) | cr_strt);
        const Status s = finish(100);  // generous: a sector erase takes milliseconds
        NSCR::write(0);
        return s;
    }

    /// Program `data` at `address` (both multiples of 16 bytes; the area must be erased).
    Status program(std::uintptr_t address, std::span<const std::byte> data) noexcept {
        using namespace detail;
        if (address % quad_word != 0 || data.size() % quad_word != 0 || address < flash_base ||
            address + data.size() > flash_base + flash_bytes) {
            return Status::OutOfRange;
        }
        if (const Status s = prepare(); s != Status::Ok) {
            return s;
        }
        NSCR::write(cr_pg);
        Status s = Status::Ok;
        for (std::size_t i = 0; i < data.size() && s == Status::Ok; i += quad_word) {
            std::uint32_t words[4];
            for (std::size_t w = 0; w < 4; ++w) {
                const std::byte* b = data.data() + i + 4 * w;
                words[w] = std::to_integer<std::uint32_t>(b[0]) | std::to_integer<std::uint32_t>(b[1]) << 8 |
                           std::to_integer<std::uint32_t>(b[2]) << 16 | std::to_integer<std::uint32_t>(b[3]) << 24;
            }
            auto* dest = reinterpret_cast<volatile std::uint32_t*>(address + i);
            std::uint32_t primask;
            __asm volatile("mrs %0, primask\n cpsid i" : "=r"(primask)::"memory");
            for (std::size_t w = 0; w < 4; ++w) {
                dest[w] = words[w];
            }
            __asm volatile("msr primask, %0" ::"r"(primask) : "memory");
            s = finish(10);
        }
        NSCR::write(0);
        return s;
    }

private:
    static constexpr std::uint32_t cycles_per_ms = CpuHz / 1000;

    static Status prepare() noexcept {
        using namespace detail;
        if ((OPTSR_CUR::read() & optsr_swap_bank) != 0) {
            return Status::BankSwap;
        }
        if (!idle(100)) {
            return Status::Timeout;
        }
        NSCCR::write(ccr_all);
        return Status::Ok;
    }

    static bool idle(std::uint32_t ms) noexcept {
        using namespace detail;
        const std::uint32_t start = cm33::CycleCounter::now();
        while ((NSSR::read() & (sr_bsy | sr_wbne | sr_dbne)) != 0) {
            if (cm33::CycleCounter::now() - start > ms * cycles_per_ms) {
                return false;
            }
        }
        return true;
    }

    static Status finish(std::uint32_t ms) noexcept {
        using namespace detail;
        if (!idle(ms)) {
            return Status::Timeout;
        }
        const std::uint32_t sr = NSSR::read();
        NSCCR::write(ccr_all);
        if ((sr & sr_wrperr) != 0) {
            return Status::WriteProtected;
        }
        if ((sr & sr_pgserr) != 0) {
            return Status::SequenceError;
        }
        if ((sr & sr_strberr) != 0) {
            return Status::StrobeError;
        }
        if ((sr & sr_incerr) != 0) {
            return Status::Inconsistency;
        }
        return Status::Ok;
    }
};

/// A part of the main flash kept for data, as rtos::storage::FlashMemory (littlefs): blocks are
/// the 8 KB sectors, the program unit is the quad-word. Its place is checked at compile time:
/// whole sectors, inside one bank, inside the flash. Keep the image out of it with
/// rtos_add_firmware(... FLASH_DATA_START <Base>).
template <std::uintptr_t Base, std::size_t Bytes, std::uint32_t CpuHz>
class Region {
    static_assert(Bytes > 0 && Base % sector_bytes == 0 && Bytes % sector_bytes == 0, "whole 8 KB sectors");
    static_assert(Base >= flash_base && Base + Bytes <= flash_base + flash_bytes, "inside the main flash");
    static_assert((Base - flash_base) / bank_bytes == (Base + Bytes - 1 - flash_base) / bank_bytes,
                  "inside one bank");

public:
    static constexpr std::uintptr_t base = Base;
    static constexpr std::uint32_t block_bytes = sector_bytes;
    static constexpr std::uint32_t block_count = Bytes / sector_bytes;
    static constexpr std::uint32_t program_bytes = quad_word;

    bool read(std::uint32_t block, std::uint32_t offset, std::span<std::byte> out) noexcept {
        if (!fits(block, offset, out.size())) {
            return false;
        }
        std::memcpy(out.data(), reinterpret_cast<const void*>(Base + block * block_bytes + offset), out.size());
        return true;
    }

    bool program(std::uint32_t block, std::uint32_t offset, std::span<const std::byte> data) noexcept {
        if (!fits(block, offset, data.size())) {
            return false;
        }
        Writer<CpuHz> w;
        last_ = w.program(Base + block * block_bytes + offset, data);
        return last_ == Status::Ok;
    }

    bool erase(std::uint32_t block) noexcept {
        if (block >= block_count) {
            return false;
        }
        Writer<CpuHz> w;
        last_ = w.erase(Base + block * block_bytes);
        return last_ == Status::Ok;
    }

    /// The status of the last program or erase.
    [[nodiscard]] Status last_status() const noexcept { return last_; }

private:
    static constexpr bool fits(std::uint32_t block, std::uint32_t offset, std::size_t size) noexcept {
        return block < block_count && offset + size <= block_bytes;
    }
    Status last_ = Status::Ok;
};

}  // namespace rtos::port::stm32h5::flash
