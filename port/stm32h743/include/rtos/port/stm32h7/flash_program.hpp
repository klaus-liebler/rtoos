// SPDX-License-Identifier: MIT
//
// STM32H743 flash programming: sector erase and flash-word programming of the main flash
// (RM0433 chapter "Embedded flash memory"; registers, bits: SVD v2.4; keys, sector and flash-word
// size, the program and erase sequences: ST's stm32h7xx_hal_flash.h/.c, stm32h7xx_hal_flash_ex.c,
// stm32h743xx.h FLASH_SECTOR_SIZE, FLASH_NB_32BITWORD_IN_FLASHWORD).
//
//   {
//       flash::Writer<clocks.cpu_hz()> w;               // unlocks; locks again at scope end
//       w.erase(region);                                 // the 128 KB sector at that address
//       w.program(region, data);                         // 32-byte aligned, multiple of 32
//   }
//
// The interface of the STM32H5's driver; what differs: two banks of 1 MB with their own
// registers (KEYR/CR/SR/CCR at +0x100 for bank 2), 8 sectors of 128 KB per bank, 256-bit flash
// words (32 bytes, with ECC; each programmable once after an erase), erase in 64-bit
// parallelism (PSIZE = 11, VDD 2.7..3.6 V). Code keeps running from one bank while the other is
// erased or programmed; touching the bank the CPU executes from stalls it. The 8 words of a
// flash word go to the write buffer with interrupts masked (PRIMASK), as on the H5.
//
// The flash is cached (MPU write-through, stm32h7/mpu.hpp): after every operation the data cache
// lines of the touched range are invalidated and the instruction cache is invalidated, so reads
// see the new contents. A bank swap (SWAP_BANK) exchanges the banks' addresses; the driver
// refuses to work then instead of erasing the wrong bank.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "rtos/port/cm7/cache.hpp"
#include "rtos/port/cortex_m/cycle_counter.hpp"
#include "rtos/port/stm32h7/chip.hpp"

namespace rtos::port::stm32h7::flash {

inline constexpr std::size_t sector_bytes = flash_sector_bytes;  // FLASH_SECTOR_SIZE, 128 KB
inline constexpr std::size_t bank_bytes = flash_bytes / 2;
inline constexpr std::size_t sectors_per_bank = bank_bytes / sector_bytes;  // FLASH_SECTOR_TOTAL, 8
inline constexpr std::size_t flash_word = 32;                             // 8 x 32 bit

enum class Status : std::uint8_t {
    Ok,
    WriteProtected,     ///< WRPERR
    SequenceError,      ///< PGSERR: e.g. a flash word programmed twice without erase
    StrobeError,        ///< STRBERR
    Inconsistency,      ///< INCERR
    OperationError,     ///< OPERR
    Timeout,
    BankSwap,           ///< SWAP_BANK set: refused
    OutOfRange,         ///< not in the main flash, or misaligned
};

namespace detail {
inline constexpr std::uintptr_t base = 0x5200'2000;
/// The registers of bank `b` (1, 2).
template <unsigned B>
struct Bank {
    static constexpr std::uintptr_t at = base + (B == 1 ? 0x000 : 0x100);
    using KEYR = Reg<at + 0x04>;
    using CR = Reg<at + 0x0C>;
    using SR = Reg<at + 0x10>;
    using CCR = Reg<at + 0x14>;
};
using OPTSR_CUR = Reg<base + 0x1C>;
inline constexpr std::uint32_t key1 = 0x4567'0123, key2 = 0xCDEF'89AB;
inline constexpr std::uint32_t sr_bsy = 1u << 0, sr_wbne = 1u << 1, sr_qw = 1u << 2, sr_wrperr = 1u << 17,
                               sr_pgserr = 1u << 18, sr_strberr = 1u << 19, sr_incerr = 1u << 21, sr_operr = 1u << 22;
inline constexpr std::uint32_t cr_lock = 1u << 0, cr_pg = 1u << 1, cr_ser = 1u << 2, cr_psize_x64 = 3u << 4,
                               cr_start = 1u << 7, cr_snb_pos = 8;
inline constexpr std::uint32_t ccr_all = 0x1FEF'0000u;  // CLR_EOP .. CLR_CRCRDERR (16..28 without 20)
inline constexpr std::uint32_t optsr_swap_bank = 1u << 31;
}  // namespace detail

template <std::uint32_t CpuHz>
class Writer {
public:
    Writer() noexcept {
        unlock<detail::Bank<1>>();
        unlock<detail::Bank<2>>();
    }
    ~Writer() {
        detail::Bank<1>::CR::write(detail::cr_lock);
        detail::Bank<2>::CR::write(detail::cr_lock);
    }
    Writer(const Writer&) = delete;
    Writer& operator=(const Writer&) = delete;

    /// Erase the 128 KB sector that contains `address` (to 0xFF).
    Status erase(std::uintptr_t address) noexcept {
        if (address < flash_base || address >= flash_base + flash_bytes) {
            return Status::OutOfRange;
        }
        const std::size_t offset = address - flash_base;
        const std::uint32_t sector = static_cast<std::uint32_t>((offset % bank_bytes) / sector_bytes);
        const std::uintptr_t start = flash_base + offset / sector_bytes * sector_bytes;
        const Status s = offset < bank_bytes ? erase<detail::Bank<1>>(sector) : erase<detail::Bank<2>>(sector);
        refresh(start, sector_bytes);
        return s;
    }

    /// Program `data` at `address` (both multiples of 32 bytes; the area must be erased).
    Status program(std::uintptr_t address, std::span<const std::byte> data) noexcept {
        if (address % flash_word != 0 || data.size() % flash_word != 0 || address < flash_base ||
            address + data.size() > flash_base + flash_bytes ||
            (address - flash_base) / bank_bytes != (address + data.size() - 1 - flash_base) / bank_bytes) {
            return Status::OutOfRange;
        }
        const Status s = address - flash_base < bank_bytes ? program<detail::Bank<1>>(address, data)
                                                           : program<detail::Bank<2>>(address, data);
        refresh(address, data.size());
        return s;
    }

private:
    static constexpr std::uint32_t cycles_per_ms = CpuHz / 1000;

    template <class B>
    static void unlock() noexcept {
        if ((B::CR::read() & detail::cr_lock) != 0) {
            B::KEYR::write(detail::key1);
            B::KEYR::write(detail::key2);
        }
    }

    template <class B>
    static Status erase(std::uint32_t sector) noexcept {
        using namespace detail;
        if (const Status s = prepare<B>(); s != Status::Ok) {
            return s;
        }
        B::CR::write(cr_ser | cr_psize_x64 | (sector << cr_snb_pos));
        B::CR::write(cr_ser | cr_psize_x64 | (sector << cr_snb_pos) | cr_start);
        const Status s = finish<B>(5'000);  // generous: a 128 KB sector erase takes about a second
        B::CR::write(0);
        return s;
    }

    template <class B>
    static Status program(std::uintptr_t address, std::span<const std::byte> data) noexcept {
        using namespace detail;
        if (const Status s = prepare<B>(); s != Status::Ok) {
            return s;
        }
        B::CR::write(cr_pg);
        __asm volatile("isb \n dsb" ::: "memory");
        Status s = Status::Ok;
        for (std::size_t i = 0; i < data.size() && s == Status::Ok; i += flash_word) {
            std::uint32_t words[8];
            std::memcpy(words, data.data() + i, flash_word);
            auto* dest = reinterpret_cast<volatile std::uint32_t*>(address + i);
            std::uint32_t primask;
            __asm volatile("mrs %0, primask\n cpsid i" : "=r"(primask)::"memory");
            for (std::size_t w = 0; w < 8; ++w) {
                dest[w] = words[w];
            }
            __asm volatile("isb \n dsb \n msr primask, %0" ::"r"(primask) : "memory");
            s = finish<B>(10);
        }
        B::CR::write(0);
        return s;
    }

    template <class B>
    static Status prepare() noexcept {
        using namespace detail;
        if ((OPTSR_CUR::read() & optsr_swap_bank) != 0) {
            return Status::BankSwap;
        }
        if (!idle<B>(100)) {
            return Status::Timeout;
        }
        B::CCR::write(ccr_all);
        return Status::Ok;
    }

    template <class B>
    static bool idle(std::uint32_t ms) noexcept {
        using namespace detail;
        const std::uint32_t start = cortex_m::CycleCounter::now();
        while ((B::SR::read() & (sr_bsy | sr_wbne | sr_qw)) != 0) {
            if (cortex_m::CycleCounter::now() - start > ms * cycles_per_ms) {
                return false;
            }
        }
        return true;
    }

    template <class B>
    static Status finish(std::uint32_t ms) noexcept {
        using namespace detail;
        if (!idle<B>(ms)) {
            return Status::Timeout;
        }
        const std::uint32_t sr = B::SR::read();
        B::CCR::write(ccr_all);
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
        if ((sr & sr_operr) != 0) {
            return Status::OperationError;
        }
        return Status::Ok;
    }

    /// The caches may hold the old contents: invalidate the range (D) and everything (I).
    static void refresh(std::uintptr_t address, std::size_t bytes) noexcept {
        cm7::cache::invalidate(reinterpret_cast<const void*>(address), bytes);
        cm7::cache::scb::ICIALLU::write(0);
        cm7::cache::barrier();
    }
};

/// A part of the main flash kept for data, as rtos::storage::FlashMemory (littlefs): blocks are
/// the 128 KB sectors, the program unit is the 32-byte flash word. Its place is checked at
/// compile time: whole sectors, inside one bank, inside the flash. Keep the image out of it with
/// rtos_add_firmware(... FLASH_DATA_START <Base>); the MPU (stm32h7/mpu.hpp) leaves it writable.
template <std::uintptr_t Base, std::size_t Bytes, std::uint32_t CpuHz>
class Region {
    static_assert(Bytes > 0 && Base % sector_bytes == 0 && Bytes % sector_bytes == 0, "whole 128 KB sectors");
    static_assert(Base >= flash_base && Base + Bytes <= flash_base + flash_bytes, "inside the main flash");
    static_assert((Base - flash_base) / bank_bytes == (Base + Bytes - 1 - flash_base) / bank_bytes,
                  "inside one bank");

public:
    static constexpr std::uintptr_t base = Base;
    static constexpr std::uint32_t block_bytes = sector_bytes;
    static constexpr std::uint32_t block_count = Bytes / sector_bytes;
    static constexpr std::uint32_t program_bytes = flash_word;

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

}  // namespace rtos::port::stm32h7::flash
