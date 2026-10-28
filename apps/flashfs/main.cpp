// SPDX-License-Identifier: MIT
//
// littlefs in the MCU's own flash, on the Factory Control Unit: the last 256 KB of bank 2 (32
// sectors, the FCU's mission store) carry a littlefs volume. The firmware mounts it (formats
// it the first time), writes a mission file in a directory, rewrites a counter file 100 times
// - more than the free blocks hold, so sectors get erased and reused - remounts and reads both
// back. Ends with one line:
//
//     FLASHFS RESULT: PASS (<erases> sector erases)      or      FLASHFS RESULT: FAIL <step>
//
// emulation/flashfs_test.py runs it in Renode (own flash model: unlock, sector erase).
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "board/fcu.hpp"
#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/stm32h5/flash_program.hpp"
#include "rtos/rtos.hpp"
#include "rtos/storage/littlefs.hpp"

namespace {

struct Config : rtos::DefaultConfig {
    static constexpr std::size_t max_threads = 1;
    static constexpr std::size_t notify_slots = 1;
};
using Port = rtos::port::cm33::Cm33Port;
using Kernel = rtos::Kernel<Port, Config>;

}  // namespace

Kernel kernel;
RTOS_CM33_BIND_KERNEL(kernel)

namespace {

namespace flash = rtos::port::stm32h5::flash;

// FLASH_DATA_START in apps/CMakeLists.txt keeps the image out of this region
constexpr std::uintptr_t data_base = 0x081C'0000;
struct CountingRegion : flash::Region<data_base, 256 * 1024, board::sysclk_hz> {
    std::uint32_t erases = 0;
    bool erase(std::uint32_t block) noexcept {
        ++erases;
        return flash::Region<data_base, 256 * 1024, board::sysclk_hz>::erase(block);
    }
};
CountingRegion region;
rtos::storage::LittleFs<CountingRegion> fs{region};
using Fs = decltype(fs);

Kernel::Thread check_tcb;
alignas(8) std::array<std::byte, 4096> check_stack;

[[noreturn]] void finish(const char* failed_step, int code) {
    if (failed_step == nullptr) {
        board::print("FLASHFS RESULT: PASS (");
        board::print_dec(region.erases);
        board::print(" sector erases)\n");
    } else {
        board::print("FLASHFS RESULT: FAIL ");
        board::print(failed_step);
        board::print(" (");
        board::print_dec(static_cast<std::uint32_t>(-code));
        board::print(")\n");
    }
    board::flush();
    for (;;) {
        kernel.sleep(1000);
    }
}

int write_file(const char* path, const void* data, std::size_t size) {
    Fs::File f;
    if (const int r = fs.open(f, path, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC); r != LFS_ERR_OK) {
        return r;
    }
    const lfs_ssize_t n = lfs_file_write(fs.get(), f.get(), data, static_cast<lfs_size_t>(size));
    const int c = lfs_file_close(fs.get(), f.get());
    return n < 0 ? static_cast<int>(n) : c;
}

lfs_ssize_t read_file(const char* path, void* data, std::size_t size) {
    Fs::File f;
    if (const int r = fs.open(f, path, LFS_O_RDONLY); r != LFS_ERR_OK) {
        return r;
    }
    const lfs_ssize_t n = lfs_file_read(fs.get(), f.get(), data, static_cast<lfs_size_t>(size));
    lfs_file_close(fs.get(), f.get());
    return n;
}

std::array<std::uint8_t, 6000> mission_out, mission_in;

void check(void*) {
    if (fs.mount() != LFS_ERR_OK) {
        board::print("flashfs: no file system, formatting\n");
        if (const int r = fs.format(); r != LFS_ERR_OK) {
            finish("format", r);
        }
        if (const int r = fs.mount(); r != LFS_ERR_OK) {
            finish("mount", r);
        }
    }
    for (std::size_t i = 0; i < mission_out.size(); ++i) {
        mission_out[i] = static_cast<std::uint8_t>(i * 7 + 3);
    }
    if (const int r = lfs_mkdir(fs.get(), "missions"); r != LFS_ERR_OK && r != LFS_ERR_EXIST) {
        finish("mkdir", r);
    }
    if (const int r = write_file("missions/pick.bin", mission_out.data(), mission_out.size()); r != LFS_ERR_OK) {
        finish("write mission", r);
    }
    for (std::uint32_t i = 0; i < 100; ++i) {
        std::array<std::uint32_t, 512> counter{};  // 2 KB per rewrite
        counter.fill(i);
        if (const int r = write_file("counter", counter.data(), sizeof counter); r != LFS_ERR_OK) {
            finish("rewrite counter", r);
        }
    }
    if (const int r = fs.unmount(); r != LFS_ERR_OK) {
        finish("unmount", r);
    }
    if (const int r = fs.mount(); r != LFS_ERR_OK) {
        finish("remount", r);
    }
    if (read_file("missions/pick.bin", mission_in.data(), mission_in.size()) !=
            static_cast<lfs_ssize_t>(mission_in.size()) ||
        mission_in != mission_out) {
        finish("read mission", LFS_ERR_CORRUPT);
    }
    std::array<std::uint32_t, 512> counter{};
    if (read_file("counter", counter.data(), sizeof counter) != static_cast<lfs_ssize_t>(sizeof counter) ||
        counter[0] != 99 || counter[511] != 99) {
        finish("read counter", LFS_ERR_CORRUPT);
    }
    board::print("flashfs: missions/pick.bin and counter (99) read back after a remount\n");
    // the superblock is really in the flash: "littlefs" at offset 8 of block 0 or 1
    const auto magic_at = [](std::uint32_t block) {
        return std::memcmp(reinterpret_cast<const void*>(data_base + block * flash::sector_bytes + 8), "littlefs", 8) == 0;
    };
    if (!magic_at(0) && !magic_at(1)) {
        finish("superblock in flash", LFS_ERR_CORRUPT);
    }
    finish(nullptr, 0);
}

}  // namespace

int main() {
    board::init();
    board::print("\nrtoos flashfs - STM32H563 / Factory Control Unit\n");
    rtos::port::cm33::CycleCounter::enable();
    Port::configure(board::sysclk_hz, 1000);
    kernel.create_thread(check_tcb, {.name = "check", .entry = &check, .stack = check_stack, .priority = 10});
    kernel.start();
}
