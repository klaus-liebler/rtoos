// SPDX-License-Identifier: MIT
//
// The PLC's A/B program slots (rtos/plc/slot_store.hpp) in the FCU's real flash layout: 256 KB at
// 0x081C0000 (STM32H563: 32 sectors of 8 KB; STM32H743: the last two 128 KB sectors of bank 2).
// Commits an image, "restarts" (a new store object reads the flash), reads it back, changes the
// configuration through the journal, commits a second image into the other slot, and checks that
// the slot header is really in the memory-mapped flash. Ends with one line:
//
//     PLCSLOTS RESULT: PASS (<erases> sector erases)      or      PLCSLOTS RESULT: FAIL <step>
//
// In Renode with the chip's flash interface model (unlock, sector erase): plcslots_renode
// (preset arm) and h743_plcslots_renode (preset arm-h743).
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "board/chip.hpp"
#include "rtos/plc/slot_store.hpp"
#include "rtos/rtos.hpp"
#if RTOS_FCU_REVISION == 3
#include "rtos/port/stm32h7/flash_program.hpp"
#else
#include "rtos/port/stm32h5/flash_program.hpp"
#endif

namespace {

struct Config : rtos::DefaultConfig {
    static constexpr std::size_t max_threads = 1;
    static constexpr std::size_t notify_slots = 1;
};
using Kernel = rtos::Kernel<board::Port, Config>;

}  // namespace

Kernel kernel;
RTOS_BOARD_BIND_KERNEL(kernel)

namespace {

constexpr std::uintptr_t slots_base = 0x081C'0000;  // FLASH_DATA_START keeps the image out of it
using Base = board::chip::flash::Region<slots_base, 256 * 1024, board::sysclk_hz>;
struct CountingRegion : Base {
    std::uint32_t erases = 0;
    bool erase(std::uint32_t block) noexcept {
        ++erases;
        return Base::erase(block);
    }
};
CountingRegion region;
using Store = rtos::plc::SlotStore<CountingRegion>;

Kernel::Thread check_tcb;
alignas(8) std::array<std::byte, 4096> check_stack;
std::array<std::uint8_t, 20000> image_a, image_b, back;

[[noreturn]] void finish(const char* failed_step) {
    if (failed_step == nullptr) {
        board::print("PLCSLOTS RESULT: PASS (");
        board::print_dec(region.erases);
        board::print(" sector erases)\n");
    } else {
        board::print("PLCSLOTS RESULT: FAIL ");
        board::print(failed_step);
        board::print("\n");
    }
    board::flush();
    for (;;) kernel.sleep(1000);
}

bool read_back(Store& s, const std::array<std::uint8_t, 20000>& expected, std::size_t n) {
    return s.image_bytes() == n && s.read_image(back) == rtos::plc::SlotError::none &&
           std::memcmp(back.data(), expected.data(), n) == 0;
}

void check(void*) {
    for (std::size_t i = 0; i < image_a.size(); ++i) {
        image_a[i] = static_cast<std::uint8_t>(i * 13 + 5);
        image_b[i] = static_cast<std::uint8_t>(i * 7 + 1);
    }
    {
        Store s{region};
        s.mount();  // the emulated flash starts as zeros: no valid slot
        if (s.has_image()) finish("empty flash has an image");
        if (s.commit(std::span(image_a).first(15000), {1, 1, 0, 0, 0}) != rtos::plc::SlotError::none) finish("commit A");
    }
    {
        Store s{region};  // a restart
        s.mount();
        if (!read_back(s, image_a, 15000)) finish("read back A");
        if (s.config().control_source != 1 || s.config().autostart != 1) finish("config of A");
        for (std::uint32_t i = 1; i <= 10; ++i) {
            if (s.set_config({0, 1, 0, 0, i}) != rtos::plc::SlotError::none) finish("journal");
        }
    }
    {
        Store s{region};
        s.mount();
        if (s.config().reserved2 != 10 || s.config().control_source != 0) finish("journal after restart");
        if (!read_back(s, image_a, 15000)) finish("A after the journal");
        if (s.commit(std::span(image_b).first(20000), {1, 0, 0, 0, 0}) != rtos::plc::SlotError::none) finish("commit B");
    }
    Store s{region};
    s.mount();
    if (s.active_slot() != 1 || !read_back(s, image_b, 20000)) finish("read back B from the other slot");
    if (std::memcmp(reinterpret_cast<const void*>(slots_base + Store::slot_bytes), "FCUSLOT1", 8) != 0) {
        finish("slot header in the flash");
    }
    board::print("plcslots: image A, journal, image B in the other slot - all read back after restarts\n");
    finish(nullptr);
}

}  // namespace

int main() {
    board::init();
    board::print("\nrtoos plcslots - ");
    board::print(board::mcu_name);
    board::print(" / Factory Control Unit\n");
    board::CycleCounter::enable();
    board::Port::configure(board::sysclk_hz, 1000);
    kernel.create_thread(check_tcb, {.name = "check", .entry = &check, .stack = check_stack, .priority = 10});
    kernel.start();
}
