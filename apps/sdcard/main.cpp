// SPDX-License-Identifier: MIT
//
// SD card check for the Factory Control Unit: initialises the card in the SDMMC1 slot and
// writes, reads back and compares test patterns - one block and a multi-block run - at the
// card's last blocks (the first ones keep a file system). Ends with one line:
//
//     SDCARD RESULT: PASS (<blocks> blocks, SDHC|SDSC)      or      SDCARD RESULT: FAIL <step>
//
// emulation/sdcard_test.py runs it in Renode against a card image and also checks the image.
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "board/fcu.hpp"
#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/stm32h5/sdmmc.hpp"
#include "rtos/rtos.hpp"
#include "rtos/storage/sd_card.hpp"

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

namespace h5 = rtos::port::stm32h5;
using rtos::storage::SdStatus;

using Host = h5::Sdmmc<h5::SdmmcId::Sdmmc1, {.kernel_hz = board::pll1_q_hz, .cpu_hz = board::sysclk_hz}>;
Host host;
rtos::storage::SdCard<Host> card{host};

constexpr std::size_t run_blocks = 8;
alignas(4) std::array<std::byte, run_blocks * rtos::storage::sd_block_size> out_buffer;
alignas(4) std::array<std::byte, run_blocks * rtos::storage::sd_block_size> in_buffer;

Kernel::Thread check_tcb;
alignas(8) std::array<std::byte, 2048> check_stack;

constexpr const char* status_name(SdStatus s) {
    switch (s) {
        case SdStatus::Ok: return "ok";
        case SdStatus::Timeout: return "timeout";
        case SdStatus::CrcError: return "crc error";
        case SdStatus::CardError: return "card error";
        case SdStatus::NoCard: return "no card";
        case SdStatus::Unsupported: return "unsupported card";
        case SdStatus::NotReady: return "not ready";
    }
    return "?";
}

[[noreturn]] void finish(const char* failed_step, SdStatus s) {
    if (failed_step == nullptr) {
        board::print("SDCARD RESULT: PASS (");
        board::print_dec(static_cast<std::uint32_t>(card.blocks()));
        board::print(card.high_capacity() ? " blocks, SDHC)\n" : " blocks, SDSC)\n");
    } else {
        board::print("SDCARD RESULT: FAIL ");
        board::print(failed_step);
        board::print(": ");
        board::print(status_name(s));
        board::print("\n");
    }
    board::flush();
    for (;;) {
        kernel.sleep(1000);
    }
}

void fill(std::span<std::byte> b, std::uint32_t seed) {
    for (std::size_t i = 0; i < b.size(); ++i) {
        b[i] = static_cast<std::byte>((i * 31 + seed * 7 + (i >> 9)) & 0xFF);
    }
}

void check(void*) {
    board::print("sd: identification ");
    board::print_dec(Host::identification_clock_hz / 1000);
    board::print(" kHz, transfer ");
    board::print_dec(Host::transfer_clock_hz / 1000);
    board::print(" kHz\n");
    if (const SdStatus s = card.init(); s != SdStatus::Ok) {
        finish("init", s);
    }
    board::print("sd: ");
    board::print_dec(static_cast<std::uint32_t>(card.blocks() / 2048));
    board::print(" MB\n");

    // one block, then a run of 8, at the end of the card
    const std::uint64_t single = card.blocks() - 1;
    const std::uint64_t run = card.blocks() - 1 - run_blocks;
    const auto one_out = std::span(out_buffer).first(rtos::storage::sd_block_size);
    const auto one_in = std::span(in_buffer).first(rtos::storage::sd_block_size);
    fill(one_out, 1);
    if (const SdStatus s = card.write(single, one_out); s != SdStatus::Ok) {
        finish("write block", s);
    }
    if (const SdStatus s = card.read(single, one_in); s != SdStatus::Ok) {
        finish("read block", s);
    }
    if (!std::ranges::equal(one_in, one_out)) {
        finish("compare block", SdStatus::CardError);
    }
    fill(out_buffer, 2);
    if (const SdStatus s = card.write(run, out_buffer); s != SdStatus::Ok) {
        finish("write run", s);
    }
    in_buffer.fill(std::byte{0});
    if (const SdStatus s = card.read(run, in_buffer); s != SdStatus::Ok) {
        finish("read run", s);
    }
    if (in_buffer != out_buffer) {
        finish("compare run", SdStatus::CardError);
    }
    finish(nullptr, SdStatus::Ok);
}

}  // namespace

int main() {
    board::init();
    board::print("\nrtoos sdcard - STM32H563 / Factory Control Unit\n");
    rtos::port::cm33::CycleCounter::enable();
    board::init_sdcard();
    Port::configure(board::sysclk_hz, 1000);
    kernel.create_thread(check_tcb, {.name = "check", .entry = &check, .stack = check_stack, .priority = 10});
    kernel.start();
}
