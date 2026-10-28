// SPDX-License-Identifier: MIT
//
// SD card check for the Factory Control Unit: initialises the card in the SDMMC1 slot, writes,
// reads back and compares test patterns - one block and a multi-block run - at the card's last
// blocks, then formats the card with FAT (FileX), writes two files (one in a directory),
// remounts and reads them back. Ends with one line:
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
#include "rtos/storage/filex_media.hpp"
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
rtos::storage::FileXMedia<rtos::storage::SdCard<Host>> volume{card};

constexpr char greeting[] = "rtoos on the Factory Control Unit\n";
constexpr std::size_t data_bytes = 20'000;
alignas(4) std::array<std::byte, data_bytes> data_out;
alignas(4) std::array<std::byte, data_bytes> data_in;

constexpr std::size_t run_blocks = 8;
alignas(4) std::array<std::byte, run_blocks * rtos::storage::sd_block_size> out_buffer;
alignas(4) std::array<std::byte, run_blocks * rtos::storage::sd_block_size> in_buffer;

Kernel::Thread check_tcb;
alignas(8) std::array<std::byte, 4096> check_stack;

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

bool write_file(const char* path, std::span<const std::byte> content) {
    FX_FILE file{};
    return fx_file_create(volume.media(), const_cast<CHAR*>(path)) == FX_SUCCESS &&
           fx_file_open(volume.media(), &file, const_cast<CHAR*>(path), FX_OPEN_FOR_WRITE) == FX_SUCCESS &&
           fx_file_write(&file, const_cast<std::byte*>(content.data()), static_cast<ULONG>(content.size())) == FX_SUCCESS &&
           fx_file_close(&file) == FX_SUCCESS;
}

std::size_t read_file(const char* path, std::span<std::byte> out) {
    FX_FILE file{};
    ULONG got = 0;
    if (fx_file_open(volume.media(), &file, const_cast<CHAR*>(path), FX_OPEN_FOR_READ) != FX_SUCCESS) {
        return 0;
    }
    (void)fx_file_read(&file, out.data(), static_cast<ULONG>(out.size()), &got);
    (void)fx_file_close(&file);
    return got;
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

    // file system: format, two files, remount, read back
    if (volume.format_fat("FCU", 8) != FX_SUCCESS || volume.open() != FX_SUCCESS) {
        finish("format", SdStatus::CardError);
    }
    fill(data_out, 3);
    if (!write_file("hello.txt", std::as_bytes(std::span(greeting, sizeof greeting - 1))) ||
        fx_directory_create(volume.media(), const_cast<CHAR*>("logs")) != FX_SUCCESS ||
        !write_file("logs/data.bin", data_out) || volume.close() != FX_SUCCESS) {
        finish("write files", SdStatus::CardError);
    }
    if (volume.open() != FX_SUCCESS) {
        finish("remount", SdStatus::CardError);
    }
    std::array<std::byte, sizeof greeting - 1> text{};
    if (read_file("hello.txt", text) != text.size() || !std::ranges::equal(text, std::as_bytes(std::span(greeting, sizeof greeting - 1)))) {
        finish("read hello.txt", SdStatus::CardError);
    }
    if (read_file("logs/data.bin", data_in) != data_bytes || data_in != data_out) {
        finish("read logs/data.bin", SdStatus::CardError);
    }
    board::print("fs: FAT");
    board::print_dec(volume.media()->fx_media_32_bit_FAT ? 32u : (volume.media()->fx_media_12_bit_FAT ? 12u : 16u));
    board::print(", hello.txt and logs/data.bin read back\n");
    (void)volume.close();
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
