// SPDX-License-Identifier: MIT
// AFL++ harness: the PLC VM (rtos/plc, docs/plc.md) with a hostile program image - what a client
// uploads over the web UI / IDE. load() verifies the image completely and the interpreter relies
// on it, so a gap in the verifier is an out-of-bounds access here.
//
// Input: the image. The harness corrects the length and the CRC in the header (else nearly every
// mutation ends at the CRC check); the rest stays as mutated. A loaded image runs each task for a
// few cycles, then once more single-stepping through the debugger (halt, resume).
// Seeds: tests/plc_corpus/*.img (the compiler's differential test images).
#include <algorithm>
#include <array>
#include <cstring>
#include <vector>

#include "fuzz_main.hpp"
#include "rtos/plc/image.hpp"
#include "rtos/plc/vm.hpp"

namespace plc = rtos::plc;

namespace {
// A small budget limit: the target here is the verifier, and every input must stay fast. The limit
// itself is the VM's (an image above it is refused) - found by this harness: images with budgets
// up to 2^32 held the VM for minutes.
constexpr std::uint32_t max_budget = 100'000;
}  // namespace

void fuzz_one(std::span<const std::uint8_t> input) {
    static std::array<std::uint8_t, 32 * 1024> data;
    static std::array<plc::Context, plc::max_tasks> contexts;
    static std::vector<std::uint8_t> image;
    image.assign(input.begin(), input.end());
    // the task budgets down to the harness's limit (below), like the CRC: else the compiler's seed
    // images (2,000,000) would all be refused
    if (image.size() >= plc::hdr::task_count + 4) {
        std::uint32_t at = 0, count = 0;
        std::memcpy(&at, &image[plc::hdr::task_offset], 4);
        std::memcpy(&count, &image[plc::hdr::task_count], 4);
        for (std::uint64_t i = 0; i < count && i < plc::max_tasks; ++i) {
            const std::uint64_t budget_at = at + i * plc::task_record_size + 4;
            if (budget_at + 4 > image.size()) break;
            std::uint32_t budget = 0;
            std::memcpy(&budget, &image[budget_at], 4);
            budget = std::min(budget, max_budget);
            std::memcpy(&image[budget_at], &budget, 4);
        }
    }
    if (image.size() >= plc::hdr::crc + 4) {
        const auto n = static_cast<std::uint32_t>(image.size());
        std::memcpy(&image[plc::hdr::length], &n, 4);
        const std::uint32_t crc = plc::crc32(std::span(image).subspan(plc::hdr::crc + 4));
        std::memcpy(&image[plc::hdr::crc], &crc, 4);
    }
    data.fill(0xA5);  // a stale data segment: load() must initialize what the program reads
    contexts = {};
    plc::Vm vm{data, contexts, max_budget};
    if (vm.load(image) != plc::LoadError::none) return;

    std::uint32_t now = 0;
    for (int cycle = 0; cycle < 3; ++cycle) {
        for (std::size_t t = 0; t < vm.task_count(); ++t) {
            (void)vm.run_cycle(t, now);
        }
        now += 10;
    }
    // the debugger: single steps into every call, then on to the end of the cycle
    vm.set_debug({.step = plc::StepKind::into});
    for (std::size_t t = 0; t < vm.task_count(); ++t) {
        plc::RunResult r = vm.run_cycle(t, now);
        for (int steps = 0; steps < 64 && r.state == plc::RunState::halted; ++steps) {
            r = vm.resume(t);
        }
        if (vm.halted(t)) vm.abandon(t);
    }
}
