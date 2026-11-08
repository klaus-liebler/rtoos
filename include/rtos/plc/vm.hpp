// SPDX-License-Identifier: MIT
//
// The PLC bytecode VM (docs/plc.md). No heap, no static state: the application provides the data
// segment and one execution context per task. The image is verified completely by load(); the
// interpreter only checks what depends on run-time values (indirect addresses, stacks, division,
// conversions, CHK_*) and the instruction budget.
//
//     std::array<std::uint8_t, 32 * 1024> data;
//     std::array<rtos::plc::Context, 1> contexts;
//     rtos::plc::Vm vm{data, contexts};
//     if (vm.load(image) != LoadError::none) ...          // image must stay valid while loaded
//     const auto r = vm.run_cycle(0, now_ms);              // task 0: all its programs, once
//
// Not thread-safe: one thread runs a task's cycle at a time (several tasks of one Vm: several
// threads, each with its own context, share the data segment like IEC tasks share globals).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/plc/image.hpp"
#include "rtos/plc/isa.hpp"

namespace rtos::plc {

inline constexpr std::size_t operand_stack_slots = 64;
inline constexpr std::size_t call_depth = 32;
inline constexpr std::size_t max_tasks = 8;

/// The execution state of one task (docs/plc.md "Machine model").
struct Context {
    struct Call {
        std::uint32_t return_pc;  ///< no_return: the task's top call of a program
        std::uint32_t fp;
        std::uint32_t self;
    };
    static constexpr std::uint32_t no_return = 0xFFFF'FFFF;

    std::array<std::uint64_t, operand_stack_slots> stack{};
    std::array<Call, call_depth> calls{};
    std::uint32_t sp = 0;     ///< slots in use
    std::uint32_t depth = 0;  ///< calls in use
    std::uint32_t pc = 0;
    std::uint32_t fp = 0;
    std::uint32_t self = 0;   ///< THIS
    std::uint32_t frame_top = 0;
    std::uint32_t frame_end = 0;
    std::uint32_t now_ms = 0;
};

enum class RunState : std::uint8_t { done, trapped };

struct RunResult {
    RunState state = RunState::done;
    Trap trap = Trap::NONE;
    std::uint16_t trap_code = 0;     ///< TRAP's operand
    std::uint32_t pc = 0;            ///< the instruction that trapped
    std::uint32_t instructions = 0;  ///< executed in this call
};

class Vm {
public:
    Vm(std::span<std::uint8_t> data, std::span<Context> contexts) noexcept : data_(data), contexts_(contexts) {}
    Vm(const Vm&) = delete;
    Vm& operator=(const Vm&) = delete;

    /// Verifies the image and initializes the data segment. On an error nothing is loaded.
    [[nodiscard]] LoadError load(std::span<const std::uint8_t> image) noexcept;
    void unload() noexcept { loaded_ = false; }
    [[nodiscard]] bool loaded() const noexcept { return loaded_; }

    /// One cycle of `task`: every program of it, in order. `now_ms`: the cycle's start time (TIME_MS).
    [[nodiscard]] RunResult run_cycle(std::size_t task, std::uint32_t now_ms) noexcept;

    [[nodiscard]] std::size_t task_count() const noexcept { return task_count_; }
    [[nodiscard]] TaskInfo task(std::size_t i) const noexcept;
    [[nodiscard]] std::size_t io_count() const noexcept { return io_count_; }
    [[nodiscard]] IoBinding io(std::size_t i) const noexcept;
    [[nodiscard]] std::uint32_t io_map_hash() const noexcept { return rd32(&image_[hdr::io_map_hash]); }
    [[nodiscard]] std::uint32_t project_hash() const noexcept { return rd32(&image_[hdr::project_hash]); }

    /// The program's variables (watch, force, process image, tests): data_size() bytes.
    [[nodiscard]] std::span<std::uint8_t> data() noexcept { return data_.first(data_size_); }
    [[nodiscard]] std::size_t data_size() const noexcept { return data_size_; }

private:
    LoadError verify_code() noexcept;
    RunResult execute(Context& c, std::uint32_t budget) noexcept;

    std::span<std::uint8_t> data_;
    std::span<Context> contexts_;
    std::span<const std::uint8_t> image_;
    std::span<const std::uint8_t> code_;
    std::uint32_t data_size_ = 0;
    std::uint32_t task_offset_ = 0;
    std::size_t task_count_ = 0;
    std::uint32_t io_offset_ = 0;
    std::size_t io_count_ = 0;
    std::array<bool, native_count> natives_listed_{};
    bool loaded_ = false;
};

}  // namespace rtos::plc
