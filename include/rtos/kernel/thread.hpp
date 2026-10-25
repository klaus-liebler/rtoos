// SPDX-License-Identifier: MIT
//
// Thread control block (TCB). Statically allocated by the application, never by the
// kernel. Contains the FreeRTOS-style notification array: per slot one 32-bit value
// and one flag byte, both lock-free atomics, so any interrupt (even one above the
// kernel's BASEPRI mask) can signal a thread without entering a critical section.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "rtos/config.hpp"
#include "rtos/kernel/intrusive_list.hpp"

namespace rtos {

template <class Tcb>
class WaitQueue;

enum class ThreadState : std::uint8_t {
    Inactive,       ///< Not yet created.
    Ready,          ///< In the ready queue (the running thread is Ready, too).
    Sleeping,       ///< Delayed; in the timer wheel only.
    WaitingNotify,  ///< Blocked on a notification slot; optionally in the timer wheel.
    Waiting,        ///< Blocked on a WaitQueue (semaphore, queue, pool); optionally timed.
    Suspended,      ///< Removed from scheduling until resume().
    Terminated,     ///< Entry function returned.
};

/// Outcome of a wait on a WaitQueue.
enum class WaitResult : std::uint8_t {
    Ok,       ///< Woken by the object (the request was satisfied).
    Timeout,  ///< The timeout expired first.
};

enum class NotifyAction : std::uint8_t {
    None,       ///< Only mark the slot pending (binary semaphore semantics).
    SetBits,    ///< value |= arg (event-group semantics).
    Increment,  ///< value += 1, arg ignored (counting semaphore semantics).
    Overwrite,  ///< value = arg (mailbox semantics, last writer wins).
};

/// Flag bits of a notification slot.
namespace notify_flag {
inline constexpr std::uint8_t pending = 0x01;  ///< An unconsumed notification is present.
inline constexpr std::uint8_t waiting = 0x02;  ///< The owner thread is (about to be) blocked on it.
}  // namespace notify_flag

template <std::size_t NotifySlots>
struct ThreadControlBlock {
    // ---- context (accessed by the port's switch code) -----------------------------
    void* sp = nullptr;              ///< Saved process stack pointer.
    std::uintptr_t stack_limit = 0;  ///< Lowest valid stack address (Cortex-M33: PSPLIM).

    // ---- scheduling ----------------------------------------------------------------
    Link<ThreadControlBlock> sched_link{this};  ///< Ready queue membership.
    Link<ThreadControlBlock> timer_link{this};  ///< Timer wheel membership.
    Link<ThreadControlBlock> wait_link{this};   ///< WaitQueue membership.
    void (*entry)(void*) = nullptr;
    void* arg = nullptr;
    const char* name = "";
    Tick wake_tick = 0;
    std::uint32_t time_slice = 0;  ///< Round-robin quantum in ticks, 0 = run to block.
    std::uint32_t slice_left = 0;
    std::uint8_t priority = 0;   ///< 0 = most urgent.
    std::uint8_t threshold = 0;  ///< Preemption threshold, <= priority (ThreadX).
    std::uint8_t id = 0xFF;      ///< Index in the kernel registry, used by wake bitmaps.
    std::uint8_t wait_index = 0;
    ThreadState state = ThreadState::Inactive;

    // ---- object waits (semaphores, queues, pools) ---------------------------------
    WaitQueue<ThreadControlBlock>* wait_queue = nullptr;  ///< Queue blocked on, if any.
    std::uintptr_t wait_data = 0;  ///< Payload exchanged with the waking side.
    WaitResult wait_result = WaitResult::Ok;

    // ---- task notifications (lock-free) -------------------------------------------
    std::array<std::atomic<std::uint32_t>, NotifySlots> notify_value{};
    std::array<std::atomic<std::uint8_t>, NotifySlots> notify_flags{};

    constexpr ThreadControlBlock() noexcept = default;
    ThreadControlBlock(const ThreadControlBlock&) = delete;
    ThreadControlBlock& operator=(const ThreadControlBlock&) = delete;
};

}  // namespace rtos
