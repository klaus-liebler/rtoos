// SPDX-License-Identifier: MIT
//
// Network services run "in process" by the network thread.
//
// A service is anything with
//     void service() noexcept                       do the work that is due
//     std::uint32_t next_deadline() const noexcept  absolute time (stack ms) it must run
//                                                    again without network events; 0 = none
// (DHCP, DNS, mDNS, SNTP, TcpServer-based servers such as HTTPS, later Modbus TCP and
// OPC UA, application tasks). Services receive network input through the stack's
// callbacks (UDP handlers, TCP events) during Stack::poll(), which runs in the same
// thread: no locks anywhere.
//
// NetRuntime is the loop body of that thread:
//
//     runtime.add(dhcp); runtime.add(mdns); runtime.add(https); ...
//     for (;;) {
//         const auto wake = runtime.run_once(kernel.now());
//         kernel.notify_wait(slot, runtime.sleep_ms(wake, kernel.now()));
//     }
//
// The drivers' interrupts wake the thread (notify_from_isr); other threads that
// want a service to act (e.g. queue a message) wake it the same way.
//
// Fairness in one thread. Work comes in two kinds:
//   * foreground: service() and the TCP/UDP callbacks during Stack::poll() - short
//     and bounded (answer a Modbus or OPC UA request, a WebSocket message, a timer);
//   * background (optional, BackgroundService): expensive work in bounded units -
//     a TLS handshake, the next record of a large download:
//         bool background_pending() const noexcept
//         void background_step() noexcept      one unit
// Each pass runs all foreground work and then ONE background step, round robin over
// the services that have some (TcpServer: round robin over its sessions). While
// background work is pending the loop does not sleep (sleep_ms() == 0), but every
// step is followed by a fresh Stack::poll(): a request waits for at most one step,
// however many downloads are running. Together with the TCP bulk budget
// (TcpConnectionBase::set_bulk) - downloads cannot take the transmit buffers the
// replies need - interactive latency stays independent of the background load.
//
// Measurement: with a cycle counter (set_cycle_counter) the runtime records per
// service the longest service() call and the longest background step (stats()).
#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <contracts>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace rtos::net {

template <class S>
concept NetService = requires(S& s, const S& cs) {
    { s.service() } noexcept;
    { cs.next_deadline() } noexcept -> std::convertible_to<std::uint32_t>;
};

template <class S>
concept BackgroundService = NetService<S> && requires(S& s, const S& cs) {
    { cs.background_pending() } noexcept -> std::same_as<bool>;
    { s.background_step() } noexcept;
};

/// Free-running cycle counter (DWT_CYCCNT on the target).
using CycleCounter = std::uint32_t (*)() noexcept;

struct ServiceStats {
    std::uint32_t max_service_cycles = 0;  ///< longest service() call
    std::uint32_t max_step_cycles = 0;     ///< longest background step
    std::uint32_t steps = 0;               ///< background steps run
};

/// Earliest of two absolute deadlines (0 = none), wrap-around safe.
[[nodiscard]] constexpr std::uint32_t earliest(std::uint32_t a, std::uint32_t b) noexcept {
    if (a == 0) {
        return b;
    }
    if (b == 0) {
        return a;
    }
    return static_cast<std::int32_t>(a - b) < 0 ? a : b;
}

template <class Stack, std::size_t MaxServices = 16>
class NetRuntime {
public:
    /// Upper bound for one sleep (also the period without any deadline).
    static constexpr std::uint32_t max_sleep_ms = 1000;

    explicit NetRuntime(Stack& stack) noexcept : stack_(stack) {}

    NetRuntime(const NetRuntime&) = delete;
    NetRuntime& operator=(const NetRuntime&) = delete;

    /// Register a service (it must outlive the runtime); run in registration order.
    template <NetService S>
    bool add(S& service, std::string_view name = {}) noexcept {
        if (count_ == entries_.size()) {
            return false;
        }
        Entry e{&service, [](void* s) noexcept { static_cast<S*>(s)->service(); },
                [](const void* s) noexcept -> std::uint32_t { return static_cast<const S*>(s)->next_deadline(); },
                nullptr, nullptr, name, {}};
        if constexpr (BackgroundService<S>) {
            e.pending = [](const void* s) noexcept { return static_cast<const S*>(s)->background_pending(); };
            e.step = [](void* s) noexcept { static_cast<S*>(s)->background_step(); };
        }
        entries_[count_++] = e;
        return true;
    }

    void set_cycle_counter(CycleCounter counter) noexcept { cycles_ = counter; }

    /// One pass: receive and process frames, run every service, then one background
    /// step. Returns the absolute time of the next pass needed without new frames
    /// (0 = none); busy() tells whether background work is waiting.
    std::uint32_t run_once(std::uint32_t now_ms) noexcept {
        stack_.poll(now_ms);
        for (std::size_t i = 0; i < count_; ++i) {
            Entry& e = entries_[i];
            const std::uint32_t t0 = cycles();
            e.service(e.self);
            e.stats.max_service_cycles = std::max(e.stats.max_service_cycles, cycles() - t0);
        }
        busy_ = false;
        for (std::size_t k = 0; k < count_; ++k) {
            Entry& e = entries_[(next_ + k) % count_];
            if (e.step != nullptr && e.pending(e.self)) {
                next_ = (next_ + k + 1) % count_;
                const std::uint32_t t0 = cycles();
                e.step(e.self);
                e.stats.max_step_cycles = std::max(e.stats.max_step_cycles, cycles() - t0);
                ++e.stats.steps;
                break;
            }
        }
        stack_.flush();  // FINs, window updates the services caused: now, not next pass
        std::uint32_t wake = stack_.next_deadline();
        for (std::size_t i = 0; i < count_; ++i) {
            const Entry& e = entries_[i];
            wake = earliest(wake, e.deadline(e.self));
            busy_ = busy_ || (e.pending != nullptr && e.pending(e.self));
        }
        return wake;
    }

    /// Background work is waiting: run the next pass without sleeping.
    [[nodiscard]] bool busy() const noexcept { return busy_; }

    /// Milliseconds to sleep after run_once() returned `wake`: 0 while busy(), else
    /// until `wake` (at least 1, at most max_sleep_ms).
    [[nodiscard]] std::uint32_t sleep_ms(std::uint32_t wake, std::uint32_t now_ms) const noexcept {
        return busy_ ? 0 : timeout(wake, now_ms);
    }

    /// Milliseconds to sleep until `wake` (at least 1, at most max_sleep_ms).
    [[nodiscard]] static std::uint32_t timeout(std::uint32_t wake, std::uint32_t now_ms) noexcept {
        if (wake == 0) {
            return max_sleep_ms;
        }
        const auto left = static_cast<std::int32_t>(wake - now_ms);
        return left <= 0 ? 1 : std::min<std::uint32_t>(static_cast<std::uint32_t>(left), max_sleep_ms);
    }

    [[nodiscard]] std::size_t size() const noexcept { return count_; }
    [[nodiscard]] std::string_view name(std::size_t i) const noexcept { return entries_[i].name; }
    [[nodiscard]] const ServiceStats& stats(std::size_t i) const noexcept { return entries_[i].stats; }
    void reset_stats() noexcept {
        for (auto& e : entries_) {
            e.stats = {};
        }
    }

private:
    struct Entry {
        void* self = nullptr;
        void (*service)(void*) noexcept = nullptr;
        std::uint32_t (*deadline)(const void*) noexcept = nullptr;
        bool (*pending)(const void*) noexcept = nullptr;  ///< background (optional)
        void (*step)(void*) noexcept = nullptr;
        std::string_view name;
        ServiceStats stats;
    };

    [[nodiscard]] std::uint32_t cycles() const noexcept { return cycles_ != nullptr ? cycles_() : 0; }

    Stack& stack_;
    std::array<Entry, MaxServices> entries_{};
    std::size_t count_ = 0;
    std::size_t next_ = 0;  ///< round robin of the background steps
    bool busy_ = false;
    CycleCounter cycles_ = nullptr;
};

}  // namespace rtos::net
