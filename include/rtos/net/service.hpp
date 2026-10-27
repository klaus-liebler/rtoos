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
//         kernel.notify_wait(slot, NetRuntime::timeout(wake, kernel.now()));
//     }
//
// The drivers' interrupts wake the thread (notify_from_isr); other threads that
// want a service to act (e.g. queue a message) wake it the same way.
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
        entries_[count_++] = {&service, [](void* s) noexcept { static_cast<S*>(s)->service(); },
                              [](const void* s) noexcept -> std::uint32_t {
                                  return static_cast<const S*>(s)->next_deadline();
                              },
                              name};
        return true;
    }

    /// One pass: receive and process frames, then run every service. Returns the
    /// absolute time of the next pass needed without new frames (0 = none).
    std::uint32_t run_once(std::uint32_t now_ms) noexcept {
        stack_.poll(now_ms);
        std::uint32_t wake = stack_.next_deadline();
        for (std::size_t i = 0; i < count_; ++i) {
            entries_[i].service(entries_[i].self);
        }
        stack_.flush();  // FINs, window updates the services caused: now, not next pass
        for (std::size_t i = 0; i < count_; ++i) {
            wake = earliest(wake, entries_[i].deadline(entries_[i].self));
        }
        return earliest(wake, stack_.next_deadline());
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

private:
    struct Entry {
        void* self = nullptr;
        void (*service)(void*) noexcept = nullptr;
        std::uint32_t (*deadline)(const void*) noexcept = nullptr;
        std::string_view name;
    };

    Stack& stack_;
    std::array<Entry, MaxServices> entries_{};
    std::size_t count_ = 0;
};

}  // namespace rtos::net
