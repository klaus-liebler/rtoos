// SPDX-License-Identifier: MIT
//
// Bounded message queue of trivially copyable messages (FIFO).
//
// For zero-copy designs the message is a handle (pointer, pool index, buffer
// descriptor); the payload stays where it is.
//
//   send(msg, timeout)       thread         blocks while full
//   receive(timeout)         thread         blocks while empty
//   try_send / try_receive   thread         never block
//   try_send_from_isr /      kernel-aware ISR
//   try_receive_from_isr
//
// Direct hand-over: a sender finding a blocked receiver copies the message straight
// into the receiver's destination; a receiver making room moves the first blocked
// sender's message into the ring. Message order is preserved, every operation is
// O(1), and no woken thread can be overtaken.
#pragma once

#include <array>
#include <contracts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>

#include "rtos/config.hpp"
#include "rtos/kernel/thread.hpp"

namespace rtos {

template <class K, class T, std::size_t Capacity>
    requires(std::is_trivially_copyable_v<T> && std::is_default_constructible_v<T> && Capacity >= 1)
class MessageQueue {
public:
    explicit MessageQueue(K& kernel) noexcept : kernel_(kernel) {}
    MessageQueue(const MessageQueue&) = delete;
    MessageQueue& operator=(const MessageQueue&) = delete;

    // ---- send ------------------------------------------------------------------

    /// Phase 1: true = sent, false = full (only with no_wait), nullopt = blocked
    /// (then K::wait_end(): Ok = sent). `msg` must stay valid until then.
    std::optional<bool> send_begin(const T& msg, Tick timeout) noexcept {
        contract_assert(!K::in_isr());
        typename K::Lock lock;
        if (offer_locked(msg)) {
            return true;
        }
        if (timeout == no_wait) {
            return false;
        }
        kernel_.block_current_locked(senders_, timeout, reinterpret_cast<std::uintptr_t>(&msg));
        return std::nullopt;
    }

    bool send(const T& msg, Tick timeout = wait_forever) noexcept {
        if (auto sent = send_begin(msg, timeout)) {
            return *sent;
        }
        return kernel_.wait_end().result == WaitResult::Ok;
    }

    bool try_send(const T& msg) noexcept { return send(msg, no_wait); }

    bool try_send_from_isr(const T& msg) noexcept {
        contract_assert(K::in_isr());
        contract_assert(K::may_lock());
        typename K::Lock lock;
        return offer_locked(msg);
    }

    // ---- receive ---------------------------------------------------------------

    /// Phase 1: true = `out` filled, false = empty (only with no_wait), nullopt =
    /// blocked (then K::wait_end(): Ok = `out` filled). `out` must stay valid.
    std::optional<bool> receive_begin(T& out, Tick timeout) noexcept {
        contract_assert(!K::in_isr());
        typename K::Lock lock;
        if (take_locked(out)) {
            return true;
        }
        if (timeout == no_wait) {
            return false;
        }
        kernel_.block_current_locked(receivers_, timeout, reinterpret_cast<std::uintptr_t>(&out));
        return std::nullopt;
    }

    std::optional<T> receive(Tick timeout = wait_forever) noexcept {
        T out;
        if (auto received = receive_begin(out, timeout)) {
            return *received ? std::optional<T>(out) : std::nullopt;
        }
        return kernel_.wait_end().result == WaitResult::Ok ? std::optional<T>(out) : std::nullopt;
    }

    std::optional<T> try_receive() noexcept { return receive(no_wait); }

    std::optional<T> try_receive_from_isr() noexcept {
        contract_assert(K::in_isr());
        contract_assert(K::may_lock());
        typename K::Lock lock;
        T out;
        return take_locked(out) ? std::optional<T>(out) : std::nullopt;
    }

    [[nodiscard]] std::size_t size() const noexcept {
        typename K::Lock lock;
        return count_;
    }
    [[nodiscard]] static constexpr std::size_t capacity() noexcept { return Capacity; }

private:
    /// Deliver to a blocked receiver or append to the ring.
    bool offer_locked(const T& msg) noexcept {
        if (auto* receiver = K::front_waiter_locked(receivers_)) {
            *reinterpret_cast<T*>(receiver->wait_data) = msg;
            kernel_.wake_one_locked(receivers_, receiver->wait_data);
            return true;
        }
        if (count_ == Capacity) {
            return false;
        }
        ring_[(head_ + count_) % Capacity] = msg;
        ++count_;
        return true;
    }

    /// Pop the oldest message; refill from the first blocked sender.
    bool take_locked(T& out) noexcept {
        if (count_ == 0) {
            return false;
        }
        out = ring_[head_];
        head_ = (head_ + 1) % Capacity;
        --count_;
        if (auto* sender = K::front_waiter_locked(senders_)) {
            ring_[(head_ + count_) % Capacity] = *reinterpret_cast<const T*>(sender->wait_data);
            ++count_;
            kernel_.wake_one_locked(senders_);
        }
        return true;
    }

    K& kernel_;
    typename K::WaitQueue senders_{};
    typename K::WaitQueue receivers_{};
    std::array<T, Capacity> ring_{};
    std::size_t head_ = 0;
    std::size_t count_ = 0;
};

}  // namespace rtos
