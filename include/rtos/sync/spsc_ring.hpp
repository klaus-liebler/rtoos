// SPDX-License-Identifier: MIT
//
// Lock-free single-producer / single-consumer ring of N elements (N a power of two).
// One context pushes (e.g. an interrupt handler), one other context pops (e.g. the
// network thread); neither ever blocks or takes a lock, so it works from zero-latency
// interrupts too. Element copies only (T trivially copyable).
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>

namespace rtos {

template <class T, std::size_t N>
class SpscRing {
    static_assert(N >= 2 && (N & (N - 1)) == 0, "N must be a power of two");
    static_assert(std::is_trivially_copyable_v<T>);

public:
    /// Producer side. False if full (the element is dropped).
    bool push(const T& value) noexcept {
        const std::uint32_t head = head_.load(std::memory_order_relaxed);
        if (head - tail_.load(std::memory_order_acquire) == N) {
            return false;
        }
        slots_[head % N] = value;
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    /// Consumer side.
    std::optional<T> pop() noexcept {
        const std::uint32_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return std::nullopt;
        }
        const T value = slots_[tail % N];
        tail_.store(tail + 1, std::memory_order_release);
        return value;
    }

    [[nodiscard]] std::size_t size() const noexcept {
        return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
    }
    [[nodiscard]] bool empty() const noexcept { return size() == 0; }
    [[nodiscard]] static constexpr std::size_t capacity() noexcept { return N; }

private:
    std::array<T, N> slots_{};
    std::atomic<std::uint32_t> head_{0};
    std::atomic<std::uint32_t> tail_{0};
};

/// Byte stream version: one producer writes, one consumer reads (block copies).
template <std::size_t N>
class ByteRing {
    static_assert(N >= 2 && (N & (N - 1)) == 0, "N must be a power of two");

public:
    /// Producer: copy as much of `data` as fits; returns the bytes taken.
    std::size_t write(std::span<const std::byte> data) noexcept {
        const std::uint32_t head = head_.load(std::memory_order_relaxed);
        const std::uint32_t tail = tail_.load(std::memory_order_acquire);
        const std::size_t n = std::min<std::size_t>(data.size(), N - (head - tail));
        for (std::size_t i = 0; i < n; ++i) {
            bytes_[(head + i) % N] = data[i];
        }
        head_.store(head + static_cast<std::uint32_t>(n), std::memory_order_release);
        return n;
    }

    /// Consumer: copy up to out.size() bytes; returns the bytes taken.
    std::size_t read(std::span<std::byte> out) noexcept {
        const std::uint32_t tail = tail_.load(std::memory_order_relaxed);
        const std::uint32_t head = head_.load(std::memory_order_acquire);
        const std::size_t n = std::min<std::size_t>(out.size(), head - tail);
        for (std::size_t i = 0; i < n; ++i) {
            out[i] = bytes_[(tail + i) % N];
        }
        tail_.store(tail + static_cast<std::uint32_t>(n), std::memory_order_release);
        return n;
    }

    [[nodiscard]] std::size_t size() const noexcept {
        return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
    }
    [[nodiscard]] std::size_t space() const noexcept { return N - size(); }
    [[nodiscard]] static constexpr std::size_t capacity() noexcept { return N; }
    /// Only while neither side runs (e.g. reset by the consumer's owner).
    void clear() noexcept { tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release); }

private:
    std::array<std::byte, N> bytes_{};
    std::atomic<std::uint32_t> head_{0};
    std::atomic<std::uint32_t> tail_{0};
};

}  // namespace rtos
