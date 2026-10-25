// SPDX-License-Identifier: MIT
//
// Fixed-size block pool (ThreadX tx_block_pool) - deterministic replacement for heap.
//
// Lock-free Treiber stack over block *indices*:
//   head_ = [ 16-bit ABA tag | 16-bit index of first free block ]
// A single 32-bit CAS updates both, so the pool is lock-free on ARMv8-M, which has
// LDREX/STREX but no 64-bit LDREXD. The free-list links live in a separate atomic
// array (not inside the blocks), so a racing pop never reads memory the winner has
// already handed out -> no data race on block contents.
//
// allocate / deallocate are O(1); the CAS loop retries only when an interrupt
// touched the same pool in between (bounded by the number of nested interrupts).
#pragma once

#include <array>
#include <atomic>
#include <contracts>
#include <cstddef>
#include <cstdint>

namespace rtos {

template <std::size_t BlockSize, std::size_t BlockCount, std::size_t Align = alignof(std::max_align_t)>
    requires(BlockSize >= 1 && BlockCount >= 1 && BlockCount < 0xFFFF && (Align & (Align - 1)) == 0)
class BlockPool {
public:
    /// Distance between blocks: BlockSize rounded up to the alignment.
    static constexpr std::size_t stride = (BlockSize + Align - 1) & ~(Align - 1);
    static constexpr std::size_t capacity = BlockCount;

    BlockPool() noexcept {
        for (std::size_t i = 0; i < BlockCount; ++i) {
            next_[i].store(static_cast<std::uint16_t>(i + 1 < BlockCount ? i + 1 : nil),
                           std::memory_order_relaxed);
        }
        head_.store(pack(0, 0), std::memory_order_relaxed);
        free_.store(BlockCount, std::memory_order_relaxed);
    }

    BlockPool(const BlockPool&) = delete;
    BlockPool& operator=(const BlockPool&) = delete;

    /// Returns a block or nullptr if the pool is exhausted. ISR-safe.
    [[nodiscard]] void* allocate() noexcept {
        std::uint32_t head = head_.load(std::memory_order_acquire);
        for (;;) {
            const std::uint16_t index = index_of(head);
            if (index == nil) {
                return nullptr;
            }
            const std::uint16_t next = next_[index].load(std::memory_order_relaxed);
            if (head_.compare_exchange_weak(head, pack(tag_of(head) + 1, next),
                                            std::memory_order_acq_rel, std::memory_order_acquire)) {
                free_.fetch_sub(1, std::memory_order_relaxed);
                return block(index);
            }
        }
    }

    /// Returns `p` to the pool. ISR-safe.
    void deallocate(void* p) noexcept {
        contract_assert(owns(p));
        const auto index = static_cast<std::uint16_t>(
            (static_cast<std::byte*>(p) - storage_) / static_cast<std::ptrdiff_t>(stride));
        std::uint32_t head = head_.load(std::memory_order_acquire);
        for (;;) {
            next_[index].store(index_of(head), std::memory_order_relaxed);
            if (head_.compare_exchange_weak(head, pack(tag_of(head) + 1, index),
                                            std::memory_order_acq_rel, std::memory_order_acquire)) {
                free_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
        }
    }

    /// True if `p` is the start of a block of this pool.
    [[nodiscard]] bool owns(const void* p) const noexcept {
        const auto addr = reinterpret_cast<std::uintptr_t>(p);
        const auto base = reinterpret_cast<std::uintptr_t>(storage_);
        return addr >= base && addr < base + sizeof(storage_) && (addr - base) % stride == 0;
    }

    [[nodiscard]] std::size_t available() const noexcept { return free_.load(std::memory_order_relaxed); }

private:
    static constexpr std::uint16_t nil = 0xFFFF;

    static constexpr std::uint32_t pack(std::uint32_t tag, std::uint16_t index) noexcept {
        return (tag << 16) | index;
    }
    static constexpr std::uint16_t index_of(std::uint32_t v) noexcept { return static_cast<std::uint16_t>(v); }
    static constexpr std::uint32_t tag_of(std::uint32_t v) noexcept { return v >> 16; }

    void* block(std::uint16_t index) noexcept { return storage_ + std::size_t{index} * stride; }

    alignas(Align) std::byte storage_[stride * BlockCount];
    std::array<std::atomic<std::uint16_t>, BlockCount> next_{};
    std::atomic<std::uint32_t> head_{};
    std::atomic<std::size_t> free_{};
};

}  // namespace rtos
