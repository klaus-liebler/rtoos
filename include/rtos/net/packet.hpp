// SPDX-License-Identifier: MIT
//
// Zero-copy packet buffers.
//
// A PacketBuffer holds one Ethernet frame. Every layer works in place:
//   receive : the DMA writes the frame at the start of the buffer; each layer
//             strips its header with pull_front() and hands the same buffer on.
//   transmit: the payload is written after `headroom` bytes; each layer prepends
//             its header with push_front(). The driver DMAs from the buffer.
// Buffers are reference counted (PacketRef): TCP keeps a reference to every unacked
// segment for retransmission while the driver holds one until the DMA is done, so
// a segment is never copied. Counting is atomic: the last reference may be dropped
// in an interrupt.
//
// PacketPool<N> is a lock-free fixed pool (BlockPool), usable from any context. Put
// it in DMA-capable, non-cacheable memory (RTOS_DMA_BUFFER on the STM32H5).
#pragma once

#include <array>
#include <atomic>
#include <contracts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#include <span>
#include <utility>

#include "rtos/kernel/block_pool.hpp"

namespace rtos::net {

class PacketBuffer {
public:
    /// 1518-byte maximum frame (incl. FCS) rounded up to a multiple of 32: DMA and
    /// cache-line friendly.
    static constexpr std::size_t capacity = 1536;

    PacketBuffer(const PacketBuffer&) = delete;
    PacketBuffer& operator=(const PacketBuffer&) = delete;

    [[nodiscard]] std::byte* data() noexcept { return storage_.data() + offset_; }
    [[nodiscard]] const std::byte* data() const noexcept { return storage_.data() + offset_; }
    [[nodiscard]] std::size_t size() const noexcept { return length_; }
    [[nodiscard]] std::span<std::byte> bytes() noexcept { return {data(), length_}; }
    [[nodiscard]] std::span<const std::byte> bytes() const noexcept { return {data(), length_}; }

    /// Bytes available in front of data() (for headers) and behind it (for payload).
    [[nodiscard]] std::size_t headroom() const noexcept { return offset_; }
    [[nodiscard]] std::size_t tailroom() const noexcept { return capacity - offset_ - length_; }

    /// Start an empty buffer with `headroom` bytes reserved for headers.
    void reset(std::size_t headroom) noexcept {
        contract_assert(headroom <= capacity);
        offset_ = static_cast<std::uint16_t>(headroom);
        length_ = 0;
    }

    /// Grow at the front (prepend a header). Returns the new data().
    std::byte* push_front(std::size_t n) noexcept {
        contract_assert(n <= headroom());
        offset_ = static_cast<std::uint16_t>(offset_ - n);
        length_ = static_cast<std::uint16_t>(length_ + n);
        return data();
    }

    /// Shrink at the front (strip a header).
    void pull_front(std::size_t n) noexcept {
        contract_assert(n <= size());
        offset_ = static_cast<std::uint16_t>(offset_ + n);
        length_ = static_cast<std::uint16_t>(length_ - n);
    }

    /// Append `n` bytes (already written behind the payload, e.g. by the caller).
    std::byte* append(std::size_t n) noexcept {
        contract_assert(n <= tailroom());
        std::byte* tail = data() + length_;
        length_ = static_cast<std::uint16_t>(length_ + n);
        return tail;
    }

    void append(std::span<const std::byte> src) noexcept {
        contract_assert(src.size() <= tailroom());
        std::memcpy(append(src.size()), src.data(), src.size());
    }

    /// Drop bytes at the end (e.g. Ethernet padding).
    void truncate(std::size_t n) noexcept {
        contract_assert(n <= size());
        length_ = static_cast<std::uint16_t>(n);
    }

    /// Absolute view, used by the DMA driver and for retransmission.
    [[nodiscard]] std::size_t offset() const noexcept { return offset_; }
    void set_view(std::size_t offset, std::size_t length) noexcept {
        contract_assert(offset + length <= capacity);
        offset_ = static_cast<std::uint16_t>(offset);
        length_ = static_cast<std::uint16_t>(length);
    }
    [[nodiscard]] std::byte* storage() noexcept { return storage_.data(); }

    /// Number of PacketRefs to this buffer.
    [[nodiscard]] std::uint32_t use_count() const noexcept { return refs_.load(std::memory_order_acquire); }

    /// Free list / queue link for the owner of the buffer.
    PacketBuffer* next = nullptr;

private:
    template <std::size_t N>
    friend class PacketPool;
    friend class PacketRef;

    using Release = void (*)(void* pool, PacketBuffer* buffer) noexcept;
    PacketBuffer(Release release, void* pool) noexcept : release_(release), pool_(pool) {}

    alignas(32) std::array<std::byte, capacity> storage_;
    std::uint16_t offset_ = 0;
    std::uint16_t length_ = 0;
    std::atomic<std::uint16_t> refs_{1};
    Release release_;
    void* pool_;
};

/// Shared ownership of a PacketBuffer (intrusive, atomic count).
class PacketRef {
public:
    constexpr PacketRef() noexcept = default;
    explicit PacketRef(PacketBuffer* adopt) noexcept : buffer_(adopt) {}
    PacketRef(const PacketRef& other) noexcept : buffer_(other.buffer_) {
        if (buffer_ != nullptr) {
            buffer_->refs_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    PacketRef(PacketRef&& other) noexcept : buffer_(std::exchange(other.buffer_, nullptr)) {}
    PacketRef& operator=(PacketRef other) noexcept {
        std::swap(buffer_, other.buffer_);
        return *this;
    }
    ~PacketRef() { reset(); }

    void reset() noexcept {
        if (PacketBuffer* b = std::exchange(buffer_, nullptr)) {
            if (b->refs_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
                b->release_(b->pool_, b);
            }
        }
    }

    /// Give up ownership without dropping the reference (e.g. into a DMA descriptor).
    [[nodiscard]] PacketBuffer* release() noexcept { return std::exchange(buffer_, nullptr); }

    [[nodiscard]] PacketBuffer* get() const noexcept { return buffer_; }
    PacketBuffer& operator*() const noexcept {
        contract_assert(buffer_ != nullptr);
        return *buffer_;
    }
    PacketBuffer* operator->() const noexcept {
        contract_assert(buffer_ != nullptr);
        return buffer_;
    }
    explicit operator bool() const noexcept { return buffer_ != nullptr; }

private:
    PacketBuffer* buffer_ = nullptr;
};

/// Lock-free pool of N packet buffers. Any context.
template <std::size_t N>
class PacketPool {
public:
    PacketPool() noexcept = default;
    PacketPool(const PacketPool&) = delete;
    PacketPool& operator=(const PacketPool&) = delete;

    /// A fresh buffer with `headroom` reserved, or an empty ref if exhausted.
    [[nodiscard]] PacketRef allocate(std::size_t headroom = 0) noexcept {
        void* block = blocks_.allocate();
        if (block == nullptr) {
            return {};
        }
        auto* buffer = ::new (block) PacketBuffer(&PacketPool::release, this);
        buffer->reset(headroom);
        return PacketRef(buffer);
    }

    [[nodiscard]] std::size_t available() const noexcept { return blocks_.available(); }
    [[nodiscard]] static constexpr std::size_t capacity() noexcept { return N; }

private:
    static void release(void* self, PacketBuffer* buffer) noexcept {
        buffer->~PacketBuffer();
        static_cast<PacketPool*>(self)->blocks_.deallocate(buffer);
    }

    BlockPool<sizeof(PacketBuffer), N, alignof(PacketBuffer)> blocks_{};
};

/// Anything that hands out packet buffers.
template <class P>
concept PacketAllocator = requires(P& p, std::size_t headroom) {
    { p.allocate(headroom) } noexcept -> std::same_as<PacketRef>;
    { p.available() } noexcept -> std::convertible_to<std::size_t>;
};

}  // namespace rtos::net
