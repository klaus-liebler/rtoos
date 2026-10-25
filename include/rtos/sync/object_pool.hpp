// SPDX-License-Identifier: MIT
//
// Typed object pool with O(1) allocate/free, built on the lock-free BlockPool.
//
//   try_make(args...)          any context, incl. zero-latency ISR   never blocks
//   make(timeout, args...)     thread                                blocks while empty
//   Ptr (returned handle)      unique ownership; destroys + frees on scope exit (thread)
//   destroy(p)                 thread
//   destroy_from_isr(p)        any ISR: kernel-aware -> direct hand-over,
//                              zero-latency -> lock-free free + deferred wake-up
//
// Raw storage API (no construction): try_allocate / allocate / allocate_begin /
// deallocate / deallocate_from_isr.
//
// A freed block goes straight to the first blocked allocator (FIFO), so a woken
// thread can never lose it to another thread. Zero-latency interrupts cannot take
// the kernel lock: they return the block to the lock-free free list and post a
// Deferred refill that hands free blocks to waiters at the next scheduling point.
#pragma once

#include <atomic>
#include <contracts>
#include <cstddef>
#include <cstdint>
#include <new>
#include <optional>
#include <type_traits>
#include <utility>

#include "rtos/config.hpp"
#include "rtos/kernel/block_pool.hpp"
#include "rtos/kernel/thread.hpp"
#include "rtos/kernel/wait_queue.hpp"

namespace rtos {

template <class K, class T, std::size_t Count>
class ObjectPool {
public:
    /// Owning handle; frees the object when it goes out of scope (thread context).
    /// In an ISR use release() and destroy_from_isr().
    class Ptr {
    public:
        constexpr Ptr() noexcept = default;
        Ptr(Ptr&& other) noexcept : pool_(other.pool_), object_(std::exchange(other.object_, nullptr)) {}
        Ptr& operator=(Ptr&& other) noexcept {
            if (this != &other) {
                reset();
                pool_ = other.pool_;
                object_ = std::exchange(other.object_, nullptr);
            }
            return *this;
        }
        ~Ptr() { reset(); }

        void reset() noexcept {
            if (object_ != nullptr) {
                pool_->destroy(std::exchange(object_, nullptr));
            }
        }
        [[nodiscard]] T* release() noexcept { return std::exchange(object_, nullptr); }
        [[nodiscard]] T* get() const noexcept { return object_; }
        T& operator*() const noexcept {
            contract_assert(object_ != nullptr);
            return *object_;
        }
        T* operator->() const noexcept {
            contract_assert(object_ != nullptr);
            return object_;
        }
        explicit operator bool() const noexcept { return object_ != nullptr; }

    private:
        friend class ObjectPool;
        Ptr(ObjectPool* pool, T* object) noexcept : pool_(pool), object_(object) {}

        ObjectPool* pool_ = nullptr;
        T* object_ = nullptr;
    };

    explicit ObjectPool(K& kernel) noexcept : kernel_(kernel) {}
    ObjectPool(const ObjectPool&) = delete;
    ObjectPool& operator=(const ObjectPool&) = delete;

    // ---- typed API -------------------------------------------------------------

    template <class... Args>
    [[nodiscard]] Ptr try_make(Args&&... args) noexcept(std::is_nothrow_constructible_v<T, Args...>) {
        return construct(try_allocate(), std::forward<Args>(args)...);
    }

    template <class... Args>
    [[nodiscard]] Ptr make(Tick timeout, Args&&... args) noexcept(std::is_nothrow_constructible_v<T, Args...>) {
        return construct(allocate(timeout), std::forward<Args>(args)...);
    }

    void destroy(T* object) noexcept {
        contract_assert(owns(object));
        contract_assert(!K::in_isr());
        object->~T();
        deallocate(object);
    }

    void destroy_from_isr(T* object) noexcept {
        contract_assert(owns(object));
        contract_assert(K::in_isr());
        object->~T();
        deallocate_from_isr(object);
    }

    // ---- raw storage API ------------------------------------------------------

    /// Lock-free, any context. nullptr if exhausted.
    [[nodiscard]] void* try_allocate() noexcept { return blocks_.allocate(); }

    /// Phase 1: block address, nullptr (exhausted, only with no_wait) or nullopt
    /// (blocked; then K::wait_end(): Ok -> data is the block address).
    std::optional<void*> allocate_begin(Tick timeout) noexcept {
        contract_assert(!K::in_isr());
        typename K::Lock lock;
        if (void* block = blocks_.allocate()) {
            return block;
        }
        if (timeout == no_wait) {
            return nullptr;
        }
        kernel_.block_current_locked(waiters_, timeout);
        // A zero-latency ISR may have freed a block between the failed attempt and
        // the registration above (it saw no waiter, so it posted no refill). The
        // registration is seq_cst, as is the ISR's waiter check: retrying here
        // closes the window.
        std::atomic_thread_fence(std::memory_order_seq_cst);
        if (void* block = blocks_.allocate()) {
            kernel_.cancel_wait_locked();
            return block;
        }
        return std::nullopt;
    }

    /// Wait up to `timeout` ticks for a block. nullptr on timeout.
    [[nodiscard]] void* allocate(Tick timeout = wait_forever) noexcept {
        if (auto block = allocate_begin(timeout)) {
            return *block;
        }
        const auto outcome = kernel_.wait_end();
        return outcome.result == WaitResult::Ok ? reinterpret_cast<void*>(outcome.data) : nullptr;
    }

    void deallocate(void* block) noexcept {
        contract_assert(owns(block));
        contract_assert(!K::in_isr());
        release_locked(block);
    }

    void deallocate_from_isr(void* block) noexcept {
        contract_assert(owns(block));
        contract_assert(K::in_isr());
        if (K::may_lock()) {
            release_locked(block);
            return;
        }
        blocks_.deallocate(block);  // lock-free
        std::atomic_thread_fence(std::memory_order_seq_cst);
        if (waiters_.has_waiters()) {
            kernel_.defer_from_isr(refill_);
        }
    }

    [[nodiscard]] bool owns(const void* p) const noexcept { return blocks_.owns(p); }
    [[nodiscard]] std::size_t available() const noexcept { return blocks_.available(); }
    [[nodiscard]] static constexpr std::size_t capacity() noexcept { return Count; }

private:
    template <class... Args>
    Ptr construct(void* block, Args&&... args) {
        if (block == nullptr) {
            return {};
        }
        return Ptr(this, ::new (block) T(std::forward<Args>(args)...));
    }

    void release_locked(void* block) noexcept {
        typename K::Lock lock;
        if (kernel_.wake_one_locked(waiters_, reinterpret_cast<std::uintptr_t>(block)) == nullptr) {
            blocks_.deallocate(block);
        }
    }

    /// Deferred work (kernel lock held): hand free blocks to blocked allocators.
    static void refill(void* self) noexcept {
        auto& pool = *static_cast<ObjectPool*>(self);
        while (pool.waiters_.has_waiters()) {
            void* block = pool.blocks_.allocate();
            if (block == nullptr) {
                return;
            }
            pool.kernel_.wake_one_locked(pool.waiters_, reinterpret_cast<std::uintptr_t>(block));
        }
    }

    K& kernel_;
    BlockPool<sizeof(T), Count, alignof(T)> blocks_{};
    typename K::WaitQueue waiters_{};
    Deferred refill_{&ObjectPool::refill, this};
};

}  // namespace rtos
