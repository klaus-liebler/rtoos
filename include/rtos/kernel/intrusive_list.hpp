// SPDX-License-Identifier: MIT
//
// Circular, doubly linked intrusive list. The nodes live inside the objects they
// link (e.g. the TCB), so no operation allocates and insert/remove are O(1).
#pragma once

#include <contracts>
#include <utility>

namespace rtos {

template <class T>
class Link {
public:
    /// A detached node points to itself; `owner` is the object that embeds the node.
    constexpr explicit Link(T* owner = nullptr) noexcept : next_(this), prev_(this), owner_(owner) {}

    // A node's address is its identity: copying would corrupt the neighbours' pointers.
    Link(const Link&) = delete;
    Link& operator=(const Link&) = delete;

    [[nodiscard]] bool linked() const noexcept { return next_ != this; }
    [[nodiscard]] T* owner() const noexcept { return owner_; }
    [[nodiscard]] Link* next() const noexcept { return next_; }

    /// Remove this node from whatever list it is in. No-op on a detached node.
    void unlink() noexcept {
        prev_->next_ = next_;
        next_->prev_ = prev_;
        next_ = prev_ = this;
    }

    /// Insert this (detached) node directly in front of `pos`.
    void insert_before(Link& pos) noexcept {
        contract_assert(!linked());
        next_ = &pos;
        prev_ = pos.prev_;
        pos.prev_->next_ = this;
        pos.prev_ = this;
    }

private:
    Link* next_;
    Link* prev_;
    T* owner_;
};

template <class T>
class IntrusiveList {
public:
    constexpr IntrusiveList() noexcept = default;
    IntrusiveList(const IntrusiveList&) = delete;
    IntrusiveList& operator=(const IntrusiveList&) = delete;

    [[nodiscard]] bool empty() const noexcept { return !head_.linked(); }

    void push_back(Link<T>& node) noexcept { node.insert_before(head_); }
    void push_front(Link<T>& node) noexcept { node.insert_before(*head_.next()); }

    [[nodiscard]] T* front() const noexcept { return empty() ? nullptr : head_.next()->owner(); }

    [[nodiscard]] bool is_front(const Link<T>& node) const noexcept { return head_.next() == &node; }

    /// True if the list holds at least two elements.
    [[nodiscard]] bool has_multiple() const noexcept {
        return !empty() && head_.next()->next() != &head_;
    }

    /// Move the first element to the back (round robin). O(1).
    void rotate() noexcept {
        if (has_multiple()) {
            Link<T>* first = head_.next();
            first->unlink();
            push_back(*first);
        }
    }

    /// Visit every element; `fn` may unlink the element it is given.
    template <class Fn>
    void for_each_safe(Fn&& fn) noexcept(noexcept(fn(std::declval<T&>()))) {
        for (Link<T>* node = head_.next(); node != &head_;) {
            Link<T>* following = node->next();
            fn(*node->owner());
            node = following;
        }
    }

private:
    Link<T> head_{};  // sentinel
};

}  // namespace rtos
