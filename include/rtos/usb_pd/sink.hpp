// SPDX-License-Identifier: MIT
//
// USB Power Delivery sink: the protocol layer (message IDs, GoodCRC, retries; USB PD 3.1
// 6.6-6.7) and a sink policy that asks the source for a voltage and keeps it.
//
//   h5::Ucpd<ucpd_config> phy;                   // PHY: UCPD1 + GPDMA
//   h5::MicrosecondTimer<...> timer;             // deadlines: TIM2 compare
//   rtos::usb_pd::Sink<decltype(phy), decltype(timer), Kernel::Lock> sink{phy, timer};
//   sink.request_power(12'000);                  // remembered until a source offers it
//   phy.start(sink);                             // CC pull-downs on, attach events
//   UCPD1 IRQ:  phy.on_irq(sink);   TIM2 IRQ: if (timer.take()) sink.on_timer();
//   thread:     sink.poll([](SinkEvent e) { ... });  sink.status();
//
// Everything protocol-related runs in the PHY and timer interrupts, which must have the same
// priority (they don't preempt each other). Application calls (request_power, poll, status)
// run under `Guard`, which must mask both interrupts (target: Kernel::Lock with kernel-aware
// priorities). The sink offers revision 2.0 headers and answers what it does not support with
// Reject; it never sends anything a source would have to wait for.
//
// Ported from "USB Power Delivery for Arduino", Copyright (c) 2023 Manuel Bleichenbacher,
// MIT License (https://github.com/manuelbl/usb-pd-arduino), via the FCU firmware: no heap,
// no std::function, no busy waits, no global instance; a busy transmitter defers a request
// instead of spinning.
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/usb_pd/message.hpp"
#include "rtos/usb_pd/source_capability.hpp"

namespace rtos::usb_pd {

/// What a PHY reports, from its interrupt.
class PhyEvents {
public:
    /// CC line with a source attached (1, 2), or 0: detached.
    virtual void on_attach(std::uint8_t cc) noexcept = 0;
    /// A message with a valid CRC (the PHY already filtered by ordered set).
    virtual void on_message(const Message& m) noexcept = 0;
    virtual void on_rx_error() noexcept = 0;
    /// The message passed to transmit() went out (true), or was discarded/aborted (false).
    virtual void on_transmitted(bool ok) noexcept = 0;
    /// Hard Reset received, or our own Hard Reset sent.
    virtual void on_hard_reset() noexcept = 0;

protected:
    ~PhyEvents() = default;
};

template <class P>
concept Phy = requires(P& p, const Message& m) {
    { p.transmit(m) } -> std::same_as<bool>;  // copies the message; false: line not usable
    { p.transmit_hard_reset() } -> std::same_as<bool>;
};

/// A free-running microsecond clock with one compare: arm(at) calls back (Sink::on_timer)
/// at `at`, at once if `at` already passed.
template <class T>
concept DeadlineTimer = requires(T& t, std::uint32_t at) {
    { t.now() } -> std::same_as<std::uint32_t>;
    t.arm(at);
    t.disarm();
};

enum class SinkEvent : std::uint8_t { SourceCapabilitiesChanged, VoltageChanged, PowerRejected };

struct SinkStatus {
    bool attached = false;  ///< CC line shows a source
    bool connected = false; ///< the source sent its capabilities (PD contract possible)
    int active_mv = 0, active_ma = 0;
    int requested_mv = 0, requested_ma = 0;
    std::size_t capability_count = 0;
    std::array<SourceCapability, Message::max_objects> capabilities{};
};

struct NoGuard {};

template <Phy P, DeadlineTimer T, class Guard = NoGuard>
class Sink final : public PhyEvents {
public:
    // Timing (USB PD 3.1 table 6.68)
    static constexpr std::uint32_t receive_timeout_us = 1'100;      ///< tReceive: GoodCRC
    static constexpr unsigned retries = 2;                         ///< nRetryCount
    static constexpr std::uint32_t caps_first_ask_us = 100'000;     ///< after attach
    static constexpr std::uint32_t caps_retry_us = 200'000;
    static constexpr std::uint32_t busy_retry_us = 10'000;
    static constexpr std::uint32_t wait_retry_us = 100'000;         ///< tSinkRequest after Wait
    static constexpr std::uint32_t pps_refresh_us = 8'000'000;      ///< PPS: below tPPSRequest 10 s
    static constexpr unsigned caps_asks_before_hard_reset = 3;

    /// `bus_powered`: the board runs from this VBUS, so 5 V is there before any contract.
    Sink(P& phy, T& timer, bool bus_powered = false) noexcept
        : phy_(phy), timer_(timer), bus_powered_(bus_powered) {
        reset_policy();
    }

    // ---- application (thread) --------------------------------------------------------------

    /// Ask for `mv` with at most `ma` (0: what the source offers). Remembered: a source that
    /// attaches later or sends new capabilities gets this request. False if no current
    /// capability matches (the sink then falls back to 5 V on new capabilities).
    bool request_power(int mv, int ma = 0) noexcept {
        [[maybe_unused]] const Guard guard{};
        desired_mv_ = mv;
        desired_ma_ = ma;
        if (cap_count_ == 0) {
            return false;
        }
        return request(mv, ma) != RequestResult::NoMatch;
    }

    /// The first of `mv_by_priority` the source offers; false if none.
    bool request_first_available(std::span<const int> mv_by_priority) noexcept {
        for (const int mv : mv_by_priority) {
            if (can_provide(mv)) {
                return request_power(mv);
            }
        }
        return false;
    }

    [[nodiscard]] bool can_provide(int mv) const noexcept {
        [[maybe_unused]] const Guard guard{};
        return std::ranges::any_of(std::span{caps_}.first(cap_count_),
                                   [mv](const SourceCapability& c) { return c.min_mv <= mv && mv <= c.max_mv; });
    }

    [[nodiscard]] SinkStatus status() const noexcept {
        [[maybe_unused]] const Guard guard{};
        return {.attached = cc_ != 0, .connected = cap_count_ != 0, .active_mv = active_mv_,
                .active_ma = active_ma_, .requested_mv = requested_mv_, .requested_ma = requested_ma_,
                .capability_count = cap_count_, .capabilities = caps_};
    }

    /// Deliver the events since the last call to `on_event(SinkEvent)`.
    template <class F>
    void poll(F&& on_event) {
        const std::uint8_t bits = events_.exchange(0);
        for (const auto e : {SinkEvent::SourceCapabilitiesChanged, SinkEvent::VoltageChanged, SinkEvent::PowerRejected}) {
            if ((bits & bit(e)) != 0) {
                on_event(e);
            }
        }
    }

    // ---- interrupts ----------------------------------------------------------------------

    void on_attach(std::uint8_t cc) noexcept override {
        cc_ = cc;
        reset_protocol();
        reset_policy();
    }

    void on_hard_reset() noexcept override {
        reset_protocol();
        reset_policy();
    }

    void on_rx_error() noexcept override { ++rx_errors_; }

    void on_message(const Message& m) noexcept override {
        if (tx_retries_ > 0 && tx_.type() != MessageType::GoodCrc) {
            // waiting for the GoodCRC of our message: anything else ends the attempt
            cancel(GoodCrcDue);
            finish_tx();
            if (m.type() != MessageType::GoodCrc || m.sop != Sop::Sop || m.message_id() != sent_id_) {
                ++tx_failures_;
            }
        }
        if (m.type() == MessageType::GoodCrc || m.sop != Sop::Sop) {
            run_pending();
            return;
        }
        const bool soft_reset = m.type() == MessageType::SoftReset;
        if (soft_reset || static_cast<int>(m.message_id()) != last_rx_id_) {
            rx_ = m;
            has_rx_ = true;
            last_rx_id_ = static_cast<int>(m.message_id());
            revision_ = std::min(m.spec_revision(), 1u);
        }
        tx_retries_ = 0;  // a GoodCRC goes out even over a pending GoodCRC
        if (!send(MessageType::GoodCrc, {})) {
            forget_rx();
            return;
        }
        if (soft_reset) {
            tx_id_ = 0;
            last_rx_id_ = -1;
        }
    }

    void on_transmitted(bool ok) noexcept override {
        if (tx_.type() != MessageType::GoodCrc) {
            if (ok) {
                schedule(GoodCrcDue, receive_timeout_us);
            } else {
                no_good_crc();
            }
            return;
        }
        finish_tx();
        if (!ok) {
            forget_rx();
        } else if (has_rx_) {
            has_rx_ = false;
            handle(rx_);
        }
        run_pending();
    }

    /// From the timer interrupt.
    void on_timer() noexcept {
        const std::uint32_t now = timer_.now();
        std::uint8_t due = 0;
        for (unsigned d = 0; d < deadline_count; ++d) {
            if ((armed_ & (1u << d)) != 0 && static_cast<std::int32_t>(now - due_[d]) >= 0) {
                due |= static_cast<std::uint8_t>(1u << d);
            }
        }
        armed_ &= static_cast<std::uint8_t>(~due);
        if ((due & (1u << GoodCrcDue)) != 0) {
            no_good_crc();
        }
        if ((due & (1u << SourceCapsDue)) != 0) {
            ask_for_capabilities();
        }
        if ((due & (1u << RequestDue)) != 0) {
            if (cap_count_ != 0 && request(requested_mv_, requested_ma_) == RequestResult::NoMatch) {
                request(5'000, 0);
            }
        }
        rearm();
    }

    /// Diagnostics.
    [[nodiscard]] unsigned rx_errors() const noexcept { return rx_errors_; }
    [[nodiscard]] unsigned tx_failures() const noexcept { return tx_failures_; }

private:
    enum Deadline : unsigned { GoodCrcDue, SourceCapsDue, RequestDue, deadline_count };
    enum class RequestResult : std::uint8_t { Sent, Deferred, NoMatch };

    static constexpr std::uint8_t bit(SinkEvent e) noexcept { return static_cast<std::uint8_t>(1u << static_cast<unsigned>(e)); }
    void raise(SinkEvent e) noexcept { events_.fetch_or(bit(e)); }

    // ---- protocol layer ------------------------------------------------------------------

    bool send(MessageType type, std::span<const std::uint32_t> objects) noexcept {
        if (tx_retries_ > 0) {
            return false;
        }
        tx_.init(type, objects.size(), revision_);
        std::ranges::copy(objects, tx_.objects.begin());
        tx_retries_ = retries + 1;
        return transmit();
    }

    bool transmit() noexcept {
        sent_id_ = tx_.type() == MessageType::GoodCrc ? static_cast<unsigned>(last_rx_id_) : tx_id_;
        tx_.set_message_id(sent_id_);
        if (!phy_.transmit(tx_)) {
            tx_retries_ = 0;
            return false;
        }
        return true;
    }

    void no_good_crc() noexcept {
        cancel(GoodCrcDue);
        if (--tx_retries_ > 0) {
            if (transmit()) {
                return;
            }
        }
        ++tx_failures_;
        finish_tx();
        run_pending();
    }

    /// Our GoodCRC did not go out: the source repeats the message, take it as new then.
    void forget_rx() noexcept {
        has_rx_ = false;
        last_rx_id_ = -1;
    }

    /// The transmitter is free again; the next message gets the next ID.
    void finish_tx() noexcept {
        tx_retries_ = 0;
        if (tx_.type() != MessageType::GoodCrc) {
            tx_id_ = (tx_id_ + 1) & 0x07u;
        }
    }

    void reset_protocol() noexcept {
        last_rx_id_ = -1;
        tx_id_ = 0;
        tx_retries_ = 0;
        has_rx_ = false;
        revision_ = 1;
        tx_ = {};
        cancel(GoodCrcDue);
    }

    // ---- sink policy ---------------------------------------------------------------------

    void reset_policy() noexcept {
        const bool powered = bus_powered_ || cc_ != 0;
        if (active_mv_ != (powered ? 5'000 : 0)) {
            raise(SinkEvent::VoltageChanged);
        }
        active_mv_ = powered ? 5'000 : 0;
        active_ma_ = powered ? 900 : 0;
        requested_mv_ = 5'000;
        requested_ma_ = 900;
        if (cap_count_ != 0) {
            raise(SinkEvent::SourceCapabilitiesChanged);
        }
        cap_count_ = 0;
        caps_changed_ = false;
        pending_request_ = false;
        pps_ = false;
        caps_asks_ = 0;
        cancel(RequestDue);
        cancel(SourceCapsDue);
        if (cc_ != 0) {
            schedule(SourceCapsDue, caps_first_ask_us);
        }
    }

    void handle(const Message& m) noexcept {
        switch (m.type()) {
            case MessageType::SourceCapabilities:
                cap_count_ = parse_source_capabilities(m, caps_);
                caps_changed_ = true;
                cancel(SourceCapsDue);
                if (request(desired_mv_, desired_ma_) == RequestResult::NoMatch) {
                    request(5'000, 0);
                }
                break;
            case MessageType::Reject:
                requested_mv_ = active_mv_;
                requested_ma_ = active_ma_;
                raise(SinkEvent::PowerRejected);
                break;
            case MessageType::Wait:
                schedule(RequestDue, wait_retry_us);
                break;
            case MessageType::PsReady:
                if (caps_changed_) {
                    caps_changed_ = false;
                    raise(SinkEvent::SourceCapabilitiesChanged);
                }
                if (active_mv_ != requested_mv_ || active_ma_ != requested_ma_) {
                    raise(SinkEvent::VoltageChanged);
                }
                active_mv_ = requested_mv_;
                active_ma_ = requested_ma_;
                if (pps_) {
                    schedule(RequestDue, pps_refresh_us);
                }
                break;
            case MessageType::SoftReset:
                send(MessageType::Accept, {});
                break;
            case MessageType::GetSinkCap:
            case MessageType::DrSwap:
            case MessageType::PrSwap:
            case MessageType::VconnSwap:
            case MessageType::GotoMin:
                send(MessageType::Reject, {});
                break;
            default:
                break;  // Accept: PS_RDY follows; Ping and the rest need no answer
        }
    }

    RequestResult request(int mv, int ma) noexcept {
        const PowerRequest r = power_request(std::span{caps_}.first(cap_count_), mv, ma);
        if (!r.valid()) {
            return RequestResult::NoMatch;
        }
        cancel(RequestDue);
        const std::uint32_t object = r.object;
        if (!send(MessageType::Request, std::span{&object, 1})) {
            pending_request_ = true;  // after the current transmission
            pending_mv_ = mv;
            pending_ma_ = ma;
            return RequestResult::Deferred;
        }
        pending_request_ = false;
        pps_ = caps_[r.index].type == SupplyType::Pps;
        requested_mv_ = r.mv;
        requested_ma_ = r.ma;
        return RequestResult::Sent;
    }

    void run_pending() noexcept {
        if (pending_request_ && tx_retries_ == 0 && cap_count_ != 0) {
            request(pending_mv_, pending_ma_);
        }
    }

    void ask_for_capabilities() noexcept {
        if (cc_ == 0 || cap_count_ != 0) {
            return;
        }
        if (caps_asks_ >= caps_asks_before_hard_reset) {
            // some sources only send their capabilities again around a Hard Reset
            caps_asks_ = 0;
            phy_.transmit_hard_reset();
            schedule(SourceCapsDue, caps_first_ask_us);
        } else if (send(MessageType::GetSourceCap, {})) {
            ++caps_asks_;
            schedule(SourceCapsDue, caps_retry_us);
        } else {
            schedule(SourceCapsDue, busy_retry_us);
        }
    }

    // ---- deadlines -----------------------------------------------------------------------

    void schedule(Deadline d, std::uint32_t after_us) noexcept {
        due_[d] = timer_.now() + after_us;
        armed_ |= static_cast<std::uint8_t>(1u << d);
        rearm();
    }

    void cancel(Deadline d) noexcept {
        if ((armed_ & (1u << d)) != 0) {
            armed_ &= static_cast<std::uint8_t>(~(1u << d));
            rearm();
        }
    }

    void rearm() noexcept {
        if (armed_ == 0) {
            timer_.disarm();
            return;
        }
        const std::uint32_t now = timer_.now();
        std::uint32_t next = 0;
        std::int32_t nearest = INT32_MAX;
        for (unsigned d = 0; d < deadline_count; ++d) {
            const auto left = static_cast<std::int32_t>(due_[d] - now);
            if ((armed_ & (1u << d)) != 0 && left < nearest) {
                nearest = left;
                next = due_[d];
            }
        }
        timer_.arm(next);
    }

    P& phy_;
    T& timer_;
    const bool bus_powered_;

    // protocol
    std::uint8_t cc_ = 0;
    Message tx_{};
    Message rx_{};
    bool has_rx_ = false;
    unsigned tx_id_ = 0, sent_id_ = 0, tx_retries_ = 0, revision_ = 1;
    int last_rx_id_ = -1;
    unsigned rx_errors_ = 0, tx_failures_ = 0;

    // policy
    std::array<SourceCapability, Message::max_objects> caps_{};
    std::size_t cap_count_ = 0;
    int active_mv_ = 0, active_ma_ = 0, requested_mv_ = 5'000, requested_ma_ = 900;
    int desired_mv_ = 5'000, desired_ma_ = 0, pending_mv_ = 0, pending_ma_ = 0;
    bool caps_changed_ = false, pending_request_ = false, pps_ = false;
    unsigned caps_asks_ = 0;

    std::array<std::uint32_t, deadline_count> due_{};
    std::uint8_t armed_ = 0;
    std::atomic<std::uint8_t> events_{0};
};

}  // namespace rtos::usb_pd
