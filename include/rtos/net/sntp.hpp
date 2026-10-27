// SPDX-License-Identifier: MIT
//
// SNTP client (RFC 4330, version 4, client mode).
//
//   * request every `interval_ms`; up to 3 retries 2 s apart without an answer
//   * a reply is accepted only if its originate timestamp equals our transmit
//     timestamp (guards against stale and blind spoofed replies), mode = server,
//     stratum 1..15, non-zero transmit timestamp
//   * offset: server time at reception = T3 + round-trip delay / 2
//     with delay = (T4 - T1) - (T3 - T2)   (T1/T4 local, T2/T3 server)
// The local clock is the millisecond tick of the stack (Stack::now_ms()).
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/net/stack.hpp"

namespace rtos::net {

inline constexpr std::uint64_t ntp_unix_offset_s = 2'208'988'800ull;  // 1900-01-01 -> 1970-01-01

template <class Stack>
class SntpClient {
public:
    static constexpr std::size_t message_size = 48;
    static constexpr std::uint32_t retry_ms = 2000;
    static constexpr std::uint8_t max_retries = 3;

    SntpClient(Stack& stack, Ipv4Address server, std::uint16_t server_port = 123,
               std::uint32_t interval_ms = 64'000, std::uint16_t local_port = 12'300) noexcept
        : stack_(stack), server_(server), server_port_(server_port), interval_ms_(interval_ms), local_port_(local_port) {}

    SntpClient(const SntpClient&) = delete;
    SntpClient& operator=(const SntpClient&) = delete;

    bool start() noexcept {
        if (!stack_.udp_bind(local_port_, handler_)) {
            return false;
        }
        next_ms_ = stack_.now_ms();
        return true;
    }

    void service() noexcept {
        if (!active() || static_cast<std::int32_t>(stack_.now_ms() - next_ms_) < 0) {
            return;
        }
        if (awaiting_ && retries_ >= max_retries) {
            awaiting_ = false;
            retries_ = 0;
            next_ms_ = stack_.now_ms() + interval_ms_;
            return;
        }
        send_request();
    }

    /// 0 while there is nothing to do (no server yet, no route to it).
    [[nodiscard]] std::uint32_t next_deadline() const noexcept { return active() ? next_ms_ : 0; }

    /// Use another server (e.g. from DHCP option 42 or a DNS lookup); asks it at once.
    void set_server(Ipv4Address server, std::uint16_t port) noexcept {
        server_ = server;
        server_port_ = port;
        awaiting_ = false;
        retries_ = 0;
        next_ms_ = stack_.now_ms();
    }
    [[nodiscard]] Ipv4Address server() const noexcept { return server_; }
    [[nodiscard]] bool synchronized() const noexcept { return synchronized_; }

    /// Milliseconds since 1970-01-01 UTC for the local tick `now_ms` (0 if unsynchronized).
    [[nodiscard]] std::uint64_t unix_ms(std::uint32_t now_ms) const noexcept {
        return synchronized_ ? base_unix_ms_ + static_cast<std::uint32_t>(now_ms - base_tick_ms_) : 0;
    }

private:
    [[nodiscard]] bool active() const noexcept { return server_ != Ipv4Address::any() && stack_.routable(server_); }

    /// 64-bit NTP timestamp <-> milliseconds (unix epoch)
    static std::uint64_t ntp_to_unix_ms(std::uint32_t seconds, std::uint32_t fraction) noexcept {
        return (static_cast<std::uint64_t>(seconds) - ntp_unix_offset_s) * 1000 +
               ((static_cast<std::uint64_t>(fraction) * 1000) >> 32);
    }

    void send_request() noexcept {
        PacketRef p = stack_.allocate();
        if (!p) {
            next_ms_ = stack_.now_ms() + 100;
            return;
        }
        std::byte* m = p->append(message_size);
        for (std::size_t i = 0; i < message_size; ++i) {
            m[i] = std::byte{0};
        }
        m[0] = std::byte{0x23};  // LI 0, version 4, mode 3 (client)
        // Transmit timestamp: an unguessable value that the server echoes as
        // "originate"; our local send tick is kept separately.
        transmit_hi_ = stack_.random32();
        transmit_lo_ = stack_.random32();
        store_be32(m + 40, transmit_hi_);
        store_be32(m + 44, transmit_lo_);
        t1_ms_ = stack_.now_ms();
        awaiting_ = true;
        ++retries_;
        next_ms_ = t1_ms_ + retry_ms;
        stack_.udp_send(std::move(p), server_, server_port_, local_port_);
    }

    static void received(void* self, const UdpDatagram& d) noexcept { static_cast<SntpClient*>(self)->reply(d); }

    void reply(const UdpDatagram& d) noexcept {
        const auto m = d.payload;
        if (!awaiting_ || d.source != server_ || d.source_port != server_port_ || m.size() < message_size) {
            return;
        }
        const auto first = std::to_integer<std::uint8_t>(m[0]);
        const auto stratum = std::to_integer<std::uint8_t>(m[1]);
        if ((first & 0x7) != 4 || stratum == 0 || stratum > 15 || (first >> 6) == 3) {
            return;  // not a server reply, kiss-o'-death or unsynchronized server
        }
        if (load_be32(m.data() + 24) != transmit_hi_ || load_be32(m.data() + 28) != transmit_lo_) {
            return;  // not an answer to our latest request
        }
        const std::uint32_t t4_ms = stack_.now_ms();
        const std::uint64_t t2 = ntp_to_unix_ms(load_be32(m.data() + 32), load_be32(m.data() + 36));
        const std::uint64_t t3 = ntp_to_unix_ms(load_be32(m.data() + 40), load_be32(m.data() + 44));
        if (load_be32(m.data() + 40) == 0) {
            return;
        }
        const std::int64_t round_trip = static_cast<std::int64_t>(t4_ms - t1_ms_);
        const std::int64_t server_hold = static_cast<std::int64_t>(t3 - t2);
        const std::int64_t delay = round_trip > server_hold ? round_trip - server_hold : 0;
        base_unix_ms_ = t3 + static_cast<std::uint64_t>(delay / 2);
        base_tick_ms_ = t4_ms;
        synchronized_ = true;
        awaiting_ = false;
        retries_ = 0;
        next_ms_ = t4_ms + interval_ms_;
    }

    Stack& stack_;
    Ipv4Address server_;
    std::uint16_t server_port_;
    std::uint32_t interval_ms_;
    std::uint16_t local_port_;
    std::uint32_t next_ms_ = 0;
    std::uint32_t t1_ms_ = 0;
    std::uint32_t transmit_hi_ = 0;
    std::uint32_t transmit_lo_ = 0;
    std::uint64_t base_unix_ms_ = 0;
    std::uint32_t base_tick_ms_ = 0;
    std::uint8_t retries_ = 0;
    bool awaiting_ = false;
    bool synchronized_ = false;
    UdpHandler handler_{this, &SntpClient::received};
};

}  // namespace rtos::net
