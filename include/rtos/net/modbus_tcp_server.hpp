// SPDX-License-Identifier: MIT
//
// Modbus TCP server (Modbus Messaging on TCP/IP Implementation Guide V1.0b) on
// TcpServer: one Session per client connection, the PDU handled by
// rtos::modbus::process_pdu against the application's RegisterMap.
//
//   * MBAP header: transaction id and unit id are echoed; a protocol id other than 0
//     or a length outside 2..254 closes the connection (the stream cannot be
//     re-synchronised).
//   * Stream framing: a request split over several TCP segments is reassembled,
//     several requests in one segment (pipelining) are answered in order. A response
//     the TCP window cannot take at once is finished before the next request is
//     read (back pressure on the client).
//   * Unit id: `unit_id` 0 answers every unit id; otherwise only that one and 0xFF
//     (the "no gateway" value of the specification); others get no answer.
//   * Idle connections close after idle_timeout_ms (0: never) and may be evicted
//     for a new client when all MaxClients slots are taken.
//
// Interactive service: requests are answered inline in the network thread (a few
// microseconds), never as background work - see the fairness rules in service.hpp.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/modbus/modbus.hpp"
#include "rtos/net/interface.hpp"
#include "rtos/net/tcp_server.hpp"

namespace rtos::net {

struct ModbusTcpConfig {
    std::uint32_t idle_timeout_ms = 120'000;  ///< close a silent client (0: never)
    std::uint8_t unit_id = 0;                 ///< 0: answer every unit id
};

struct ModbusTcpStats {
    std::uint32_t requests = 0;
    std::uint32_t exceptions = 0;       ///< exception responses
    std::uint32_t protocol_errors = 0;  ///< connections closed for a bad MBAP header
    std::uint32_t other_unit = 0;       ///< requests for another unit id (not answered)
};

template <class Stack, std::size_t MaxClients = 4>
class ModbusTcpServer {
public:
    using Connection = typename Stack::TcpConnection;
    static constexpr std::uint16_t default_port = 502;
    static constexpr std::size_t mbap_size = 7;
    static constexpr std::size_t max_adu = mbap_size + modbus::max_pdu;  ///< 260

    ModbusTcpServer(Stack& stack, modbus::RegisterMap& map, const ModbusTcpConfig& config = {}) noexcept
        : tcp_(stack), map_(map), config_(config) {
        for (std::size_t i = 0; i < MaxClients; ++i) {
            tcp_.session(static_cast<std::uint8_t>(i)).owner = this;
        }
    }

    ModbusTcpServer(const ModbusTcpServer&) = delete;
    ModbusTcpServer& operator=(const ModbusTcpServer&) = delete;

    /// Serve Modbus TCP on `port` on the interfaces in `on` (default: all).
    bool start(std::uint16_t port = default_port, InterfaceSet on = InterfaceSet::all()) noexcept {
        return tcp_.listen(port, on);
    }

    // ---- NetService ------------------------------------------------------------
    void service() noexcept { tcp_.service(); }
    [[nodiscard]] std::uint32_t next_deadline() const noexcept { return tcp_.next_deadline(); }

    [[nodiscard]] const ModbusTcpStats& stats() const noexcept { return stats_; }
    [[nodiscard]] std::size_t clients() const noexcept { return tcp_.active_count(); }
    [[nodiscard]] const TcpServerStats& connection_stats() const noexcept { return tcp_.stats(); }

private:
    struct Session {
        ModbusTcpServer* owner = nullptr;
        std::array<std::byte, max_adu> rx{};
        std::size_t rx_size = 0;
        std::array<std::byte, max_adu> tx{};
        std::size_t tx_size = 0;
        std::size_t tx_sent = 0;
        std::uint32_t last_ms = 0;
        bool done = false;

        bool open(Connection&, std::uint32_t now) noexcept {
            rx_size = tx_size = tx_sent = 0;
            last_ms = now;
            done = false;
            return true;
        }

        void pump(Connection& c, std::uint32_t now, bool event) noexcept {
            if (event) {
                last_ms = now;
            }
            if (done) {
                return;
            }
            while (flush(c)) {
                rx_size += c.read(std::span(rx).subspan(rx_size));
                if (rx_size < mbap_size) {
                    break;
                }
                const std::uint16_t protocol = load_be16(&rx[2]);
                const std::uint16_t length = load_be16(&rx[4]);
                if (protocol != 0 || length < 2 || length > 1 + modbus::max_pdu) {
                    ++owner->stats_.protocol_errors;
                    close(c);
                    return;
                }
                const std::size_t total = 6 + std::size_t{length};
                if (rx_size < total) {
                    break;  // the rest of the request is still on its way
                }
                answer(std::span<const std::byte>(rx).first(total));
                std::copy(rx.begin() + static_cast<std::ptrdiff_t>(total),
                          rx.begin() + static_cast<std::ptrdiff_t>(rx_size), rx.begin());
                rx_size -= total;
            }
            const std::uint32_t idle = owner->config_.idle_timeout_ms;
            if (tx_size == 0 && (c.eof() || (idle != 0 && static_cast<std::int32_t>(now - (last_ms + idle)) >= 0))) {
                close(c);
            }
        }

        void closed() noexcept { done = false; }
        [[nodiscard]] bool finished() const noexcept { return done; }
        [[nodiscard]] bool evictable() const noexcept { return rx_size == 0 && tx_size == 0; }
        [[nodiscard]] std::uint32_t deadline() const noexcept {
            const std::uint32_t idle = owner->config_.idle_timeout_ms;
            return done || idle == 0 ? 0 : last_ms + idle;
        }

        /// Send what is left of the last response. True when nothing is left.
        bool flush(Connection& c) noexcept {
            if (tx_sent < tx_size) {
                tx_sent += c.write(std::span<const std::byte>(tx).subspan(tx_sent, tx_size - tx_sent));
                if (tx_sent < tx_size) {
                    return false;
                }
            }
            tx_size = tx_sent = 0;
            return true;
        }

        void answer(std::span<const std::byte> request) noexcept {
            ++owner->stats_.requests;
            const auto unit = std::to_integer<std::uint8_t>(request[6]);
            const std::uint8_t mine = owner->config_.unit_id;
            if (mine != 0 && unit != mine && unit != 0xFF) {
                ++owner->stats_.other_unit;
                return;
            }
            const std::size_t length =
                modbus::process_pdu(owner->map_, request.subspan(mbap_size), std::span(tx).subspan(mbap_size));
            if ((std::to_integer<std::uint8_t>(tx[mbap_size]) & 0x80) != 0) {
                ++owner->stats_.exceptions;
            }
            std::copy_n(request.begin(), 4, tx.begin());  // transaction id, protocol id
            store_be16(&tx[4], static_cast<std::uint16_t>(length + 1));
            tx[6] = request[6];
            tx_size = mbap_size + length;
            tx_sent = 0;
        }

        void close(Connection& c) noexcept {
            c.close();
            done = true;
        }
    };

    TcpServer<Stack, Session, MaxClients> tcp_;
    modbus::RegisterMap& map_;
    ModbusTcpConfig config_;
    ModbusTcpStats stats_{};
};

}  // namespace rtos::net
