// SPDX-License-Identifier: MIT
//
// Log terminal: an SSH server whose sessions show the log journal (rtos/log/log.hpp), first
// the records the journal still holds (the boot log), then every new one as it is written.
//
//   ssh -i <key> log@fcu.local          # any user name; the key must be authorized
//
//   rtos::net::SshServerContext ssh_context;      // host key, authorized keys, static pool
//   rtos::net::LogTerminal<Stack, Journal, 2> terminal{stack, ssh_context, journal};
//   terminal.start(22);
//   runtime.add(terminal, "log-terminal");        // network thread
//
// Keys: Ctrl-C, Ctrl-D or q end the session. Lines are coloured by level (ANSI).
//
// Fairness (service.hpp): the key exchange is background work (one wolfSSH step per pass);
// a pass writes at most lines_per_pass lines per session, and only while wolfSSH holds no
// unsent data, so a slow client never buffers more than one SSH packet in the device.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/log/log.hpp"
#include "rtos/net/ssh.hpp"
#include "rtos/net/tcp_server.hpp"

namespace rtos::net {

namespace log_terminal_detail {
template <class J>
struct JournalText;
template <std::size_t Records, std::size_t Text, class Guard>
struct JournalText<log::Journal<Records, Text, Guard>> {
    static constexpr std::size_t value = Text;
};
}  // namespace log_terminal_detail

struct LogTerminalStats {
    std::uint32_t sessions = 0;  ///< opened terminals
    std::uint32_t lines = 0;     ///< log lines sent
    std::uint32_t lost = 0;      ///< records a slow terminal missed (the journal overtook it)
};

template <class Stack, class Journal, std::size_t MaxSessions = 2>
class LogTerminal {
public:
    using Connection = typename Stack::TcpConnection;
    static constexpr std::uint16_t default_port = 22;
    static constexpr std::size_t lines_per_pass = 8;
    static constexpr std::uint32_t poll_ms = 50;  ///< new records: looked for this often

    LogTerminal(Stack& stack, SshServerContext& context, Journal& journal) noexcept
        : stack_(stack), context_(context), journal_(journal), tcp_(stack) {
        for (std::size_t i = 0; i < MaxSessions; ++i) {
            tcp_.session(static_cast<std::uint8_t>(i)).owner = this;
        }
    }
    LogTerminal(const LogTerminal&) = delete;
    LogTerminal& operator=(const LogTerminal&) = delete;

    bool start(std::uint16_t port = default_port, InterfaceSet on = InterfaceSet::all()) noexcept {
        return tcp_.listen(port, on);
    }

    // ---- NetService ------------------------------------------------------------------------
    void service() noexcept { tcp_.service(); }
    [[nodiscard]] std::uint32_t next_deadline() const noexcept { return tcp_.next_deadline(); }
    [[nodiscard]] bool background_pending() const noexcept { return tcp_.background_pending(); }
    void background_step() noexcept { tcp_.background_step(); }

    [[nodiscard]] std::size_t terminals() const noexcept { return tcp_.active_count(); }
    [[nodiscard]] const LogTerminalStats& stats() const noexcept { return stats_; }

private:
    static constexpr std::size_t text_bytes = log_terminal_detail::JournalText<Journal>::value;

    struct Session {
        LogTerminal* owner = nullptr;
        SshSession ssh;
        log::Cursor cursor{};
        bool greeted = false;
        bool done = false;
        std::uint32_t next_poll = 0;
        std::array<char, text_bytes + 64> line{};  ///< a formatted line not yet taken by SSH
        std::size_t line_size = 0, line_sent = 0;

        bool open(Connection& c, std::uint32_t now) noexcept {
            done = greeted = false;
            line_size = line_sent = 0;
            next_poll = now;
            cursor = owner->journal_.oldest();
            return ssh.open(owner->context_, c);
        }

        [[nodiscard]] bool background() const noexcept { return ssh.status() == SshStatus::Handshake; }

        /// One wolfSSH accept step (the key exchange signs and derives keys: milliseconds).
        bool step(Connection& c, std::uint32_t now) noexcept {
            (void)ssh.advance();
            if (!background()) {
                pump(c, now, false);
            }
            return false;  // the next step waits for the client's next message
        }

        void pump(Connection& c, std::uint32_t now, bool) noexcept {
            if (done) {
                return;
            }
            if (ssh.status() == SshStatus::Handshake) {
                return;  // background work
            }
            if (ssh.advance() != SshStatus::Open) {
                finish(c);
                return;
            }
            if (!greeted) {
                greeted = true;
                ++owner->stats_.sessions;
                RTOS_LOG_INFO("log terminal: %.*s connected", static_cast<int>(ssh.user().size()), ssh.user().data());
            }
            std::array<std::byte, 16> keys{};
            for (std::size_t n = ssh.read(keys); n != 0; n = ssh.read(keys)) {
                for (std::size_t i = 0; i < n; ++i) {
                    const auto k = std::to_integer<char>(keys[i]);
                    if (k == '\x03' || k == '\x04' || k == 'q') {
                        ssh.shutdown();
                        finish(c);
                        return;
                    }
                }
            }
            if (static_cast<std::int32_t>(now - next_poll) < 0 && line_size == 0) {
                return;
            }
            next_poll = now + poll_ms;
            for (std::size_t lines = 0; lines < lines_per_pass;) {
                if (line_size == 0) {
                    if (!next_line()) {
                        break;
                    }
                    ++lines;
                }
                if (!ssh.can_write()) {
                    break;
                }
                line_sent += ssh.write(std::as_bytes(std::span(line).subspan(line_sent, line_size - line_sent)));
                if (line_sent < line_size) {
                    break;  // window full: the rest when the client opens it
                }
                line_size = line_sent = 0;
                ++owner->stats_.lines;
            }
            if (line_size != 0) {
                next_poll = now;  // more to write as soon as there is room
            }
        }

        /// Format the next journal record into `line`; false if none.
        bool next_line() noexcept {
            std::array<char, text_bytes> text{};
            log::Record record;
            std::uint32_t lost = 0;
            if (!owner->journal_.read(cursor, record, text, lost)) {
                return false;
            }
            std::size_t n = 0;
            if (lost != 0) {
                owner->stats_.lost += lost;
                const int m = std::snprintf(line.data(), line.size(), "\x1b[90m... %lu lines lost\x1b[0m\r\n",
                                            static_cast<unsigned long>(lost));
                n = m > 0 ? static_cast<std::size_t>(m) : 0;
            }
            n += log::format_line(record, std::span(line).subspan(n), true);
            line_size = std::min(n, line.size());
            line_sent = 0;
            return line_size != 0;
        }

        void finish(Connection& c) noexcept {
            c.close();
            done = true;
        }

        void closed() noexcept {
            ssh.reset();
            done = false;
        }
        [[nodiscard]] bool finished() const noexcept { return done; }
        [[nodiscard]] bool evictable() const noexcept { return false; }
        [[nodiscard]] std::uint32_t deadline() const noexcept {
            return ssh.status() == SshStatus::Open && !done ? next_poll : 0;
        }
    };

    Stack& stack_;
    SshServerContext& context_;
    Journal& journal_;
    LogTerminalStats stats_{};
    TcpServer<Stack, Session, MaxSessions> tcp_;
};

}  // namespace rtos::net
