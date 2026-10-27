// SPDX-License-Identifier: MIT
//
// TcpServer (tcp_server.hpp) with a minimal line protocol, the way a Modbus TCP or
// OPC UA server would plug in: accept, pump on data, eviction of idle sessions,
// refusal, early release after FIN, deadlines without events.
#include <gtest/gtest.h>

#include <string>

#include "net_fixture.hpp"
#include "rtos/net/service.hpp"
#include "rtos/net/tcp_server.hpp"

namespace {

using namespace test;

/// Echoes every chunk it receives; "quit" closes; idle after `idle_timeout` closes.
/// A session that has received nothing yet may be evicted.
struct LineSession {
    static constexpr std::uint32_t idle_timeout = 1000;
    TestStack::TcpConnection* conn = nullptr;
    std::uint32_t last = 0;
    bool got_data = false;
    bool done = false;
    int closes = 0;

    bool open(TestStack::TcpConnection& c, std::uint32_t now) noexcept {
        conn = &c;
        last = now;
        got_data = done = false;
        return true;
    }
    void pump(TestStack::TcpConnection& c, std::uint32_t now, bool event) noexcept {
        if (event) {
            last = now;
        }
        std::array<std::byte, 256> buf{};
        if (const std::size_t n = c.read(buf); n != 0) {
            got_data = true;
            const std::string_view text(reinterpret_cast<const char*>(buf.data()), n);
            if (text == "quit") {
                c.close();
                done = true;
                return;
            }
            c.write({buf.data(), n});
        }
        if (static_cast<std::int32_t>(now - (last + idle_timeout)) >= 0) {
            c.close();
            done = true;
        }
    }
    void closed() noexcept {
        ++closes;
        conn = nullptr;
    }
    [[nodiscard]] bool finished() const noexcept { return done; }
    [[nodiscard]] bool evictable() const noexcept { return !got_data; }
    [[nodiscard]] std::uint32_t deadline() const noexcept { return done ? 0 : last + idle_timeout; }
};
static_assert(TcpSession<LineSession, TestStack::TcpConnection>);

using LineServer = TcpServer<TestStack, LineSession, 2>;
static_assert(NetService<LineServer>);

class TcpServerTest : public NetTest {
protected:
    static constexpr std::uint16_t port = 1000;
    LineServer server{stack};

    void SetUp() override {
        NetTest::SetUp();
        ASSERT_TRUE(server.listen(port));
    }

    struct Peer {
        std::uint16_t port;
        std::uint32_t seq;  ///< next sequence number to send
        std::uint32_t ack;  ///< next sequence number expected from the server
    };

    /// Three-way handshake from peer port `from`.
    Peer connect(std::uint16_t from) {
        Peer p{from, 7000u + from, 0};
        deliver(tcp(from, port, p.seq, 0, tcp_flag::syn, {}, 8192, 1460));
        const auto f = sent();
        EXPECT_EQ(f.size(), 1u);
        p.ack = f.at(0).seq() + 1;
        ++p.seq;
        deliver(tcp(from, port, p.seq, p.ack, tcp_flag::ack));
        return p;
    }

    /// Send `text`; returns the frames the stack sent in response.
    std::vector<Frame> say(Peer& p, std::string_view text) {
        deliver(tcp(p.port, port, p.seq, p.ack, tcp_flag::ack | tcp_flag::psh, text));
        p.seq += static_cast<std::uint32_t>(text.size());
        server.service();
        return sent();
    }
};

TEST_F(TcpServerTest, AcceptsAndPumpsOnData) {
    Peer a = connect(5000);
    EXPECT_EQ(server.active_count(), 1u);
    const auto f = say(a, "hello");
    ASSERT_FALSE(f.empty());
    EXPECT_EQ(f.back().tcp_payload(), "hello");
    EXPECT_EQ(server.stats().accepted, 1u);
}

TEST_F(TcpServerTest, EvictsTheLongestIdleSessionWhenFull) {
    (void)connect(5000);
    poll(10);
    (void)connect(5001);
    poll(10);
    (void)connect(5002);  // table full: 5000 has been idle longest
    EXPECT_EQ(server.stats().evictions, 1u);
    EXPECT_EQ(server.active_count(), 2u);
    EXPECT_EQ(server.session(0).closes + server.session(1).closes, 1);
}

TEST_F(TcpServerTest, RefusesWhenNothingIsEvictable) {
    Peer a = connect(5000);
    Peer b = connect(5001);
    (void)say(a, "busy");
    (void)say(b, "busy");
    (void)connect(5002);  // both sessions busy: refused
    EXPECT_TRUE(sent().back().flags() & tcp_flag::rst);
    EXPECT_EQ(server.stats().refused, 1u);
    EXPECT_EQ(server.active_count(), 2u);
}

TEST_F(TcpServerTest, FinishedSessionFreesItsSlotAtOnce) {
    Peer a = connect(5000);
    const auto f = say(a, "quit");
    ASSERT_FALSE(f.empty());
    EXPECT_TRUE(f.back().flags() & tcp_flag::fin);
    EXPECT_EQ(server.active_count(), 0u);  // TCP still closes; the session is free
    EXPECT_EQ(server.session(0).closes, 1);
}

TEST_F(TcpServerTest, DeadlinePumpsWithoutEvents) {
    NetRuntime<TestStack> runtime{stack};
    ASSERT_TRUE(runtime.add(server, "line"));
    (void)connect(5000);
    const std::uint32_t wake = runtime.run_once(now);
    EXPECT_EQ(wake, now + LineSession::idle_timeout);
    now = wake;
    (void)runtime.run_once(now);  // the deadline: the session closes itself
    EXPECT_EQ(server.active_count(), 0u);
    EXPECT_TRUE(sent().back().flags() & tcp_flag::fin);
}

}  // namespace
