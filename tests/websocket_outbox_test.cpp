// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <cstddef>
#include <span>
#include <string>
#include <vector>

#include "rtos/net/websocket_outbox.hpp"

namespace {

/// The part of HttpsServer the outbox uses.
struct FakeServer {
    using SessionId = std::uint8_t;
    struct Sent {
        int session;  ///< -1: broadcast
        std::string data;
        bool binary;
    };
    std::vector<Sent> sent;
    std::size_t clients = 2;
    std::size_t broadcast_accepts = 2;
    bool send_accepts = true;

    bool send(SessionId id, std::span<const std::byte> m, bool binary) {
        if (send_accepts) {
            sent.push_back({id, std::string(reinterpret_cast<const char*>(m.data()), m.size()), binary});
        }
        return send_accepts;
    }
    std::size_t broadcast(std::span<const std::byte> m, bool binary) {
        sent.push_back({-1, std::string(reinterpret_cast<const char*>(m.data()), m.size()), binary});
        return broadcast_accepts;
    }
    std::size_t websocket_clients() const { return clients; }
};

std::span<const std::byte> bytes(const std::string& s) { return std::as_bytes(std::span(s.data(), s.size())); }

using Outbox = rtos::net::WebSocketOutbox<FakeServer, 64>;

TEST(WebSocketOutbox, DeliversInOrderInTheNetworkThread) {
    FakeServer server;
    Outbox outbox{server};
    EXPECT_TRUE(outbox.broadcast(bytes("status")));
    EXPECT_TRUE(outbox.send(1, bytes("reply"), false));
    EXPECT_TRUE(server.sent.empty()) << "nothing goes out from the posting thread";
    outbox.service();
    ASSERT_EQ(server.sent.size(), 2u);
    EXPECT_EQ(server.sent[0].session, -1);
    EXPECT_EQ(server.sent[0].data, "status");
    EXPECT_TRUE(server.sent[0].binary);
    EXPECT_EQ(server.sent[1].session, 1);
    EXPECT_EQ(server.sent[1].data, "reply");
    EXPECT_FALSE(server.sent[1].binary);
    EXPECT_EQ(outbox.stats().posted, 2u);
    outbox.service();
    EXPECT_EQ(server.sent.size(), 2u);
}

TEST(WebSocketOutbox, FullRingRefusesInsteadOfWaiting) {
    FakeServer server;
    Outbox outbox{server};
    const std::string m(20, 'x');  // 4 header + 20 bytes: two fit into 64, a third doesn't
    EXPECT_TRUE(outbox.broadcast(bytes(m)));
    EXPECT_TRUE(outbox.broadcast(bytes(m)));
    EXPECT_FALSE(outbox.broadcast(bytes(m)));
    EXPECT_EQ(outbox.stats().dropped, 1u);
    outbox.service();
    EXPECT_EQ(server.sent.size(), 2u);
    EXPECT_TRUE(outbox.broadcast(bytes(m))) << "room again after delivery";
}

TEST(WebSocketOutbox, WrapsAroundTheRing) {
    FakeServer server;
    Outbox outbox{server};
    for (int i = 0; i < 20; ++i) {
        const std::string m = "message " + std::to_string(i) + std::string(static_cast<std::size_t>(i % 7), '.');
        ASSERT_TRUE(outbox.send(0, bytes(m)));
        outbox.service();
        ASSERT_EQ(server.sent.back().data, m);
    }
}

TEST(WebSocketOutbox, CountsWhatSessionsCouldNotTake) {
    FakeServer server;
    Outbox outbox{server};
    server.broadcast_accepts = 1;
    server.send_accepts = false;
    outbox.broadcast(bytes("a"));
    outbox.send(0, bytes("b"));
    outbox.service();
    EXPECT_EQ(outbox.stats().undelivered, 2u);
}

TEST(WebSocketOutbox, TooLargeIsRefusedAndWakeRunsOnlyOnSuccess) {
    FakeServer server;
    rtos::net::WebSocketOutbox<FakeServer, 2048> outbox{server};
    int wakes = 0;
    outbox.set_wake([](void* n) noexcept { ++*static_cast<int*>(n); }, &wakes);
    EXPECT_FALSE(outbox.broadcast(bytes(std::string(513, 'x'))));
    EXPECT_TRUE(outbox.broadcast(bytes(std::string(512, 'x'))));
    EXPECT_EQ(wakes, 1);
}

}  // namespace
