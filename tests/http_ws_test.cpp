// SPDX-License-Identifier: MIT
//
// HTTP request parser and WebSocket framing (RFC 9112 / RFC 6455 test vectors).
#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "rtos/net/http.hpp"
#include "rtos/net/websocket.hpp"

namespace {

using namespace rtos::net;

template <class P>
typename P::State feed(P& p, std::string_view text) {
    auto space = p.free_space();
    const std::size_t n = std::min(space.size(), text.size());
    std::memcpy(space.data(), text.data(), n);
    return p.commit(n);
}

// ---------------------------------------------------------------------------
// HTTP
// ---------------------------------------------------------------------------

TEST(Http, ParsesRequestLineAndHeadersCaseInsensitively) {
    HttpRequestParser<> p;
    using S = HttpRequestParser<>::State;
    EXPECT_EQ(feed(p, "GET /ws HTTP/1.1\r\nHost: fcu.local\r\nUPGRADE:  websocket \r\n"), S::Incomplete);
    EXPECT_EQ(feed(p, "Connection: keep-alive, Upgrade\r\n\r\n"), S::Complete);
    EXPECT_EQ(p.method(), "GET");
    EXPECT_EQ(p.target(), "/ws");
    EXPECT_EQ(p.header("host"), "fcu.local");
    EXPECT_EQ(p.header("Upgrade"), "websocket");
    EXPECT_TRUE(contains_token(*p.header("connection"), "upgrade"));
    EXPECT_FALSE(p.header("Origin").has_value());
    EXPECT_TRUE(p.keep_alive());
}

TEST(Http, AcceptEncodingTokensWithQuality) {
    EXPECT_TRUE(contains_token("gzip, deflate, br;q=1.0, zstd", "br"));
    EXPECT_FALSE(contains_token("gzip, deflate", "br"));
}

TEST(Http, RejectsMalformedAndOversizedRequests) {
    HttpRequestParser<> bad;
    EXPECT_EQ(feed(bad, "GARBAGE\r\n\r\n"), HttpRequestParser<>::State::Error);
    HttpRequestParser<64> small;
    EXPECT_EQ(feed(small, "GET / HTTP/1.1\r\nX-Long: " + std::string(100, 'a')), HttpRequestParser<64>::State::TooLarge);
}

TEST(Http, PipelinedRequestIsKeptForNext) {
    HttpRequestParser<> p;
    using S = HttpRequestParser<>::State;
    EXPECT_EQ(feed(p, "GET /a HTTP/1.1\r\n\r\nGET /b HTTP/1.1\r\nConnection: close\r\n\r\n"), S::Complete);
    EXPECT_EQ(p.target(), "/a");
    p.next();
    EXPECT_EQ(p.state(), S::Complete);
    EXPECT_EQ(p.target(), "/b");
    EXPECT_FALSE(p.keep_alive());
}

// ---------------------------------------------------------------------------
// WebSocket
// ---------------------------------------------------------------------------

TEST(WebSocket, AcceptKeyRfc6455Example) {
    std::array<char, 29> accept{};
    ASSERT_TRUE(websocket_accept_key("dGhlIHNhbXBsZSBub25jZQ==", accept));
    EXPECT_STREQ(accept.data(), "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");  // RFC 6455 section 1.3
    EXPECT_FALSE(websocket_accept_key("short", accept));
}

std::vector<std::byte> bytes(std::initializer_list<int> v) {
    std::vector<std::byte> out;
    for (int x : v) {
        out.push_back(static_cast<std::byte>(x));
    }
    return out;
}

/// Masked client frame.
std::vector<std::byte> client_frame(std::uint8_t first, std::span<const std::byte> payload,
                                    std::array<std::byte, 4> mask = {std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4}}) {
    std::vector<std::byte> f{static_cast<std::byte>(first)};
    if (payload.size() < 126) {
        f.push_back(static_cast<std::byte>(0x80 | payload.size()));
    } else {
        f.push_back(std::byte{0x80 | 126});
        f.push_back(static_cast<std::byte>(payload.size() >> 8));
        f.push_back(static_cast<std::byte>(payload.size()));
    }
    f.insert(f.end(), mask.begin(), mask.end());
    for (std::size_t i = 0; i < payload.size(); ++i) {
        f.push_back(payload[i] ^ mask[i & 3]);
    }
    return f;
}

using Decoder = WebSocketDecoder<512>;

TEST(WebSocket, DecodesMaskedHelloRfc6455Example) {
    // RFC 6455 5.7: a single-frame masked text message "Hello"
    const auto f = bytes({0x81, 0x85, 0x37, 0xfa, 0x21, 0x3d, 0x7f, 0x9f, 0x4d, 0x51, 0x58});
    Decoder d;
    Decoder::Event e;
    EXPECT_EQ(d.feed(f, e), f.size());
    ASSERT_EQ(e, Decoder::Event::Message);
    EXPECT_EQ(d.message_opcode(), ws_opcode::text);
    EXPECT_EQ(std::string(reinterpret_cast<const char*>(d.message().data()), d.message().size()), "Hello");
}

TEST(WebSocket, BinaryMessageOf512BytesInSmallPieces) {
    std::vector<std::byte> payload(512);
    for (std::size_t i = 0; i < payload.size(); ++i) {
        payload[i] = static_cast<std::byte>(i * 7);
    }
    const auto f = client_frame(0x82, payload);
    Decoder d;
    Decoder::Event e = Decoder::Event::None;
    std::size_t pos = 0;
    while (pos < f.size() && e == Decoder::Event::None) {
        const std::size_t chunk = std::min<std::size_t>(13, f.size() - pos);  // arbitrary TCP segmentation
        pos += d.feed({f.data() + pos, chunk}, e);
    }
    ASSERT_EQ(e, Decoder::Event::Message);
    EXPECT_EQ(pos, f.size());
    EXPECT_EQ(d.message_opcode(), ws_opcode::binary);
    ASSERT_EQ(d.message().size(), 512u);
    EXPECT_TRUE(std::equal(payload.begin(), payload.end(), d.message().begin()));
}

TEST(WebSocket, FragmentedMessageWithInterleavedPing) {
    const auto hel = bytes({'H', 'e', 'l'});
    const auto lo = bytes({'l', 'o'});
    const auto ping = bytes({'p'});
    std::vector<std::byte> stream = client_frame(0x01, hel);  // text, FIN = 0
    const auto p = client_frame(0x89, ping);
    stream.insert(stream.end(), p.begin(), p.end());
    const auto last = client_frame(0x80, lo);  // continuation, FIN = 1
    stream.insert(stream.end(), last.begin(), last.end());

    Decoder d;
    Decoder::Event e;
    std::size_t pos = d.feed(stream, e);
    ASSERT_EQ(e, Decoder::Event::Ping);
    EXPECT_EQ(d.control_payload().size(), 1u);
    pos += d.feed({stream.data() + pos, stream.size() - pos}, e);
    ASSERT_EQ(e, Decoder::Event::Message);
    EXPECT_EQ(std::string(reinterpret_cast<const char*>(d.message().data()), d.message().size()), "Hello");
    EXPECT_EQ(pos, stream.size());
}

TEST(WebSocket, ProtocolErrors) {
    Decoder d;
    Decoder::Event e;
    const auto unmasked = bytes({0x82, 0x01, 0x00});
    d.feed(unmasked, e);
    EXPECT_EQ(e, Decoder::Event::Error);
    EXPECT_EQ(d.close_code(), 1002);

    Decoder big;
    const std::vector<std::byte> payload(600);
    big.feed(client_frame(0x82, payload), e);
    EXPECT_EQ(e, Decoder::Event::Error);
    EXPECT_EQ(big.close_code(), 1009);
}

TEST(WebSocket, ServerFrameHeader) {
    std::array<std::byte, 4> h{};
    EXPECT_EQ(websocket_frame_header(h.data(), ws_opcode::binary, 100), 2u);
    EXPECT_EQ(h[0], std::byte{0x82});
    EXPECT_EQ(h[1], std::byte{100});
    EXPECT_EQ(websocket_frame_header(h.data(), ws_opcode::binary, 512), 4u);
    EXPECT_EQ(h[1], std::byte{126});
    EXPECT_EQ(h[2], std::byte{0x02});
    EXPECT_EQ(h[3], std::byte{0x00});
}

}  // namespace
