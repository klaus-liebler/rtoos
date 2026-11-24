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

TEST(Http, ContentLengthBodyIsWaitedForAndKeptApartFromTheNextRequest) {
    HttpRequestParser<> p;
    using S = HttpRequestParser<>::State;
    EXPECT_EQ(feed(p, "POST /api/login HTTP/1.1\r\nContent-Length: 11\r\n\r\n{\"a\":"), S::Incomplete);
    EXPECT_EQ(feed(p, "\"xyz\"}GET /b HTTP/1.1\r\n\r\n"), S::Complete);
    EXPECT_EQ(p.body(), "{\"a\":\"xyz\"}");
    p.next();
    EXPECT_EQ(p.state(), S::Complete);
    EXPECT_EQ(p.target(), "/b");
    EXPECT_EQ(p.body(), "");
}

TEST(Http, BodiesThatCouldSmuggleARequestAreRefused) {
    using S = HttpRequestParser<>::State;
    for (const char* head : {"POST / HTTP/1.1\r\nTransfer-Encoding: chunked\r\n\r\n",
                             "POST / HTTP/1.1\r\nContent-Length: 1\r\nContent-Length: 1\r\n\r\n",
                             "POST / HTTP/1.1\r\nContent-Length: -1\r\n\r\n",
                             "POST / HTTP/1.1\r\nContent-Length: 0x10\r\n\r\n",
                             "POST / HTTP/1.1\r\nContent-Length: \r\n\r\n"}) {
        HttpRequestParser<> p;
        EXPECT_EQ(feed(p, head), S::Error) << head;
    }
    HttpRequestParser<128> small;
    EXPECT_EQ(feed(small, "POST / HTTP/1.1\r\nContent-Length: 999999\r\n\r\n"), HttpRequestParser<128>::State::TooLarge);
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

// Combination: a fragmented message (message_size_ > 0) and a continuation frame with a 64-bit
// length close to 2^64. The size check added both, the sum wrapped to a small number, passed, and
// the decoder wrote the peer's bytes past the end of its message buffer (found by AFL++,
// tests/fuzz/websocket.cpp) - before any authentication, from any client that reaches port 443.
TEST(WebSocket, ContinuationLengthCannotWrapTheSizeCheck) {
    Decoder d;
    Decoder::Event e;
    d.feed(client_frame(0x02, bytes({1, 2, 3, 4, 5, 6, 7, 8, 9, 10})), e);  // binary, FIN = 0: 10 bytes
    ASSERT_EQ(e, Decoder::Event::None);
    // continuation, length 2^64 - 8 (10 + it wraps to 2); the MSB is set as well
    std::vector<std::byte> f = bytes({0x80, 0x80 | 127, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 1, 2, 3, 4});
    f.resize(f.size() + 600, std::byte{0x55});
    const std::size_t used = d.feed(f, e);
    EXPECT_EQ(e, Decoder::Event::Error);
    EXPECT_LE(used, 14u) << "no payload byte accepted";

    // the same without the MSB: 2^63 - 8, still far beyond the limit
    Decoder d2;
    d2.feed(client_frame(0x02, bytes({1, 2, 3, 4, 5, 6, 7, 8, 9, 10})), e);
    f[2] = std::byte{0x7F};
    d2.feed(f, e);
    EXPECT_EQ(e, Decoder::Event::Error);
    EXPECT_EQ(d2.close_code(), 1009);
}

// RFC 6455 5.2: the most significant bit of a 64-bit payload length must be 0.
TEST(WebSocket, SixtyFourBitLengthWithTheTopBitIsAProtocolError) {
    Decoder d;
    Decoder::Event e;
    d.feed(bytes({0x82, 0x80 | 127, 0x80, 0, 0, 0, 0, 0, 0, 1, 1, 2, 3, 4}), e);
    EXPECT_EQ(e, Decoder::Event::Error);
    EXPECT_EQ(d.close_code(), 1002);
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
