// SPDX-License-Identifier: MIT
//
// TLS 1.3 server (rtos::net::TlsSession) against a real wolfSSL client over an
// in-memory pipe: handshake with certificate + host name verification, data in
// both directions, one record per write, partial transport writes, KeyUpdate,
// close_notify, tampering, all three cipher suites.
#include <gtest/gtest.h>

#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/ssl.h>
#include <wolfssl/wolfcrypt/memory.h>
#include <wolfssl/wolfio.h>

#include <array>
#include <cstring>
#include <deque>
#include <random>
#include <string>

#include "rtos/net/tls.hpp"
#include "rtos/net/tls_memory.hpp"

namespace {

using namespace rtos::net;

constexpr unsigned char server_cert[] = {
#embed "server.der"
};
constexpr unsigned char server_key[] = {
#embed "server.key.der"
};
constexpr unsigned char ca_cert[] = {
#embed "ca.der"
};

std::span<const std::byte> bytes(const unsigned char* p, std::size_t n) { return {reinterpret_cast<const std::byte*>(p), n}; }
std::span<const std::byte> bytes(std::string_view s) { return std::as_bytes(std::span(s.data(), s.size())); }

/// Both directions of the pipe plus an optional write limit (simulates a full TCP window).
struct Pipe {
    std::deque<std::byte> to_server;
    std::deque<std::byte> to_client;
    std::size_t server_write_limit = SIZE_MAX;
    std::size_t server_writes = 0;  ///< transport write() calls that took bytes
};

/// The server's view of the pipe (ByteTransport).
struct ServerTransport {
    Pipe* pipe;
    std::size_t read(std::span<std::byte> out) noexcept {
        const std::size_t n = std::min(out.size(), pipe->to_server.size());
        for (std::size_t i = 0; i < n; ++i) {
            out[i] = pipe->to_server.front();
            pipe->to_server.pop_front();
        }
        return n;
    }
    std::size_t write(std::span<const std::byte> in) noexcept {
        const std::size_t n = std::min(in.size(), pipe->server_write_limit);
        pipe->to_client.insert(pipe->to_client.end(), in.begin(), in.begin() + static_cast<std::ptrdiff_t>(n));
        pipe->server_writes += n != 0;
        return n;
    }
    bool eof() const noexcept { return false; }
};
static_assert(ByteTransport<ServerTransport>);

int client_receive(WOLFSSL*, char* buf, int sz, void* ctx) {
    auto& q = static_cast<Pipe*>(ctx)->to_client;
    if (q.empty()) {
        return WOLFSSL_CBIO_ERR_WANT_READ;
    }
    const int n = std::min<int>(sz, static_cast<int>(q.size()));
    for (int i = 0; i < n; ++i) {
        buf[i] = static_cast<char>(q.front());
        q.pop_front();
    }
    return n;
}

int client_send(WOLFSSL*, char* buf, int sz, void* ctx) {
    auto& q = static_cast<Pipe*>(ctx)->to_server;
    for (int i = 0; i < sz; ++i) {
        q.push_back(static_cast<std::byte>(buf[i]));
    }
    return sz;
}

bool host_entropy(std::span<std::byte> out) noexcept {
    static std::random_device rd;
    for (auto& b : out) {
        b = static_cast<std::byte>(rd());
    }
    return true;
}

class TlsTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        ASSERT_TRUE(TlsServerContext::initialize_library(&host_entropy, []() noexcept -> std::uint32_t { return 0; }));
    }

    void SetUp() override {
        ASSERT_EQ(server_ctx.init(server_memory, {bytes(server_cert, sizeof server_cert), bytes(server_key, sizeof server_key)}, 2),
                  TlsInitResult::Ok)
            << "wolfSSL error " << server_ctx.last_error();
        ASSERT_EQ(wolfSSL_CTX_load_static_memory(&client_ctx, wolfTLSv1_3_client_method_ex, client_memory.data(),
                                                 client_memory.size(), WOLFMEM_GENERAL, 1),
                  WOLFSSL_SUCCESS);
        ASSERT_EQ(wolfSSL_CTX_load_verify_buffer(client_ctx, ca_cert, sizeof ca_cert, WOLFSSL_FILETYPE_ASN1), WOLFSSL_SUCCESS);
        wolfSSL_CTX_set_verify(client_ctx, WOLFSSL_VERIFY_PEER, nullptr);
        wolfSSL_CTX_SetIORecv(client_ctx, &client_receive);
        wolfSSL_CTX_SetIOSend(client_ctx, &client_send);
        client = wolfSSL_new(client_ctx);
        ASSERT_NE(client, nullptr);
        wolfSSL_SetIOReadCtx(client, &pipe);
        wolfSSL_SetIOWriteCtx(client, &pipe);
        ASSERT_EQ(wolfSSL_check_domain_name(client, "fcu.local"), WOLFSSL_SUCCESS);
    }

    void TearDown() override {
        server.reset();
        if (client != nullptr) {
            wolfSSL_free(client);
        }
        wolfSSL_CTX_free(client_ctx);
    }

    /// Run client and server until the handshake completes (or a round limit).
    bool handshake() {
        server.open(server_ctx, transport);
        for (int round = 0; round < 50; ++round) {
            const int c = wolfSSL_connect(client);
            server.advance();
            if (c == WOLFSSL_SUCCESS && server.status() == TlsStatus::Open) {
                return true;
            }
            if (server.status() == TlsStatus::Failed) {
                return false;
            }
        }
        return false;
    }

    void client_write(std::string_view data) {
        ASSERT_EQ(wolfSSL_write(client, data.data(), static_cast<int>(data.size())), static_cast<int>(data.size()));
    }

    std::string client_read() {
        std::array<char, 4096> buf{};
        std::string out;
        for (;;) {
            const int n = wolfSSL_read(client, buf.data(), static_cast<int>(buf.size()));
            if (n <= 0) {
                return out;
            }
            out.append(buf.data(), static_cast<std::size_t>(n));
        }
    }

    std::string server_read() {
        std::array<std::byte, 4096> buf{};
        std::string out;
        for (std::size_t n = server.read(buf); n != 0; n = server.read(buf)) {
            out.append(reinterpret_cast<const char*>(buf.data()), n);
        }
        return out;
    }

    alignas(16) static inline std::array<std::byte, 160 * 1024> server_memory{};
    alignas(16) static inline std::array<unsigned char, 160 * 1024> client_memory{};
    TlsServerContext server_ctx;
    WOLFSSL_CTX* client_ctx = nullptr;
    WOLFSSL* client = nullptr;
    Pipe pipe;
    ServerTransport transport{&pipe};
    TlsSession server;
};

TEST_F(TlsTest, HandshakeVerifiesCertificateAndHostName) {
    ASSERT_TRUE(handshake());
    EXPECT_STREQ(server.cipher_suite(), "TLS13-CHACHA20-POLY1305-SHA256");
    EXPECT_STREQ(wolfSSL_get_version(client), "TLSv1.3");
}

// A pool with its own bucket layout; the tracking counts its buckets while it is loaded and
// every block taken by a handshake. (fcu_net's layout is measured on the target: wolfSSL's
// structures are larger on x64, its 5632-byte blocks would not hold them here.)
TEST_F(TlsTest, PoolLayoutAndHighWaterMarks) {
    rtos::net::tls_pool_tracking_start();
    alignas(16) static std::array<std::byte, 96 * 1024> memory{};
    static constexpr std::array<std::uint32_t, 7> sizes{64, 128, 256, 512, 1024, 4096, 16128};
    static constexpr std::array<std::uint32_t, 7> counts{64, 24, 36, 8, 6, 6, 2};
    const TlsCredentials credentials{bytes(server_cert, sizeof server_cert), bytes(server_key, sizeof server_key)};
    TlsServerContext ctx;
    ASSERT_EQ(ctx.init(memory, credentials, 2, {sizes, counts}), TlsInitResult::Ok) << ctx.last_error();
    const rtos::net::TlsPoolStats& pool = rtos::net::tls_pool_stats();
    for (std::size_t i = 0; i < sizes.size(); ++i) {
        const auto it = std::find_if(pool.buckets.begin(), pool.buckets.begin() + static_cast<std::ptrdiff_t>(pool.count),
                                     [&](const rtos::net::TlsPoolBucket& b) { return b.size == sizes[i]; });
        ASSERT_NE(it, pool.buckets.begin() + static_cast<std::ptrdiff_t>(pool.count)) << sizes[i];
        EXPECT_GE(it->total, counts[i]) << "at least one round of the layout";
    }
    EXPECT_GT(pool.total_bytes, 80'000u);
    EXPECT_LE(pool.total_bytes, memory.size());
    // the fixture's handshake (its own pool): the global callback counts it too
    const std::uint32_t before = pool.peak_bytes;
    ASSERT_TRUE(handshake());
    EXPECT_GT(pool.peak_bytes, before);
    EXPECT_EQ(pool.failures, 0u);

    TlsServerContext bad;
    static constexpr std::array<std::uint32_t, 2> two{64, 128};
    EXPECT_EQ(bad.init(memory, credentials, 2, {sizes, two}), TlsInitResult::StaticMemory) << "sizes and counts differ";
}

TEST_F(TlsTest, ApplicationDataBothDirections) {
    ASSERT_TRUE(handshake());
    const std::string request = "GET / HTTP/1.1\r\nHost: fcu.local\r\n\r\n";
    client_write(request);
    EXPECT_EQ(server_read(), request);

    const std::string response = "HTTP/1.1 200 OK\r\n\r\nhello";
    EXPECT_EQ(server.write(bytes(response)), response.size());
    EXPECT_EQ(client_read(), response);
}

TEST_F(TlsTest, EachWriteIsOneRecordSentAtOnce) {
    ASSERT_TRUE(handshake());
    const std::string frame(516, 'w');  // WebSocket header + 512-byte payload
    const std::size_t before = pipe.server_writes;
    EXPECT_EQ(server.write(bytes(frame)), frame.size());
    EXPECT_EQ(pipe.server_writes - before, 1u);                 // handed to the transport immediately
    EXPECT_EQ(pipe.to_client.size(), frame.size() + 5 + 1 + 16);  // exactly one record
    EXPECT_EQ(client_read(), frame);
}

TEST_F(TlsTest, LargeWritesAreChunkedIntoRecords) {
    ASSERT_TRUE(handshake());
    const std::string blob(20'000, 'x');
    std::size_t sent = 0;
    while (sent < blob.size()) {
        const std::size_t n = server.write(bytes(std::string_view(blob).substr(sent)));
        ASSERT_GT(n, 0u);
        ASSERT_LE(n, TlsSession::max_write_chunk);
        sent += n;
    }
    EXPECT_EQ(client_read(), blob);
}

TEST_F(TlsTest, FullTransportQueuesRecordUntilFlush) {
    ASSERT_TRUE(handshake());
    const std::string text(1000, 'y');
    pipe.server_write_limit = 0;  // TCP window closed
    EXPECT_EQ(server.write(bytes(text)), text.size());
    EXPECT_TRUE(server.output_pending());
    EXPECT_FALSE(server.flush());
    pipe.server_write_limit = 100;  // trickles out in pieces
    for (int i = 0; i < 50 && !server.flush(); ++i) {
    }
    EXPECT_FALSE(server.output_pending());
    EXPECT_EQ(client_read(), text);
}

TEST_F(TlsTest, KeyUpdateFromClient) {
    ASSERT_TRUE(handshake());
    ASSERT_EQ(wolfSSL_update_keys(client), WOLFSSL_SUCCESS);  // update_requested
    client_write("after update");
    EXPECT_EQ(server_read(), "after update");
    EXPECT_GT(server.write(bytes("reply with new keys")), 0u);
    EXPECT_EQ(client_read(), "reply with new keys");
}

TEST_F(TlsTest, CloseNotifyEndsSession) {
    ASSERT_TRUE(handshake());
    wolfSSL_shutdown(client);
    std::array<std::byte, 16> buf{};
    EXPECT_EQ(server.read(buf), 0u);
    EXPECT_EQ(server.status(), TlsStatus::Closed);
}

TEST_F(TlsTest, TamperedRecordFails) {
    ASSERT_TRUE(handshake());
    client_write("secret");
    ASSERT_GT(pipe.to_server.size(), 7u);
    pipe.to_server[6] ^= std::byte{0x01};  // inside the ciphertext
    std::array<std::byte, 16> buf{};
    EXPECT_EQ(server.read(buf), 0u);
    EXPECT_EQ(server.status(), TlsStatus::Failed);
}

TEST_F(TlsTest, GarbageFailsHandshake) {
    server.open(server_ctx, transport);
    const std::string junk = "GET / HTTP/1.1\r\n\r\n";  // plain HTTP to the TLS port
    for (char ch : junk) {
        pipe.to_server.push_back(static_cast<std::byte>(ch));
    }
    server.advance();
    EXPECT_EQ(server.status(), TlsStatus::Failed);
}

class TlsCipherTest : public TlsTest, public ::testing::WithParamInterface<const char*> {};

TEST_P(TlsCipherTest, DataBothDirections) {
    ASSERT_EQ(wolfSSL_set_cipher_list(client, GetParam()), WOLFSSL_SUCCESS);
    ASSERT_TRUE(handshake());
    EXPECT_STREQ(server.cipher_suite(), GetParam());
    client_write("ping");
    EXPECT_EQ(server_read(), "ping");
    EXPECT_EQ(server.write(bytes("pong")), 4u);
    EXPECT_EQ(client_read(), "pong");
}

INSTANTIATE_TEST_SUITE_P(Suites, TlsCipherTest,
                         ::testing::Values("TLS13-CHACHA20-POLY1305-SHA256", "TLS13-AES128-GCM-SHA256",
                                           "TLS13-AES256-GCM-SHA384"));

}  // namespace
