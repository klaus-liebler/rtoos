// SPDX-License-Identifier: MIT
//
// SSH server sessions (rtos::net::SshSession, the log terminal's transport) against a real
// wolfSSH client over an in-memory pipe: authorized_keys parsing, key exchange and public key
// authentication, terminal data in both directions, a full transport, an unauthorized key.
#include <gtest/gtest.h>

#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/wolfcrypt/memory.h>
#include <wolfssh/error.h>
#include <wolfssh/ssh.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <deque>
#include <random>
#include <string>
#include <string_view>

#include "rtos/net/ssh.hpp"

namespace {

using namespace rtos::net;

constexpr unsigned char host_key[] = {
#embed "ssh_host.key.der"
};
constexpr unsigned char client_key[] = {
#embed "ssh_client.key.der"
};
constexpr char client_pub[] = {
#embed "ssh_client.pub"
    , '\0'};

std::span<const std::byte> bytes(const unsigned char* p, std::size_t n) { return {reinterpret_cast<const std::byte*>(p), n}; }

// ---- authorized_keys lines ------------------------------------------------------------------

TEST(SshAuthorizedKeys, ParsesEd25519AndEcdsaLines) {
    // ssh-ed25519 blob: string "ssh-ed25519", string 00 01 .. 1f
    constexpr auto ed = parse_authorized_key("ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIAABAgMEBQYHCAkKCwwNDg8QERITFBUWFxgZGhscHR4f klaus@pc");
    static_assert(ed.has_value());
    static_assert(ed->size == 51);
    EXPECT_EQ(ed->blob[50], std::byte{0x1F});
    const auto ec = parse_authorized_key(std::string_view(client_pub).substr(0, std::string_view(client_pub).find('\n')));
    ASSERT_TRUE(ec.has_value());
    EXPECT_EQ(ec->size, 104u) << "string type, string curve, string point(65)";
}

TEST(SshAuthorizedKeys, RejectsMalformedAndForeignLines) {
    EXPECT_FALSE(parse_authorized_key("ssh-rsa AAAAB3NzaC1yc2EAAAABeA=="));  // RSA is not offered
    EXPECT_FALSE(parse_authorized_key("ssh-ed25519"));
    EXPECT_FALSE(parse_authorized_key("ssh-ed25519 AAAA$$$$"));
    // the type inside the blob differs from the one in front
    EXPECT_FALSE(parse_authorized_key("ecdsa-sha2-nistp256 AAAAC3NzaC1lZDI1NTE5AAAAIAABAgMEBQYHCAkKCwwNDg8QERITFBUWFxgZGhscHR4f"));
}

// ---- sessions against a wolfSSH client --------------------------------------------------------

struct Pipe {
    std::deque<std::byte> to_server, to_client;
    std::size_t server_write_limit = SIZE_MAX;
};

struct ServerTransport {
    Pipe* pipe;
    bool closed = false;
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
        return n;
    }
    bool eof() const noexcept { return closed; }
};
static_assert(ByteTransport<ServerTransport>);

int client_receive(WOLFSSH*, void* buf, word32 sz, void* ctx) {
    auto& q = static_cast<Pipe*>(ctx)->to_client;
    if (q.empty()) {
        return WS_CBIO_ERR_WANT_READ;
    }
    const word32 n = std::min<word32>(sz, static_cast<word32>(q.size()));
    for (word32 i = 0; i < n; ++i) {
        static_cast<std::byte*>(buf)[i] = q.front();
        q.pop_front();
    }
    return static_cast<int>(n);
}
int client_send(WOLFSSH*, void* buf, word32 sz, void* ctx) {
    auto& q = static_cast<Pipe*>(ctx)->to_server;
    q.insert(q.end(), static_cast<std::byte*>(buf), static_cast<std::byte*>(buf) + sz);
    return static_cast<int>(sz);
}
int accept_any_host_key(const byte*, word32, void*) { return 0; }

bool host_entropy(std::span<std::byte> out) noexcept {
    static std::random_device rd;
    for (auto& b : out) {
        b = static_cast<std::byte>(rd());
    }
    return true;
}

/// The client's user key (DER and SSH blob, loaded through wolfSSH as its client does).
struct UserKey {
    std::array<byte, 256> priv{};
    word32 priv_size = 0;
    std::array<byte, 256> pub{};
    word32 pub_size = 0;
    const byte* type = nullptr;
    word32 type_size = 0;
};
UserKey g_user;

int client_user_auth(byte type, WS_UserAuthData* data, void*) {
    if (type != WOLFSSH_USERAUTH_PUBLICKEY) {
        return WOLFSSH_USERAUTH_FAILURE;
    }
    auto& pk = data->sf.publicKey;
    pk.publicKeyType = g_user.type;
    pk.publicKeyTypeSz = g_user.type_size;
    pk.publicKey = g_user.pub.data();
    pk.publicKeySz = g_user.pub_size;
    pk.privateKey = g_user.priv.data();
    pk.privateKeySz = g_user.priv_size;
    return WOLFSSH_USERAUTH_SUCCESS;
}

class SshTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        ASSERT_TRUE(TlsServerContext::initialize_library(&host_entropy, []() noexcept -> std::uint32_t { return 0; }));
    }

    // wolfSSH objects: session 3.7 KB, handshake state 4.7 KB (MinGW host)
    static constexpr std::array<std::uint32_t, 9> pool_sizes{64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384};
    static constexpr std::array<std::uint32_t, 9> pool_counts{128, 64, 64, 32, 16, 16, 8, 8, 4};

    void init_server(std::span<const std::string_view> keys) {
        ASSERT_EQ(server_ctx.init(server_memory, {bytes(host_key, sizeof host_key), keys}, 2, {pool_sizes, pool_counts}),
                  SshInitResult::Ok)
            << "error " << server_ctx.last_error();
    }

    void SetUp() override {
        WOLFSSL_HEAP_HINT* hint = nullptr;
        std::array<word32, 9> sizes{}, counts{};
        std::copy(pool_sizes.begin(), pool_sizes.end(), sizes.begin());
        std::copy(pool_counts.begin(), pool_counts.end(), counts.begin());
        ASSERT_EQ(wc_LoadStaticMemory_ex(&hint, 9, sizes.data(), counts.data(), client_memory.data(),
                                         static_cast<unsigned>(client_memory.size()), WOLFMEM_GENERAL, 1),
                  0);
        if (wolfSSL_GetGlobalHeapHint() == nullptr) {
            wolfSSL_SetGlobalHeapHint(hint);  // the DRBG's seed test allocates without a hint
        }
        client_ctx = wolfSSH_CTX_new(WOLFSSH_ENDPOINT_CLIENT, hint);
        ASSERT_NE(client_ctx, nullptr);
        wolfSSH_SetIORecv(client_ctx, &client_receive);
        wolfSSH_SetIOSend(client_ctx, &client_send);
        wolfSSH_SetUserAuth(client_ctx, &client_user_auth);
        wolfSSH_CTX_SetPublicKeyCheck(client_ctx, &accept_any_host_key);
        // the user key: DER private key, SSH public key line
        byte* out = g_user.priv.data();
        g_user.priv_size = static_cast<word32>(g_user.priv.size());
        const byte* type = nullptr;
        word32 type_size = 0;
        ASSERT_EQ(wolfSSH_ReadKey_buffer(client_key, sizeof client_key, WOLFSSH_FORMAT_ASN1, &out, &g_user.priv_size,
                                         &type, &type_size, hint),
                  WS_SUCCESS);
        out = g_user.pub.data();
        g_user.pub_size = static_cast<word32>(g_user.pub.size());
        const std::string_view line(client_pub);
        ASSERT_EQ(wolfSSH_ReadKey_buffer(reinterpret_cast<const byte*>(line.data()), static_cast<word32>(line.find('\n')),
                                         WOLFSSH_FORMAT_SSH, &out, &g_user.pub_size, &g_user.type, &g_user.type_size, hint),
                  WS_SUCCESS);
        client = wolfSSH_new(client_ctx);
        ASSERT_NE(client, nullptr);
        wolfSSH_SetIOReadCtx(client, &pipe);
        wolfSSH_SetIOWriteCtx(client, &pipe);
        ASSERT_EQ(wolfSSH_SetUsername(client, "log"), WS_SUCCESS);
    }

    void TearDown() override {
        server.reset();
        wolfSSH_free(client);
        wolfSSH_CTX_free(client_ctx);
    }

    /// Run both ends until the session is open (or a round limit).
    bool connect() {
        server.open(server_ctx, transport);
        for (int round = 0; round < 100; ++round) {
            const int c = wolfSSH_connect(client);
            server.advance();
            if (c == WS_SUCCESS && server.status() == SshStatus::Open) {
                return true;
            }
            if (server.status() == SshStatus::Failed || server.status() == SshStatus::Closed) {
                return false;
            }
        }
        return false;
    }

    std::string client_read() {
        std::string text;
        std::array<byte, 512> buf{};
        for (int round = 0; round < 20; ++round) {
            const int n = wolfSSH_stream_read(client, buf.data(), static_cast<word32>(buf.size()));
            if (n > 0) {
                text.append(reinterpret_cast<const char*>(buf.data()), static_cast<std::size_t>(n));
            } else if (pipe.to_client.empty()) {
                break;
            }
        }
        return text;
    }

    alignas(16) static inline std::array<std::byte, 320 * 1024> server_memory{};
    alignas(16) static inline std::array<unsigned char, 320 * 1024> client_memory{};
    SshServerContext server_ctx;
    SshSession server;
    Pipe pipe;
    ServerTransport transport{&pipe};
    WOLFSSH_CTX* client_ctx = nullptr;
    WOLFSSH* client = nullptr;
};

TEST_F(SshTest, AuthorizedKeyOpensTheTerminal) {
    const std::array<std::string_view, 1> keys{std::string_view(client_pub).substr(0, std::string_view(client_pub).find('\n'))};
    init_server(keys);
    ASSERT_TRUE(connect()) << "server error " << server.last_error() << ", client " << wolfSSH_get_error(client);
    EXPECT_EQ(server.user(), "log");
    EXPECT_GE(SshServerContext::stats().sessions, 1u);

    const std::string_view line = "12:00:00 INF hello\r\n";
    EXPECT_EQ(server.write(std::as_bytes(std::span(line.data(), line.size()))), line.size());
    EXPECT_EQ(client_read(), line);

    std::array<byte, 1> q{'q'};
    ASSERT_EQ(wolfSSH_stream_send(client, q.data(), 1), 1);
    std::array<std::byte, 8> in{};
    server.advance();
    std::size_t n = server.read(in);
    for (int i = 0; i < 5 && n == 0; ++i) {
        server.advance();
        n = server.read(in);
    }
    ASSERT_EQ(n, 1u);
    EXPECT_EQ(in[0], std::byte{'q'});
}

TEST_F(SshTest, FullTransportKeepsOutputUntilThereIsRoom) {
    const std::array<std::string_view, 1> keys{std::string_view(client_pub).substr(0, std::string_view(client_pub).find('\n'))};
    init_server(keys);
    ASSERT_TRUE(connect());
    pipe.server_write_limit = 0;  // TCP window closed
    const std::string_view line = "a line that waits\r\n";
    EXPECT_EQ(server.write(std::as_bytes(std::span(line.data(), line.size()))), line.size()) << "framed and kept";
    EXPECT_FALSE(server.can_write());
    pipe.server_write_limit = SIZE_MAX;
    server.advance();  // flushes
    EXPECT_TRUE(server.can_write());
    EXPECT_EQ(client_read(), line);
}

TEST_F(SshTest, UnauthorizedKeyIsRejected) {
    const std::array<std::string_view, 1> keys{
        "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIAABAgMEBQYHCAkKCwwNDg8QERITFBUWFxgZGhscHR4f someone-else"};
    init_server(keys);
    const std::uint32_t rejected = SshServerContext::stats().rejected_keys;
    EXPECT_FALSE(connect());
    EXPECT_NE(server.status(), SshStatus::Open);
    EXPECT_GT(SshServerContext::stats().rejected_keys, rejected);
}

}  // namespace
