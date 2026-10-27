// SPDX-License-Identifier: MIT
//
// HttpsServer + NetRuntime end to end on the host: real wolfSSL clients over simulated
// TCP peers (a minimal client-side TCP in this file) download a 100 KB resource while
// a Modbus TCP server (the interactive service, as OPC UA later) answers
// requests. Checks the fairness rules of service.hpp:
//   * downloads and handshakes are background steps: a request that arrives in the
//     same receive batch as download ACKs or ClientHellos is answered BEFORE the next
//     download record / handshake flight goes out (transmit order on the wire);
//   * at most Cfg::tcp_bulk_connections downloads send at a time, so the packet pool
//     always has buffers for interactive replies;
//   * every download completes byte-exact, round robin, nothing starves.
#include <gtest/gtest.h>

#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/ssl.h>
#include <wolfssl/wolfcrypt/memory.h>
#include <wolfssl/wolfio.h>

#include <array>
#include <cstring>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "net_fixture.hpp"
#include "rtos/modbus/register_bank.hpp"
#include "rtos/net/https_server.hpp"
#include "rtos/net/modbus_tcp_server.hpp"
#include "rtos/net/service.hpp"
#include "rtos/net/tcp_server.hpp"

namespace {

using namespace test;

constexpr unsigned char server_cert[] = {
#embed "server.der"
};
constexpr unsigned char server_key[] = {
#embed "server.key.der"
};
constexpr unsigned char ca_cert[] = {
#embed "ca.der"
};

struct FairCfg : DefaultStackConfig {
    static constexpr std::size_t tcp_connections = 8;
    static constexpr std::size_t tcp_listeners = 2;
    static constexpr std::size_t tcp_bulk_connections = 2;
};

/// Pool that checks on destruction that every buffer came back (as TestPool).
struct FairPool : PacketPool<48> {
    ~FairPool() { EXPECT_EQ(available(), capacity()) << "packet buffers leaked"; }
};

using FStack = Stack<FairPool, FairCfg>;
using Server = HttpsServer<FStack, 6>;
constexpr std::uint16_t https_port = 443;
constexpr std::uint16_t echo_port = 502;  ///< the interactive service: Modbus TCP

using ModbusServer = ModbusTcpServer<FStack, 2>;

/// Client side of one TCP connection, loss-free, every segment acknowledged at once.
struct TcpPeer {
    std::uint16_t port = 0;
    std::uint16_t server_port = 0;
    std::uint32_t snd_nxt = 1000;
    std::uint32_t rcv_nxt = 0;
    bool established = false;
    bool fin = false;
    bool reset = false;
    std::deque<std::byte> rx;
};

/// wolfSSL client doing GET /blob over a TcpPeer.
struct Client {
    TcpPeer peer;
    WOLFSSL* ssl = nullptr;
    std::deque<std::byte> out;  ///< TLS bytes waiting to be sent
    enum class State { Connecting, Request, Reading, Done, Failed } state = State::Connecting;
    std::string response;
    std::size_t expected = std::string::npos;  ///< header + body, once the header is in
};

int client_receive(WOLFSSL*, char* buf, int sz, void* ctx) {
    auto& rx = static_cast<Client*>(ctx)->peer.rx;
    if (rx.empty()) {
        return WOLFSSL_CBIO_ERR_WANT_READ;
    }
    const int n = std::min<int>(sz, static_cast<int>(rx.size()));
    for (int i = 0; i < n; ++i) {
        buf[i] = static_cast<char>(rx.front());
        rx.pop_front();
    }
    return n;
}

int client_send(WOLFSSL*, char* buf, int sz, void* ctx) {
    auto& out = static_cast<Client*>(ctx)->out;
    for (int i = 0; i < sz; ++i) {
        out.push_back(static_cast<std::byte>(buf[i]));
    }
    return sz;
}

bool host_entropy(std::span<std::byte> out) noexcept {
    static std::mt19937 rng(42);
    for (auto& b : out) {
        b = static_cast<std::byte>(rng());
    }
    return true;
}

std::span<const std::byte> bytes(const unsigned char* p, std::size_t n) {
    return {reinterpret_cast<const std::byte*>(p), n};
}

/// A transmitted frame, classified for the order checks.
struct Sent {
    std::uint16_t src_port = 0;
    std::size_t payload = 0;
};

class HttpsFairness : public ::testing::Test {
protected:
    static constexpr std::size_t blob_size = 100 * 1024;

    static void SetUpTestSuite() {
        ASSERT_TRUE(TlsServerContext::initialize_library(&host_entropy, []() noexcept -> std::uint32_t { return 0; }));
    }

    void SetUp() override {
        for (std::size_t i = 0; i < blob.size(); ++i) {
            blob[i] = static_cast<std::byte>(i * 7 + (i >> 8));
        }
        ASSERT_EQ(tls.init(tls_memory, {bytes(server_cert, sizeof server_cert), bytes(server_key, sizeof server_key)}, 6),
                  TlsInitResult::Ok);
        ASSERT_EQ(wolfSSL_CTX_load_static_memory(&client_ctx, wolfTLSv1_3_client_method_ex, client_memory.data(),
                                                 client_memory.size(), WOLFMEM_GENERAL, 6),
                  WOLFSSL_SUCCESS);
        ASSERT_EQ(wolfSSL_CTX_load_verify_buffer(client_ctx, ca_cert, sizeof ca_cert, WOLFSSL_FILETYPE_ASN1), WOLFSSL_SUCCESS);
        wolfSSL_CTX_set_verify(client_ctx, WOLFSSL_VERIFY_PEER, nullptr);
        wolfSSL_CTX_SetIORecv(client_ctx, &client_receive);
        wolfSSL_CTX_SetIOSend(client_ctx, &client_send);
        stack.set_random([](void*) noexcept -> std::uint32_t { return 100000; }, nullptr);
        ASSERT_TRUE(server.start(https_port));
        ASSERT_TRUE(echo.start(echo_port));
        runtime.add(server, "https");
        runtime.add(echo, "modbus");
        // The device knows the peer's MAC.
        std::array<std::byte, 28> a{};
        store_be16(a.data(), 1);
        store_be16(a.data() + 2, ethertype_ipv4);
        a[4] = std::byte{6};
        a[5] = std::byte{4};
        store_be16(a.data() + 6, 2);
        peer_mac.store(a.data() + 8);
        peer_ip.store(a.data() + 14);
        device_mac.store(a.data() + 18);
        device_ip.store(a.data() + 24);
        driver.rx.push_back(frame(ethertype_arp, a));
        pass();
    }

    void TearDown() override {
        for (auto& c : clients) {
            if (c->ssl != nullptr) {
                wolfSSL_free(c->ssl);
            }
        }
        clients.clear();
        wolfSSL_CTX_free(client_ctx);
    }

    // ---- network simulation ---------------------------------------------------------

    PacketRef frame(std::uint16_t ethertype, std::span<const std::byte> payload) {
        PacketRef p = pool.allocate(0);
        std::byte* h = p->append(14);
        device_mac.store(h);
        peer_mac.store(h + 6);
        store_be16(h + 12, ethertype);
        p->append(payload);
        return p;
    }

    PacketRef segment(const TcpPeer& peer, std::uint32_t seq, std::uint8_t flags, std::span<const std::byte> data) {
        const std::size_t hlen = (flags & tcp_flag::syn) ? 24 : 20;  // SYN: MSS 1460 like any real client
        std::vector<std::byte> ip(20 + hlen + data.size());
        std::byte* t = ip.data() + 20;
        store_be16(t, peer.port);
        store_be16(t + 2, peer.server_port);
        store_be32(t + 4, seq);
        store_be32(t + 8, (flags & tcp_flag::ack) ? peer.rcv_nxt : 0);
        t[12] = static_cast<std::byte>((hlen / 4) << 4);
        t[13] = static_cast<std::byte>(flags);
        store_be16(t + 14, 65535);
        if (hlen == 24) {
            t[20] = std::byte{2};
            t[21] = std::byte{4};
            store_be16(t + 22, 1460);
        }
        std::memcpy(t + hlen, data.data(), data.size());
        const std::size_t tlen = hlen + data.size();
        store_be16(t + 16, checksum_finish(checksum_add({t, tlen}, FStack::pseudo_sum(peer_ip, device_ip, ip_proto_tcp, tlen))));
        ip[0] = std::byte{0x45};
        store_be16(ip.data() + 2, static_cast<std::uint16_t>(ip.size()));
        ip[8] = std::byte{64};
        ip[9] = std::byte{ip_proto_tcp};
        peer_ip.store(ip.data() + 12);
        device_ip.store(ip.data() + 16);
        store_be16(ip.data() + 10, checksum({ip.data(), 20}));
        return frame(ethertype_ipv4, ip);
    }

    void connect(TcpPeer& peer, std::uint16_t server_port) {
        peer.port = next_port++;
        peer.server_port = server_port;
        driver.rx.push_back(segment(peer, peer.snd_nxt, tcp_flag::syn, {}));
        ++peer.snd_nxt;
    }

    void send(TcpPeer& peer, std::span<const std::byte> data) {
        for (std::size_t done = 0; done < data.size();) {
            const std::size_t n = std::min<std::size_t>(data.size() - done, 1460);
            driver.rx.push_back(segment(peer, peer.snd_nxt, tcp_flag::ack | tcp_flag::psh, data.subspan(done, n)));
            peer.snd_nxt += static_cast<std::uint32_t>(n);
            done += n;
        }
    }

    TcpPeer* find(std::uint16_t port) {
        for (auto& c : clients) {
            if (c->peer.port == port) {
                return &c->peer;
            }
        }
        return echo_peer.port == port ? &echo_peer : nullptr;
    }

    /// Hand the frames the device sent to the peers; they acknowledge at once.
    void network() {
        for (auto& p : driver.tx) {
            const Frame f{std::vector<std::byte>(p->data(), p->data() + p->size())};
            if (f.ethertype() != ethertype_ipv4 || f.ip_protocol() != ip_proto_tcp) {
                continue;
            }
            const std::uint16_t src = load_be16(f.l4());
            wire.push_back({src, f.tcp_payload().size()});
            TcpPeer* peer = find(load_be16(f.l4() + 2));
            if (peer == nullptr) {
                continue;
            }
            if (f.flags() & tcp_flag::rst) {
                peer->reset = true;
                continue;
            }
            bool ack = false;
            if ((f.flags() & tcp_flag::syn) && !peer->established) {
                peer->established = true;
                peer->rcv_nxt = f.seq() + 1;
                ack = true;
            }
            const auto payload = f.tcp_payload();
            if (!payload.empty()) {
                if (f.seq() == peer->rcv_nxt) {
                    for (char ch : payload) {
                        peer->rx.push_back(static_cast<std::byte>(ch));
                    }
                    peer->rcv_nxt += static_cast<std::uint32_t>(payload.size());
                }
                ack = true;
            }
            if ((f.flags() & tcp_flag::fin) && f.seq() + payload.size() == peer->rcv_nxt && !peer->fin) {
                peer->fin = true;
                ++peer->rcv_nxt;
                ack = true;
            }
            if (ack) {
                driver.rx.push_back(segment(*peer, peer->snd_nxt, tcp_flag::ack, {}));
            }
        }
        driver.complete_tx();
    }

    void drive_clients() {
        static constexpr std::string_view request =
            "GET /blob HTTP/1.1\r\nHost: fcu.local\r\nConnection: close\r\n\r\n";
        for (auto& cp : clients) {
            Client& c = *cp;
            if (!c.peer.established || c.peer.reset) {
                continue;
            }
            switch (c.state) {
                case Client::State::Connecting:
                    if (wolfSSL_connect(c.ssl) == WOLFSSL_SUCCESS) {
                        c.state = Client::State::Request;
                    } else if (const int e = wolfSSL_get_error(c.ssl, 0);
                               e != WOLFSSL_ERROR_WANT_READ && e != WOLFSSL_ERROR_WANT_WRITE) {
                        c.state = Client::State::Failed;
                    }
                    break;
                case Client::State::Request:
                    if (wolfSSL_write(c.ssl, request.data(), static_cast<int>(request.size())) > 0) {
                        c.state = Client::State::Reading;
                    }
                    break;
                case Client::State::Reading: {
                    std::array<char, 4096> buf{};
                    int n = 0;
                    while ((n = wolfSSL_read(c.ssl, buf.data(), static_cast<int>(buf.size()))) > 0) {
                        c.response.append(buf.data(), static_cast<std::size_t>(n));
                    }
                    if (c.expected == std::string::npos) {
                        if (const auto end = c.response.find("\r\n\r\n"); end != std::string::npos) {
                            const auto at = c.response.find("Content-Length: ");
                            c.expected = end + 4 + std::stoul(c.response.substr(at + 16));
                        }
                    }
                    if (c.response.size() >= c.expected) {
                        c.state = Client::State::Done;
                    }
                    break;
                }
                case Client::State::Done:
                case Client::State::Failed: break;
            }
            if (!c.out.empty()) {
                const std::vector<std::byte> data(c.out.begin(), c.out.end());
                c.out.clear();
                send(c.peer, data);
            }
        }
    }

    /// One pass of the network thread, then the peers react.
    void pass() {
        now += 1;
        runtime.run_once(now);
        max_bulk_active = std::max(max_bulk_active, stack.tcp().bulk_active());
        min_pool_available = std::min(min_pool_available, pool.available());
        network();
        drive_clients();
    }

    Client& add_client() {
        clients.push_back(std::make_unique<Client>());
        Client& c = *clients.back();
        c.ssl = wolfSSL_new(client_ctx);
        EXPECT_NE(c.ssl, nullptr);
        wolfSSL_SetIOReadCtx(c.ssl, &c);
        wolfSSL_SetIOWriteCtx(c.ssl, &c);
        connect(c.peer, https_port);
        return c;
    }

    bool all_done() const {
        for (const auto& c : clients) {
            if (c->state != Client::State::Done) {
                return false;
            }
        }
        return true;
    }

    void expect_downloads_complete() {
        EXPECT_EQ(server.tls_failures(), 0u) << "wolfSSL error " << server.last_tls_error();
        EXPECT_EQ(server.handshake_failures(), 0u);
        for (const auto& c : clients) {
            ASSERT_EQ(c->state, Client::State::Done) << "client on port " << c->peer.port;
            const std::size_t body = c->response.find("\r\n\r\n") + 4;
            ASSERT_EQ(c->response.size() - body, blob_size);
            EXPECT_EQ(std::memcmp(c->response.data() + body, blob.data(), blob_size), 0);
        }
    }

    /// Echo request in the same receive batch as whatever the peers queued (ACKs,
    /// ClientHellos); returns the frames sent in the pass that handles it.
    std::vector<Sent> echo_round_trip() {
        // Modbus TCP: read holding registers 0..9 (transaction 7, unit 1)
        static constexpr std::string_view ping{"\x00\x07\x00\x00\x00\x06\x01\x03\x00\x00\x00\x0A", 12};
        wire.clear();
        send(echo_peer, std::as_bytes(std::span(ping.data(), ping.size())));
        now += 1;
        runtime.run_once(now);
        network();
        std::vector<Sent> out = wire;
        drive_clients();
        return out;
    }

    // ---- the device -----------------------------------------------------------------
    FairPool pool;
    MockDriver driver;
    FStack stack{pool};
    InterfaceId eth = stack.attach(
        driver, Ipv4Config{device_ip, Ipv4Address::from(255, 255, 255, 0), Ipv4Address::from(192, 168, 77, 254)});
    alignas(16) static inline std::array<std::byte, 160 * 1024> tls_memory{};
    TlsServerContext tls;
    std::array<std::byte, blob_size> blob{};
    std::array<HttpResource, 1> resources{
        HttpResource{"/blob", "application/octet-stream", "", std::span<const std::byte>(blob)}};
    Server::WebSocketHandler ws{};
    Server server{stack, tls, resources, "/ws", ws};
    rtos::modbus::RegisterBank<32, 0> registers;
    ModbusServer echo{stack, registers};
    NetRuntime<FStack> runtime{stack};
    std::uint32_t now = 1000;

    // ---- the peers ------------------------------------------------------------------
    alignas(16) static inline std::array<unsigned char, 384 * 1024> client_memory{};
    WOLFSSL_CTX* client_ctx = nullptr;
    std::vector<std::unique_ptr<Client>> clients;
    TcpPeer echo_peer;
    std::uint16_t next_port = 40000;
    std::vector<Sent> wire;  ///< frames sent since the last clear, in order

    std::uint32_t max_bulk_active = 0;
    std::size_t min_pool_available = 1000;
};

TEST_F(HttpsFairness, ParallelDownloadsCompleteByteExact) {
    for (int i = 0; i < 5; ++i) {
        add_client();
    }
    for (int i = 0; i < 20'000 && !all_done(); ++i) {
        pass();
    }
    expect_downloads_complete();
}

TEST_F(HttpsFairness, BulkAdmissionKeepsBuffersForInteractiveReplies) {
    for (int i = 0; i < 5; ++i) {
        add_client();
    }
    for (int i = 0; i < 20'000 && !all_done(); ++i) {
        pass();
    }
    expect_downloads_complete();
    EXPECT_LE(max_bulk_active, FairCfg::tcp_bulk_connections);
    // RX ring of the mock driver: none; the receive reserve and 2 windows (4 + FIN
    // each) may be in use - everything else stays free for interactive replies.
    EXPECT_GE(min_pool_available, FStack::rx_reserve);
}

TEST_F(HttpsFairness, RequestIsAnsweredBeforeTheNextDownloadRecord) {
    connect(echo_peer, echo_port);
    for (int i = 0; i < 5; ++i) {
        add_client();
    }
    // Run until all downloads are well under way.
    for (int i = 0; i < 20'000; ++i) {
        pass();
        bool sending = true;
        for (const auto& c : clients) {
            sending = sending && c->state == Client::State::Reading && c->response.size() > 20'000;
        }
        if (sending) {
            break;
        }
    }
    ASSERT_GT(stack.tcp().bulk_active(), 0u);
    for (int round = 0; round < 20 && !all_done(); ++round) {
        const auto frames = echo_round_trip();
        // The echo reply must leave before any download data of this pass.
        std::size_t echo_at = frames.size(), first_bulk = frames.size();
        for (std::size_t i = 0; i < frames.size(); ++i) {
            if (frames[i].src_port == echo_port && frames[i].payload != 0 && echo_at == frames.size()) {
                echo_at = i;
            }
            if (frames[i].src_port == https_port && frames[i].payload != 0 && first_bulk == frames.size()) {
                first_bulk = i;
            }
        }
        ASSERT_LT(echo_at, frames.size()) << "round " << round << ": no echo reply in the pass";
        EXPECT_LT(echo_at, first_bulk) << "round " << round << ": download data went out before the reply";
        for (int i = 0; i < 7; ++i) {
            pass();
        }
    }
    for (int i = 0; i < 20'000 && !all_done(); ++i) {
        pass();
    }
    expect_downloads_complete();
}

TEST_F(HttpsFairness, RequestIsAnsweredBeforePendingHandshakes) {
    connect(echo_peer, echo_port);
    for (int i = 0; i < 4 && !echo_peer.established; ++i) {
        pass();
    }
    // Three clients: SYN, then their ClientHellos arrive together with the request.
    for (int i = 0; i < 3; ++i) {
        add_client();
    }
    pass();  // SYN -> SYN-ACK; the clients queue ACK + ClientHello
    for (const auto& c : clients) {
        ASSERT_TRUE(c->peer.established);
        ASSERT_FALSE(driver.rx.empty());
    }
    // ACKs + ClientHellos + the request in one receive batch.
    const auto frames = echo_round_trip();
    std::size_t echo_at = frames.size(), first_flight = frames.size();
    for (std::size_t i = 0; i < frames.size(); ++i) {
        if (frames[i].src_port == echo_port && frames[i].payload != 0 && echo_at == frames.size()) {
            echo_at = i;
        }
        if (frames[i].src_port == https_port && frames[i].payload != 0 && first_flight == frames.size()) {
            first_flight = i;
        }
    }
    ASSERT_LT(echo_at, frames.size());
    EXPECT_LT(echo_at, first_flight) << "a handshake flight went out before the reply";
    for (int i = 0; i < 20'000 && !all_done(); ++i) {
        pass();
    }
    expect_downloads_complete();
}

TEST_F(HttpsFairness, RuntimeMeasuresStepsAndSleepsOnlyWhenIdle) {
    std::uint32_t fake_cycles = 0;
    static std::uint32_t* counter = nullptr;
    counter = &fake_cycles;
    runtime.set_cycle_counter([]() noexcept { return *counter += 10; });
    add_client();
    bool was_busy = false;
    for (int i = 0; i < 20'000 && !all_done(); ++i) {
        pass();
        was_busy = was_busy || runtime.busy();
        if (runtime.busy()) {
            EXPECT_EQ(runtime.sleep_ms(0, now), 0u);
        }
    }
    expect_downloads_complete();
    EXPECT_TRUE(was_busy);
    EXPECT_GT(runtime.stats(0).steps, 0u);           // https
    EXPECT_GT(runtime.stats(0).max_step_cycles, 0u);
    EXPECT_EQ(runtime.stats(1).steps, 0u);           // Modbus has no background work
    for (int i = 0; i < 100; ++i) {
        pass();
    }
    EXPECT_FALSE(runtime.busy());
    EXPECT_GT(runtime.sleep_ms(0, now), 0u);
}

}  // namespace
