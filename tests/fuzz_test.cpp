// SPDX-License-Identifier: MIT
//
// Mutation fuzzing of everything that parses network input (idea: lwIP test/fuzz,
// FreeRTOS-Plus-TCP CBMC parser proofs). GCC has no libFuzzer and MinGW no
// sanitizers, so this is a deterministic, seeded mutator that runs as an ordinary
// test: valid seed messages for every protocol are mutated (bit flips, boundary
// values, truncation, extension, copied chunks). For frames the checksums are
// recomputed most of the time, so mutations get past the checksum checks into the
// parsers behind them; the stack runs with all services attached.
//
// A finding is a contract violation (the host enforces contracts), a crash, a hang
// (ctest timeout), a leaked packet buffer (TestPool), or a stack that no longer
// answers afterwards. The seed is fixed: a failure reproduces exactly.
//
// Environment for longer runs: RTOS_FUZZ_ITERATIONS (default 20000 frames, twice
// that per parser), RTOS_FUZZ_SEED (other mutation sequences).
#include <gtest/gtest.h>

#include <cstdlib>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "net_fixture.hpp"
#include "rtos/net/dhcp.hpp"
#include "rtos/net/dhcp_server.hpp"
#include "rtos/net/dns.hpp"
#include "rtos/net/dns_resolver.hpp"
#include "rtos/net/http.hpp"
#include "rtos/net/mdns.hpp"
#include "rtos/net/sntp.hpp"
#include "rtos/net/websocket.hpp"

namespace {

using namespace test;
using Bytes = std::vector<std::byte>;

std::size_t iterations(std::size_t base) {
    if (const char* s = std::getenv("RTOS_FUZZ_ITERATIONS")) {
        return std::strtoul(s, nullptr, 10);
    }
    return base;
}

/// RTOS_FUZZ_SEED (environment) varies the mutation sequences.
std::uint32_t seed(std::uint32_t base) {
    if (const char* s = std::getenv("RTOS_FUZZ_SEED")) {
        return base ^ static_cast<std::uint32_t>(std::strtoul(s, nullptr, 0));
    }
    return base;
}

class Mutator {
public:
    explicit Mutator(std::uint32_t seed) : rng_(seed) {}

    std::uint32_t below(std::uint32_t n) { return n == 0 ? 0 : static_cast<std::uint32_t>(rng_() % n); }
    bool chance(std::uint32_t one_in) { return below(one_in) == 0; }

    /// 1-3 mutations; `keep` leading bytes stay untouched (e.g. the Ethernet header).
    void mutate(Bytes& b, std::size_t keep = 0, std::size_t max_size = 1514) {
        static constexpr std::uint8_t interesting8[] = {0x00, 0x01, 0x3F, 0x40, 0x7F, 0x80, 0xC0, 0xFF};
        static constexpr std::uint16_t interesting16[] = {0x0000, 0x0001, 0x00FF, 0x0100, 0x7FFF, 0x8000, 0xFFFF};
        for (std::uint32_t n = 1 + below(3); n != 0; --n) {
            if (b.size() <= keep) {
                b.resize(keep + 1 + below(16));
            }
            const std::size_t at = keep + below(static_cast<std::uint32_t>(b.size() - keep));
            switch (below(7)) {
                case 0: b[at] ^= static_cast<std::byte>(1u << below(8)); break;
                case 1: b[at] = std::byte{interesting8[below(std::size(interesting8))]}; break;
                case 2:
                    if (at + 1 < b.size()) {
                        store_be16(b.data() + at, interesting16[below(std::size(interesting16))]);
                    }
                    break;
                case 3: b.resize(keep + below(static_cast<std::uint32_t>(b.size() - keep))); break;
                case 4:
                    for (std::uint32_t k = 1 + below(64); k != 0 && b.size() < max_size; --k) {
                        b.push_back(static_cast<std::byte>(rng_()));
                    }
                    break;
                case 5: {  // copy a chunk within the message
                    const std::size_t from = keep + below(static_cast<std::uint32_t>(b.size() - keep));
                    const std::size_t len = std::min<std::size_t>({1 + below(16), b.size() - from, b.size() - at});
                    std::memmove(b.data() + at, b.data() + from, len);
                    break;
                }
                default: b[at] = static_cast<std::byte>(rng_()); break;
            }
        }
    }

private:
    std::mt19937 rng_;
};

/// Recompute the IPv4 header and ICMP/UDP/TCP checksums of an Ethernet frame as far
/// as its (possibly mutated) length fields allow.
void fix_checksums(Bytes& f) {
    if (f.size() < ethernet_header_size + ipv4_header_size || load_be16(f.data() + 12) != ethertype_ipv4) {
        return;
    }
    std::byte* ip = f.data() + ethernet_header_size;
    const std::size_t ihl = (std::to_integer<std::size_t>(ip[0]) & 0x0F) * 4;
    const std::size_t avail = f.size() - ethernet_header_size;
    if (ihl < ipv4_header_size || ihl > avail) {
        return;
    }
    store_be16(ip + 10, 0);
    store_be16(ip + 10, checksum({ip, ihl}));
    const std::size_t total = load_be16(ip + 2);
    if (total < ihl || total > avail) {
        return;
    }
    std::byte* l4 = ip + ihl;
    std::size_t len = total - ihl;
    const auto src = Ipv4Address::load(ip + 12);
    const auto dst = Ipv4Address::load(ip + 16);
    switch (std::to_integer<std::uint8_t>(ip[9])) {
        case ip_proto_icmp:
            if (len >= 8) {
                store_be16(l4 + 2, 0);
                store_be16(l4 + 2, checksum({l4, len}));
            }
            break;
        case ip_proto_udp:
            if (len >= 8) {
                const std::size_t udp_len = load_be16(l4 + 4);
                len = udp_len >= 8 && udp_len <= len ? udp_len : len;
                store_be16(l4 + 6, 0);
                const std::uint16_t sum =
                    checksum_finish(checksum_add({l4, len}, TestStack::pseudo_sum(src, dst, ip_proto_udp, len)));
                store_be16(l4 + 6, sum == 0 ? 0xFFFF : sum);
            }
            break;
        case ip_proto_tcp:
            if (len >= tcp_header_size) {
                store_be16(l4 + 16, 0);
                store_be16(l4 + 16, checksum_finish(checksum_add(
                                        {l4, len}, TestStack::pseudo_sum(src, dst, ip_proto_tcp, len))));
            }
            break;
        default: break;
    }
}

Bytes bytes_of(const PacketRef& p) { return {p->data(), p->data() + p->size()}; }

/// TCP echo listener that tracks its single connection.
struct Echo {
    TcpConnectionBase* connection = nullptr;
    int accepted = 0;

    static void on_accepted(void* self, TcpConnectionBase& c) noexcept {
        auto& e = *static_cast<Echo*>(self);
        e.connection = &c;
        ++e.accepted;
    }
    static void on_received(void* self, TcpConnectionBase& base) noexcept {
        (void)self;
        auto& c = static_cast<TestStack::TcpConnection&>(base);
        std::array<std::byte, 1500> buf{};
        const std::size_t n = c.read(buf);
        if (n != 0) {
            c.write({buf.data(), n});
        }
        if (c.eof()) {
            c.close();
        }
    }
    static void on_closed(void* self, TcpConnectionBase& c, TcpCloseReason) noexcept {
        auto& e = *static_cast<Echo*>(self);
        if (e.connection == &c) {
            e.connection = nullptr;
        }
    }
    TcpHandler handler{this, &on_accepted, &on_received, nullptr, &on_closed};
};

constexpr std::uint16_t echo_port = 7;
constexpr std::uint16_t sntp_local_port = 12'300;
constexpr std::uint16_t dns_query_port = 49152 + 100000 % 16384;  // fixture random32() == 100000
constexpr std::uint16_t dns_query_id = static_cast<std::uint16_t>(100000);

class StackFuzz : public NetTest {
protected:
    MdnsResponder<TestStack> mdns{stack, "fcu"};
    SntpClient<TestStack> sntp{stack, peer_ip, 123, 64'000, sntp_local_port};
    DnsResolver<TestStack> resolver{stack};
    DhcpServer<TestStack> dhcp_server{stack, eth, {.first = Ipv4Address::from(192, 168, 77, 100), .count = 4}};
    Echo echo;
    Mutator m{seed(0x5EED'0001u)};

    std::uint16_t tcp_peer_port = 40'000;
    std::uint32_t tcp_peer_seq = 0;
    std::uint32_t tcp_iss = 0;
    std::string query_name = "q0.example";
    int queries = 0;

    void SetUp() override {
        NetTest::SetUp();
        ASSERT_TRUE(stack.tcp().listen(echo_port, echo.handler));
        ASSERT_TRUE(mdns.start());
        ASSERT_TRUE(sntp.start());
        ASSERT_TRUE(dhcp_server.start());
        const std::array servers{peer_ip};
        resolver.set_servers(servers);
        for (int i = 0; i < 100 && !mdns.ready(eth); ++i) {
            step(50);
        }
        ASSERT_TRUE(mdns.ready(eth));
    }

    void services() {
        mdns.service();
        sntp.service();
        resolver.service();
        dhcp_server.service();
        stack.flush();
    }

    void step(std::uint32_t ms) {
        poll(ms);
        services();
        (void)sent();
    }

    void inject(const Bytes& f) {
        PacketRef p = pool.allocate(0);
        ASSERT_TRUE(p);
        p->append(f);
        driver.rx.push_back(std::move(p));
        step(m.below(20));
    }

    /// Keep a TCP connection and a DNS query alive, so their seeds reach real state.
    void keep_targets_alive() {
        if (!stack.configured(eth)) {
            // A mutated ARP claimed our address twice within 10 s: the stack gave it up
            // (RFC 5227 2.4). The application takes it again - so do we.
            stack.set_config(eth, {device_ip, Ipv4Address::from(255, 255, 255, 0), Ipv4Address::from(192, 168, 77, 254)});
        }
        if (echo.connection != nullptr) {
            // A fresh connection, so the data seeds hit its exact sequence number again.
            static_cast<TestStack::TcpConnection*>(echo.connection)->abort();
            (void)sent();
        }
        if (echo.connection == nullptr) {
            ++tcp_peer_port;
            tcp_peer_seq = m.below(0xFFFF'FFFFu);
            deliver(tcp(tcp_peer_port, echo_port, tcp_peer_seq, 0, tcp_flag::syn, {}, 8192, 1460));
            services();
            for (const Frame& f : sent()) {
                if (f.ethertype() == ethertype_ipv4 && f.ip_protocol() == ip_proto_tcp &&
                    f.flags() == (tcp_flag::syn | tcp_flag::ack)) {
                    tcp_iss = f.seq();
                    ++tcp_peer_seq;
                    deliver(tcp(tcp_peer_port, echo_port, tcp_peer_seq, tcp_iss + 1, tcp_flag::ack));
                    services();
                    (void)sent();
                }
            }
        }
        query_name = "q" + std::to_string(queries) + ".example";
        if (resolver.resolve(query_name, [](void*, std::string_view, std::optional<Ipv4Address>) noexcept {}, nullptr)) {
            ++queries;
        }
        services();
        (void)sent();
    }

    // ---- seeds ----------------------------------------------------------------

    Bytes dns_message(std::uint16_t id, std::uint16_t flags, std::string_view name, bool answer) {
        Bytes msg(dns::header_size);
        store_be16(msg.data(), id);
        store_be16(msg.data() + 2, flags);
        store_be16(msg.data() + 4, 1);
        store_be16(msg.data() + 6, answer ? 1 : 0);
        std::array<std::byte, 160> n{};
        const std::size_t len = dns::write_name(n.data(), n.size(), name);
        msg.insert(msg.end(), n.begin(), n.begin() + static_cast<std::ptrdiff_t>(len));
        msg.resize(msg.size() + 4);
        store_be16(msg.data() + msg.size() - 4, dns::type_a);
        store_be16(msg.data() + msg.size() - 2, dns::class_in);
        if (answer) {
            const std::size_t rr = msg.size();
            msg.resize(rr + 16);
            store_be16(msg.data() + rr, 0xC00C);  // pointer to the question name
            store_be16(msg.data() + rr + 2, dns::type_a);
            store_be16(msg.data() + rr + 4, dns::class_in);
            store_be32(msg.data() + rr + 6, 300);
            store_be16(msg.data() + rr + 10, 4);
            Ipv4Address::from(10, 0, 0, 1).store(msg.data() + rr + 12);
        }
        return msg;
    }

    Bytes dhcp_request(std::uint8_t type) {
        Bytes msg(dhcp::options_offset);
        msg[0] = std::byte{1};
        msg[1] = std::byte{1};
        msg[2] = std::byte{6};
        store_be32(msg.data() + 4, 0x1234'5678);
        MacAddress::from(0x02, 0, 0, 0, 0, 0x42).store(msg.data() + 28);
        store_be32(msg.data() + dhcp::bootp_size, dhcp::magic_cookie);
        const std::byte opts[] = {std::byte{53}, std::byte{1},   std::byte{type},  std::byte{50}, std::byte{4},
                                  std::byte{192}, std::byte{168}, std::byte{77},   std::byte{100}, std::byte{54},
                                  std::byte{4},   std::byte{192}, std::byte{168},  std::byte{77},  std::byte{2},
                                  std::byte{255}};
        msg.insert(msg.end(), std::begin(opts), std::end(opts));
        return msg;
    }

    std::vector<std::function<Bytes()>> seeds() {
        return {
            [&] { return bytes_of(arp(1, peer_mac, peer_ip, device_ip)); },
            [&] { return bytes_of(arp(2, peer_mac, peer_ip, device_ip, device_mac)); },
            [&] {
                Bytes echo_req(12);
                echo_req[0] = std::byte{8};
                store_be16(echo_req.data() + 2, checksum(echo_req));
                return bytes_of(ipv4(ip_proto_icmp, echo_req));
            },
            [&] {
                return bytes_of(udp(mdns_port, mdns_port, dns_message(0, 0, "fcu.local", false), mdns_group));
            },
            [&] {
                return bytes_of(udp(mdns_port, mdns_port, dns_message(0, dns::flag_qr | dns::flag_aa, "fcu.local", true),
                                    mdns_group, Ipv4Address::from(192, 168, 77, 200)));
            },
            [&] {  // legacy unicast query
                return bytes_of(udp(40'000, mdns_port, dns_message(7, 0, "fcu.local", false), mdns_group));
            },
            [&] {
                Bytes ntp(48);
                ntp[0] = std::byte{0x24};
                ntp[1] = std::byte{2};
                store_be32(ntp.data() + 40, 0xE000'0000u);
                return bytes_of(udp(123, sntp_local_port, ntp));
            },
            [&] {
                return bytes_of(udp(53, dns_query_port, dns_message(dns_query_id, dns::flag_qr | 0x0180, query_name, true)));
            },
            [&] {
                return bytes_of(udp(68, 67, dhcp_request(dhcp::discover), Ipv4Address::broadcast(), Ipv4Address::any()));
            },
            [&] {
                return bytes_of(udp(68, 67, dhcp_request(dhcp::request), Ipv4Address::broadcast(), Ipv4Address::any()));
            },
            [&] { return bytes_of(tcp(tcp_peer_port + 1000, echo_port, 1, 0, tcp_flag::syn, {}, 8192, 1460)); },
            [&] {
                return bytes_of(tcp(tcp_peer_port, echo_port, tcp_peer_seq, tcp_iss + 1, tcp_flag::ack | tcp_flag::psh,
                                    "GET / HTTP/1.1\r\n\r\n"));
            },
            [&] {
                return bytes_of(tcp(tcp_peer_port, echo_port, tcp_peer_seq, tcp_iss + 1, tcp_flag::ack | tcp_flag::fin));
            },
            [&] { return bytes_of(tcp(tcp_peer_port, echo_port, tcp_peer_seq, tcp_iss + 1, tcp_flag::rst)); },
        };
    }
};

TEST_F(StackFuzz, ReceivePathSurvivesMutatedFrames) {
    const auto seed = seeds();
    const std::size_t n = iterations(20'000);
    for (std::size_t i = 0; i < n; ++i) {
        if (i % 64 == 0) {
            keep_targets_alive();
        }
        Bytes f = seed[m.below(static_cast<std::uint32_t>(seed.size()))]();
        m.mutate(f, m.chance(16) ? 0 : ethernet_header_size);
        if (!m.chance(8)) {
            fix_checksums(f);
        }
        inject(f);
        ASSERT_FALSE(HasFatalFailure()) << "iteration " << i;
    }

    // Let every timer run out, then the stack must still work.
    for (int s = 0; s < 400; ++s) {
        step(1000);
    }
    ASSERT_GE(pool.available(), pool.capacity() - 1);  // nothing parked forever
    keep_targets_alive();  // takes the address again if a mutated ARP made the stack give it up
    prime_arp();
    Bytes echo_req(12);
    echo_req[0] = std::byte{8};
    store_be16(echo_req.data() + 2, checksum(echo_req));
    deliver(ipv4(ip_proto_icmp, echo_req));
    auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].ip_protocol(), ip_proto_icmp);

    deliver(tcp(60'000, echo_port, 1, 0, tcp_flag::syn, {}, 8192, 1460));
    f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].flags(), tcp_flag::syn | tcp_flag::ack);
    deliver(tcp(60'000, echo_port, 2, f[0].seq() + 1, tcp_flag::ack | tcp_flag::psh, "alive"));
    f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].tcp_payload(), "alive");
}

// ---------------------------------------------------------------------------
// Parsers on their own (no checksums in the way, many more iterations)
// ---------------------------------------------------------------------------

TEST(ParserFuzz, HttpRequestParser) {
    const std::string seeds[] = {
        "GET /index.html HTTP/1.1\r\nHost: fcu.local\r\nAccept-Encoding: gzip, br\r\n\r\n",
        "GET /ws HTTP/1.1\r\nHost: fcu.local\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
        "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\nOrigin: https://fcu.local\r\n\r\n",
        "GET /a HTTP/1.1\r\n\r\nGET /b HTTP/1.0\r\nConnection: close\r\n\r\n",
    };
    Mutator m{seed(0x5EED'0002u)};
    const std::size_t n = iterations(20'000) * 2;
    for (std::size_t i = 0; i < n; ++i) {
        const std::string& s = seeds[m.below(std::size(seeds))];
        Bytes b(reinterpret_cast<const std::byte*>(s.data()), reinterpret_cast<const std::byte*>(s.data() + s.size()));
        m.mutate(b, 0, 600);
        HttpRequestParser<512> p;
        using S = HttpRequestParser<512>::State;
        std::size_t pos = 0;
        for (int rounds = 0; rounds < 64; ++rounds) {
            const auto space = p.free_space();
            const std::size_t chunk = std::min({b.size() - pos, space.size(), std::size_t{1} + m.below(64)});
            std::memcpy(space.data(), b.data() + pos, chunk);
            pos += chunk;
            const S st = p.commit(chunk);
            if (st == S::Complete) {
                (void)p.header("host");
                (void)p.keep_alive();
                ASSERT_LE(p.method().size() + p.target().size() + p.version().size(), 512u);
                p.next();
            } else if (st != S::Incomplete || (pos == b.size() && chunk == 0)) {
                break;
            }
        }
    }
}

TEST(ParserFuzz, WebSocketDecoder) {
    using Decoder = WebSocketDecoder<512>;
    const std::vector<Bytes> seeds = {
        {std::byte{0x81}, std::byte{0x85}, std::byte{0x37}, std::byte{0xfa}, std::byte{0x21}, std::byte{0x3d},
         std::byte{0x7f}, std::byte{0x9f}, std::byte{0x4d}, std::byte{0x51}, std::byte{0x58}},  // "Hello"
        {std::byte{0x01}, std::byte{0x81}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{'a'},
         std::byte{0x89}, std::byte{0x80}, std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4},
         std::byte{0x80}, std::byte{0x81}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{'b'}},
        {std::byte{0x88}, std::byte{0x82}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0x03},
         std::byte{0xE8}},  // close 1000
        {std::byte{0x82}, std::byte{0xFE}, std::byte{0x01}, std::byte{0x00}, std::byte{0}, std::byte{0},
         std::byte{0}, std::byte{0}},  // 256-byte header, payload follows via mutation
    };
    Mutator m{seed(0x5EED'0003u)};
    const std::size_t n = iterations(20'000) * 2;
    for (std::size_t i = 0; i < n; ++i) {
        Bytes b = seeds[m.below(static_cast<std::uint32_t>(seeds.size()))];
        m.mutate(b, 0, 700);
        Decoder d;
        std::size_t pos = 0;
        for (int rounds = 0; rounds < 256 && pos < b.size(); ++rounds) {
            const std::size_t chunk = std::min<std::size_t>(b.size() - pos, 1 + m.below(32));
            Decoder::Event e{};
            const std::size_t used = d.feed({b.data() + pos, chunk}, e);
            ASSERT_LE(used, chunk);
            pos += used;
            if (e == Decoder::Event::Message) {
                ASSERT_LE(d.message().size(), 512u);
            } else if (e == Decoder::Event::Ping || e == Decoder::Event::Close) {
                ASSERT_LE(d.control_payload().size(), 125u);
            } else if (e == Decoder::Event::Error) {
                break;
            } else if (used == 0 && e == Decoder::Event::None && chunk != 0) {
                FAIL() << "decoder made no progress";
            }
        }
    }
}

TEST(ParserFuzz, DnsNames) {
    Mutator m{seed(0x5EED'0004u)};
    const std::size_t n = iterations(20'000) * 2;
    for (std::size_t i = 0; i < n; ++i) {
        Bytes msg(dns::header_size);
        std::array<std::byte, 64> name{};
        msg.insert(msg.end(), name.begin(),
                   name.begin() + static_cast<std::ptrdiff_t>(dns::write_name(name.data(), name.size(), "fcu.local")));
        msg.push_back(std::byte{0xC0});
        msg.push_back(std::byte{12});
        m.mutate(msg, 0, 300);
        const std::size_t at = m.below(static_cast<std::uint32_t>(msg.size() + 2));
        if (const auto end = dns::skip_name(msg, at)) {
            ASSERT_LE(*end, msg.size());
        }
        if (const auto end = dns::match_name(msg, at, "fcu.local")) {
            ASSERT_LE(*end, msg.size());
        }
        std::array<char, 128> out{};
        if (const auto len = dns::read_name(msg, at, out)) {  // length of the dotted name
            ASSERT_LE(*len, out.size());
        }
    }
}

TEST(ParserFuzz, DhcpOptions) {
    Mutator m{seed(0x5EED'0005u)};
    const Bytes seed = {std::byte{53}, std::byte{1},  std::byte{5},  std::byte{54}, std::byte{4},  std::byte{10},
                        std::byte{0},  std::byte{0},  std::byte{1},  std::byte{6},  std::byte{8},  std::byte{10},
                        std::byte{0},  std::byte{0},  std::byte{1},  std::byte{10}, std::byte{0},  std::byte{0},
                        std::byte{2},  std::byte{51}, std::byte{4},  std::byte{0},  std::byte{0},  std::byte{14},
                        std::byte{16}, std::byte{0},  std::byte{255}};
    const std::size_t n = iterations(20'000) * 2;
    for (std::size_t i = 0; i < n; ++i) {
        Bytes b = seed;
        m.mutate(b, 0, 312);
        dhcp::Options o{};
        (void)dhcp::parse_options(b, o);
    }
}

}  // namespace
