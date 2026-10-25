// SPDX-License-Identifier: MIT
//
// Network firmware for the Factory Control Unit.
//
//   IPv4   DHCP (host name "fcu"); optionally a static fallback address when no
//          DHCP server answers within 10 s
//   DNS    servers from the DHCP lease (option 6)
//   mDNS   fcu.local
//   SNTP   server from DHCP option 42, else the DNS name sntp_host
//   Build configuration (fcu_net_config.hpp, generated from CMake cache variables):
//          preset "board" = real network (port 123, no fallback), preset "arm" =
//          emulation (SNTP port 12300, fallback 192.168.77.2/24 via 192.168.77.1)
//   HTTPS  https://fcu.local/        web app (brotli, compiled in with #embed)
//          https://fcu.local/blob    100 KB test blob (throughput)
//          wss://fcu.local/ws        binary WebSocket, messages <= 512 bytes:
//            device -> browser  0x01 status (10 Hz, see send_status()), incl. cycle
//                               counts of TLS handshake, record crypto and echo
//                               0x21 echo reply (payload of the request)
//            browser -> device  0x10 set LED (1 byte: 0/1)
//                               0x20 echo request (up to 511 bytes)
//
// One network thread owns the whole stack: it runs Stack::poll() whenever the ETH
// interrupt signals it (notify_from_isr) or the next protocol timer is due.
//
// Timing: DWT_CYCCNT at SYSCLK (250 MHz). On the hardware the cycle counts are
// real; in the emulators (Renode, QEMU -icount) they count instructions (CPI 1).
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string_view>

#include "board/fcu.hpp"
#include "fcu_net_config.hpp"
#include "rtos/net/dhcp.hpp"
#include "rtos/net/dns_resolver.hpp"
#include "rtos/net/https_server.hpp"
#include "rtos/net/mdns.hpp"
#include "rtos/net/sntp.hpp"
#include "rtos/net/stack.hpp"
#include "rtos/net/tls.hpp"
#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/stm32h5/cache.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/ethernet.hpp"
#include "rtos/port/stm32h5/rng.hpp"
#include "rtos/rtos.hpp"

namespace {

struct KernelConfig : rtos::DefaultConfig {
    static constexpr std::size_t max_threads = 4;
    static constexpr std::size_t notify_slots = 2;
};

using Port = rtos::port::cm33::Cm33Port;
using Kernel = rtos::Kernel<Port, KernelConfig>;

}  // namespace

Kernel kernel;
RTOS_CM33_BIND_KERNEL(kernel)

namespace {

namespace h5 = rtos::port::stm32h5;
namespace net = rtos::net;
namespace cm33 = rtos::port::cm33;

// ---- compiled-in assets (C++26 #embed) ------------------------------------------------
constexpr unsigned char index_html_br[] = {
#embed "index.html.br"
};
constexpr unsigned char test_blob[] = {
#embed "blob.bin"
};
constexpr unsigned char server_certificate[] = {
#embed "server.der"
};
constexpr unsigned char server_private_key[] = {
#embed "server.key.der"
};

template <std::size_t N>
std::span<const std::byte> as_span(const unsigned char (&a)[N]) {
    return {reinterpret_cast<const std::byte*>(a), N};
}

// ---- network stack ---------------------------------------------------------------------
using Pool = net::PacketPool<32>;
constexpr std::size_t eth_rx_descriptors = 8;
using Ethernet = h5::Ethernet<Pool, eth_rx_descriptors>;
struct NetConfig : net::DefaultStackConfig {
    static constexpr std::size_t udp_sockets = 6;  // DHCP, mDNS, SNTP, 2 DNS queries, spare
    // Browsers open several connections per page (spare sockets, keep-alive, the
    // WebSocket); closing ones linger in TIME_WAIT.
    static constexpr std::size_t tcp_connections = 8;
};
using Stack = net::Stack<Ethernet, Pool, NetConfig>;
// Buffer budget (stack.hpp): the receive ring keeps its buffers, the reserve is for
// receiving only, and every connection must still get at least one segment out.
static_assert(Pool::capacity() >= eth_rx_descriptors + NetConfig::rx_reserve + NetConfig::tcp_connections,
              "packet pool too small for RX ring + receive reserve + one segment per connection");
constexpr std::size_t max_https_sessions = 6;
using Server = net::HttpsServer<Stack, max_https_sessions>;

using fcu_net_config::sntp_host;
using fcu_net_config::sntp_port;
constexpr rtos::Tick dhcp_fallback_ms = 10'000;

RTOS_DMA_BUFFER Pool packet_pool;
RTOS_DMA_BUFFER Ethernet ethernet{packet_pool, board::mac_address(), board::phy_address};
Stack stack{ethernet, packet_pool, {}};  // unconfigured until DHCP (or the fallback)
net::MdnsResponder<Stack> mdns{stack, "fcu"};
net::SntpClient<Stack> sntp{stack, net::Ipv4Address::any(), sntp_port, 60'000};
net::DnsResolver<Stack> dns{stack};

// ---- TLS + HTTPS ----------------------------------------------------------------------
alignas(16) std::array<std::byte, 160 * 1024> tls_memory;  // wolfSSL static pool (no heap)
net::TlsServerContext tls_context;

const std::array<net::HttpResource, 2> resources{{
    {"/", "text/html; charset=utf-8", "br", as_span(index_html_br)},
    {"/blob", "application/octet-stream", "", as_span(test_blob)},
}};

constexpr std::uint8_t msg_status = 0x01, msg_set_led = 0x10, msg_echo = 0x20, msg_echo_reply = 0x21;
bool led_state = false;
std::uint32_t echo_cycles = 0;  ///< last echo: start of the TLS read -> reply handed to TCP

void on_ws_opened(void*, Server::SessionId id) noexcept;
void on_ws_message(void*, Server::SessionId id, std::span<const std::byte> data, bool binary) noexcept;
void on_ws_closed(void*, Server::SessionId id) noexcept;

const Server::WebSocketHandler ws_handler{nullptr, &on_ws_opened, &on_ws_message, &on_ws_closed};
Server server{stack, tls_context, resources, "/ws", ws_handler};

// ---- thread ---------------------------------------------------------------------------
Kernel::Thread net_tcb;
alignas(8) std::array<std::byte, 12 * 1024> net_stack;
constexpr std::size_t slot_wake = 0;
constexpr rtos::Tick link_poll_ms = 500;
constexpr rtos::Tick status_period_ms = 100;

// ---- console helpers -----------------------------------------------------------------

void print_ip(net::Ipv4Address a) {
    for (int shift = 24; shift >= 0; shift -= 8) {
        board::print_dec((a.value >> shift) & 0xFF);
        if (shift != 0) {
            board::print(".");
        }
    }
}

void print_mac(const net::MacAddress& m) {
    constexpr const char* hex = "0123456789abcdef";
    for (std::size_t i = 0; i < 6; ++i) {
        const auto b = std::to_integer<unsigned>(m.octets[i]);
        const char s[3] = {hex[b >> 4], hex[b & 15], ':'};
        board::print(std::string_view(s, i == 5 ? 2 : 3));
    }
}

/// "YYYY-MM-DD hh:mm:ss UTC" (H. Hinnant's civil_from_days).
void print_utc(std::uint64_t unix_ms) {
    const std::int64_t secs = static_cast<std::int64_t>(unix_ms / 1000);
    std::int64_t days = secs / 86400;
    const std::int64_t rem = secs % 86400;
    days += 719468;
    const std::int64_t era = days / 146097;
    const std::int64_t doe = days - era * 146097;
    const std::int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const std::int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const std::int64_t mp = (5 * doy + 2) / 153;
    const std::int64_t day = doy - (153 * mp + 2) / 5 + 1;
    const std::int64_t month = mp < 10 ? mp + 3 : mp - 9;
    const std::int64_t year = yoe + era * 400 + (month <= 2);
    auto two = [](std::int64_t v) {
        const char s[2] = {static_cast<char>('0' + v / 10), static_cast<char>('0' + v % 10)};
        board::print(std::string_view(s, 2));
    };
    board::print_dec(static_cast<std::uint32_t>(year));
    board::print("-");
    two(month);
    board::print("-");
    two(day);
    board::print(" ");
    two(rem / 3600);
    board::print(":");
    two(rem / 60 % 60);
    board::print(":");
    two(rem % 60);
    board::print(" UTC");
}

std::uint32_t earliest(std::uint32_t a, std::uint32_t b) {
    if (a == 0) {
        return b;
    }
    if (b == 0) {
        return a;
    }
    return static_cast<std::int32_t>(a - b) < 0 ? a : b;
}

// ---- addressing: DHCP, DNS, SNTP server ---------------------------------------------

void on_sntp_host(void*, std::string_view name, std::optional<net::Ipv4Address> address) noexcept {
    board::print("dns: ");
    board::print(name);
    if (!address) {
        board::print(" not resolved\n");
        return;
    }
    board::print(" -> ");
    print_ip(*address);
    board::print("\n");
    sntp.set_server(*address, sntp_port);
}

void use_time_server(net::Ipv4Address from_dhcp) noexcept {
    if (from_dhcp != net::Ipv4Address::any()) {
        sntp.set_server(from_dhcp, sntp_port);
    } else if (!dns.resolve(sntp_host, &on_sntp_host, nullptr)) {
        board::print("dns: no server for the SNTP host name\n");
    }
}

void on_dhcp_bound(void*, const net::DhcpLease& lease, bool renewed) noexcept {
    board::print(renewed ? "dhcp: renewed " : "dhcp: bound ");
    print_ip(lease.config.address);
    board::print(" mask ");
    print_ip(lease.config.netmask);
    board::print(", router ");
    print_ip(lease.config.gateway);
    board::print(", dns ");
    print_ip(lease.dns[0]);
    board::print(", lease ");
    board::print_dec(lease.lease_s);
    board::print(" s\n");
    if (!renewed) {
        dns.set_servers(lease.dns);
        mdns.announce();
        use_time_server(lease.ntp[0]);
    }
}

void on_dhcp_lost(void*) noexcept { board::print("dhcp: lease lost\n"); }

const net::DhcpClient<Stack>::Handler dhcp_handler{nullptr, &on_dhcp_bound, &on_dhcp_lost};
net::DhcpClient<Stack> dhcp{stack, "fcu", dhcp_handler};

// ---- application ---------------------------------------------------------------------

void store_le16(std::byte* p, std::uint16_t v) {
    p[0] = static_cast<std::byte>(v);
    p[1] = static_cast<std::byte>(v >> 8);
}
void store_le32(std::byte* p, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) {
        p[i] = static_cast<std::byte>(v >> (8 * i));
    }
}

/// Integer sine (Bhaskara I approximation), amplitude 32767, period 4 s.
std::int16_t sample(std::uint32_t now_ms) {
    const std::int32_t x = static_cast<std::int32_t>(now_ms % 4000) * 360 / 4000;  // degrees
    const std::int32_t d = x % 180;
    const std::int32_t v = 4 * d * (180 - d) * 32767 / (40500 - d * (180 - d));
    return static_cast<std::int16_t>(x < 180 ? v : -v);
}

std::uint16_t per_byte_x16(const net::TlsCostModel& m) {
    return static_cast<std::uint16_t>(std::clamp(m.per_byte() * 16 + 0.5, 0.0, 65535.0));
}
std::uint32_t fixed_cycles(const net::TlsCostModel& m) {
    return static_cast<std::uint32_t>(std::clamp(m.fixed() + 0.5, 0.0, 4.0e9));
}

/// Status (47 bytes, little endian):
///   0 type 0x01 | 1 uptime ms u32 | 5 sample i16 | 7 LED u8 | 8 clients u8
///   9 rx frames u32 | 13 tx frames u32 | 17 unix ms u64 (0: not synchronised)
///   25 last TLS handshake, CPU cycles u32 | 29 encrypt cycles/byte x16 u16
///   31 decrypt cycles/byte x16 u16 | 33 last echo cycles u32 | 37 SYSCLK MHz u16
///   39 encrypt fixed cycles per record u32 | 43 decrypt fixed cycles per record u32
///   (cycles = fixed + per byte x bytes, least-squares fit over all records)
std::array<std::byte, 47> status_message() {
    std::array<std::byte, 47> m{};
    const std::uint32_t now = kernel.now();
    m[0] = std::byte{msg_status};
    store_le32(&m[1], now);
    store_le16(&m[5], static_cast<std::uint16_t>(sample(now)));
    m[7] = std::byte{led_state ? std::uint8_t{1} : std::uint8_t{0}};
    m[8] = static_cast<std::byte>(server.websocket_clients());
    store_le32(&m[9], stack.stats().rx_frames);
    store_le32(&m[13], stack.stats().tx_frames);
    const std::uint64_t unix_ms = sntp.unix_ms(now);
    store_le32(&m[17], static_cast<std::uint32_t>(unix_ms));
    store_le32(&m[21], static_cast<std::uint32_t>(unix_ms >> 32));
    const net::TlsStats& tls = net::TlsServerContext::stats();
    store_le32(&m[25], tls.handshake_cycles);
    store_le16(&m[29], per_byte_x16(tls.write));
    store_le16(&m[31], per_byte_x16(tls.read));
    store_le32(&m[33], echo_cycles);
    store_le16(&m[37], static_cast<std::uint16_t>(board::sysclk_hz / 1'000'000));
    store_le32(&m[39], fixed_cycles(tls.write));
    store_le32(&m[43], fixed_cycles(tls.read));
    return m;
}

void send_status() {
    const auto m = status_message();
    server.broadcast(m);
}

void on_ws_opened(void*, Server::SessionId id) noexcept {
    board::print("ws: client ");
    board::print_dec(id);
    board::print(" connected\n");
    const auto m = status_message();
    server.send(id, m);  // first data at once, not with the next 100 ms tick
}

void on_ws_closed(void*, Server::SessionId id) noexcept {
    board::print("ws: client ");
    board::print_dec(id);
    board::print(" closed\n");
}

void on_ws_message(void*, Server::SessionId id, std::span<const std::byte> data, bool binary) noexcept {
    if (!binary || data.empty()) {
        return;
    }
    switch (std::to_integer<std::uint8_t>(data[0])) {
        case msg_set_led:
            if (data.size() >= 2) {
                led_state = data[1] != std::byte{0};
                board::led_info(led_state);
                board::print(led_state ? "app: led on\n" : "app: led off\n");
            }
            break;
        case msg_echo: {
            std::array<std::byte, 512> reply{};
            reply[0] = std::byte{msg_echo_reply};
            std::memcpy(reply.data() + 1, data.data() + 1, data.size() - 1);
            if (server.send(id, std::span(reply).first(data.size()))) {
                echo_cycles = cm33::CycleCounter::now() - server.read_cycle_stamp(id);
            }
            break;
        }
        default: break;
    }
}

// ---- network thread ------------------------------------------------------------------

bool rng_fill(std::span<std::byte> out) noexcept {
    for (std::size_t i = 0; i < out.size(); i += 4) {
        const std::uint32_t r = h5::Rng::next();
        for (std::size_t j = 0; j < 4 && i + j < out.size(); ++j) {
            out[i + j] = static_cast<std::byte>(r >> (8 * j));
        }
    }
    return true;
}

void net_main(void*) {
    board::init_ethernet();
    if (!ethernet.init(board::sysclk_hz)) {
        board::print("net: ETH DMA reset timed out (no RMII reference clock?)\n");
        return;
    }
    h5::Rng::init();
    stack.set_random([](void*) noexcept { return h5::Rng::next(); }, nullptr);

    constexpr unsigned eth_irq = 106;
    cm33::nvic::ipr(eth_irq) = 6u << (8 - RTOS_CM33_PRIO_BITS);
    cm33::nvic::enable(eth_irq);

    board::print("net: mac ");
    print_mac(ethernet.mac());
    board::print("\n");

    if (!net::TlsServerContext::initialize_library(
            &rng_fill, []() noexcept -> std::uint32_t { return kernel.now() / 1000; },
            []() noexcept { return cm33::CycleCounter::now(); })) {
        board::print("tls: library init failed\n");
        return;
    }
    if (const auto r = tls_context.init(tls_memory, {as_span(server_certificate), as_span(server_private_key)},
                                        static_cast<int>(max_https_sessions));
        r != net::TlsInitResult::Ok) {
        board::print("tls: context init failed, step ");
        board::print_dec(static_cast<std::uint32_t>(r));
        board::print("\n");
        return;
    }

    stack.poll(kernel.now());
    mdns.start();
    sntp.start();
    server.start(443);
    board::print("net: mdns ");
    board::print(mdns.name());
    board::print(", https://");
    board::print(mdns.name());
    board::print("/\n");

    bool link = false;
    bool synced = false;
    bool addressing_started = false;
    rtos::Tick fallback_at = 0;
    std::uint32_t handshakes = 0;
    rtos::Tick next_link_poll = 0;
    rtos::Tick next_status = 0;
    for (;;) {
        const rtos::Tick now = kernel.now();
        if (static_cast<std::int32_t>(now - next_link_poll) >= 0) {
            next_link_poll = now + link_poll_ms;
            if (ethernet.poll_link() != link) {
                link = !link;
                board::print(link ? "net: link up\n" : "net: link down\n");
                if (link && !addressing_started) {
                    addressing_started = true;
                    dhcp.start();
                    if constexpr (fcu_net_config::static_fallback) {
                        fallback_at = now + dhcp_fallback_ms;
                    }
                }
            }
        }
        if (fallback_at != 0 && static_cast<std::int32_t>(now - fallback_at) >= 0) {
            fallback_at = 0;
            if (!dhcp.bound()) {
                dhcp.stop();
                stack.set_config(fcu_net_config::fallback);
                board::print("dhcp: no server, static ");
                print_ip(fcu_net_config::fallback.address);
                board::print("\n");
                mdns.announce();
                sntp.set_server(fcu_net_config::fallback.gateway, sntp_port);
            }
        }
        stack.poll(now);
        dhcp.service();
        dns.service();
        mdns.service();
        sntp.service();
        server.service();
        if (static_cast<std::int32_t>(now - next_status) >= 0) {
            next_status = now + status_period_ms;
            if (server.websocket_clients() != 0) {
                send_status();
            }
        }
        if (const auto& tls = net::TlsServerContext::stats(); tls.handshakes != handshakes) {
            handshakes = tls.handshakes;
            board::print("tls: handshake ");
            board::print_dec(tls.handshake_cycles);
            board::print(" cycles = ");
            board::print_dec(tls.handshake_cycles / (board::sysclk_hz / 1000));
            board::print(" ms CPU\n");
        }
        if (sntp.synchronized() && !synced) {
            synced = true;
            board::print("net: sntp synchronized, ");
            print_utc(sntp.unix_ms(kernel.now()));
            board::print("\n");
        }

        rtos::Tick wake = earliest(next_link_poll, stack.next_deadline());
        for (const rtos::Tick t : {dhcp.next_deadline(), dns.next_deadline(), mdns.next_deadline(),
                                   sntp.next_deadline(), server.next_deadline(), next_status, fallback_at}) {
            wake = earliest(wake, t);
        }
        const rtos::Tick after = kernel.now();
        const rtos::Tick timeout = static_cast<std::int32_t>(wake - after) > 0 ? wake - after : 1;
        (void)kernel.notify_wait(slot_wake, timeout);
    }
}

}  // namespace

extern "C" void ETH_IRQHandler() noexcept {
    if (ethernet.irq()) {
        kernel.notify_from_isr(net_tcb, slot_wake, 1, rtos::NotifyAction::SetBits);
    }
}

int main() {
    board::init();
    board::print("\nrtoos net - STM32H563 / Factory Control Unit\n");
    cm33::CycleCounter::enable();
    Port::configure(board::sysclk_hz, 1000);
    kernel.create_thread(net_tcb, {.name = "net", .entry = &net_main, .stack = net_stack, .priority = 8});
    kernel.start();
}
