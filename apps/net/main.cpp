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
//   Modbus TCP  port 502 (2 clients), unit id: any
//            holding 0     LED (0/1), the same LED as the WebSocket command
//            input 0..1    uptime in s (u32, high word first)
//            input 2       WebSocket clients
//            input 3       HTTPS requests (low 16 bits)
//            input 4       Modbus requests (low 16 bits)
//            input 5       OPC UA messages (low 16 bits)
//   OPC UA   opc.tcp://fcu.local:4840 (2 clients), Objects/FCU: Led (UInt16, holding 0),
//            Uptime (UInt32, input 0..1), WebSocketClients, HttpsRequests, ModbusRequests,
//            OpcUaMessages (UInt16, inputs 2..5) - the same registers as Modbus.
//            Basic256Sha256 Sign or SignAndEncrypt with a client certificate signed by the CA: read and
//            write; SecurityPolicy None: read only. RSA-2048 (OPC UA only, see CLAUDE.md).
//            Subscriptions (data changes) on every variable.
//
// One network thread owns the whole stack: it runs Stack::poll() whenever the ETH
// interrupt signals it (notify_from_isr) or the next protocol timer is due.
//
// Timing: DWT_CYCCNT at SYSCLK (250 MHz). On the hardware the cycle counts are
// real; in the emulators (Renode, QEMU -icount) they count instructions (CPI 1).
#include <algorithm>
#include <array>
#include <contracts>
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
#include "rtos/modbus/register_bank.hpp"
#include "rtos/net/https_server.hpp"
#include "rtos/net/modbus_tcp_server.hpp"
#include "rtos/net/opcua_server.hpp"
#include "rtos/net/service.hpp"
#include "rtos/net/mdns.hpp"
#include "rtos/net/sntp.hpp"
#include "rtos/net/stack.hpp"
#include "rtos/net/tls.hpp"
#include "rtos/opcua/clock.hpp"
#include "rtos/opcua/ns0_base.hpp"
#include "rtos/opcua/registers.hpp"
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
constexpr unsigned char opcua_certificate[] = {
#embed "opcua.der"
};
constexpr unsigned char opcua_private_key[] = {
#embed "opcua.key.der"
};
constexpr unsigned char ca_certificate[] = {
#embed "ca.der"
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
    // WebSocket); closing ones linger in TIME_WAIT. Plus Modbus and OPC UA clients.
    static constexpr std::size_t tcp_connections = 10;
    static constexpr std::size_t tcp_listeners = 3;  // HTTPS, Modbus TCP, OPC UA
    // At most 2 downloads (bulk connections) send at a time, holding at most 2 x 5
    // transmit buffers: with the RX ring (8) and the receive reserve (8) that leaves
    // 6 of the 32 buffers for the replies of interactive connections, however many
    // downloads run.
    static constexpr std::size_t tcp_bulk_connections = 2;
};
using Stack = net::Stack<Pool, NetConfig>;
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
Stack stack{packet_pool};
constexpr net::InterfaceId eth_if = 0;  // first attach(): Ethernet, unconfigured until DHCP
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
constexpr net::HttpsServerConfig https_config{.ws_ping_interval_ms = fcu_net_config::ws_ping_interval_ms,
                                              .ws_pong_timeout_ms = fcu_net_config::ws_pong_timeout_ms};
Server server{stack, tls_context, resources, "/ws", ws_handler, https_config};

// ---- Modbus TCP -----------------------------------------------------------------------
// The register bank is shared with application threads: every access takes the
// kernel lock (one BASEPRI section per request block, register_bank.hpp).
void on_registers_written(void*, std::uint16_t start, std::uint16_t count) noexcept;
using Registers = rtos::modbus::RegisterBank<8, 8, Kernel::Lock>;
Registers registers{rtos::modbus::WriteHook{nullptr, &on_registers_written}};
net::ModbusTcpServer<Stack, 2> modbus{stack, registers};
constexpr std::uint16_t reg_led = 0;
constexpr std::uint16_t reg_uptime = 0, reg_ws_clients = 2, reg_https_requests = 3, reg_modbus_requests = 4,
                        reg_opcua_messages = 5;

// ---- OPC UA ---------------------------------------------------------------------------
// Objects/FCU: variables backed by the Modbus registers above (rtos/opcua/registers.hpp).
namespace ua = rtos::opcua;
using rtos::modbus::Table;
constexpr ua::registers::Register ua_led{Table::Holding, reg_led}, ua_uptime{Table::Input, reg_uptime},
    ua_ws_clients{Table::Input, reg_ws_clients}, ua_https_requests{Table::Input, reg_https_requests},
    ua_modbus_requests{Table::Input, reg_modbus_requests}, ua_opcua_messages{Table::Input, reg_opcua_messages};
constexpr ua::NodeId ua_fcu(1, 1000);

constexpr ua::Node ua_variable(std::uint32_t id, const char* name, std::uint32_t type, const ua::registers::Register& r,
                               bool writable = false) {
    return ua::Node{
        .nodeId = ua::NodeId(1, id),
        .nodeClass = ua::NodeClass::Variable,
        .browseName = ua::QualifiedName(1, name),
        .displayName = ua::LocalizedText("en-US", name),
        .dataType = ua::NodeId(0, type),
        .accessLevel = static_cast<ua::Byte>(ua::ns0::ACCESS_LEVEL_CURRENT_READ |
                                             (writable ? ua::ns0::ACCESS_LEVEL_CURRENT_WRITE : 0)),
        .readCallback = type == ua::ns0::DataType_UInt32 ? &ua::registers::ReadUInt32 : &ua::registers::ReadUInt16,
        .writeCallback = writable ? &ua::registers::WriteUInt16 : nullptr,
        .userContext = ua::registers::Context(r)};
}

constexpr std::array ua_app_nodes = {
    ua::Node{.nodeId = ua_fcu, .nodeClass = ua::NodeClass::Object, .browseName = ua::QualifiedName(1, "FCU"),
             .displayName = ua::LocalizedText("en-US", "FCU")},
    ua_variable(5000, "Led", ua::ns0::DataType_UInt16, ua_led, true),
    ua_variable(5001, "Uptime", ua::ns0::DataType_UInt32, ua_uptime),
    ua_variable(5002, "WebSocketClients", ua::ns0::DataType_UInt16, ua_ws_clients),
    ua_variable(5003, "HttpsRequests", ua::ns0::DataType_UInt16, ua_https_requests),
    ua_variable(5004, "ModbusRequests", ua::ns0::DataType_UInt16, ua_modbus_requests),
    ua_variable(5005, "OpcUaMessages", ua::ns0::DataType_UInt16, ua_opcua_messages),
};

constexpr auto ua_app_references = [] {
    std::array<ua::ReferenceEntry, 3 + 2 * (ua_app_nodes.size() - 1)> r{};
    r[0] = {ua::NodeId(0, ua::ns0::ObjectsFolder), ua::NodeId(0, ua::ns0::Organizes), true, ua_fcu};
    r[1] = {ua_fcu, ua::NodeId(0, ua::ns0::Organizes), false, ua::NodeId(0, ua::ns0::ObjectsFolder)};
    r[2] = {ua_fcu, ua::NodeId(0, ua::ns0::HasTypeDefinition), true, ua::NodeId(0, ua::ns0::FolderType)};
    for (std::size_t i = 1; i < ua_app_nodes.size(); ++i) {
        r[1 + 2 * i] = {ua_fcu, ua::NodeId(0, ua::ns0::HasComponent), true, ua_app_nodes[i].nodeId};
        r[2 + 2 * i] = {ua_app_nodes[i].nodeId, ua::NodeId(0, ua::ns0::HasComponent), false, ua_fcu};
    }
    return r;
}();
constexpr auto ua_nodes = ua::JoinNodes(ua::kBaseNodes, ua_app_nodes);
constexpr auto ua_references = ua::JoinReferences(ua::kBaseReferences, ua_app_references);
static_assert(ua::AllTypedNodesHaveDataType(ua_nodes));
static_assert(ua::AllDataTypesResolve(ua_nodes));
static_assert(ua::AllReferencesResolve(ua_nodes, ua_references));
constexpr ua::AddressSpace ua_space(ua_nodes, ua_references);

// The OpenSecureChannel's RSA-2048 work runs in background steps (opcua_server.hpp).
alignas(16) std::array<std::byte, 48 * 1024> opcua_crypto_memory;  // wolfCrypt pool: keys, RNG, CA
net::OpcUaServer<Stack, 2> opcua{stack, ua_space, "opc.tcp://fcu.local:4840"};

void set_led(bool on, std::string_view source) noexcept {
    led_state = on;
    registers.set_holding(reg_led, on ? 1 : 0);
    board::led_info(on);
    board::print(source);
    board::print(on ? ": led on\n" : ": led off\n");
}

void on_registers_written(void*, std::uint16_t start, std::uint16_t count) noexcept {
    if (start <= reg_led && reg_led < start + count) {
        set_led(registers.holding(reg_led) != 0, "registers");
    }
}

void update_input_registers() noexcept {
    const std::uint32_t seconds = kernel.now() / 1000;
    const std::array<std::uint16_t, 6> values{
        static_cast<std::uint16_t>(seconds >> 16),         static_cast<std::uint16_t>(seconds),
        static_cast<std::uint16_t>(server.websocket_clients()), static_cast<std::uint16_t>(server.requests()),
        static_cast<std::uint16_t>(modbus.stats().requests), static_cast<std::uint16_t>(opcua.stats().messages)};
    registers.set_inputs(reg_uptime, values);
}

// Everything the network thread runs: the stack, then the services in this order.
net::NetRuntime<Stack> runtime{stack};

// ---- thread ---------------------------------------------------------------------------
Kernel::Thread net_tcb;
// Deepest path: an OPC UA OpenSecureChannel step with the RSA-2048 private key -
// SecureChannel::HandleOpen (1.1 KB) -> wolfCrypt -> sp_RsaPrivate_2048 (1.4 KB) ->
// sp_2048_mod_exp_32 (8.6 KB, window table) plus the runtime above it: ~13.5 KB.
// The paint below does not see such frames reliably (they are not written densely), and
// Renode does not enforce PSPLIM: size by the frames (arm-none-eabi-objdump, "sub sp").
alignas(8) std::array<std::byte, 20 * 1024> net_stack;
constexpr std::byte stack_paint{0xA5};  ///< untouched stack (main() paints it, report_latency() measures)

std::size_t net_stack_used() noexcept {
    const auto untouched = std::find_if(net_stack.begin(), net_stack.end(), [](std::byte b) { return b != stack_paint; });
    return static_cast<std::size_t>(net_stack.end() - untouched);
}
constexpr std::size_t slot_wake = 0;
constexpr rtos::Tick link_poll_ms = 500;
constexpr rtos::Tick status_period_ms = 100;
constexpr rtos::Tick latency_report_ms = 10'000;

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
        use_time_server(lease.ntp[0]);  // mDNS re-probes and announces by itself (stack event)
    }
}

void on_dhcp_lost(void*) noexcept { board::print("dhcp: lease lost\n"); }

void on_dhcp_declined(void*, net::Ipv4Address address) noexcept {
    board::print("dhcp: ");
    print_ip(address);
    board::print(" is in use by another station, declined\n");
}

void on_mdns_renamed(void*, std::string_view name) noexcept {
    board::print("mdns: name conflict, now ");
    board::print(name);
    board::print("\n");
}

const net::DhcpClient<Stack>::Handler dhcp_handler{nullptr, &on_dhcp_bound, &on_dhcp_lost, &on_dhcp_declined};
net::DhcpClient<Stack> dhcp{stack, eth_if, "fcu", dhcp_handler};

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
                set_led(data[1] != std::byte{0}, "app");
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

// ---- application tasks of the network thread (a NetService) ----------------------------

/// PHY link polling (MDIO), the static fallback, the 10 Hz status broadcast and the
/// log lines for TLS handshakes and SNTP.
struct AppTasks {
    rtos::Tick next_link_poll = 0;
    rtos::Tick next_status = 0;
    rtos::Tick fallback_at = 0;
    bool link = false;
    bool fallback_armed = false;
    bool synced = false;
    std::uint32_t handshakes = 0;
    rtos::Tick next_latency_report = latency_report_ms;

    /// Longest service() call and background step since the last report (service.hpp).
    static void report_latency() noexcept {
        std::size_t worst_service = 0, worst_step = 0;
        for (std::size_t i = 0; i < runtime.size(); ++i) {
            if (runtime.stats(i).max_service_cycles > runtime.stats(worst_service).max_service_cycles) {
                worst_service = i;
            }
            if (runtime.stats(i).max_step_cycles > runtime.stats(worst_step).max_step_cycles) {
                worst_step = i;
            }
        }
        constexpr std::uint32_t cycles_per_us = board::sysclk_hz / 1'000'000;
        board::print("net: longest service ");
        board::print(runtime.name(worst_service));
        board::print(" ");
        board::print_dec(runtime.stats(worst_service).max_service_cycles / cycles_per_us);
        board::print(" us, longest step ");
        if (runtime.stats(worst_step).steps == 0) {
            board::print("none\n");
        } else {
            board::print(runtime.name(worst_step));
            board::print(" ");
            board::print_dec(runtime.stats(worst_step).max_step_cycles / cycles_per_us);
            board::print(" us\n");
        }
        board::print("net: stack ");
        board::print_dec(static_cast<std::uint32_t>(net_stack_used()));
        board::print(" of ");
        board::print_dec(static_cast<std::uint32_t>(net_stack.size()));
        board::print(" bytes\n");
        runtime.reset_stats();
    }

    void service() noexcept {
        const rtos::Tick now = stack.now_ms();
        if (static_cast<std::int32_t>(now - next_latency_report) >= 0) {
            next_latency_report = now + latency_report_ms;
            report_latency();
        }
        if (static_cast<std::int32_t>(now - next_link_poll) >= 0) {
            next_link_poll = now + link_poll_ms;
            if (ethernet.poll_link() != link) {  // the stack sees it at its next poll
                link = !link;
                board::print(link ? "net: link up\n" : "net: link down\n");
                if (link && !fallback_armed && fcu_net_config::static_fallback) {
                    fallback_armed = true;  // DHCP restarts by itself on link changes
                    fallback_at = now + dhcp_fallback_ms;
                }
            }
        }
        if (fallback_at != 0 && static_cast<std::int32_t>(now - fallback_at) >= 0) {
            fallback_at = 0;
            using DhcpState = net::DhcpClient<Stack>::State;
            if (dhcp.state() == DhcpState::Requesting || dhcp.state() == DhcpState::Checking) {
                // A server answered and DHCP is still at work (the RFC 5227 check of the
                // offered address alone takes up to 7 s): don't take its address away.
                fallback_at = now + dhcp_fallback_ms;
            } else if (!dhcp.bound()) {
                dhcp.stop();
                stack.claim_address(eth_if, fcu_net_config::fallback);  // probed like a lease (RFC 5227)
                board::print("dhcp: no server, static ");
                print_ip(fcu_net_config::fallback.address);
                board::print("\n");
                sntp.set_server(fcu_net_config::fallback.gateway, sntp_port);
            }
        }
        if (static_cast<std::int32_t>(now - next_status) >= 0) {
            next_status = now + status_period_ms;
            update_input_registers();
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
            print_utc(sntp.unix_ms(now));
            board::print("\n");
        }
    }

    [[nodiscard]] std::uint32_t next_deadline() const noexcept {
        return net::earliest(net::earliest(net::earliest(next_link_poll, next_status), fallback_at),
                             next_latency_report);
    }
} app_tasks;

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
    const net::InterfaceId attached = stack.attach(ethernet);
    contract_assert(attached == eth_if);
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

    // OPC UA: after the TLS library (wolfCrypt is initialised, the global heap hint set).
    ua::SetClock([]() noexcept { return sntp.unix_ms(kernel.now()); }, []() noexcept { return kernel.now64(); });
    if (const auto r = opcua.init({as_span(opcua_certificate), as_span(opcua_private_key), as_span(ca_certificate)},
                                  opcua_crypto_memory);
        r != net::OpcUaInitResult::Ok) {
        board::print("opcua: init failed, step ");
        board::print_dec(static_cast<std::uint32_t>(r));
        board::print("\n");
        return;
    }
    ua::registers::Bind(registers);

    mdns.start(net::InterfaceSet::all(), &on_mdns_renamed);
    dhcp.start();
    sntp.start();
    server.start(443);  // all interfaces (a list: net::InterfaceSet::of({...}))
    board::print("net: mdns ");
    board::print(mdns.name());
    board::print(", https://");
    board::print(mdns.name());
    board::print("/\n");

    // Services run in the network thread ("in process"); a further server is one
    // more add().
    runtime.add(app_tasks, "app");
    runtime.add(dhcp, "dhcp");
    runtime.add(dns, "dns");
    runtime.add(mdns, "mdns");
    runtime.add(sntp, "sntp");
    runtime.add(server, "https");
    if (!modbus.start()) {
        board::print("modbus: listen failed\n");
    }
    runtime.add(modbus, "modbus");
    if (!opcua.start()) {
        board::print("opcua: listen failed\n");
    }
    runtime.add(opcua, "opcua");
    runtime.set_cycle_counter([]() noexcept { return cm33::CycleCounter::now(); });
    for (;;) {
        const std::uint32_t wake = runtime.run_once(kernel.now());
        // 0 while background work (downloads, handshakes) waits: no sleep, but every
        // pass receives frames first (service.hpp).
        (void)kernel.notify_wait(slot_wake, runtime.sleep_ms(wake, kernel.now()));
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
    std::ranges::fill(net_stack, stack_paint);
    kernel.create_thread(net_tcb, {.name = "net", .entry = &net_main, .stack = net_stack, .priority = 8});
    kernel.start();
}
