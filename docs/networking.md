# Networking

The IP stack ([include/rtos/net](../include/rtos/net)) is a zero-copy IPv4 stack for one thread. It runs with several interfaces, and every service runs in the same network thread.

## Network thread

```
 network thread (one loop, no locks)
   NetRuntime::run_once(now)
     Stack::poll        frames of all interfaces, link changes, ARP, address checks, TCP timers
     services           AppTasks, DHCP, DNS, mDNS, SNTP, HTTPS, Modbus, OPC UA, USB ... (service / next_deadline)
     one background step   round robin: one TLS handshake flight, one download record, one RSA open
     Stack::flush       FINs, ACKs and windows the services triggered
   -> background work pending: next pass at once (no sleep),
      otherwise sleep until the earliest deadline or until a driver IRQ wakes the thread

 Stack ── interface 0: Ethernet (STM32H5 ETH)   own address, ARP, multicast, link
       └─ interface 1: e.g. USB CDC-NCM          own address, ARP, multicast, link
       shared: TCP, UDP, packet pool, routing (directly connected net, else the first interface with a gateway)
```

A **service** is anything with `void service() noexcept` and `std::uint32_t next_deadline() const noexcept` ([service.hpp](../include/rtos/net/service.hpp)). Services receive network input through the stack's callbacks during `Stack::poll()`, in the same thread. They react to address and link changes through `stack.subscribe(NetEventHandler)` instead of polling.

## Protocols

| Protocol | File | Notes |
|---|---|---|
| ARP, IPv4, ICMP, UDP, TCP | [stack.hpp](../include/rtos/net/stack.hpp), [tcp.hpp](../include/rtos/net/tcp.hpp) | no Nagle: every write goes out at once with PSH; RFC 5961 checks, zero-window probes, SYN flood handling |
| DHCP client | [dhcp.hpp](../include/rtos/net/dhcp.hpp) | per interface: address, router, DNS and NTP servers, renewal at T1/T2. The address is used only after an ARP check (RFC 5227), with DHCPDECLINE on a conflict. INIT-REBOOT after a link change. |
| DHCP server | [dhcp_server.hpp](../include/rtos/net/dhcp_server.hpp) | for a point-to-point interface such as USB CDC-NCM; hands out no router |
| DNS resolver | [dns_resolver.hpp](../include/rtos/net/dns_resolver.hpp) | A records with CNAME, cache with TTL, random ID and source port per query (RFC 5452) |
| mDNS | [mdns.hpp](../include/rtos/net/mdns.hpp) | `<name>.local` on all interfaces, probing before use, renaming (`name-2`, …) on a conflict (RFC 6762) |
| SNTP | [sntp.hpp](../include/rtos/net/sntp.hpp) | Unix time in ms, server from DHCP option 42 or DNS |
| TLS 1.3 | [tls.hpp](../include/rtos/net/tls.hpp) | wolfSSL with a static pool; X25519/P-256, ChaCha20-Poly1305 (preferred: the H563 has no AES accelerator) and AES-GCM; Thumb-2 assembly for ECC, ChaCha20, Poly1305, SHA-2 |
| HTTP, WebSocket | [http.hpp](../include/rtos/net/http.hpp), [websocket.hpp](../include/rtos/net/websocket.hpp) | parsers checked by the fuzzer |
| HTTPS server | [https_server.hpp](../include/rtos/net/https_server.hpp) | static resources with content encoding, one WebSocket endpoint |
| Modbus TCP | [modbus_tcp_server.hpp](../include/rtos/net/modbus_tcp_server.hpp) | see [Modbus](modbus.md) |
| OPC UA | [opcua_server.hpp](../include/rtos/net/opcua_server.hpp) | see [OPC UA](opcua.md) |

**Address conflicts (RFC 5227):** every new address is probed with ARP before use and announced afterwards, including a static fallback address. If another station uses the address, the device defends it once; if the conflict repeats within 10 s, it gives the address up.

**ARP hardening:** see [Security](security.md#arp-hardening).

## Packet buffers

- `PacketPool<N>` holds all frames of all interfaces. It lives in the non-cacheable DMA region (`RTOS_DMA_BUFFER`).
- Every transmit buffer comes from `Stack::allocate()` with 64 bytes of headroom. Ethernet, IPv4 and TCP need 54 bytes. Each layer only moves the start of the data forward (`push_front`) and writes its header there.
- Budget: the RX ring keeps its buffers, a receive reserve is for receiving only, and every TCP connection must be able to send at least one segment. The examples check this with a `static_assert`.

## HTTPS and WebSocket

```cpp
const std::array<net::HttpResource, 2> resources{{
    {"/", "text/html; charset=utf-8", "br", as_span(index_html_br)},   // path, type, encoding, data
    {"/blob", "application/octet-stream", "", as_span(test_blob)},
}};
const Server::WebSocketHandler ws{context, &on_opened, &on_message, &on_closed};
net::HttpsServer<Stack, 6> server{stack, tls_context, resources, "/ws", ws, {.ws_ping_interval_ms = 20'000}};
server.start(443);
runtime.add(server, "https");
```

- **Records:** one `write()` makes one TLS record (at most 4 KB). A WebSocket message, header and payload together, is always exactly one record.
- **Sessions:** browsers keep keep-alive connections open for minutes and open spare connections. A new connection that finds no free session evicts the keep-alive session that has been idle the longest. WebSockets and running responses are never evicted.
- **Timeouts:** a session without a complete request closes after 10 s; an unfinished handshake is aborted after 10 s. After close_notify and FIN the session is free at once; TCP gives up FIN_WAIT_2 after 15 s.
- **WebSocket keep-alive:** a client that sent nothing for `ws_ping_interval_ms` gets a ping; if nothing comes back within `ws_pong_timeout_ms`, it is closed.
- **Origin check:** WebSocket upgrades from foreign origins are rejected with 403.
- **Measurements:** `TlsServerContext::stats()` holds the cycles of the last handshake and a per-record cost model (fixed + per byte, least-squares fit).
- **Single-page apps:** a resource with the path `"*"` answers every GET path that no other resource matches, so the app's client routes (`/system`, `/power`, …) survive a reload. Exact paths always win, wherever they are listed.

### WebSocket messages from other threads

The server belongs to the network thread. Other threads, such as a control loop that publishes its state or a log mirror, post their messages into a [`WebSocketOutbox`](../include/rtos/net/websocket_outbox.hpp), which the network thread delivers:

```cpp
rtos::net::WebSocketOutbox<Server, 2048, Kernel::Lock> outbox{server};
outbox.set_wake([](void*) noexcept { kernel.notify(net_tcb, slot_wake, 1, rtos::NotifyAction::SetBits); }, nullptr);
runtime.add(outbox, "ws-outbox");

outbox.broadcast(status);        // any thread or kernel-aware ISR, never waits
outbox.send(session, reply);
```

Posting copies the message into a byte ring under one short lock. When the ring is full, the message is refused and counted (`stats().dropped`). At delivery, a session whose transmit queue is full misses the message (`stats().undelivered`): status messages are superseded by the next ones anyway.

## Fairness in one thread

All servers share the network thread. A web app download must not delay the answer of a Modbus or OPC UA server. Three mechanisms ensure this ([service.hpp](../include/rtos/net/service.hpp)):

| Mechanism | Effect |
|---|---|
| **Background steps** (`BackgroundSession`: `background()`, `step()`) | Expensive phases work in bounded units: one TLS handshake flight, one download record (`bulk_quantum_bytes`, default 2 KB). TCP events of such sessions are only noted. Each pass runs exactly one step, round robin over all servers and sessions. The stack reads new frames in between, so a request waits at most one step. |
| **Bulk admission** (`set_bulk()`, `tcp_bulk_connections`) | At most N downloads send at the same time, each with a full window (a partial window would hit the client's 200 ms delayed ACK). The packet pool therefore always keeps buffers for interactive replies, however many downloads run. |
| **Handshake limit** (`max_handshakes`) | At most two TLS handshakes at the same time. Each one holds several KB of the wolfSSL pool between its flights. |

The runtime measures the longest `service()` call and the longest background step per service (`set_cycle_counter`, `stats()`). Single crypto operations cannot be split: a TLS handshake flight (ECDHE and an ECDSA signature) and the OPC UA OpenSecureChannel (two RSA-2048 decryptions, one signature). Both run as background steps and are measured.

The host test [https_fairness_test.cpp](../tests/https_fairness_test.cpp) lets real wolfSSL clients download over simulated TCP peers and checks the **order on the wire**: a request that arrives in the same receive batch as download ACKs or ClientHellos is answered before the next download record or handshake flight goes out. Every fairness rule is backed by such a test. The Renode test measures the WebSocket, Modbus and OPC UA response times during five parallel downloads.

## Interfaces

Which interfaces a service serves is an `InterfaceSet` ([interface.hpp](../include/rtos/net/interface.hpp)). The default is always "all":

| Service | Call | Default |
|---|---|---|
| TCP server, HTTPS, Modbus TCP, OPC UA | `server.listen(port, set)` / `https.start(443, set)` | all |
| UDP socket | `stack.udp_bind(port, handler, set)` | all |
| mDNS | `mdns.start(set, ...)` | all |
| DHCP client, DHCP server | constructor with *one* `InterfaceId` | per interface |

A port may be bound more than once if the sets do not overlap, e.g. two different servers on port 80, one per interface. A SYN on an interface without a matching listener is answered with RST. Replies leave through the interface that owns the local address (strong host model).

### A second interface: USB CDC-NCM

1. **Driver:** [`usb::CdcNcm`](../include/rtos/usb/cdc_ncm.hpp) is a USB function and a `NetDriver` ([driver.hpp](../include/rtos/net/driver.hpp)) at the same time. It takes its buffers from the same packet pool as Ethernet.
2. **Attach:** `const auto usb = stack.attach(ncm, {192.168.7.1/24, no gateway});` (`Cfg::interfaces`, default 2).
3. **Addresses for the PC:** `net::DhcpServer<Stack> dhcpd{stack, usb, {.first = 192.168.7.10, .count = 2}}; dhcpd.start(); runtime.add(dhcpd);` The server hands out no router, so the PC does not route its internet traffic through the device.
4. **Automatic:** mDNS answers on the new interface with its address, and HTTPS, Modbus TCP and OPC UA are reachable there too.

## Your own server

A TCP server is just a session class. [`TcpServer`](../include/rtos/net/tcp_server.hpp) handles accepting, evicting idle connections, deadlines and cleanup. [modbus_tcp_server.hpp](../include/rtos/net/modbus_tcp_server.hpp) is a compact model:

```cpp
struct MySession {                            // one connection
    bool open(Connection& c, std::uint32_t now) noexcept;              // accepted
    void pump(Connection& c, std::uint32_t now, bool event) noexcept;  // read, answer
    void closed() noexcept;                   // connection gone
    bool finished() const noexcept;           // FIN sent: slot free
    bool evictable() const noexcept;          // idle: may be evicted
    std::uint32_t deadline() const noexcept;  // next deadline (0: none)
};
rtos::net::TcpServer<Stack, MySession, 4> my{stack};
my.listen(5000, rtos::net::InterfaceSet::of({eth}));  // Ethernet only; without a set: all
runtime.add(my, "my");
```

Keep `pump()` short and bounded. Expensive work becomes a background step (`BackgroundSession`), and large responses use `set_bulk()`. UDP services bind with `stack.udp_bind(port, handler)` and are services too ([dhcp.hpp](../include/rtos/net/dhcp.hpp), [mdns.hpp](../include/rtos/net/mdns.hpp)). [tcp_server_test.cpp](../tests/tcp_server_test.cpp) shows how to test a server on the host.

## Ethernet driver (STM32H5)

[ethernet.hpp](../port/stm32h563/include/rtos/port/stm32h5/ethernet.hpp) drives the EQoS MAC with RMII: DMA descriptor rings in the non-cacheable region, receive buffers taken directly from the packet pool (zero copy), MDIO for the PHY link (`poll_link()`), and an interrupt that wakes the network thread.
