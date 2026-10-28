# Examples

The `apps/` directory holds four firmware images for the Factory Control Unit. They build with the `arm` preset (for the emulators) and the `board` preset (for the real board):

| Firmware | Source | Shows |
|---|---|---|
| `fcu_demo` | [apps/demo/main.cpp](../apps/demo/main.cpp) | the smallest complete program: board init, kernel, three threads, notifications, preemption threshold, DMA buffer, block pool |
| `fcu_selftest` | [apps/selftest/main.cpp](../apps/selftest/main.cpp) | the kernel on real hardware mechanisms: context switches, FPU, interrupts of both classes, stack limit |
| `fcu_net` | [apps/net/main.cpp](../apps/net/main.cpp) | a complete network product: Ethernet, DHCP, DNS, mDNS, SNTP, HTTPS with a web app and WebSocket, Modbus TCP, OPC UA, all in one network thread |
| `fcu_bench` | [apps/bench/main.cpp](../apps/bench/main.cpp) | a measurement firmware (cost of copies in the TLS record path) |
| `fcu_stepper` | [apps/stepper/main.cpp](../apps/stepper/main.cpp) | a stepper axis with a compile-time sigmoid ramp on a timer interrupt ([Motion](motion.md)) |

Each one is a single `main.cpp` built with one line in [apps/CMakeLists.txt](../apps/CMakeLists.txt):

```cmake
rtos_add_firmware(fcu_demo SOURCES demo/main.cpp LIBRARIES rtos_board_fcu)
rtos_add_firmware(fcu_net  SOURCES net/main.cpp  LIBRARIES rtos_board_fcu rtos_tls rtos_opcua rtos_certs)
```

`rtos_add_firmware` links the chip's startup code and linker script and writes `<name>.elf`, `<name>.bin` and `<name>.map`.

Run them in an emulator:

```
python emulation/run.py renode build/arm/apps/fcu_demo.elf --interactive
python emulation/run.py qemu   build/arm/apps/fcu_selftest.elf
python emulation/net_test.py --browser build/arm/apps/fcu_net.elf     # Renode on the TAP network
```

## `fcu_demo`: the skeleton of every rtoos program

Every rtoos firmware has the same skeleton. The demo shows it with nothing else around it.

**1. Kernel configuration and type.** The kernel is a template over the CPU port and a configuration type. The `KernelConfig` concept checks the configuration at compile time.

```cpp
struct AppConfig : rtos::DefaultConfig {
    static constexpr std::size_t max_threads = 4;   // application threads (idle is extra)
    static constexpr std::size_t notify_slots = 2;  // notification slots per thread
};
using Port = rtos::port::cm33::Cm33Port;
using Kernel = rtos::Kernel<Port, AppConfig>;
```

**2. One kernel object, bound to the port.** `RTOS_CM33_BIND_KERNEL` connects the PendSV, SVC and SysTick handlers of the port to this kernel object.

```cpp
Kernel kernel;
RTOS_CM33_BIND_KERNEL(kernel)
```

**3. Static threads.** A thread is a control block plus a stack array. Priority 0 is the most urgent.

```cpp
Kernel::Thread sampler_tcb, worker_tcb, logger_tcb;
alignas(8) std::array<std::byte, 1024> sampler_stack, worker_stack, logger_stack;
```

**4. `main()`:** board init, SysTick, threads, start.

```cpp
int main() {
    board::init();                                   // clock tree, MPU, caches, LED, console
    Port::configure(board::sysclk_hz, 1000);         // 1 ms tick
    kernel.create_thread(sampler_tcb, {.name = "sampler", .entry = &sampler, .stack = sampler_stack, .priority = 5});
    kernel.create_thread(worker_tcb,  {.name = "worker",  .entry = &worker,  .stack = worker_stack,  .priority = 10});
    kernel.create_thread(logger_tcb,  {.name = "logger",  .entry = &logger,  .stack = logger_stack,
                                       .priority = 20, .preemption_threshold = 10});
    kernel.start();                                  // never returns
}
```

What the threads demonstrate:

- **Task notifications** instead of semaphores for one-to-one signalling: `kernel.notify(worker_tcb, 0, bit, SetBits)` in the sampler, `kernel.notify_wait(0, 100)` with a timeout in the worker. The same call works from an interrupt (`notify_from_isr`), e.g. a DMA-complete interrupt.
- **Preemption threshold instead of a mutex:** the logger (priority 20) runs with threshold 10, the worker's priority. The two share `processed` and never preempt each other, so the data needs no lock. Higher-priority threads (the sampler) still preempt both.
- **DMA memory:** `RTOS_DMA_BUFFER rtos::port::stm32h5::DmaBuffer<std::uint16_t, 64> adc_samples;` lies in the non-cacheable region, cache-line aligned and padded.
- **Deterministic allocation:** `rtos::BlockPool<32, 8>` replaces the heap: 8 blocks of 32 bytes.

## `fcu_selftest`: the kernel on the real mechanisms

The host tests run the kernel on a mock port. The self-test checks what a mock cannot: real PendSV context switches, lazy FPU stacking, interrupts above and below the kernel's BASEPRI mask, SysTick timing, the PSPLIM stack limit and the DWT cycle counter. It runs on the board and in both emulators and ends with exactly one line:

```
SELFTEST RESULT: PASS (n/n)
SELFTEST RESULT: PASS (n/n, 1 unsupported)       # Renode: no PSPLIM enforcement
```

Cases: priority preemption, preemption threshold, FPU context, ISR notify, sleep, timeout, round robin, semaphore from ISR, queue from ISR, pool free from a zero-latency ISR, stack limit, cycle counter. Fault handlers print the fault and end the test. When you port rtoos to a new board, this firmware is the first thing to bring up: if it passes, the CPU port and clock tree are right.

## `fcu_net`: a complete network product

`fcu_net` is the reference for a real product. It serves a web app over HTTPS with a live WebSocket connection, a Modbus TCP server and an OPC UA server. All three see the same data: one LED and a few counters.

| Port | Service |
|---|---|
| 68/UDP | DHCP client (host name `fcu`), with an optional static fallback address |
| 5353/UDP | mDNS: `fcu.local` |
| 443/TCP | HTTPS: `/` web app (brotli), `/blob` 100 KB throughput test, `/ws` WebSocket |
| 502/TCP | Modbus TCP: holding register 0 = LED, input registers 0..5 = uptime and counters |
| 4840/TCP | OPC UA: `Objects/FCU` with `Led`, `Uptime` and counters, bound to the same registers |
| SNTP client | server from DHCP option 42, otherwise `RTOS_NET_SNTP_HOST` via DNS |

The file is long because it also prints diagnostics. Its structure is what matters:

### 1. Static objects: the whole system at a glance

```cpp
using Pool = net::PacketPool<32>;                         // 32 packet buffers
using Ethernet = h5::Ethernet<Pool, 8>;                   // ETH driver with 8 RX descriptors
struct NetConfig : net::DefaultStackConfig {              // stack limits
    static constexpr std::size_t udp_sockets = 6;
    static constexpr std::size_t tcp_connections = 10;
    static constexpr std::size_t tcp_listeners = 3;       // HTTPS, Modbus TCP, OPC UA
    static constexpr std::size_t tcp_bulk_connections = 2;
};
using Stack = net::Stack<Pool, NetConfig>;

RTOS_DMA_BUFFER Pool packet_pool;                                         // DMA memory
RTOS_DMA_BUFFER Ethernet ethernet{packet_pool, board::mac_address(), board::phy_address};
Stack stack{packet_pool};
net::MdnsResponder<Stack> mdns{stack, "fcu"};
net::SntpClient<Stack> sntp{stack, net::Ipv4Address::any(), sntp_port, 60'000};
net::DnsResolver<Stack> dns{stack};
net::DhcpClient<Stack> dhcp{stack, eth_if, "fcu", dhcp_handler};
```

A `static_assert` checks the packet budget: the pool must hold the RX ring, the receive reserve and one segment per TCP connection.

### 2. TLS and HTTPS

```cpp
constexpr std::array<std::uint32_t, 7> tls_bucket_sizes{64, 128, 256, 512, 1024, 2176, 5632};
constexpr std::array<std::uint32_t, 7> tls_bucket_counts{64, 24, 36, 8, 6, 10, 3};
alignas(16) std::array<std::byte, 72 * 1024> tls_memory;  // wolfSSL pool for TLS and OPC UA
net::TlsServerContext tls_context;

const std::array<net::HttpResource, 2> resources{{
    {"/", "text/html; charset=utf-8", "br", as_span(index_html_br)},
    {"/blob", "application/octet-stream", "", as_span(test_blob)},
}};
Server server{stack, tls_context, resources, "/ws", ws_handler, https_config};
```

The web app and the certificates are compiled in with C++26 `#embed`:

```cpp
constexpr unsigned char index_html_br[] = {
#embed "index.html.br"
};
constexpr unsigned char server_certificate[] = {
#embed "server.der"
};
```

CMake brotli-compresses `web/index.html` and adds the output directory with `--embed-dir`. The certificate directory comes from the `rtos_certs` target (see [Security](security.md)).

The WebSocket handler is three plain function pointers (`opened`, `message`, `closed`). `server.send(id, data)` answers one client, and `server.broadcast(data)` sends to all of them. The protocol is binary, and the first byte is the type:

| Type | Direction | Content |
|---|---|---|
| `0x01` | device → browser | status every 100 ms (47 bytes, LE): uptime, a sample, LED, clients, frame counters, Unix time, TLS and echo cycle counts |
| `0x10` | browser → device | set the LED (1 byte: 0/1) |
| `0x20` / `0x21` | browser → device / back | echo, up to 511 bytes |

### 3. Modbus TCP and OPC UA on shared registers

```cpp
using Registers = rtos::modbus::RegisterBank<8, 8, Kernel::Lock>;   // 8 holding, 8 input
Registers registers{rtos::modbus::WriteHook{nullptr, &on_registers_written}};
net::ModbusTcpServer<Stack, 2> modbus{stack, registers};
```

The OPC UA address space is a `constexpr` table in flash. Each application variable is bound to a register:

```cpp
constexpr ua::registers::Register ua_led{Table::Holding, reg_led};
constexpr std::array ua_app_nodes = {
    ua::Node{.nodeId = ua_fcu, .nodeClass = ua::NodeClass::Object, ...},
    ua_variable(5000, "Led", ua::ns0::DataType_UInt16, ua_led, /*writable*/ true),
    ...
};
constexpr auto ua_nodes = ua::JoinNodes(ua::kBaseNodes, ua_app_nodes);
constexpr auto ua_references = ua::JoinReferences(ua::kBaseReferences, ua_app_references);
static_assert(ua::AllReferencesResolve(ua_nodes, ua_references));   // checked at compile time
constexpr ua::AddressSpace ua_space(ua_nodes, ua_references);
net::OpcUaServer<Stack, 2> opcua{stack, ua_space, "opc.tcp://fcu.local:4840"};
```

A write from Modbus or from OPC UA ends in the same `RegisterBank`, and its write hook switches the LED. The WebSocket command also writes the register. Every path sees the same value.

### 4. Application work inside the network thread

`AppTasks` is a service like DHCP or HTTPS: a struct with `service()` and `next_deadline()`. It polls the PHY link, arms the static fallback, refreshes the input registers and broadcasts the WebSocket status every 100 ms. This is the right place for short periodic work that touches network objects. Long-running work belongs in its own thread, which then exchanges data through the register bank or a message queue.

### 5. The network thread

```cpp
void net_main(void*) {
    board::init_ethernet();
    ethernet.init(board::sysclk_hz);
    h5::Rng::init();
    stack.attach(ethernet);                                     // interface 0
    stack.set_random([](void*) noexcept { return h5::Rng::next(); }, nullptr);
    cm33::nvic::ipr(eth_irq) = 6u << (8 - RTOS_CM33_PRIO_BITS); // kernel-aware priority
    cm33::nvic::enable(eth_irq);

    net::TlsServerContext::initialize_library(&rng_fill, unix_seconds, cycle_counter);
    tls_context.init(tls_memory, {cert, key}, max_https_sessions, {tls_bucket_sizes, tls_bucket_counts});
    ua::SetClock(unix_ms_from_sntp, monotonic_ms_64);
    opcua.init({opcua_cert, opcua_key, ca_cert}, {});           // shares the TLS pool
    ua::registers::Bind(registers);

    mdns.start(net::InterfaceSet::all(), &on_mdns_renamed);
    dhcp.start();
    sntp.start();
    server.start(443);
    modbus.start();
    opcua.start();
    runtime.add(app_tasks, "app");  runtime.add(dhcp, "dhcp");  runtime.add(dns, "dns");
    runtime.add(mdns, "mdns");      runtime.add(sntp, "sntp");  runtime.add(server, "https");
    runtime.add(modbus, "modbus");  runtime.add(opcua, "opcua");

    for (;;) {
        const std::uint32_t wake = runtime.run_once(kernel.now());
        (void)kernel.notify_wait(slot_wake, runtime.sleep_ms(wake, kernel.now()));
    }
}

extern "C" void ETH_IRQHandler() noexcept {
    if (ethernet.irq()) {
        kernel.notify_from_isr(net_tcb, slot_wake, 1, rtos::NotifyAction::SetBits);
    }
}
```

Adding a service means constructing it, starting it and calling `runtime.add()` once more. The loop never changes.

### 6. `main()` and the stack size

```cpp
alignas(8) std::array<std::byte, 20 * 1024> net_stack;   // sized from the deepest frames
int main() {
    board::init();
    cm33::CycleCounter::enable();
    Port::configure(board::sysclk_hz, 1000);
    kernel.create_thread(net_tcb, {.name = "net", .entry = &net_main, .stack = net_stack, .priority = 8});
    kernel.start();
}
```

The network thread needs 20 KB because an OPC UA OpenSecureChannel step runs RSA-2048 with the private key (`sp_2048_mod_exp_32` alone has an 8.6 KB frame). Size your own network thread the same way: follow the deepest call path through the frames, then add margin.

### Build-time configuration

[apps/CMakeLists.txt](../apps/CMakeLists.txt) turns cache variables into the constexpr header `fcu_net_config.hpp`, and invalid values stop the configure step:

| Variable | `arm` preset | `board` preset |
|---|---|---|
| `RTOS_NET_SNTP_HOST` | `pool.ntp.org` | `pool.ntp.org` |
| `RTOS_NET_SNTP_PORT` | 12300 (the test script's server; Windows owns port 123) | 123 |
| `RTOS_NET_STATIC_FALLBACK` | `192.168.77.2/24,192.168.77.1` | empty: DHCP keeps trying |
| `RTOS_NET_WS_PING_MS`, `RTOS_NET_WS_PONG_TIMEOUT_MS` | 4000, 3000 | 20000, 10000 |

Values that only exist for emulation belong in such cache variables, never as constants in the firmware.

### Console output

Every 10 s `fcu_net` prints its own measurements: the longest `service()` call and the longest background step, the wolfSSL pool high-water marks per bucket, and the network stack use. Use these lines to size your own product.

## `fcu_bench`: measuring before deciding

The bench firmware produces the same ChaCha20-Poly1305 record with and without the copies of the wolfSSL record path and prints the cycle counts. It is the basis of the decision against an own zero-copy record layer ([decisions](decisions.md#zero-copy-tls-record-layer-measured-and-rejected)).

```
python emulation/run.py qemu build/arm/apps/fcu_bench.elf --timeout 30
```

## USB examples

The USB stack has no example firmware yet, because the register driver for the STM32H5 `USB_DRD_FS` controller is still missing (neither emulator models the controller). Until then, the host tests show the complete composition: [tests/usb_ncm_test.cpp](../tests/usb_ncm_test.cpp) runs CDC-NCM as a second network interface with a DHCP server, and [tests/usb_cdc_test.cpp](../tests/usb_cdc_test.cpp) runs CDC-ACM with Modbus RTU. See [USB](usb.md).

## Next step

[Your own project and board](integration.md) shows how to build a product on rtoos with the same pieces.
