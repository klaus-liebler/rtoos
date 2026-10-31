# Your own project and board

This guide shows how a product uses rtoos as its base: as a CMake subproject, with its own board package, its own selection of services and its own certificates. The [`fcu_net` example](examples.md#fcu_net-a-complete-network-product) is the template for the firmware itself.

## 1. Project layout

```
my_product/
  CMakeLists.txt
  CMakePresets.json
  rtoos/                  git submodule (or FetchContent)
  board/                  your board package (only if it is not the FCU)
    CMakeLists.txt
    include/board/my_board.hpp
    src/my_board.cpp
  src/
    main.cpp              kernel, threads, network thread, services
    app/...               your application
  web/                    optional: web app for the HTTPS server
```

## 2. CMake

rtoos detects that it is a subproject and then builds only its libraries: no example firmware, no tests (`RTOS_BUILD_EXAMPLES` defaults to `OFF`).

```cmake
cmake_minimum_required(VERSION 3.28)
project(my_product LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(rtoos)        # rtos_kernel, rtos_tls, rtos_opcua, chip and FCU board packages
add_subdirectory(board)        # rtos_board_my_board (skip this on the FCU: use rtos_board_fcu)

rtos_add_firmware(my_product
    SOURCES src/main.cpp src/app/control.cpp
    LIBRARIES rtos_board_my_board rtos_tls rtos_opcua my_certs)
```

The targets you link:

| Target | Brings in |
|---|---|
| `rtos_kernel` | the headers (kernel, net, modbus, opcua, usb) and the contract flags `-fcontracts -fcontract-evaluation-semantic=…`. Every other rtoos target depends on it. |
| `rtos_stm32h563` | chip drivers and the linker script; pulls in the CPU port `rtos_port_cm33` |
| `rtos_board_fcu` | the FCU board package (only when you build for the FCU) |
| `rtos_tls` | TLS, HTTPS, WebSocket (wolfSSL glue, static pool tracking) |
| `rtos_opcua` | the OPC UA server |
| `rtos_wolfssl` | wolfSSL/wolfCrypt (fetched by CMake, configured by `third_party/wolfssl/user_settings.h`) |

`rtos_add_firmware(<name> SOURCES … LIBRARIES … [FLASH_DATA_START <address>])` creates `<name>.elf` with the chip's startup code and linker script, plus `.bin` and `.map`. `FLASH_DATA_START` keeps the flash from that address to the end free for data such as a littlefs volume; the link fails if the image grows into it.

### Toolchain and presets

Use the rtoos toolchain files and copy the structure of [CMakePresets.json](../CMakePresets.json):

```json
{
  "version": 6,
  "configurePresets": [
    { "name": "gcc16", "hidden": true, "generator": "Ninja",
      "cacheVariables": { "RTOS_GCC_ROOT": "C:/opt/msys64/ucrt64" },
      "environment": { "PATH": "C:/opt/msys64/ucrt64/bin;$penv{PATH}" } },
    { "name": "board", "inherits": "gcc16", "binaryDir": "${sourceDir}/build/board",
      "toolchainFile": "${sourceDir}/rtoos/cmake/arm-none-eabi.cmake",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "MinSizeRel", "RTOS_CONTRACT_SEMANTIC": "quick_enforce" } }
  ],
  "buildPresets": [ { "name": "board", "configurePreset": "board" } ]
}
```

The toolchain sets the Cortex-M33 flags, `-fno-exceptions -fno-rtti` and `nano.specs`. rtoos stops the configure step for any compiler other than GCC 16 or newer. Always build through a preset (`cmake --build --preset board`): the preset puts the GCC DLLs on the PATH.

Set `RTOS_CONTRACT_SEMANTIC` to `quick_enforce` on the target. A violated precondition then traps at once, and no library code for reporting is linked.

## 3. Board package

A board package is a small static library that knows the pins, the clock tree and the external chips. Copy [boards/fcu](../boards/fcu) and adapt it:

```cmake
# board/CMakeLists.txt
add_library(rtos_board_my_board STATIC src/my_board.cpp)
target_include_directories(rtos_board_my_board PUBLIC include)
target_link_libraries(rtos_board_my_board PUBLIC rtos_stm32h563)
```

The interface that the examples and the self-test use ([boards/fcu/include/board/fcu.hpp](../boards/fcu/include/board/fcu.hpp)):

| Function | Does |
|---|---|
| `board::init()` | clock tree in the order RM0481 requires (voltage scale → flash wait states → PLL), MPU (flash, RAM, non-cacheable DMA region), ICACHE, LED, console UART. First call in `main()`. |
| `board::init_ethernet()` | RMII pins and the PHY reset pulse, before `Ethernet::init()` |
| `board::mac_address()` | a unique, locally administered MAC from the 96-bit device UID |
| `board::clocks` | the clock tree (`rcc::ClockTree`): PLL1, prescalers; every frequency (`pclk1_hz()`, `timer1_hz()`, `pll1_q_hz()` …) as a compile-time constant |
| `board::phy_address`, `board::sysclk_hz` | constants for the drivers |
| `board::print…()`, `board::flush()` | polled console output, safe in fault handlers |

Take every hardware fact (addresses, bits, alternate functions, IRQ numbers) from the STM32H563 SVD, RM0481 and the datasheet, never from memory. `python tools/svd.py STM32H563.svd <PERIPHERAL> [REGISTER] [FIELD]` prints them.

Pins are named by their peripheral signal, not by an AF number: `gpio::alternate({GpioPort::D, 8}, Signal::USART3_TX)`, `gpio::analog({GpioPort::A, 4}, Signal::ADC1_INP18)`. The AF number comes from ST's pin database ([pin_functions.hpp](../port/stm32h563/include/rtos/port/stm32h5/pin_functions.hpp), generated by `tools/gen_pin_functions.py` from the STM32CubeMX database). A pin without that function, or a pin the LQFP100 package lacks, does not compile.

Peripheral configurations are compile-time values. The compiler computes the register values and rejects impossible ones:

```cpp
constexpr h5::UartConfig tmc_bus{.kernel_hz = board::clocks.pclk1_hz(), .baud = 115'200, .half_duplex = true};
h5::BufferedUart<h5::UsartId::Uart5, tmc_bus> tmc_uart;          // BRR computed and range-checked
using Sensors = h5::I2cMaster<h5::I2cId::I2c4, {.kernel_hz = board::clocks.pclk3_hz(),
                                               .cpu_hz = board::sysclk_hz, .bus_hz = 400'000}>;   // TIMINGR
```

A baud rate the kernel clock cannot produce within 2 %, or an I2C timing outside the bus specification, stops the build with a `static_assert` message.

Bring the board up in this order:

1. `fcu_demo` compiled against your board package: the console prints and the LED blinks.
2. `fcu_selftest`: `SELFTEST RESULT: PASS` proves the CPU port, SysTick and interrupt priorities.
3. The network firmware.

A different STM32H5 part with the same peripherals needs only its own linker script. A different MCU needs its own chip package under `port/` that provides the same drivers, and a CPU port that satisfies `rtos::CpuPort`.

## 4. The firmware: putting the services together

Start from [apps/net/main.cpp](../apps/net/main.cpp) and delete what you don't need. The fixed pattern:

```cpp
// kernel
struct KernelConfig : rtos::DefaultConfig { static constexpr std::size_t max_threads = 6; };
using Kernel = rtos::Kernel<rtos::port::cm33::Cm33Port, KernelConfig>;
Kernel kernel;
RTOS_CM33_BIND_KERNEL(kernel)

// network: pool, driver, stack, services (all static)
RTOS_DMA_BUFFER Pool packet_pool;
RTOS_DMA_BUFFER Ethernet ethernet{packet_pool, board::mac_address(), board::phy_address};
Stack stack{packet_pool};
net::NetRuntime<Stack> runtime{stack};
// ... DhcpClient, MdnsResponder, HttpsServer, ModbusTcpServer, OpcUaServer ...

// the one network thread: attach, init, start, runtime.add(), loop
void net_main(void*);

// driver interrupts wake the network thread
extern "C" void ETH_IRQHandler() noexcept {
    if (ethernet.irq()) kernel.notify_from_isr(net_tcb, slot_wake, 1, rtos::NotifyAction::SetBits);
}

int main() {
    board::init();
    Port::configure(board::sysclk_hz, 1000);
    kernel.create_thread(net_tcb, {.name = "net", .entry = &net_main, .stack = net_stack, .priority = 8});
    kernel.create_thread(control_tcb, {.name = "control", .entry = &control_main, .stack = control_stack, .priority = 4});
    kernel.start();
}
```

Choosing the pieces:

| You want | Add | Details |
|---|---|---|
| an address | `DhcpClient` (+ optional static fallback via `stack.claim_address`) | [networking](networking.md) |
| a name in the LAN | `MdnsResponder{stack, "my-device"}` | |
| wall-clock time | `SntpClient`, `DnsResolver` | |
| a web UI | `HttpsServer` with `HttpResource`s from `#embed`, WebSocket handler | [networking](networking.md#https-and-websocket) |
| Modbus TCP | `RegisterBank` + `ModbusTcpServer` | [modbus](modbus.md) |
| OPC UA | `constexpr` address space + `OpcUaServer` | [opcua](opcua.md) |
| a USB serial port, Modbus RTU over USB, Ethernet over USB | `usb::Device` with `CdcAcm`, `RtuService`, `CdcNcm` + `DhcpServer` | [usb](usb.md) |
| your own TCP protocol | a session class for `TcpServer` | [networking](networking.md#your-own-server) |
| periodic work next to the services | a struct with `service()` / `next_deadline()` | `AppTasks` in `fcu_net` |

### Where application code runs

- **Control loops, drivers, anything that blocks or computes for long:** own threads with a higher priority than the network thread (a lower number). They must not call network objects directly.
- **Data shared with the protocols:** the `RegisterBank` with `Kernel::Lock` (one short lock per request), a `MessageQueue`, or `Kernel::notify` to wake the network thread.
- **Short work that touches network objects** (broadcast a WebSocket status, react to a link change): a service in the network thread.

## 5. Certificates

HTTPS needs a server certificate and key. OPC UA needs an RSA-2048 application certificate whose SubjectAltName carries the ApplicationUri, plus the CA certificate that client certificates must chain to. The firmware embeds DER files: `server.der`, `server.key.der`, `opcua.der`, `opcua.key.der`, `ca.der`.

There are two ways:

- **Your product's own mechanism** (recommended for products): generate or provision the files with your build tool, put them in a directory and give the firmware target `--embed-dir=<that directory>`. rtoos needs nothing else.
- **The rtoos default:** `rtos_certificates()` runs [tools/make_certs.py](../tools/make_certs.py) at configure time. It checks the certificates strictly against the CA and issues them again only when they are missing, about to expire, or name another host, IP or ApplicationUri:

  ```cmake
  rtos_certificates(my_certs DIR ${CMAKE_SOURCE_DIR}/certs
                    HOST my-device.local IP 192.168.1.50 OPCUA_URI urn:my-company:my-device)
  ```

  The CA comes from `RTOS_CA_CERT` / `RTOS_CA_KEY` (cache or environment). Without them, a local test CA is created in the directory. The rtoos examples use this way.

Keep certificate directories out of version control: the development keys are not encrypted. See [Security](security.md).

## 6. Sizing memory

All limits are compile-time constants. Size them from the numbers the firmware prints rather than by guesswork:

| What | Where | Measured by |
|---|---|---|
| packet buffers | `PacketPool<N>` | `static_assert` budget: RX ring + receive reserve + one segment per TCP connection |
| TCP connections, listeners, UDP sockets | `NetConfig` | count your servers and clients |
| wolfSSL pool | bucket sizes and counts in `TlsServerContext::init` | `tls: pool peak …` every 10 s (`tls_memory.hpp`), the failure counter must stay 0 under full load |
| HTTPS sessions, OPC UA sessions, Modbus clients | template arguments | |
| thread stacks | stack arrays | frames along the deepest path (`arm-none-eabi-objdump -d`, `sub sp`); RSA-2048 needs ~13.5 KB |
| subscriptions | `rtos/opcua/subscriptions.hpp` limits | |

## 7. Testing your product

- **Host tests:** link your application logic against `rtos_kernel` and build it with the host toolchain ([cmake/gcc-host.cmake](../cmake/gcc-host.cmake)). The rtoos fixtures in `tests/` (`net_fixture.hpp`, `usb_fixture.hpp`, `opcua_service_harness.hpp`) show how to drive the stack without hardware.
- **Emulation:** if your board is close to the FCU, add your peripherals to the Renode platform ([emulation/renode](../emulation/renode)) and reuse `emulation/run.py` and `emulation/net_test.py`.
- **Board:** run the self-test first, then your firmware.
