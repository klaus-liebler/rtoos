# rtoos

**rtoos** is an efficient real-time operating system for the STM32H563 (Cortex-M33), with its own IP stack and its own USB device stack. It is written in C++26. It uses no heap, no exceptions and no RTTI. Every layer is covered by a three-level test pyramid:

1. **host-based unit tests** for all of the logic,
2. **QEMU** for the kernel on the emulated CPU,
3. **Renode** for everything that also involves the CPU's peripherals, such as networking over the emulated Ethernet controller.

Built in:

- **Kernel:** preemptive scheduler with preemption threshold, task notifications, semaphores, message queues, object and block pools, a non-blocking log journal. Interrupts above the kernel mask have zero latency.
- **IP stack:** IPv4, ARP, ICMP, UDP, TCP, DHCP client and server, DNS, mDNS, SNTP. Supports several interfaces and uses zero-copy packet buffers.
- **HTTPS** with TLS 1.3 (wolfSSL), including **WebSocket**
- **Modbus TCP**
- **OPC UA** (binary, opc.tcp): Basic256Sha256 (Sign, SignAndEncrypt), sessions, browse, subscriptions
- **USB device stack** with CDC-ACM (virtual serial port), CDC-NCM (Ethernet over USB) and a **Modbus RTU server** over the CDC serial port

All network services share one thread. Expensive work (TLS handshakes, large downloads, RSA) runs in bounded background steps, so short requests are not kept waiting.

## Test pyramid

| Level | Where | Covers |
|---|---|---|
| 1: host unit tests | GoogleTest on the PC; the kernel runs on a mock CPU port, the IP stack over simulated links, the USB stack against a simulated host | all of the logic: kernel, IP stack, TLS, HTTPS, WebSocket, Modbus, OPC UA, USB classes. Also deterministic mutation fuzzers and test cases ported from lwIP, FreeRTOS-Plus-TCP, TinyUSB, USBX and open62541. Fast, runs in seconds. |
| 2: QEMU | own QEMU machine `fcu` | the kernel on the real CPU mechanisms: PendSV context switches, lazy FPU stacking, interrupt priorities and BASEPRI masking, SysTick, PSPLIM stack limit (on-target self-test) |
| 3: Renode | Renode platform of the FCU, connected to the PC through a TAP adapter | everything that involves the CPU's peripherals: the same self-test, plus an end-to-end network test of the real firmware over the emulated Ethernet controller (DHCP, DNS, mDNS, SNTP, HTTPS, WebSocket, Modbus TCP, OPC UA with asyncua, behaviour under load) |

The target board is the **Factory Control Unit (FCU)**: STM32H563VIT6 with a LAN8742A PHY.

## Quick start

Requirements: GCC 16 (host and arm-none-eabi), CMake ≥ 3.28, Ninja, Python 3. Levels 2 and 3 also need QEMU with the `fcu` machine and Renode (see [Testing](docs/testing.md)).

```
cmake --preset host  && cmake --build --preset host  && ctest --preset host
cmake --preset arm   && cmake --build --preset arm   && ctest --preset arm
cmake --preset board && cmake --build --preset board
```

`arm` builds the firmware for the emulators, and `board` builds it for the real board.

## Documentation

| Document | Contents |
|---|---|
| [Architecture](docs/architecture.md) | layers, threads and interrupts, memory model, source tree |
| [Examples](docs/examples.md) | the apps in `apps/`, explained step by step |
| [Your own project and board](docs/integration.md) | rtoos as the base of a product: CMake, board package, putting the services together |
| [Kernel](docs/kernel.md) | threads, synchronisation, execution contexts, rules |
| [Logging](docs/logging.md) | non-blocking log journal with independent readers (console, network) |
| [Motion](docs/motion.md) | stepper moves with a sigmoid speed ramp computed by the compiler |
| [Networking](docs/networking.md) | IP stack, interfaces, services, HTTPS and WebSocket, fairness |
| [Modbus](docs/modbus.md) | Modbus TCP and RTU, register bank |
| [OPC UA](docs/opcua.md) | server, security, address space, subscriptions, tests ported from open62541 |
| [USB](docs/usb.md) | device stack, CDC-ACM, CDC-NCM, Modbus RTU over USB |
| [Security](docs/security.md) | certificates and CA, TLS, OPC UA trust, ARP hardening |
| [Testing](docs/testing.md) | host tests, fuzzing, QEMU, Renode, network test |
| [Design decisions](docs/decisions.md) | decisions backed by measurements (zero-copy TLS, memory pool) |

Rules for contributors (compiler, language standard, workarounds) are in [CLAUDE.md](CLAUDE.md).

## License

The rtoos sources are marked `SPDX-License-Identifier: MIT`. wolfSSL (TLS and crypto) is licensed under GPLv3 or under a commercial license from wolfSSL Inc.
