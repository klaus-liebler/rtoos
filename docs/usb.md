# USB

rtoos has its own small USB device stack ([include/rtos/usb](../include/rtos/usb)) instead of USBX or TinyUSB. It runs as a service in the network thread, which the USB interrupt wakes. No class driver has its own thread.

> **Status:** the stack, the classes and the STM32H5 `USB_DRD_FS` driver are complete and tested on the host. Neither QEMU nor Renode models this controller, so the driver is tested against a simulation of its registers (below). It has not yet run on the board.

## STM32H5 controller: `UsbDrdFs`

[usb_device.hpp](../port/stm32h563/include/rtos/port/stm32h5/usb_device.hpp) is the `DeviceController` for the STM32H5's USB_DRD_FS:

```cpp
namespace h5 = rtos::port::stm32h5;
h5::UsbHardware::enable_clocks();   // VDDUSB, HSI48 + CRS (trimmed by the host's SOF), kernel clock
h5::UsbDrdFs<> usb_fs;
usb_fs.start();
rtos::usb::Device<h5::UsbDrdFs<>> device{usb_fs, descriptors, functions};
device.connect();                    // D+ pull-up
extern "C" void USB_DRD_FS_IRQHandler() noexcept { usb_fs.on_irq(); /* wake the network thread */ }
```

- **ISR:** the interrupt handler moves packets between the packet memory and the transfer buffers and re-arms the endpoint. It queues an event only for a SETUP, a completed transfer, a bus reset, suspend or resume.
- **Endpoint registers:** every write follows ST's toggle rules. STAT and DTOG toggle where a 1 is written, and VTRX/VTTX are cleared where a 0 is written. Thread-side changes run with interrupts masked, because the ISR writes the same register.
- **EP0 OUT** stays ready while idle. A data packet that arrives before the stack asks for it is kept and handed over by the next `transfer()`. The driver therefore works whether or not the controller accepts SETUP packets while STAT_RX is NAK. The tests run both behaviours.
- **Register access** goes through a policy type: `UsbHardware` on the target, a register simulation in the host tests.


| Part | File | Contents |
|---|---|---|
| Port | [usb.hpp](../include/rtos/usb/usb.hpp) | `DeviceController` concept: open endpoints, transfers, stall, address. The USB interrupt puts events into a lock-free ring ([spsc_ring.hpp](../include/rtos/sync/spsc_ring.hpp)) and wakes the network thread. |
| Core | [device.hpp](../include/rtos/usb/device.hpp) | `usb::Device`: enumeration, all standard requests of USB 2.0 chapter 9 (direction and data stage checked), control transfer stages with the ZLP rule, SET_ADDRESS after the status stage, dispatch to the functions |
| Descriptors | [descriptors.hpp](../include/rtos/usb/descriptors.hpp) | built at compile time (`constexpr`); total length and interface count are computed |
| CDC-ACM | [cdc_acm.hpp](../include/rtos/usb/cdc_acm.hpp) | virtual serial port: line coding, DTR/RTS, ring buffers with flow control, ZLP after a full last packet |
| CDC-NCM | [cdc_ncm.hpp](../include/rtos/usb/cdc_ncm.hpp) | Ethernet to the PC, and a `NetDriver` for the IP stack: NTB16 with validated receive (including chained NDPs, no loops), bundled transmit with 4-byte alignment (divisor 4, which Windows' usbncm.sys requires since KB5124008), notifications |
| Modbus RTU | [rtu_service.hpp](../include/rtos/modbus/rtu_service.hpp) | Modbus RTU over any byte channel, e.g. CDC-ACM; end of frame by silence |

## Composition

A composite device with a serial port (Modbus RTU) and an Ethernet link:

```cpp
rtos::usb::CdcAcm<> acm;                                    // serial port
rtos::usb::CdcNcm<Pool> ncm{ncm_info, packet_pool, device_mac};
std::array<rtos::usb::Function*, 2> functions{&ncm, &acm};
rtos::usb::Device<UsbController> usb{controller, descriptors, functions};

// in the network thread
const auto usb_if = stack.attach(ncm, {192.168.7.1/24, no gateway});
rtos::net::DhcpServer<Stack> dhcpd{stack, usb_if, {.first = 192.168.7.10, .count = 2}};
dhcpd.start();
rtos::modbus::RtuService<rtos::usb::CdcAcm<>, Stack> rtu{acm, stack, registers, 1, 5};
runtime.add(usb, "usb");      // first: drains the controller's events
runtime.add(dhcpd, "dhcpd");
runtime.add(rtu, "rtu");
```

Once the NCM interface is attached, mDNS answers on it, and HTTPS, Modbus TCP and OPC UA are reachable over USB, all without further code (see [Networking: interfaces](networking.md#interfaces)).

Application threads can read and write the CDC-ACM port directly (`acm.read()`, `acm.write()`, `write_space()`) if they don't use Modbus RTU on it.

## Tests

The tests follow the USBX regression tests: a simulated host ([usb_fixture.hpp](../tests/usb_fixture.hpp)) drives the real stack through a mock controller.

- **Core** ([usb_device_test.cpp](../tests/usb_device_test.cpp)): enumeration, standard requests, ZLP and overrun cases from TinyUSB's `test_usbd.c`.
- **CDC-ACM** ([usb_cdc_test.cpp](../tests/usb_cdc_test.cpp)): line coding, DTR reset on bus reset, flow control, Modbus RTU over the port.
- **CDC-NCM** ([usb_ncm_test.cpp](../tests/usb_ncm_test.cpp)): a table of malformed NTBs, the host's NTB size, and the full path with the IP stack: ARP, ping, a DHCP lease over USB.
- **Fuzzer** ([usb_fuzz_test.cpp](../tests/usb_fuzz_test.cpp)): SETUP packets against an NCM+ACM composite device, and mutated NTBs.
- **Controller** ([usb_drd_fs_test.cpp](../tests/usb_drd_fs_test.cpp)): the driver against a simulated USB_DRD_FS with the hardware's register semantics (toggle and clear-on-0 bits, NAK and toggle after each transaction, packet memory with buffer descriptors), driven by a simulated host at token level. It covers bus reset, SETUP, IN with a zero-length packet, OUT ending with a short packet or a full buffer, data toggles, STALL and CLEAR_FEATURE, early EP0 OUT data, packet memory allocation, and a full enumeration with bulk echo through `usb::Device`.

Bugs these tests found and that are fixed: standard requests with the wrong direction or a forbidden data stage; IN requests with `wLength 0`; NDP loops that delivered datagrams several times.

## USB Power Delivery sink

[include/rtos/usb_pd](../include/rtos/usb_pd) makes the board a USB PD sink: it asks a USB-C power supply for a voltage (fixed or PPS) and keeps the contract. It is ported from Manuel Bleichenbacher's [USB Power Delivery for Arduino](https://github.com/manuelbl/usb-pd-arduino) (MIT License), without heap, `std::function`, busy waits or a global instance.

| Part | File | Contents |
|---|---|---|
| Messages | [message.hpp](../include/rtos/usb_pd/message.hpp) | header fields, data objects, wire bytes |
| Capabilities | [source_capability.hpp](../include/rtos/usb_pd/source_capability.hpp) | parse Source_Capabilities (fixed, variable, battery, SPR PPS), build the Request data object |
| Sink | [sink.hpp](../include/rtos/usb_pd/sink.hpp) | protocol layer (message IDs, GoodCRC, `nRetryCount` retries after `tReceive`) and policy: request the wanted voltage, fall back to 5 V, ask with Get_Source_Cap and finally a Hard Reset if the source stays silent, answer Wait, Soft Reset and unsupported requests, refresh PPS contracts every 8 s |
| PHY | [ucpd.hpp](../port/stm32h563/include/rtos/port/stm32h5/ucpd.hpp) | UCPD1 with GPDMA (RX channel 0, TX channel 1); dividers computed and checked at compile time from the kernel clock (HSI / 4 = 16 MHz) |
| Deadlines | `MicrosecondTimer` in [timer.hpp](../port/stm32h563/include/rtos/port/stm32h5/timer.hpp) | free-running 1 MHz 32-bit timer (TIM2 or TIM5) with a compare interrupt |

```cpp
namespace h5 = rtos::port::stm32h5;
using PdClock = h5::MicrosecondTimer<h5::TimerId::Tim5, board::clocks.timer1_hz()>;
PdClock pd_clock;
RTOS_DMA_BUFFER h5::Ucpd<{.kernel_hz = board::clocks.ucpd_hz()}> ucpd;   // DMA buffers inside
rtos::usb_pd::Sink<decltype(ucpd), PdClock, Kernel::Lock> sink{ucpd, pd_clock};

extern "C" void UCPD1_IRQHandler() noexcept { ucpd.on_irq(sink); }
extern "C" void TIM5_IRQHandler() noexcept { if (PdClock::take()) sink.on_timer(); }

PdClock::init();
sink.request_power(12'000);      // remembered until a source offers it
ucpd.start();                    // Rd on CC1/CC2; then enable both IRQs at the same priority
// in a thread:
sink.poll([](rtos::usb_pd::SinkEvent e) { /* capabilities, voltage changed, rejected */ });
const auto s = sink.status();    // attached, connected, active_mv, active_ma, capabilities
```

The UCPD and timer interrupts must have the same priority: the protocol runs in both and they must not preempt each other. Application calls run under the `Guard` (`Kernel::Lock` masks kernel-aware interrupts). The CC pins (PB13, PB14) go into the board's pin list as analog pins.

**Tests** ([usb_pd_test.cpp](../tests/usb_pd_test.cpp)): the sink against a simulated source that plays its protocol layer and policy: negotiation, 5 V fallback, retries without GoodCRC, repeated message IDs, Reject, Wait, Get_Source_Cap with Hard Reset escalation, a request deferred while the transmitter is busy, PPS refresh, Soft Reset, a lost GoodCRC, detach. The UCPD dividers are checked in [port_config_test.cpp](../tests/port_config_test.cpp). Neither emulator models the UCPD, so the PHY has not run outside the board yet.
