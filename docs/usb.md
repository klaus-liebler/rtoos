# USB

rtoos has its own small USB device stack ([include/rtos/usb](../include/rtos/usb)) instead of USBX or TinyUSB. It runs as a service in the network thread, which the USB interrupt wakes. No class driver has its own thread.

> **Status:** the stack, the classes and their tests are complete. The register driver for the STM32H5 `USB_DRD_FS` controller is still missing. Neither QEMU nor Renode models this controller, so the driver has to be verified on the board.

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

Bugs these tests found and that are fixed: standard requests with the wrong direction or a forbidden data stage; IN requests with `wLength 0`; NDP loops that delivered datagrams several times.
