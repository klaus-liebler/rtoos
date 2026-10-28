# Modbus

Modbus is a fixed part of rtoos. One register bank serves Modbus TCP, Modbus RTU (over a UART or USB CDC-ACM) and, if wanted, OPC UA.

| Part | File | Contents |
|---|---|---|
| PDU | [modbus.hpp](../include/rtos/modbus/modbus.hpp) | function codes 03, 04, 06, 16 per Modbus Application Protocol V1.1b3; exceptions in the order of the specification (quantity → 03, address → 02); the `RegisterMap` interface; CRC-16 |
| Registers | [register_bank.hpp](../include/rtos/modbus/register_bank.hpp) | `RegisterBank<Holding, Input, Guard>`: one lock per request block (target: `Kernel::Lock`), write hook for the application |
| RTU framing | [rtu.hpp](../include/rtos/modbus/rtu.hpp) | frame length from the function code, end of frame by silence for unknown codes, broadcast (address 0), CRC check |
| RTU server | [rtu_service.hpp](../include/rtos/modbus/rtu_service.hpp) | `RtuService<Channel, Clock>`: Modbus RTU over any byte channel, e.g. `usb::CdcAcm`; a network-thread service |
| TCP server | [modbus_tcp_server.hpp](../include/rtos/net/modbus_tcp_server.hpp) | `ModbusTcpServer` on `TcpServer`: MBAP checks, requests split across segments and several requests per segment, unit-id filter, idle timeout, several clients, `InterfaceSet` |

## Usage

```cpp
// 64 holding and 32 input registers, shared with application threads
rtos::modbus::RegisterBank<64, 32, Kernel::Lock> registers{{nullptr, &on_written}};

// Modbus TCP: port 502, 2 clients, all interfaces
rtos::net::ModbusTcpServer<Stack, 2> modbus{stack, registers};
modbus.start();
runtime.add(modbus, "modbus");

// Modbus RTU over the USB serial port: the CDC-ACM function is the byte channel,
// the stack is the millisecond clock (now_ms()), unit address 1, 5 ms silence ends a frame
rtos::usb::CdcAcm<> acm;
rtos::modbus::RtuService<rtos::usb::CdcAcm<>, Stack> rtu{acm, stack, registers, 1, 5};
runtime.add(rtu, "rtu");
```

Application threads read and write with `registers.holding(i)`, `set_holding(i, v)` and `set_inputs(first, values)`. The write hook `on_written(context, start, count)` runs after a Modbus (or OPC UA) client has written holding registers, e.g. to switch an output.

Registers can be shared with OPC UA or kept separate. In the Factory Control Unit they are shared: OPC UA variables are bound to registers with `rtos::opcua::registers` (see [OPC UA](opcua.md#address-space)). The tests cover both arrangements.

## Tests

- [modbus_test.cpp](../tests/modbus_test.cpp): the examples of the specification, the exceptions, the RTU framing, and Modbus TCP over the stack.
- [usb_cdc_test.cpp](../tests/usb_cdc_test.cpp): Modbus RTU over CDC-ACM with a simulated USB host.
- The fuzzer covers the PDU, RTU and Modbus TCP in the receive path; the fairness test checks the Modbus reply during downloads.
- Renode: `net_test.py` reads and writes the registers of `fcu_net` with a Modbus TCP client and measures the response time during parallel downloads.
