// SPDX-License-Identifier: MIT
//
// USB check for the Factory Control Unit revision 3 (STM32H743 USB2_OTG_FS): a CDC-ACM device that
// echoes every byte the host sends. emulation/renode/STM32_OTG_FS.cs (Mode "echo") plays the
// host - enumeration, SET_ADDRESS, SET_CONFIGURATION, a two-packet echo - and writes the
// result; on the board any terminal program on the virtual COM port shows the echo.
//
//   usb_test.py build/arm-h743/apps/stm32h743/h743_usb.elf
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "board/chip.hpp"
#include "board/fcu.hpp"
#include "rtos/rtos.hpp"
#include "rtos/usb/cdc_acm.hpp"
#include "rtos/usb/descriptors.hpp"
#include "rtos/usb/device.hpp"

namespace {

struct Config : rtos::DefaultConfig {
    static constexpr std::size_t max_threads = 1;
    static constexpr std::size_t notify_slots = 1;
};
using Kernel = rtos::Kernel<board::Port, Config>;

}  // namespace

Kernel kernel;
RTOS_BOARD_BIND_KERNEL(kernel)

namespace {

namespace usb = rtos::usb;

constexpr usb::CdcAcmInfo acm_info{.interface = 0, .notify_ep = 0x81, .out_ep = 0x02, .in_ep = 0x82, .string = 4};
constexpr auto device_descriptor = usb::device_descriptor({.vendor = 0x1209, .product = 0x0001});
constexpr auto configuration = usb::configuration_descriptor({.max_power_ma = 100}, usb::cdc_acm_function(acm_info));
constexpr std::array<std::string_view, 4> strings{"rtoos", "FCU USB check", "0001", "rtoos echo"};

board::UsbDevice controller;
usb::CdcAcm<> acm{acm_info};
std::array<usb::Function*, 1> functions{&acm};
usb::Device<board::UsbDevice> device{controller, {device_descriptor, configuration, strings}, functions};

Kernel::Thread usb_tcb;
alignas(8) std::array<std::byte, 4096> usb_stack;

void usb_main(void*) {
    board::init_usb();
    controller.start();
    board::nvic::ipr(board::UsbDevice::irq) = static_cast<std::uint8_t>(6u << (8 - board::irq_priority_bits));
    board::nvic::enable(board::UsbDevice::irq);
    device.connect();
    board::print("usb: connected, CDC-ACM echo\n");
    bool configured = false;
    std::array<std::byte, 128> buffer{};
    for (;;) {
        (void)kernel.notify_wait(0, 10);
        device.service();
        if (device.configured() != configured) {
            configured = device.configured();
            board::print(configured ? "usb: configured\n" : "usb: not configured\n");
        }
        if (const std::size_t n = acm.read(buffer); n > 0) {
            (void)acm.write(std::span(buffer).first(n));
            device.service();  // start sending now
        }
    }
}

}  // namespace

extern "C" void RTOS_BOARD_USB_IRQ_HANDLER() noexcept {
    controller.on_irq();
    kernel.notify_from_isr(usb_tcb, 0, 1, rtos::NotifyAction::SetBits);
}

int main() {
    board::init();
    board::print("\nrtoos usb - ");
    board::print(board::mcu_name);
    board::print(" / Factory Control Unit\n");
    board::CycleCounter::enable();
    board::Port::configure(board::sysclk_hz, 1000);
    kernel.create_thread(usb_tcb, {.name = "usb", .entry = &usb_main, .stack = usb_stack, .priority = 8});
    kernel.start();
}
