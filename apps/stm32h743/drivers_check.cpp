// SPDX-License-Identifier: MIT
//
// Compile check of the STM32H743 chip drivers: instantiates every driver template with a
// 400 MHz clock tree (HSE 25 MHz, the HSI48 on), so the compile-time configuration checks and
// the register code are built even before a board uses them. Not linked into any firmware.
// The chip package is still growing; every driver it gets is instantiated here.
#include <array>
#include <cstddef>
#include <cstdint>

#include "rtos/devices/hx711.hpp"
#include "rtos/devices/ina226.hpp"
#include "rtos/port/cm7/cache.hpp"
#include "rtos/port/stm32h7/clocks.hpp"
#include "rtos/port/stm32h7/dma.hpp"
#include "rtos/port/stm32h7/exti.hpp"
#include "rtos/port/stm32h7/gpio.hpp"
#include "rtos/port/stm32h7/mpu.hpp"
#include "rtos/port/stm32h7/i2c.hpp"
#include "rtos/port/stm32h7/rng.hpp"
#include "rtos/port/stm32h7/sdmmc.hpp"
#include "rtos/port/stm32h7/spi.hpp"
#include "rtos/port/stm32h7/timer.hpp"
#include "rtos/port/stm32h7/usart.hpp"
#include "rtos/storage/sd_card.hpp"

namespace h7 = rtos::port::stm32h7;

namespace {
// HSE 25 MHz / 5 = 5 MHz reference, x 160 = 800 MHz VCO; P / 2 = 400 MHz, Q / 8 = 100 MHz,
// R / 4 = 200 MHz
constexpr h7::rcc::ClockTree clocks{
    .pll1 = {.source = h7::rcc::PllSource::Hse, .input_hz = 25'000'000, .m = 5, .n = 160, .p = 2, .q = 8, .r = 4},
    .hsi48 = true};
static_assert(clocks.cpu_hz() == 400'000'000 && clocks.hclk_hz() == 200'000'000 && clocks.wait_states() == 2 &&
              clocks.pll1_q_hz() == 100'000'000 && clocks.timer1_hz() == 200'000'000);

constexpr std::array pins{
    h7::gpio::output({h7::gpio::GpioPort::E, 1}, /*high*/ true),
    h7::gpio::alternate({h7::gpio::GpioPort::D, 8}, h7::gpio::Signal::USART3_TX),
    h7::gpio::alternate({h7::gpio::GpioPort::D, 9}, h7::gpio::Signal::USART3_RX, h7::gpio::Speed::Low,
                        h7::gpio::Pull::Up),
    h7::gpio::alternate({h7::gpio::GpioPort::B, 14}, h7::gpio::Signal::USART1_TX),
    h7::gpio::analog({h7::gpio::GpioPort::A, 4}, h7::gpio::Signal::ADC1_INP18),
};

h7::BufferedUart<h7::UsartId::Usart1, {.kernel_hz = clocks.pclk2_hz(), .baud = 1'000'000, .half_duplex = true}> servo;
using Console = h7::Usart<h7::UsartId::Usart3>;
alignas(rtos::port::cm7::cache::line_bytes) std::array<std::byte, 64> dma_buffer{};

constexpr std::uint32_t cpu_hz = clocks.cpu_hz();
using I2c4 = h7::I2cMaster<h7::I2cId::I2c4, {.kernel_hz = clocks.pclk4_hz(), .cpu_hz = cpu_hz}>;
struct I2c1Wait {
    static void reset() noexcept {}
    static bool wait(std::uint32_t) noexcept { return true; }
    static void signal() noexcept {}
};
using I2c1 = h7::I2cMaster<h7::I2cId::I2c1, {.kernel_hz = clocks.pclk1_hz(), .cpu_hz = cpu_hz, .bus_hz = 400'000},
                           I2c1Wait>;
using Spi2 = h7::SpiMaster<h7::SpiId::Spi2, {.kernel_hz = clocks.pll1_q_hz(), .max_bus_hz = 1'000'000, .cpu_hz = cpu_hz}>;
using Pwm = h7::Timer<h7::TimerId::Tim4, {.kernel_hz = clocks.timer1_hz(), .update_hz = 1'000, .min_steps = 1000}>;
using Leds = h7::Timer<h7::TimerId::Tim15, {.kernel_hz = clocks.timer2_hz(), .update_hz = 800'000}>;
using Steps = h7::Timer<h7::TimerId::Tim2, {.kernel_hz = clocks.timer1_hz(), .tick_hz = 1'000'000}>;
using PdClock = h7::MicrosecondTimer<h7::TimerId::Tim5, clocks.timer1_hz()>;
using Host = h7::Sdmmc<h7::SdmmcId::Sdmmc1, {.kernel_hz = clocks.pll1_q_hz(), .cpu_hz = cpu_hz}>;
Host sd_host;
rtos::storage::SdCard<Host> card{sd_host};
using Current = rtos::devices::Ina226<I2c4, {.shunt_micro_ohm = 10'000, .max_current_ma = 8'000}>;
bool scale_ready() noexcept { return !h7::gpio::read({h7::gpio::GpioPort::C, 2}); }
using Scale = rtos::devices::Hx711<Spi2, &scale_ready>;
using LedDma = h7::DmaChannel<{.controller = 1, .stream = 2, .request = h7::DmaRequest::Tim15Ch1,
                               .direction = h7::DmaDirection::MemoryToPeripheral, .memory_bytes = 2,
                               .peripheral_bytes = 2}>;
using StepDma = h7::DmaChannel<{.controller = 2, .stream = 5, .request = h7::DmaRequest::Tim2Up,
                                .direction = h7::DmaDirection::MemoryToPeripheral, .memory_bytes = 4,
                                .peripheral_bytes = 4, .priority = 3}>;
RTOS_DMA_BUFFER std::array<std::uint16_t, 24> led_bits{};
using Barrier = h7::ExtiLine<{h7::gpio::GpioPort::B, 10}, h7::ExtiEdge::Falling>;
using Diag = h7::ExtiLine<{h7::gpio::GpioPort::D, 6}, h7::ExtiEdge::Both>;
}  // namespace

void drivers_check() noexcept;
void drivers_check() noexcept {
    h7::configure_mpu();
    rtos::port::cm7::cache::enable_icache();
    rtos::port::cm7::cache::enable_dcache();
    rtos::port::cm7::cache::clean(dma_buffer.data(), dma_buffer.size());
    rtos::port::cm7::cache::invalidate(dma_buffer.data(), dma_buffer.size());
    rtos::port::cm7::cache::clean_invalidate(dma_buffer.data(), dma_buffer.size());

    h7::rcc::configure_clocks<clocks>();
    h7::gpio::configure<pins>();
    h7::gpio::toggle({h7::gpio::GpioPort::E, 1});
    (void)h7::gpio::parse_pin("PD8");
    Console::configure<{.kernel_hz = clocks.pclk1_hz()}>();
    Console::write("x\n");

    servo.start();
    servo.set_baud<115'200>();
    std::array<std::byte, 4> b{};
    (void)servo.write(b);
    (void)servo.read(b);
    servo.on_irq();

    I2c4::init();
    (void)I2c4::read_register(0x40, 0x02, b);
    I2c4::recover();
    I2c1::init();
    (void)I2c1::probe(0x29);
    I2c1::on_event_irq();
    I2c1::on_error_irq();
    if (Current::probe() && Current::init()) {
        (void)Current::read();
    }

    Spi2::init();
    (void)Spi2::transfer(b, b);
    (void)Spi2::start(b);
    (void)Spi2::finish(b);
    (void)Scale::init();
    (void)Scale::poll();

    Pwm::init();
    Pwm::configure_pwm<3>();
    Pwm::set_duty_permille<3>(500);
    Pwm::start();
    Leds::init();
    Leds::configure_pwm<1>();
    Leds::enable_compare_dma<1>();
    Leds::configure_dma_burst(0x38, 1);
    Steps::init();
    Steps::configure_toggle<1>();
    Steps::set_one_pulse(false);
    Steps::generate_update();
    Steps::enable_update_dma();
    PdClock::init();
    PdClock clock;
    clock.arm(clock.now() + 1000);
    (void)PdClock::take();

    h7::Rng::init();
    (void)h7::Rng::next();

    (void)card.init();

    LedDma::init();
    (void)LedDma::start(Leds::compare_address<1>(), led_bits.data(), led_bits.size(), true);
    (void)LedDma::done();
    (void)LedDma::failed();
    LedDma::clear();
    StepDma::init();
    (void)StepDma::remaining();
    StepDma::stop();

    Barrier::init();
    Barrier::enable();
    (void)Barrier::take();
    Diag::init();
    Diag::trigger();
    Diag::disable();
}
