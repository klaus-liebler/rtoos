// SPDX-License-Identifier: MIT
//
// Compile-time configuration of the STM32H743 chip drivers: the register values the drivers
// write are computed here exactly as on the target (they are constexpr), so their limits can be
// checked against RM0433 and ST's tables on the host. Expected encodings: ST's stm32h743xx.h
// (RCC fields, prescalers), stm32h7xx_hal_rcc.h / stm32h7xx_ll_utils.c (PLL ranges),
// STM32CubeMX db (wait states, pin database), stm32h7xx_hal_flash_ex.h (programming delay).
#include <gtest/gtest.h>

#include "rtos/port/cm7/mpu.hpp"
#include "rtos/port/stm32h7/adc.hpp"
#include "rtos/port/stm32h7/clocks.hpp"
#include "rtos/port/stm32h7/dma.hpp"
#include "rtos/port/stm32h7/ethernet.hpp"
#include "rtos/port/stm32h7/exti.hpp"
#include "rtos/port/stm32h7/fdcan.hpp"
#include "rtos/port/stm32h7/flash_program.hpp"
#include "rtos/port/stm32h7/gpio.hpp"
#include "rtos/port/stm32h7/i2c.hpp"
#include "rtos/port/stm32h7/mpu.hpp"
#include "rtos/port/stm32h7/sdmmc.hpp"
#include "rtos/port/stm32h7/spi.hpp"
#include "rtos/port/stm32h7/timer.hpp"
#include "rtos/port/stm32h7/usart.hpp"

namespace {

namespace h7 = rtos::port::stm32h7;

TEST(H7Rcc, Pll1FromHsiAt400MHz) {
    constexpr h7::rcc::PllConfig pll{.m = 4, .n = 50, .p = 2, .q = 8};
    static_assert(pll.valid());
    EXPECT_EQ(pll.reference_hz(), 16'000'000u);
    EXPECT_EQ(pll.vco_hz(), 800'000'000u);
    EXPECT_EQ(pll.p_hz(), 400'000'000u);
    EXPECT_EQ(pll.q_hz(), 100'000'000u);
    EXPECT_EQ(pll.range(), 3u) << "8-16 MHz reference";
    EXPECT_FALSE(pll.medium_vco());
    // PLLSRC = 00 (HSI), DIVM1 = m (not m - 1: 0 switches the prescaler off)
    EXPECT_EQ(pll.pllckselr(), 4u << 4);
    // PLL1RGE = 11, VCOSEL = 0 (wide), integer mode, DIVP1EN, DIVQ1EN
    EXPECT_EQ(pll.pllcfgr(), (3u << 2) | (1u << 16) | (1u << 17));
    // DIVN1 = n - 1, DIVP1 = p - 1, DIVQ1 = q - 1, DIVR1 = 2 - 1 (R unused)
    EXPECT_EQ(pll.pll1divr(), 49u | (1u << 9) | (7u << 16) | (1u << 24));
}

TEST(H7Rcc, Pll1FromHseAndTheMediumVco) {
    constexpr h7::rcc::PllConfig hse{.source = h7::rcc::PllSource::Hse, .input_hz = 25'000'000, .m = 5, .n = 160,
                                     .p = 2, .q = 8, .r = 4};
    static_assert(hse.valid());
    EXPECT_EQ(hse.range(), 2u) << "5 MHz: 4-8 MHz range";
    EXPECT_EQ(hse.pllckselr(), 2u | (5u << 4)) << "PLLSRC = 10 (HSE)";
    EXPECT_EQ(hse.pllcfgr(), (2u << 2) | (1u << 16) | (1u << 17) | (1u << 18));
    EXPECT_EQ(hse.r_hz(), 200'000'000u);
    // a 1-2 MHz reference needs the medium VCO (ST: IS_LL_UTILS_CHECK_VCO_RANGES)
    constexpr h7::rcc::PllConfig medium{.m = 40, .n = 200, .p = 2};
    static_assert(medium.valid());
    EXPECT_EQ(medium.reference_hz(), 1'600'000u);
    EXPECT_TRUE(medium.medium_vco());
    EXPECT_EQ(medium.pllcfgr() & 0xFu, 1u << 1) << "PLL1RGE = 00, VCOSEL = 1";
}

TEST(H7Rcc, Pll1Limits) {
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 4, .n = 56, .p = 2}.valid())) << "VCO 896 MHz > 836 MHz (wide)";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 4, .n = 11, .p = 2}.valid())) << "VCO 176 MHz < 192 MHz (wide)";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 40, .n = 300, .p = 2}.valid())) << "VCO 480 MHz > 420 MHz (medium)";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 3, .n = 30, .p = 2}.valid())) << "reference 21.3 MHz > 16 MHz";
    EXPECT_TRUE((h7::rcc::PllConfig{.m = 63, .n = 300, .p = 2}.valid())) << "reference 1.02 MHz, medium VCO 305 MHz";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 63, .n = 100, .p = 2}.valid())) << "medium VCO 102 MHz < 150 MHz";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 4, .n = 50, .p = 3}.valid())) << "P even only";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 4, .n = 50, .p = 1}.valid())) << "P >= 2";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 64, .n = 200, .p = 2}.valid())) << "M is 1..63";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 4, .n = 50, .p = 2, .q = 129}.valid())) << "Q is 1..128";
}

TEST(H7Clocks, TreeDerivesFrequenciesAndPrescalers) {
    constexpr h7::rcc::ClockTree t{.pll1 = {.m = 4, .n = 50, .p = 2, .q = 8}};
    EXPECT_EQ(t.cpu_hz(), 400'000'000u);
    EXPECT_EQ(t.hclk_hz(), 200'000'000u);
    EXPECT_EQ(t.pclk1_hz(), 100'000'000u);
    EXPECT_EQ(t.pclk2_hz(), 100'000'000u);
    EXPECT_EQ(t.pclk3_hz(), 100'000'000u);
    EXPECT_EQ(t.pclk4_hz(), 100'000'000u);
    EXPECT_EQ(t.timer1_hz(), 200'000'000u) << "APB prescaler 2: the timers run at HCLK";
    EXPECT_EQ(t.wait_states(), 2u);
    // HPRE 1000 (/2), D1PPRE 100 (/2), D1CPRE 0000 (/1)
    EXPECT_EQ(t.d1cfgr(), 0x8u | (0x4u << 4));
    EXPECT_EQ(t.d2cfgr(), (0x4u << 4) | (0x4u << 8));
    EXPECT_EQ(t.d3cfgr(), 0x4u << 4);
    constexpr h7::rcc::ClockTree slow{.pll1 = {.m = 4, .n = 50, .p = 2}, .ahb_div = 4, .apb1_div = 4,
                                      .apb2_div = 1, .apb3_div = 1, .apb4_div = 16};
    EXPECT_EQ(slow.hclk_hz(), 100'000'000u);
    EXPECT_EQ(slow.d1cfgr(), 0x9u);
    EXPECT_EQ(slow.d2cfgr(), 0x5u << 4);
    EXPECT_EQ(slow.d3cfgr(), 0x7u << 4);
    EXPECT_EQ(slow.timer1_hz(), 50'000'000u) << "2 x PCLK1 (25 MHz)";
    EXPECT_EQ(slow.timer2_hz(), 100'000'000u) << "prescaler 1: PCLK2";
    EXPECT_EQ(slow.wait_states(), 1u);
}

TEST(H7Flash, WaitStatesMatchCubeMx) {
    // STM32CubeMX db IP/RCC-STM32H7_rcc_v1_0_Modes.xml, FLatency, scale1: 70/140/210/225 MHz
    const std::array<std::uint32_t, 4> limits{70'000'000u, 140'000'000u, 210'000'000u, 225'000'000u};
    for (std::uint32_t ws = 0; ws < limits.size(); ++ws) {
        EXPECT_EQ(h7::flash::vos1_wait_states(limits[ws]), ws) << limits[ws];
        EXPECT_EQ(h7::flash::vos1_wait_states(limits[ws] + 1), ws + 1 < limits.size() ? ws + 1 : 0xFFu) << limits[ws];
    }
    // stm32h7xx_hal_flash_ex.h: FLASH_PROGRAMMING_DELAY_0/1/2 up to 70/185/225 MHz
    EXPECT_EQ(h7::flash::programming_delay(70'000'000), 0u);
    EXPECT_EQ(h7::flash::programming_delay(70'000'001), 1u);
    EXPECT_EQ(h7::flash::programming_delay(185'000'001), 2u);
}

TEST(H7Gpio, PinDatabaseOfTheLqfp100) {
    using h7::gpio::GpioPort, h7::gpio::Signal;
    EXPECT_EQ(h7::gpio::package_pins.size(), 82u) << "LQFP100: 82 I/O pins";
    // the FCU revision 3's pins (pcb_factory_control_unit, U17)
    static_assert(h7::gpio::function_af({GpioPort::D, 8}, Signal::USART3_TX) == 7);
    static_assert(h7::gpio::function_af({GpioPort::D, 9}, Signal::USART3_RX) == 7);
    static_assert(h7::gpio::function_af({GpioPort::B, 11}, Signal::ETH_TX_EN) == 11);
    static_assert(h7::gpio::function_af({GpioPort::B, 12}, Signal::ETH_TXD0) == 11);
    static_assert(h7::gpio::function_af({GpioPort::D, 12}, Signal::I2C4_SCL) == 4);
    static_assert(h7::gpio::function_af({GpioPort::A, 8}, Signal::I2C3_SCL) == 4);
    static_assert(h7::gpio::function_af({GpioPort::E, 5}, Signal::TIM15_CH1) == 4);
    static_assert(h7::gpio::function_af({GpioPort::C, 11}, Signal::QUADSPI_BK2_NCS) == 9);
    static_assert(h7::gpio::function_af({GpioPort::C, 15}, Signal::QUADSPI_BK2_NCS) == -1,
                  "PC15 has no QUADSPI function (the revision 3 schematic had QSPI_CS there)");
    static_assert(h7::gpio::function_af({GpioPort::A, 4}, Signal::ADC1_INP18) == h7::gpio::additional);
    EXPECT_EQ(h7::gpio::parse_pin("PE1"), (h7::gpio::Pin{GpioPort::E, 1}));
    EXPECT_FALSE(h7::gpio::parse_pin("PF0")) << "port F is not bonded out in the LQFP100";
}

TEST(H7Gpio, PinListFoldsIntoOneValuePerPort) {
    using h7::gpio::GpioPort, h7::gpio::Signal;
    constexpr std::array pins{
        h7::gpio::output({GpioPort::E, 1}, true),
        h7::gpio::alternate({GpioPort::D, 8}, Signal::USART3_TX),
        h7::gpio::alternate({GpioPort::D, 9}, Signal::USART3_RX, h7::gpio::Speed::Low, h7::gpio::Pull::Up),
    };
    constexpr auto plans = h7::gpio::plan(pins);
    static_assert(plans.size() == 8);
    EXPECT_FALSE(plans[0].used);
    EXPECT_EQ(plans[3].mode, (2u << 16) | (2u << 18));
    EXPECT_EQ(plans[3].afh, 7u | (7u << 4));
    EXPECT_EQ(plans[3].pull, 1u << 18);
    EXPECT_EQ(plans[4].mode, 1u << 2);
    EXPECT_EQ(plans[4].bsrr, 1u << 1);
}

TEST(H7Usart, InstancesAndConsoleBaudRate) {
    EXPECT_EQ(h7::usart_hw(h7::UsartId::Usart3).base, 0x4000'4800u);
    EXPECT_EQ(h7::usart_hw(h7::UsartId::Usart3).irq, 39u);
    EXPECT_EQ(h7::usart_hw(h7::UsartId::Usart1).base, 0x4001'1000u);
    EXPECT_EQ(h7::usart_hw(h7::UsartId::Uart8).irq, 83u);
    // RCC_APB1LENR.USART3EN = bit 18, reset register 0x58 below
    EXPECT_EQ(h7::usart_hw(h7::UsartId::Usart3).gate.enr, 0x5802'44E8u);
    EXPECT_EQ(h7::usart_hw(h7::UsartId::Usart3).gate.rstr, 0x5802'4490u);
    EXPECT_EQ(h7::usart_hw(h7::UsartId::Usart3).gate.bit, 18u);
    EXPECT_EQ(h7::uart_brr(100'000'000, 115'200), 868u) << "100 MHz / 115200, rounded";
}

TEST(H7Timer, DmaRegistersSitAt0x48) {
    // SVD v2.4: TIMx_DCR at +0x048, DMAR at +0x04C (the H5 and G4: +0x3DC/+0x3E0)
    using Leds = h7::Timer<h7::TimerId::Tim15, {.kernel_hz = 200'000'000, .update_hz = 800'000}>;
    EXPECT_EQ(Leds::dmar_address(), 0x4001'4000u + 0x4Cu);
    EXPECT_EQ(Leds::compare_address<1>(), 0x4001'4034u);
    EXPECT_EQ(Leds::period_ticks, 250u) << "200 MHz / 800 kHz";
    using Steps = h7::Timer<h7::TimerId::Tim2, {.kernel_hz = 200'000'000, .tick_hz = 1'000'000}>;
    EXPECT_EQ(Steps::arr_address(), 0x4000'002Cu);
    EXPECT_EQ(Steps::max_period, 0xFFFF'FFFFu) << "TIM2 counts 32 bit";
    EXPECT_EQ(h7::timer_hw(h7::TimerId::Tim15).irq, 116u);
    EXPECT_FALSE(h7::timer_hw(h7::TimerId::Tim15).burst_source) << "no TIMx_DCR.DBSS on the H7";
}

TEST(H7Buses, I2cSpiSdmmcInstances) {
    EXPECT_EQ(h7::i2c_hw(h7::I2cId::I2c4).base, 0x5800'1C00u);
    EXPECT_EQ(h7::i2c_hw(h7::I2cId::I2c4).irq_event, 95u);
    EXPECT_EQ(h7::i2c_hw(h7::I2cId::I2c3).irq_error, 73u);
    EXPECT_TRUE((h7::checked_i2c_timing<{.kernel_hz = 100'000'000, .cpu_hz = 400'000'000}>().valid));
    EXPECT_EQ(h7::spi_hw(h7::SpiId::Spi2).fifo_bytes, 16u) << "SPI1-3: 16-byte FIFO (HX711)";
    EXPECT_EQ(h7::spi_hw(h7::SpiId::Spi4).fifo_bytes, 8u);
    EXPECT_EQ(h7::spi_mbr({.kernel_hz = 100'000'000, .max_bus_hz = 1'000'000}), 6u) << "/128 -> 781 kHz";
    EXPECT_EQ(h7::sdmmc_hw(h7::SdmmcId::Sdmmc1).base, 0x5200'7000u);
    EXPECT_EQ(h7::sdmmc_hw(h7::SdmmcId::Sdmmc1).irq, 49u);
    EXPECT_EQ(h7::sdmmc_clkdiv(100'000'000, 400'000), 125u);
}

TEST(H7Dma, StreamRegistersRequestsAndInterrupts) {
    using h7::DmaDirection, h7::DmaRequest;
    // DIR = 01 (memory to peripheral), MINC, PSIZE = MSIZE = 01 (16 bit), PL = 11
    EXPECT_EQ(h7::dma_cr({.stream = 2, .request = DmaRequest::Tim15Ch1, .direction = DmaDirection::MemoryToPeripheral,
                          .memory_bytes = 2, .peripheral_bytes = 2, .priority = 3}),
              (1u << 6) | (1u << 10) | (1u << 11) | (1u << 13) | (3u << 16));
    EXPECT_EQ(h7::dma_cr({.stream = 0, .request = DmaRequest::Spi2Rx, .direction = DmaDirection::PeripheralToMemory}),
              1u << 10);
    // stm32h743xx.h: DMAMUX1_Channel0_BASE = 0x40020800, Channel8 (DMA2 stream 0) = +0x20
    EXPECT_EQ(h7::dmamux_channel_address(1, 0), 0x4002'0800u);
    EXPECT_EQ(h7::dmamux_channel_address(1, 7), 0x4002'081Cu);
    EXPECT_EQ(h7::dmamux_channel_address(2, 0), 0x4002'0820u);
    EXPECT_EQ(h7::dmamux_channel_address(2, 7), 0x4002'083Cu);
    EXPECT_EQ(h7::dma_irq(1, 0), 11u);
    EXPECT_EQ(h7::dma_irq(1, 6), 17u);
    EXPECT_EQ(h7::dma_irq(1, 7), 47u) << "DMA1_Stream7_IRQn";
    EXPECT_EQ(h7::dma_irq(2, 4), 60u);
    EXPECT_EQ(h7::dma_irq(2, 5), 68u) << "DMA2_Stream5_IRQn";
    EXPECT_EQ(h7::dma_irq(2, 7), 70u);
    // LISR/HISR: streams 0..3 (and 4..7) at bits 0, 6, 16, 22
    EXPECT_EQ(h7::dma_detail::flag_shift(1), 6u);
    EXPECT_EQ(h7::dma_detail::flag_shift(6), 16u);
    // stm32h7xx_ll_dmamux.h
    EXPECT_EQ(static_cast<unsigned>(DmaRequest::Tim2Up), 22u);
    EXPECT_EQ(static_cast<unsigned>(DmaRequest::Tim15Ch1), 105u);
}

TEST(H7Exti, SharedInterruptLines) {
    EXPECT_EQ(h7::exti_detail::irq(0), 6u);
    EXPECT_EQ(h7::exti_detail::irq(4), 10u);
    EXPECT_EQ(h7::exti_detail::irq(5), 23u) << "EXTI9_5";
    EXPECT_EQ(h7::exti_detail::irq(9), 23u);
    EXPECT_EQ(h7::exti_detail::irq(10), 40u) << "EXTI15_10";
    EXPECT_EQ(h7::exti_detail::irq(15), 40u);
    using Barrier = h7::ExtiLine<{h7::gpio::GpioPort::B, 10}, h7::ExtiEdge::Falling>;
    EXPECT_EQ(Barrier::irq, 40u);
}

TEST(H7Adc, PrescalerAndTemperatureFormula) {
    // per_ck = HSI 64 MHz, at most 6.25 MHz: / 12 (PRESC 0110) -> 5.33 MHz
    constexpr h7::AdcConfig c{.cpu_hz = 400'000'000, .channel = 18};
    EXPECT_EQ(h7::adc_presc(c), 6u);
    EXPECT_EQ(h7::adc_clock_hz(c), 5'333'333u);
    EXPECT_EQ(h7::adc_presc({.kernel_hz = 64'000'000, .cpu_hz = 400'000'000, .channel = 0, .max_adc_hz = 64'000'000}), 0u);
    // __LL_ADC_CALC_TEMPERATURE at 3.3 V: 30 degC at TS_CAL1, TS_CAL2's temperature at TS_CAL2
    EXPECT_EQ(h7::temperature_centi_celsius(12'000, 12'000, 16'000, 130), 3000);
    EXPECT_EQ(h7::temperature_centi_celsius(16'000, 12'000, 16'000, 130), 13'000);
    EXPECT_EQ(h7::temperature_centi_celsius(14'000, 12'000, 16'000, 110), 7000) << "revision Y: TS_CAL2 at 110 degC";
    EXPECT_EQ(h7::temperature_centi_celsius(11'000, 12'000, 16'000, 130), 500) << "below 30 degC";
    EXPECT_FALSE(h7::temperature_centi_celsius(12'000, 0, 0, 130)) << "no calibration: no value";
}

TEST(H7Ethernet, InstanceAndMdioClock) {
    EXPECT_EQ(h7::ethernet_hw.base, 0x4002'8000u) << "SVD v2.4 Ethernet_MAC";
    EXPECT_EQ(h7::ethernet_hw.irq, 61u) << "stm32h743xx.h ETH_IRQn";
    // HAL_ETH_SetMDIOClockRange: HCLK 200 MHz -> /102 (MDC 1.96 MHz <= 2.5 MHz)
    EXPECT_EQ(h7::eth_reg::mdio_clock_range(200'000'000), 0x400u);
    EXPECT_EQ(h7::eth_detail::pmcr_epis_rmii, 0x0080'0000u) << "stm32h743xx.h SYSCFG_PMCR_EPIS_SEL_2";
}

TEST(H7Fdcan, MessageRamLayoutAndBitTiming) {
    namespace d = h7::fdcan_detail;
    // RX FIFO 0: 16 elements at byte 0; TX FIFO: 8 elements at byte 256; 16-byte elements (8 data)
    EXPECT_EQ(d::rxf0c, 16u << 16);
    EXPECT_EQ(d::txbc, 0x100u | (8u << 24)) << "TBSA = byte 256, NDTB = 0, TFQS = 8, TFQM = 0 (FIFO)";
    EXPECT_LE(d::rx_fifo0 + d::rx_fifo0_elements * d::element_bytes, d::tx_fifo) << "RX FIFO 0 overlaps the TX FIFO";
    EXPECT_LE(d::used_bytes, 10u * 1024u) << "within the 10 KB message RAM";
    EXPECT_EQ(d::sram, 0x4000'AC00u) << "stm32h743xx.h SRAMCAN_BASE";
    // pll1_q 100 MHz, 500 kbit/s: the most quanta that divide exactly, 25 (prescaler 8), sample
    // point 22/25 = 88.0 % (nearest to 87.5 %)
    using Can = h7::Fdcan<{.kernel_hz = 100'000'000, .cpu_hz = 400'000'000, .bitrate = 500'000}>;
    EXPECT_EQ(Can::bit_timing.quanta(), 25u);
    EXPECT_EQ(Can::bit_timing.prescaler, 8u);
    EXPECT_EQ(Can::bit_timing.sample_permille(), 880u);
    EXPECT_EQ(Can::irq0, 19u);
}

TEST(Cm7Mpu, RegionEncodings) {
    namespace mpu = rtos::port::cm7::mpu;
    using mpu::Memory;
    EXPECT_EQ(mpu::size_field(32), 4u) << "ARM_MPU_REGION_SIZE_32B";
    EXPECT_EQ(mpu::size_field(256 * 1024), 0x11u) << "ARM_MPU_REGION_SIZE_256KB";
    EXPECT_EQ(mpu::size_field(2 * 1024 * 1024), 0x14u);
    // non-cacheable normal (TEX 001, C 0, B 0), shareable, XN, privileged RW, 256 KB
    constexpr mpu::Region dma{.base = 0x3000'0000, .bytes = 256 * 1024, .memory = Memory::NormalNonCacheable,
                              .shareable = true};
    EXPECT_EQ(mpu::rasr(dma), (1u << 28) | (1u << 24) | (1u << 19) | (1u << 18) | (0x11u << 1) | 1u);
    EXPECT_EQ(mpu::rbar(3, dma), 0x3000'0000u | (1u << 4) | 3u);
    // write-back read/write allocate: TEX 001, C 1, B 1
    EXPECT_EQ(mpu::attributes(Memory::NormalWriteBack, false), (1u << 19) | (1u << 17) | (1u << 16));
    // flash: write-through (TEX 000, C 1, B 0), privileged read-only (AP 101), executable
    EXPECT_EQ(mpu::rasr(h7::mpu_regions::flash), (5u << 24) | (1u << 17) | (0x14u << 1) | 1u);
    EXPECT_FALSE((mpu::valid({.base = 0x3000'0000, .bytes = 288 * 1024, .memory = Memory::NormalNonCacheable})))
        << "288 KB is no power of two";
    EXPECT_FALSE((mpu::valid({.base = 0x3001'0000, .bytes = 256 * 1024, .memory = Memory::NormalNonCacheable})))
        << "base not aligned to the size";
    static_assert(mpu::valid(h7::mpu_regions::flash) && mpu::valid(h7::mpu_regions::dtcm) &&
                  mpu::valid(h7::mpu_regions::axi_sram) && mpu::valid(h7::mpu_regions::dma));
}

TEST(Cm7Mpu, FlashDataAreaStaysWritable) {
    namespace mpu = rtos::port::cm7::mpu;
    constexpr auto flash = h7::mpu_regions::flash;  // 2 MB: eighths of 256 KB
    EXPECT_EQ(mpu::subregions_from(flash, 0x081C'0000), 0x80u) << "the FCU's missions: the last 256 KB";
    EXPECT_EQ(mpu::subregions_from(flash, 0x0820'0000), 0x00u) << "no data area: the whole flash read-only";
    EXPECT_EQ(mpu::subregions_from(flash, 0x0810'0000), 0xF0u) << "bank 2";
    EXPECT_EQ(mpu::subregions_from(flash, 0x0811'0000), 0xF0u) << "unaligned: the shared eighth is writable too";
    mpu::Region r = flash;
    r.subregions_disabled = 0x80;
    EXPECT_EQ(mpu::rasr(r) & (0xFFu << 8), 0x80u << 8);
}

TEST(H7Flash, SectorsAndFlashWords) {
    namespace f = h7::flash;
    EXPECT_EQ(f::sector_bytes, 128u * 1024u);
    EXPECT_EQ(f::sectors_per_bank, 8u);
    EXPECT_EQ(f::flash_word, 32u) << "FLASH_NB_32BITWORD_IN_FLASHWORD = 8";
    static_assert(f::detail::Bank<1>::at == 0x5200'2000 && f::detail::Bank<2>::at == 0x5200'2100,
                  "bank 2's KEYR/CR/SR/CCR sit 0x100 above bank 1's (SVD: KEYR2 +0x104, CR2 +0x10C)");
    using Missions = f::Region<0x081C'0000, 256 * 1024, 400'000'000>;
    EXPECT_EQ(Missions::block_count, 2u);
    EXPECT_EQ(Missions::block_bytes, 128u * 1024u);
    EXPECT_EQ(Missions::program_bytes, 32u);
    // flash::Region<0x081E'0000, 256 * 1024, ...> (beyond the flash) and
    // flash::Region<0x080E'0000, 256 * 1024, ...> (across the banks) do not compile.
}

}  // namespace
