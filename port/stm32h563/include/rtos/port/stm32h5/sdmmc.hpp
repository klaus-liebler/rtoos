// SPDX-License-Identifier: MIT
//
// STM32H563 SDMMC host controller for rtos::storage::SdCard (RM0481 chapter SDMMC;
// registers, bits, IRQs: SVD v1.8).
//
//   using Host = Sdmmc<SdmmcId::Sdmmc1, {.kernel_hz = board::pll1_q_hz, .cpu_hz = board::sysclk_hz}>;
//   Host host;
//   rtos::storage::SdCard<Host> card{host};
//   card.init();
//
// The configuration is a template argument: the clock dividers for identification (at most
// 400 kHz) and data transfer (at most transfer_hz, default 25 MHz - default speed) are chosen
// and checked at compile time: SDMMC_CK = kernel / (2 x CLKDIV). Commands and data are polled
// with timeouts on the DWT cycle counter; data moves by the internal DMA (IDMA) straight
// between the card and the caller's buffer, which must be word aligned. Kernel clock: RCC_CCIPR4
// reset selection pll1_q.
#pragma once

#include <contracts>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"
#include "rtos/storage/sd_card.hpp"

namespace rtos::port::stm32h5 {

enum class SdmmcId : std::uint8_t { Sdmmc1, Sdmmc2 };

struct SdmmcConfig {
    std::uint32_t kernel_hz;                 ///< SDMMC kernel clock (pll1_q)
    std::uint32_t cpu_hz;                    ///< core clock: timeouts
    std::uint32_t init_hz = 400'000;         ///< identification clock limit (spec: <= 400 kHz)
    std::uint32_t transfer_hz = 25'000'000;  ///< data clock limit (default speed: 25 MHz)
    std::uint32_t data_timeout_ms = 500;     ///< read, write and busy timeout (spec: 250 ms write)
};

/// CLKDIV for at most `hz` at `kernel_hz` (SDMMC_CK = kernel / (2 x CLKDIV), CLKDIV >= 1).
[[nodiscard]] constexpr std::uint32_t sdmmc_clkdiv(std::uint32_t kernel_hz, std::uint32_t hz) noexcept {
    const std::uint32_t div = (kernel_hz + 2 * hz - 1) / (2 * hz);
    return div == 0 ? 1 : div;
}

namespace sdmmc_detail {
inline constexpr std::uint32_t power_on = 3;
inline constexpr std::uint32_t cmdr_cmdtrans = 1u << 6, cmdr_cmdstop = 1u << 7, cmdr_cpsmen = 1u << 12;
inline constexpr std::uint32_t dctrl_dtdir = 1u << 1, dctrl_block512 = 9u << 4, dctrl_fiforst = 1u << 13;
inline constexpr std::uint32_t star_ccrcfail = 1u << 0, star_dcrcfail = 1u << 1, star_ctimeout = 1u << 2,
                               star_dtimeout = 1u << 3, star_txunderr = 1u << 4, star_rxoverr = 1u << 5,
                               star_cmdrend = 1u << 6, star_cmdsent = 1u << 7, star_dataend = 1u << 8,
                               star_cpsmact = 1u << 13, star_busyd0 = 1u << 20,
                               star_idmate = 1u << 27;
inline constexpr std::uint32_t icr_all = 0x1FE0'0FFFu;  // bits 0..11, 21..28
/// The command path's flags only: a data transfer's flags (DATAEND ...) may already be set
/// when its command completes and must survive until the data path looks at them.
inline constexpr std::uint32_t icr_command = star_ccrcfail | star_ctimeout | star_cmdrend | star_cmdsent;
inline constexpr std::uint32_t idma_en = 1u << 0;
}  // namespace sdmmc_detail

template <SdmmcId Id, SdmmcConfig Config>
class Sdmmc {
    static constexpr std::uintptr_t offset = Id == SdmmcId::Sdmmc1 ? 0x0600'8000 : 0x0600'8C00;
    static constexpr rcc::Gate gate = Id == SdmmcId::Sdmmc1 ? rcc::gate::sdmmc1 : rcc::gate::sdmmc2;
    static constexpr std::uint32_t init_div = sdmmc_clkdiv(Config.kernel_hz, Config.init_hz);
    static constexpr std::uint32_t transfer_div = sdmmc_clkdiv(Config.kernel_hz, Config.transfer_hz);
    static_assert(init_div <= 1023, "kernel clock too fast for the identification clock (CLKDIV > 1023)");
    static_assert(Config.kernel_hz / (2 * init_div) <= 400'000, "identification clock above 400 kHz");
    static_assert(Config.transfer_hz <= 50'000'000, "transfer clock above high speed (50 MHz)");
    static constexpr std::uint32_t cycles_per_ms = Config.cpu_hz / 1000;
    static constexpr std::uint32_t transfer_hz = Config.kernel_hz / (2 * transfer_div);
    static constexpr std::uint32_t data_timeout_clocks = transfer_hz / 1000 * Config.data_timeout_ms;

public:
    using SdResponse = storage::SdResponse;
    using SdStatus = storage::SdStatus;
    using SdCommandResult = storage::SdCommandResult;

    static constexpr std::uint8_t irq = Id == SdmmcId::Sdmmc1 ? 79 : 102;
    static constexpr std::uint32_t identification_clock_hz = Config.kernel_hz / (2 * init_div);
    static constexpr std::uint32_t transfer_clock_hz = transfer_hz;

    void power_on() noexcept {
        rcc::enable(gate);
        POWER::write(0);
        CLKCR::write(init_div);  // 1-bit bus, rising edge, no power saving
        POWER::write(sdmmc_detail::power_on);
        delay_ms(2);  // >= 74 clocks before the first command
    }

    void set_transfer_clock() noexcept { CLKCR::write(transfer_div); }

    void delay_ms(std::uint32_t ms) noexcept {
        const std::uint32_t start = cm33::CycleCounter::now();
        while (cm33::CycleCounter::now() - start < ms * cycles_per_ms) {
        }
    }

    SdCommandResult command(std::uint8_t index, std::uint32_t arg, SdResponse r) noexcept {
        return run(index, arg, r, 0);
    }

    SdCommandResult stop(SdResponse r) noexcept { return run(12, 0, r, sdmmc_detail::cmdr_cmdstop); }

    SdCommandResult read_data(std::uint8_t index, std::uint32_t arg, std::span<std::byte> in) noexcept {
        return transfer(index, arg, in.data(), in.size(), true);
    }

    SdCommandResult write_data(std::uint8_t index, std::uint32_t arg, std::span<const std::byte> out) noexcept {
        return transfer(index, arg, const_cast<std::byte*>(out.data()), out.size(), false);
    }

private:
    static constexpr std::uint32_t waitresp(SdResponse r) noexcept {
        switch (r) {
            case SdResponse::None: return 0;
            case SdResponse::R3: return 2;  // short, no CRC
            case SdResponse::R2: return 3;  // long
            default: return 1;              // short, CRC checked
        }
    }

    template <class Done>
    static bool wait(Done done, std::uint32_t ms) noexcept {
        const std::uint32_t start = cm33::CycleCounter::now();
        while (!done()) {
            if (cm33::CycleCounter::now() - start > ms * cycles_per_ms) {
                return false;
            }
        }
        return true;
    }

    SdCommandResult run(std::uint8_t index, std::uint32_t arg, SdResponse r, std::uint32_t extra) noexcept {
        using namespace sdmmc_detail;
        if (!wait([] { return (STAR::read() & star_cpsmact) == 0; }, 10)) {
            return {SdStatus::Timeout, {}};
        }
        ICR::write(icr_all);
        ARGR::write(arg);
        CMDR::write(index | (waitresp(r) << 8) | cmdr_cpsmen | extra);
        const std::uint32_t done = r == SdResponse::None ? star_cmdsent : (star_cmdrend | star_ccrcfail | star_ctimeout);
        if (!wait([done] { return (STAR::read() & done) != 0; }, 10)) {
            return {SdStatus::Timeout, {}};
        }
        const std::uint32_t star = STAR::read();
        if ((star & star_ctimeout) != 0) {
            ICR::write(icr_command);
            return {SdStatus::Timeout, {}};
        }
        if ((star & star_ccrcfail) != 0) {
            ICR::write(icr_command);
            return {SdStatus::CrcError, {}};
        }
        SdCommandResult result{SdStatus::Ok, {RESP1::read(), RESP2::read(), RESP3::read(), RESP4::read()}};
        // R1b: the card holds D0 low while busy; BUSYD0 is the inverted line, polled directly
        // (BUSYD0END is not raised after every R1b response)
        if (r == SdResponse::R1b &&
            !wait([] { return (STAR::read() & star_busyd0) == 0; }, Config.data_timeout_ms)) {
            result.status = SdStatus::Timeout;
        }
        ICR::write(icr_command);
        return result;
    }

    SdCommandResult transfer(std::uint8_t index, std::uint32_t arg, std::byte* buffer, std::size_t bytes,
                             bool read) noexcept {
        using namespace sdmmc_detail;
        contract_assert(reinterpret_cast<std::uintptr_t>(buffer) % 4 == 0 && bytes % storage::sd_block_size == 0);
        ICR::write(icr_all);
        DTIMER::write(data_timeout_clocks);
        DLENR::write(static_cast<std::uint32_t>(bytes));
        DCTRL::write(dctrl_block512 | (read ? dctrl_dtdir : 0u));
        IDMABASER::write(static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(buffer)));
        IDMACTRLR::write(idma_en);
        SdCommandResult result = run(index, arg, SdResponse::R1, cmdr_cmdtrans);
        if (result.status == SdStatus::Ok) {
            constexpr std::uint32_t end = star_dataend | star_dcrcfail | star_dtimeout | star_rxoverr | star_txunderr |
                                          star_idmate;
            if (!wait([] { return (STAR::read() & end) != 0; }, Config.data_timeout_ms)) {
                result.status = SdStatus::Timeout;
            } else if (const std::uint32_t star = STAR::read(); (star & star_dataend) == 0) {
                result.status = (star & star_dcrcfail) != 0 ? SdStatus::CrcError
                                : (star & star_dtimeout) != 0 ? SdStatus::Timeout
                                                              : SdStatus::CardError;
            }
        }
        IDMACTRLR::write(0);
        DCTRL::write(dctrl_fiforst);
        DCTRL::write(0);
        ICR::write(icr_all);
        return result;
    }

    using POWER = PReg<offset + 0x00>;
    using CLKCR = PReg<offset + 0x04>;
    using ARGR = PReg<offset + 0x08>;
    using CMDR = PReg<offset + 0x0C>;
    using RESP1 = PReg<offset + 0x14>;
    using RESP2 = PReg<offset + 0x18>;
    using RESP3 = PReg<offset + 0x1C>;
    using RESP4 = PReg<offset + 0x20>;
    using DTIMER = PReg<offset + 0x24>;
    using DLENR = PReg<offset + 0x28>;
    using DCTRL = PReg<offset + 0x2C>;
    using STAR = PReg<offset + 0x34>;
    using ICR = PReg<offset + 0x38>;
    using IDMACTRLR = PReg<offset + 0x50>;
    using IDMABASER = PReg<offset + 0x58>;
};

}  // namespace rtos::port::stm32h5
