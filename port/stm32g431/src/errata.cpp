// SPDX-License-Identifier: MIT
//
// STM32G431 silicon errata the start-up has to handle (ES0431).
#include <cstdint>

extern "C" {
/// ES0431 2.2.6 "SRAM write error": after a system reset at a critical instant the first access
/// to an SRAM instance restores its state machine - a write there is lost. The reset entry
/// (cortex_m startup.cpp) reads each instance the firmware uses before the first write: SRAM1
/// (0x2000'0000), SRAM2 (0x2000'4000), CCM SRAM (0x1000'0000; the linker script uses its alias at
/// 0x2000'5800) - the addresses of ST's workaround.
extern const std::uintptr_t rtos_sram_first_reads[] = {0x2000'0000, 0x2000'4000, 0x1000'0000, 0};
}
