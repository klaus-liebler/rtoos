// SPDX-License-Identifier: MIT
// Forward declaration of rtos::Kernel (the default configuration lives here only).
#pragma once

#include "rtos/config.hpp"
#include "rtos/kernel/port.hpp"

namespace rtos {

template <CpuPort Port, KernelConfig Cfg = DefaultConfig>
class Kernel;

}  // namespace rtos
