// SPDX-License-Identifier: MIT
//
// printf-style formatting of a std::string_view (log records, terminal output): "%.*s" takes an
// int length and a pointer, RTOS_SV(x) passes both -
//     RTOS_LOG_INFO("user %.*s connected", RTOS_SV(name));
// `x` is evaluated twice: pass a name or a call without side effects.
#pragma once

#define RTOS_SV(x) static_cast<int>((x).size()), (x).data()
