#pragma once
// Server time (OPC UA DateTime). Wall-clock time comes from SNTP via SetClock(); before the
// first sync, Now() anchors the monotonic clock onto the firmware's __DATE__/__TIME__ build
// timestamp -- NOT the literal Unix epoch: a real OPC UA client (UAExpert confirmed,
// 2026-08-19) computes SecureChannel token expiry as the server-reported createdAt plus the
// revised lifetime, compared against the CLIENT's own real UTC clock. Anchor 56 years in the
// past (the literal epoch) and even a 1-hour lifetime reads as expired the instant the channel
// opens, so the client perpetually thinks it needs to renew -- a renew/reconnect loop that
// looks like a connection-stability bug but is really just a wrong clock. The build-timestamp
// anchor is close enough to avoid that even pre-sync, for a device that boots shortly after
// being flashed.
//
// rtoos: the server runs in the network thread, which also owns the SNTP client -- no lock.
#include <cstdint>

#include "rtos/opcua/types.hpp"

namespace rtos::opcua {

/// Milliseconds since the Unix epoch, 0 while not synchronised (e.g. SntpClient::unix_ms).
using UnixMsSource = std::uint64_t (*)() noexcept;
/// Free-running milliseconds that never wrap (e.g. Kernel::now64() at a 1 kHz tick).
using MonotonicMsSource = std::uint64_t (*)() noexcept;

void SetClock(UnixMsSource unixMs, MonotonicMsSource monotonicMs) noexcept;

DateTime Now() noexcept;
DateTime NowMonotonic() noexcept;
/// True once Now() is real UTC (SNTP), not the build-timestamp fallback.
bool IsSynced() noexcept;

}  // namespace rtos::opcua
