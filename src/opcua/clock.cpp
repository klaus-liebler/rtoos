#include "rtos/opcua/clock.hpp"

namespace rtos::opcua {

namespace {

// -- Build-timestamp epoch anchor (see clock.hpp for why the literal Unix epoch doesn't work) --
// All constexpr, evaluated once at compile time; __DATE__/__TIME__ are always "Mmm dd yyyy" /
// "hh:mm:ss" in the C/C++ standard's fixed (space-padded) format.
constexpr int MonthFromAbbrev(const char *m) {
    switch(m[0]) {
    case 'J': return m[1] == 'a' ? 1 : (m[2] == 'n' ? 6 : 7);   // Jan, Jun, Jul
    case 'F': return 2;
    case 'M': return m[2] == 'r' ? 3 : 5;                       // Mar, May
    case 'A': return m[1] == 'p' ? 4 : 8;                       // Apr, Aug
    case 'S': return 9;
    case 'O': return 10;
    case 'N': return 11;
    default:  return 12;                                        // Dec
    }
}

// Days since 1970-01-01 for a given civil (proleptic Gregorian) date -- Howard Hinnant's
// well-known constant-time algorithm (http://howardhinnant.github.io/date_algorithms.html).
constexpr int64_t DaysFromCivil(int64_t y, int m, int d) {
    y -= (m <= 2) ? 1 : 0;
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const int64_t yoe = y - era * 400;
    const int64_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

constexpr int64_t ParseBuildUnixSeconds() {
    const char *date = __DATE__; // e.g. "Aug 19 2026" (day is space-padded, e.g. "Aug  9 2026")
    const char *time = __TIME__; // "hh:mm:ss"
    int month = MonthFromAbbrev(date);
    int day = (date[4] == ' ' ? 0 : (date[4] - '0') * 10) + (date[5] - '0');
    int64_t year = (date[7] - '0') * 1000 + (date[8] - '0') * 100 + (date[9] - '0') * 10 + (date[10] - '0');
    int hour = (time[0] - '0') * 10 + (time[1] - '0');
    int minute = (time[3] - '0') * 10 + (time[4] - '0');
    int second = (time[6] - '0') * 10 + (time[7] - '0');
    return DaysFromCivil(year, month, day) * 86400 + hour * 3600 + minute * 60 + second;
}

constexpr int64_t BUILD_UNIX_SECONDS = ParseBuildUnixSeconds();

UnixMsSource g_unixMs = nullptr;
MonotonicMsSource g_monotonicMs = nullptr;

DateTime MsToHundredNs(int64_t ms) { return ms * (DATETIME_SEC / 1000); }

} // namespace

void SetClock(UnixMsSource unixMs, MonotonicMsSource monotonicMs) noexcept {
    g_unixMs = unixMs;
    g_monotonicMs = monotonicMs;
}

bool IsSynced() noexcept { return g_unixMs != nullptr && g_unixMs() != 0; }

DateTime Now() noexcept {
    if(g_unixMs != nullptr) {
        if(const std::uint64_t ms = g_unixMs(); ms != 0)
            return DATETIME_UNIX_EPOCH + MsToHundredNs(static_cast<int64_t>(ms));
    }
    return NowMonotonic() + DATETIME_UNIX_EPOCH + static_cast<DateTime>(BUILD_UNIX_SECONDS) * DATETIME_SEC;
}

DateTime NowMonotonic() noexcept {
    return g_monotonicMs != nullptr ? MsToHundredNs(static_cast<int64_t>(g_monotonicMs())) : 0;
}

}  // namespace rtos::opcua
