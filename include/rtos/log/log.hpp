// SPDX-License-Identifier: MIT
//
// Logging that never blocks the caller on an output device.
//
//   RTOS_LOG_INFO("dhcp: bound %u.%u.%u.%u", a, b, c, d);   // any thread or kernel-aware ISR
//
// A call formats its text on the caller's stack (printf syntax, checked by the compiler) and
// appends it to a Journal: a ring of fixed-size records, written under one short Guard section
// (target: Kernel::Lock). Output devices are readers of the journal that run where they belong
// and at their own pace: the console (a low-priority thread writing the UART), a WebSocket
// mirror (a service in the network thread), a test. Each reader has its own position; when a
// reader falls behind by more than the ring, it learns how many records it lost instead of
// stalling the writers. The newest records always win.
//
// Text longer than a record is cut. A Block keeps several lines together (a boot banner, a bus
// scan): they are appended in one Guard section, so no other line lands between them.
//
// Formatting uses vsnprintf (newlib-nano: no heap for integer and string conversions; floats
// are not printed by nano). Zero-latency ISRs must not log: the Guard masks only kernel-aware
// priorities.
#pragma once

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>

namespace rtos::log {

enum class Level : std::uint8_t { Trace, Debug, Info, Warn, Error, Fatal, Off };

/// Three-letter names, indexed by Level.
inline constexpr std::array<std::string_view, 6> level_names{"TRC", "DBG", "INF", "WRN", "ERR", "FTL"};

/// One record as a reader sees it (the text is copied out of the journal).
struct Record {
    std::uint32_t sequence = 0;
    std::uint32_t time_ms = 0;
    Level level = Level::Info;
    bool continuation = false;  ///< a further line of a Block: no prefix of its own
    std::uint16_t line = 0;
    const char* file = "";      ///< file name only (__FILE_NAME__), static storage
    std::string_view text;
};

/// The interface a front end needs from a journal (type-erased, so the macros need no template).
struct Sink {
    void* context = nullptr;
    /// Appends `lines` (separated by '\n') as one record per line, the first with the prefix.
    void (*append)(void* context, Level, const char* file, std::uint16_t line, std::uint32_t time_ms,
                   std::string_view lines) noexcept = nullptr;
    std::uint32_t (*now_ms)() noexcept = nullptr;
};

namespace detail {
inline Sink g_sink{};
inline Level g_level = Level::Info;
}  // namespace detail

/// Install the journal (normally once in main(), before threads start).
inline void set_sink(const Sink& sink) noexcept { detail::g_sink = sink; }
inline void set_level(Level level) noexcept { detail::g_level = level; }
[[nodiscard]] inline Level level() noexcept { return detail::g_level; }
[[nodiscard]] inline bool enabled(Level level) noexcept {
    return level >= detail::g_level && level != Level::Off && detail::g_sink.append != nullptr;
}

/// Maximum text of one log call (the caller's stack holds it while formatting).
inline constexpr std::size_t max_line = 160;

inline void append(Level level, const char* file, int line, std::string_view lines) noexcept {
    const Sink& s = detail::g_sink;
    const std::uint32_t now = s.now_ms != nullptr ? s.now_ms() : 0;
    s.append(s.context, level, file, static_cast<std::uint16_t>(line), now, lines);
}

inline void vwrite(Level level, const char* file, int line, const char* format, std::va_list args) noexcept {
    if (!enabled(level)) {
        return;
    }
    std::array<char, max_line> text;
    const int n = std::vsnprintf(text.data(), text.size(), format, args);
    if (n < 0) {
        return;
    }
    append(level, file, line, {text.data(), std::min<std::size_t>(static_cast<std::size_t>(n), text.size() - 1)});
}

[[gnu::format(printf, 4, 5)]] inline void write(Level level, const char* file, int line, const char* format,
                                                 ...) noexcept {
    std::va_list args;
    va_start(args, format);
    vwrite(level, file, line, format, args);
    va_end(args);
}

/// Several lines that stay together; appended when the Block ends. The text lives in the
/// Block (on the caller's stack): size it for the lines.
template <std::size_t Bytes = 1024>
class Block {
public:
    Block(Level level, const char* file, int line) noexcept : level_(level), file_(file), line_(line) {}
    Block(const Block&) = delete;
    Block& operator=(const Block&) = delete;
    ~Block() {
        if (used_ != 0 && enabled(level_)) {
            append(level_, file_, line_, {text_.data(), used_});
        }
    }

    [[gnu::format(printf, 2, 3)]] void add(const char* format, ...) noexcept {
        if (!enabled(level_) || used_ + 1 >= Bytes) {
            return;
        }
        if (used_ != 0) {
            text_[used_++] = '\n';
        }
        std::va_list args;
        va_start(args, format);
        const int n = std::vsnprintf(text_.data() + used_, Bytes - used_, format, args);
        va_end(args);
        if (n > 0) {
            used_ = std::min(used_ + static_cast<std::size_t>(n), Bytes - 1);
        }
    }

private:
    std::array<char, Bytes> text_{};
    std::size_t used_ = 0;
    Level level_;
    const char* file_;
    int line_;
};

// ---- journal ---------------------------------------------------------------------------

struct NoGuard {};

/// A reader's position in a Journal.
struct Cursor {
    std::uint32_t next = 0;  ///< sequence number of the next record to read
};

/// Ring of `Records` records with up to `Text` bytes each. `Guard` is a default-constructible
/// RAII lock (target: Kernel::Lock, single-threaded tests: NoGuard).
template <std::size_t Records, std::size_t Text = 120, class Guard = NoGuard>
class Journal {
    static_assert(Records >= 2 && Text >= 16);

public:
    /// The Sink for set_sink(); the journal must outlive the logging.
    [[nodiscard]] Sink sink(std::uint32_t (*now_ms)() noexcept) noexcept { return {this, &append_thunk, now_ms}; }

    /// Called after every append, outside the Guard (e.g. to wake the console thread). Runs
    /// in the logging context: thread or kernel-aware ISR.
    void set_listener(void (*listener)(void*) noexcept, void* context) noexcept {
        listener_ = listener;
        listener_context_ = context;
    }

    void append(Level level, const char* file, std::uint16_t line, std::uint32_t time_ms,
                std::string_view lines) noexcept {
        {
            [[maybe_unused]] const Guard guard{};
            bool first = true;
            for (;;) {
                const std::size_t end = std::min(lines.find('\n'), lines.size());
                Slot& s = slots_[written_ % Records];
                s.sequence = written_++;
                s.time_ms = time_ms;
                s.level = level;
                s.continuation = !first;
                s.line = line;
                s.file = file;
                s.length = static_cast<std::uint16_t>(std::min(end, Text));
                std::copy_n(lines.data(), s.length, s.text.data());
                first = false;
                if (end == lines.size()) {
                    break;
                }
                lines.remove_prefix(end + 1);
            }
        }
        if (listener_ != nullptr) {
            listener_(listener_context_);
        }
    }

    /// A cursor that starts with the oldest record still held.
    [[nodiscard]] Cursor oldest() const noexcept {
        [[maybe_unused]] const Guard guard{};
        return {written_ > Records ? written_ - static_cast<std::uint32_t>(Records) : 0};
    }

    /// Copies the next record into `out` (its text into `text`); false if there is none.
    /// `lost` counts the records the cursor missed because the ring overtook it.
    bool read(Cursor& cursor, Record& out, std::span<char, Text> text, std::uint32_t& lost) noexcept {
        [[maybe_unused]] const Guard guard{};
        lost = 0;
        if (written_ - cursor.next > Records) {
            const std::uint32_t oldest = written_ - static_cast<std::uint32_t>(Records);
            lost = oldest - cursor.next;
            cursor.next = oldest;
        }
        if (cursor.next == written_) {
            return false;
        }
        const Slot& s = slots_[cursor.next % Records];
        std::copy_n(s.text.data(), s.length, text.data());
        out = Record{s.sequence, s.time_ms, s.level, s.continuation, s.line, s.file, {text.data(), s.length}};
        ++cursor.next;
        return true;
    }

    [[nodiscard]] std::uint32_t written() const noexcept {
        [[maybe_unused]] const Guard guard{};
        return written_;
    }

private:
    struct Slot {
        std::uint32_t sequence;
        std::uint32_t time_ms;
        Level level;
        bool continuation;
        std::uint16_t line;
        const char* file;
        std::uint16_t length;
        std::array<char, Text> text;
    };

    static void append_thunk(void* self, Level level, const char* file, std::uint16_t line, std::uint32_t time_ms,
                             std::string_view lines) noexcept {
        static_cast<Journal*>(self)->append(level, file, line, time_ms, lines);
    }

    std::array<Slot, Records> slots_{};
    std::uint32_t written_ = 0;
    void (*listener_)(void*) noexcept = nullptr;
    void* listener_context_ = nullptr;
};

// ---- output format ---------------------------------------------------------------------

/// Width of the console prefix: time (6) level (3) file (8) ':' line (3) ':' and spaces.
inline constexpr std::size_t prefix_width = 25;

/// One console line with a prefix and CRLF:
///   " 12345 INF dhcp.hpp:042: text"   (time in ms, in s from 100000 ms on)
/// continuation lines are indented to the text column. ANSI colours if `colour`.
/// Returns the length (cut to `out`).
inline std::size_t format_line(const Record& r, std::span<char> out, bool colour) noexcept {
    static constexpr std::array<const char*, 6> colours{"\x1b[94m", "\x1b[36m", "\x1b[32m",
                                                        "\x1b[33m", "\x1b[31m", "\x1b[35m"};
    int n = 0;
    if (r.continuation) {
        n = std::snprintf(out.data(), out.size(), "%*s%.*s\r\n", static_cast<int>(prefix_width), "",
                          static_cast<int>(r.text.size()), r.text.data());
    } else {
        const auto level = static_cast<std::size_t>(r.level) < level_names.size() ? static_cast<std::size_t>(r.level) : 5;
        const unsigned long time = r.time_ms > 99'999 ? r.time_ms / 1000 : r.time_ms;
        n = std::snprintf(out.data(), out.size(), colour ? "%6lu %s%s\x1b[0m \x1b[90m%-8.8s:%03u:\x1b[0m %.*s\r\n"
                                                         : "%6lu %s%s %-8.8s:%03u: %.*s\r\n",
                          time, colour ? colours[level] : "", level_names[level].data(), r.file,
                          static_cast<unsigned>(r.line), static_cast<int>(r.text.size()), r.text.data());
    }
    return n < 0 ? 0 : std::min(static_cast<std::size_t>(n), out.size() - 1);
}

}  // namespace rtos::log

#define RTOS_LOG(level, ...) ::rtos::log::write(level, __FILE_NAME__, __LINE__, __VA_ARGS__)
#define RTOS_LOG_TRACE(...) RTOS_LOG(::rtos::log::Level::Trace, __VA_ARGS__)
#define RTOS_LOG_DEBUG(...) RTOS_LOG(::rtos::log::Level::Debug, __VA_ARGS__)
#define RTOS_LOG_INFO(...) RTOS_LOG(::rtos::log::Level::Info, __VA_ARGS__)
#define RTOS_LOG_WARN(...) RTOS_LOG(::rtos::log::Level::Warn, __VA_ARGS__)
#define RTOS_LOG_ERROR(...) RTOS_LOG(::rtos::log::Level::Error, __VA_ARGS__)
#define RTOS_LOG_FATAL(...) RTOS_LOG(::rtos::log::Level::Fatal, __VA_ARGS__)
/// A Block of lines at `level` that stays together: auto b = RTOS_LOG_BLOCK(Info); b.add(...);
#define RTOS_LOG_BLOCK(level) ::rtos::log::Block<>(::rtos::log::Level::level, __FILE_NAME__, __LINE__)
