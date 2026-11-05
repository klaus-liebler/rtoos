// SPDX-License-Identifier: MIT
//
// Command terminal: a line of text in, lines of text out, independent of the transport (the
// FCU's browser terminal carries it over a WebSocket; a UART or SSH session could do the same).
//
// The command catalog is generated from C# schema files (FirmwareBuilder.Common, TerminalCatalog):
// terminal_commands.hh with
//   namespace terminal_catalog {
//       enum class Command : std::uint8_t { ... };       // the commands the device runs
//       struct CommandInfo { Command id; std::string_view name, usage; };
//       inline constexpr std::array<CommandInfo, N> commands;
//       enum class Lookup : std::uint8_t { ... };        // live value lists for the client
//       inline constexpr std::size_t lookup_count;
//   }
// The product writes one handler per command and lists them; complete() checks at compile time
// that every command of the catalog has exactly one handler:
//
//   using Out = rtos::terminal::Output<1024>;
//   bool cmd_info(rtos::terminal::Args, Out& o) { o.line("uptime %lu s", ...); return true; }
//   constexpr std::array handlers{rtos::terminal::Handler<cat::Command, Out>{cat::Command::info, &cmd_info}, ...};
//   static_assert(rtos::terminal::complete(cat::commands, handlers));
//   Out out;
//   const bool ok = rtos::terminal::execute(line, cat::commands, handlers, out);
//
// A handler returns false for wrong arguments: execute() then adds the usage line. Handlers run
// in the thread that calls execute(); the transport usually queues the lines (Request) to the
// thread that owns the hardware.
#pragma once

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <span>
#include <string_view>

namespace rtos::terminal {

/// The output of a command: lines of text, cut at `Capacity` characters.
template <std::size_t Capacity>
class Output {
public:
    [[gnu::format(printf, 2, 3)]] void line(const char* format, ...) noexcept {
        va_list args;
        va_start(args, format);
        const int n = std::vsnprintf(text_.data() + size_, text_.size() - size_, format, args);
        va_end(args);
        if (n > 0) {
            size_ = std::min(size_ + static_cast<std::size_t>(n), text_.size() - 1);
        }
        if (size_ + 1 < text_.size()) {
            text_[size_++] = '\n';
        }
        text_[size_] = '\0';
    }

    /// The lines without the last line break.
    [[nodiscard]] const char* c_str() noexcept {
        if (size_ > 0 && text_[size_ - 1] == '\n') {
            text_[--size_] = '\0';
        }
        return text_.data();
    }

private:
    std::array<char, Capacity + 1> text_{};
    std::size_t size_ = 0;
};

/// The arguments of a command (the words after its name).
using Args = std::span<const std::string_view>;

template <class Id, class Out>
struct Handler {
    Id id;
    bool (*run)(Args, Out&);  ///< false: wrong arguments (the usage line is shown)
};

/// Words of a command line, separated by spaces: the command and at most max_words - 1 arguments.
inline constexpr std::size_t max_words = 8;

/// Split `line` into `words`; the count, or words.size() + 1 when there are more words.
[[nodiscard]] constexpr std::size_t split(std::string_view line, std::span<std::string_view> words) noexcept {
    std::size_t n = 0;
    for (;;) {
        const auto start = line.find_first_not_of(' ');
        if (start == std::string_view::npos) {
            return n;
        }
        if (n == words.size()) {
            return n + 1;
        }
        line.remove_prefix(start);
        const auto end = line.find(' ');
        words[n++] = line.substr(0, end);
        line = end == std::string_view::npos ? std::string_view{} : line.substr(end);
    }
}

/// Every command of `commands` has exactly one handler, and there is no other.
template <class Info, std::size_t N, class Id, class Out, std::size_t M>
consteval bool complete(const std::array<Info, N>& commands, const std::array<Handler<Id, Out>, M>& handlers) {
    for (const Info& c : commands) {
        std::size_t n = 0;
        for (const auto& h : handlers) {
            n += h.id == c.id ? 1 : 0;
        }
        if (n != 1) {
            return false;
        }
    }
    return M == N;
}

/// Run one command line. False: unknown command or wrong arguments (with a message in `out`).
/// An empty line does nothing.
template <class Info, std::size_t N, class Id, class Out, std::size_t M>
bool execute(std::string_view line, const std::array<Info, N>& commands,
             const std::array<Handler<Id, Out>, M>& handlers, Out& out) noexcept {
    std::array<std::string_view, max_words> words{};
    const std::size_t n = split(line, words);
    if (n == 0) {
        return true;
    }
    const auto c = std::ranges::find(commands, words[0], &Info::name);
    const auto h = c == commands.end() ? handlers.end() : std::ranges::find(handlers, c->id, &Handler<Id, Out>::id);
    if (h == handlers.end()) {
        out.line("unknown command '%.*s' - try help", static_cast<int>(words[0].size()), words[0].data());
        return false;
    }
    if (n > words.size() || !h->run(std::span(words).subspan(1, n - 1), out)) {
        out.line("usage: %.*s", static_cast<int>(c->usage.size()), c->usage.data());
        return false;
    }
    return true;
}

/// A number: decimal, or hexadecimal with 0x.
template <class T>
[[nodiscard]] constexpr std::optional<T> number(std::string_view s) noexcept {
    T v{};
    int base = 10;
    if (s.starts_with("0x") || s.starts_with("0X")) {
        s.remove_prefix(2);
        base = 16;
    }
    const auto [end, ec] = std::from_chars(s.data(), s.data() + s.size(), v, base);
    if (s.empty() || ec != std::errc{} || end != s.data() + s.size()) {
        return std::nullopt;
    }
    return v;
}

/// A level: 1/on/high or 0/off/low.
[[nodiscard]] constexpr std::optional<bool> level(std::string_view s) noexcept {
    if (s == "1" || s == "on" || s == "high") return true;
    if (s == "0" || s == "off" || s == "low") return false;
    return std::nullopt;
}

/// A command line or a lookup of a client session, queued from the transport's thread to the
/// thread that runs the commands (e.g. rtos::MessageQueue<Kernel, Request<...>, 4>).
template <class Session, class Lookup, std::size_t MaxLine>
struct Request {
    Session session{};
    std::uint16_t request_id = 0;
    std::optional<Lookup> lookup;  ///< a lookup instead of a command line
    std::array<char, MaxLine + 1> line{};

    /// A command line, cut at MaxLine characters.
    static Request command(Session s, std::uint16_t id, std::span<const char> text) noexcept {
        Request r{.session = s, .request_id = id, .lookup = std::nullopt};
        std::memcpy(r.line.data(), text.data(), std::min(text.size(), MaxLine));
        return r;
    }
    [[nodiscard]] std::string_view text() const noexcept { return line.data(); }
};

}  // namespace rtos::terminal
