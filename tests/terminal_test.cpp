// Command terminal (rtos/terminal/terminal.hpp) and the pin names of the STM32H5 port.
#include "rtos/terminal/terminal.hpp"

#include <gtest/gtest.h>

#include <string>

#include "rtos/port/stm32h5/gpio.hpp"

namespace {

namespace term = rtos::terminal;

// a catalog as terminal_commands.hh (FirmwareBuilder.Common TerminalCatalog) generates it
namespace catalog {
enum class Command : std::uint8_t { info, out };
struct CommandInfo {
    Command id;
    std::string_view name;
    std::string_view usage;
};
inline constexpr std::array<CommandInfo, 2> commands{{
    {Command::info, "info", "info"},
    {Command::out, "out", "out [<output>] [<state>]"},
}};
}  // namespace catalog

using Out = term::Output<64>;
using Handler = term::Handler<catalog::Command, Out>;

std::string seen;  // the arguments the last handler got, joined with '|'

bool cmd_info(term::Args a, Out& o) {
    if (!a.empty()) return false;
    o.line("info %d", 42);
    return true;
}

bool cmd_out(term::Args a, Out& o) {
    seen.clear();
    for (const auto w : a) seen += std::string(w) + "|";
    o.line("out");
    return a.size() <= 2;
}

constexpr std::array handlers{Handler{catalog::Command::info, &cmd_info}, Handler{catalog::Command::out, &cmd_out}};
static_assert(term::complete(catalog::commands, handlers));
static_assert(!term::complete(catalog::commands, std::array{Handler{catalog::Command::info, &cmd_info}}));
static_assert(!term::complete(catalog::commands, std::array{Handler{catalog::Command::info, &cmd_info},
                                                            Handler{catalog::Command::info, &cmd_info}}));

std::string run(std::string_view line, bool expect_ok) {
    Out out;
    EXPECT_EQ(term::execute(line, catalog::commands, handlers, out), expect_ok) << line;
    return out.c_str();
}

TEST(Terminal, RunsTheHandlerWithTheArguments) {
    EXPECT_EQ(run("info", true), "info 42");
    EXPECT_EQ(run("  out   valve  on ", true), "out");
    EXPECT_EQ(seen, "valve|on|");
}

TEST(Terminal, EmptyLineDoesNothing) {
    EXPECT_EQ(run("", true), "");
    EXPECT_EQ(run("    ", true), "");
}

TEST(Terminal, UnknownCommand) { EXPECT_EQ(run("infox", false), "unknown command 'infox' - try help"); }

TEST(Terminal, WrongArgumentsShowTheUsage) {
    EXPECT_EQ(run("info now", false), "usage: info");
    EXPECT_EQ(run("out a b c", false), "out\nusage: out [<output>] [<state>]");
}

TEST(Terminal, TooManyWordsAreAnErrorNotCutOff) {
    EXPECT_EQ(term::max_words, 8u);
    EXPECT_EQ(run("out 1 2 3 4 5 6 7", false), "out\nusage: out [<output>] [<state>]");  // 8 words: handler decides
    seen.clear();
    EXPECT_EQ(run("out 1 2 3 4 5 6 7 8", false), "usage: out [<output>] [<state>]");  // 9: never reaches it
    EXPECT_EQ(seen, "");
}

TEST(Terminal, OutputIsCutAtItsCapacity) {
    term::Output<8> o;
    o.line("%s", "0123456789");
    o.line("more");
    EXPECT_STREQ(o.c_str(), "01234567");  // Capacity characters, nothing of the second line
}

TEST(Terminal, Numbers) {
    static_assert(term::number<int>("-12") == -12);
    static_assert(term::number<unsigned>("0x1F") == 31u);
    static_assert(!term::number<unsigned>("0x"));
    static_assert(!term::number<unsigned>(""));
    static_assert(!term::number<unsigned>("12a"));
    static_assert(!term::number<std::uint8_t>("256"));
    static_assert(term::level("on") == true && term::level("0") == false && !term::level("maybe"));
}

TEST(Terminal, RequestCutsTheLine) {
    using Request = term::Request<std::uint8_t, catalog::Command, 4>;
    const std::string_view text = "info now";
    const auto r = Request::command(3, 7, text);
    EXPECT_EQ(r.session, 3);
    EXPECT_EQ(r.request_id, 7);
    EXPECT_FALSE(r.lookup);
    EXPECT_EQ(r.text(), "info");
}

TEST(Stm32h5Gpio, PinNames) {
    namespace gpio = rtos::port::stm32h5::gpio;
    using gpio::GpioPort;
    static_assert(gpio::parse_pin("PC7") == gpio::Pin{GpioPort::C, 7});
    static_assert(gpio::parse_pin("pa15") == gpio::Pin{GpioPort::A, 15});
    static_assert(gpio::port_letter({GpioPort::H, 1}) == 'H');
    static_assert(!gpio::parse_pin("PA16"));
    static_assert(!gpio::parse_pin("PZ1"));
    static_assert(!gpio::parse_pin("PA"));
    static_assert(!gpio::parse_pin("PA1x"));
    static_assert(!gpio::parse_pin("XA1"));
    static_assert(!gpio::parse_pin("PA001"));
}

}  // namespace
