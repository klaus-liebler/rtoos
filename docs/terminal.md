# Command terminal

`rtos/terminal/terminal.hpp` runs text commands: a line in, lines of text out. It knows no transport. The FCU carries it over a WebSocket (browser terminal); a UART or SSH session could carry it the same way.

The commands are not written by hand in C++. They are described in C# schema files, and the builder library `FirmwareBuilder.Common` (`TerminalCatalog`, in `dotnet_libs/firmware_builder_common`) generates two files from them:

| file | for | content |
|---|---|---|
| `terminal_commands.hh` | firmware | `namespace terminal_catalog`: `enum class Command`, `commands` (id, name, usage line), `enum class Lookup`, `lookup_count` |
| `terminal-commands.ts` | web app | every command with its help text, parameters and values: completion, `help`, parameter tooltip |

Commands marked `Client` (e.g. `help`, `clear`) only exist in the client; the firmware gets no `Command` for them.

## Schema

One class per command, its fields are the parameters in order. The `///` summary is the help text. Attributes are recognized by name (syntax-only Roslyn parsing, like the register map and BestBinaryBuffers schemas); their declarations in `TerminalSchemaAttributes.cs` only give IntelliSense.

```csharp
using TerminalSchema;

namespace terminal;

/// <summary>Values the client fetches live from the device.</summary>
[TerminalLookups]
public enum Lookup
{
	/// <summary>all GPIO pins with their levels</summary>
	pins,
}

/// <summary>read any pin, or drive an output pin</summary>
[TerminalCommand]
public class gpio
{
	/// <summary>pin name, e.g. PC7</summary>
	[Lookup(Lookup.pins)] public string pin;

	/// <summary>drive the pin</summary>
	[Value("1", "high"), Value("0", "low")] public string? level;
}

/// <summary>clear the console</summary>
[TerminalCommand(Client = true)]
public class clear;
```

| element | meaning |
|---|---|
| `[TerminalCommand]` class | a command; the class name is the command name (lower case, no C++ keyword; `@in` for a C# keyword) |
| `Client = true` | runs in the client only |
| field `string` / `string?` | required / optional parameter; optional ones come last |
| `[Value("v", "doc")]`, repeated | the listed values of a parameter (doc optional) |
| `[Lookup(Lookup.x)]` | values fetched live from the device (the firmware answers the lookup) |
| `[Source(Source.commands)]`, `[Source(Source.registers)]` | values computed in the client (command names, register addresses) |
| `[TerminalLookups]` enum | the lookups, at most one enum; order = `terminal_catalog::Lookup` |

The generator rejects: duplicate names, a missing help text, a required parameter after an optional one, an unknown lookup or source, more than one value kind per parameter, a value with a space, a parameter type other than `string`/`string?`.

## Firmware

```cpp
#include "rtos/terminal/terminal.hpp"
#include "terminal_commands.hh"

namespace cat = terminal_catalog;
using Out = rtos::terminal::Output<1024>;          // lines of text, cut at 1024 characters
using Handler = rtos::terminal::Handler<cat::Command, Out>;

bool cmd_gpio(rtos::terminal::Args a, Out& o) {
    const auto pin = a.empty() ? std::nullopt : gpio::parse_pin(a[0]);
    if (!pin || a.size() > 2) return false;         // false: execute() adds the usage line
    o.line("P%c%u %s", gpio::port_letter(*pin), pin->number, gpio::read(*pin) ? "high" : "low");
    return true;
}

constexpr std::array handlers{Handler{cat::Command::gpio, &cmd_gpio}, /* one per command */};
static_assert(rtos::terminal::complete(cat::commands, handlers));  // a missing handler does not compile

Out out;
const bool ok = rtos::terminal::execute(line, cat::commands, handlers, out);
```

- `execute()` splits the line at spaces into at most `max_words` (8) words: the command and up to 7 arguments. More words are an error (usage line), not cut off. An empty line does nothing and is ok.
- Unknown command: `unknown command 'x' - try help`. Handler returns false: `usage: <usage line>`.
- `Output::line()` is printf-like; the text is cut at the capacity.
- Helpers: `number<T>()` (decimal or `0x` hex, whole word only), `level()` (`1`/`on`/`high`, `0`/`off`/`low`); STM32H5 pin names: `gpio::parse_pin("PC7")` (only pins the package has), `gpio::port_letter()`.
- Lookups have no framework function: the product answers each `cat::Lookup` (a `switch` without `default`, so `-Wswitch` reports a missing one).

### Threads

Handlers run in the thread that calls `execute()`. Commands that touch hardware belong in the thread that owns it; the transport's thread queues the lines there:

```cpp
using Request = rtos::terminal::Request<SessionId, cat::Lookup, 200>;   // line cut at 200 characters
rtos::MessageQueue<Kernel, Request, 4> requests{kernel};

// transport thread
requests.try_send(Request::command(session, request_id, line));
// owning thread, every cycle
while (const auto r = requests.try_receive()) { /* execute(r->text(), ...) or answer r->lookup */ }
```

A handler runs inside the owning thread's cycle: keep it short. A command that waits (e.g. a reset that lets the answer go out first) delays that cycle.

## Build

With rtoos as a subproject the header is on the include path of `rtos_kernel`; there is no library to link. Host tests: `tests/terminal_test.cpp`.
