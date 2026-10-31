# Logging

[include/rtos/log/log.hpp](../include/rtos/log/log.hpp): logging that never makes the caller wait for an output device.

```cpp
RTOS_LOG_INFO("dhcp: bound %u.%u.%u.%u", a, b, c, d);
RTOS_LOG_WARN("supply: only %ld mV", mv);

auto banner = RTOS_LOG_BLOCK(Info);      // lines that stay together
banner.add("=== Factory Control Unit ===");
banner.add(" -> version %s", version);
```                                      // appended when `banner` ends

## How it works

- **Callers** format the text on their own stack (printf syntax, checked by the compiler through `[[gnu::format(printf)]]`, at most 160 bytes). They append it to a `Journal` in one short `Guard` section (target: `Kernel::Lock`). Logging works from any thread and from kernel-aware ISRs, but not from zero-latency ISRs.
- **The journal** is a ring of fixed-size records: `Journal<Records, Text, Guard>`. When it is full, the newest records overwrite the oldest. Writers never wait.
- **Readers** are the output devices: a console thread, a WebSocket mirror in the network thread, a test. Each one has its own `Cursor` and reads at its own pace. A reader that falls behind learns how many records it lost (`read(cursor, record, text, lost)`) instead of stalling the system.
- **Blocks** (`RTOS_LOG_BLOCK`) collect several lines and append them in one Guard section, so no other line lands between them. Their first line has the prefix; the others are `continuation` records.
- **Level filter:** `rtos::log::set_level(Level::Debug)`; filtered calls cost one comparison and are not formatted.

## Setup

```cpp
rtos::log::Journal<64, 120, Kernel::Lock> journal;          // 64 records of 120 characters: ~9 KB

int main() {
    board::init();
    rtos::log::set_sink(journal.sink([]() noexcept { return kernel.now(); }));
    journal.set_listener([](void*) noexcept { wake_console(); }, nullptr);   // optional
    ...
}

// console thread: low priority, writes the UART at its own pace
void console_main(void*) {
    rtos::log::Cursor cursor = journal.oldest();
    rtos::log::Record record;
    std::array<char, 120> text;
    std::array<char, 200> line;
    for (;;) {
        std::uint32_t lost = 0;
        while (journal.read(cursor, record, text, lost)) {
            if (lost != 0) { /* print "... N lines lost" */ }
            board::print({line.data(), rtos::log::format_line(record, line, /*colour*/ true)});
        }
        kernel.notify_wait(0, rtos::wait_forever);
    }
}
```

`format_line` produces the console format with an optional ANSI colour:

```
  4711 INF dhcp.hpp:042: bound 192.168.1.50
   123 ERR main.cpp:007: supply missing        (time in s from 100 000 ms on)
                         continuation line of a block
```

A network reader (e.g. a WebSocket log mirror) is a service in the network thread with its own cursor; it sends the records that are new since its last pass.

Readers: the console (a low-priority thread) and the SSH log terminal ([Networking](networking.md#ssh-log-terminal)), which shows the journal to every connected `ssh` client.

## Tests

[log_test.cpp](../tests/log_test.cpp): formatting and record data, level filter, cut texts, lost records of a slow reader, independent cursors, blocks, the listener, the console format.
