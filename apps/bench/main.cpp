// SPDX-License-Identifier: MIT
//
// Benchmark: cost of the copies in the TLS record path (wolfSSL's record buffer
// <-> TCP segments). It decided against an own zero-copy record layer (docs/decisions.md:
// "Zero-copy TLS record layer"): the copies cost 3-6 % of the record CPU time.
//
// For each record size (39 B status, 512 B WebSocket message, 1438 B = one full TCP
// segment of the web app) the same ChaCha20-Poly1305 record is produced twice:
//
//   zero copy   what an own record layer would do: plaintext already in the packet buffer
//               (or encrypted straight from its source, e.g. flash), sealed in place
//   with copies what the wolfSSL record buffer path did: copy the plaintext into the
//               record buffer, seal it there, copy the ciphertext into the packet
//               buffer (TCP write from the I/O callback)
//
// and received twice:
//
//   zero copy   decrypt in place in the received packet buffer, frame viewed in place
//   with copies copy into the record buffer, decrypt, copy the plaintext out
//               (wolfSSL_read), copy the payload into the WebSocket decoder
//
// The copy path is a lower bound for the old code: wolfSSL's own record handling
// (BuildTls13Message, buffer management) is not included. Cycle counts come from
// DWT_CYCCNT: real cycles on the hardware, instructions in the emulators.
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/wolfcrypt/chacha20_poly1305.h>

#include "board/fcu.hpp"
#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/stm32h5/cache.hpp"
#include "rtos/rtos.hpp"

// The port's PendSV/SysTick handlers need a kernel; the benchmark never starts it
// (no interrupts, no context switches disturb the measurement).
rtos::Kernel<rtos::port::cm33::Cm33Port> kernel;
RTOS_CM33_BIND_KERNEL(kernel)

namespace {

using rtos::port::cm33::CycleCounter;

constexpr std::size_t max_record = 1438;          // MSS 1460 - 22 bytes record overhead
constexpr std::size_t overhead = 5 + 1 + 16;     // header, content type, tag
constexpr std::size_t runs = 32;

// Source data: in flash (web app) and in SRAM (an application message).
constexpr std::array<std::byte, max_record> flash_source = [] {
    std::array<std::byte, max_record> a{};
    for (std::size_t i = 0; i < a.size(); ++i) {
        a[i] = static_cast<std::byte>(i * 31 + 7);
    }
    return a;
}();
std::array<std::byte, max_record> sram_source;

RTOS_DMA_BUFFER std::array<std::byte, 1536> packet;   // network packet buffer
std::array<std::byte, 16384 + 256> record_buffer;      // wolfSSL's record buffer
std::array<std::byte, max_record> app_buffer;          // wolfSSL_read target
std::array<std::byte, 512> decoder_buffer;             // WebSocket message buffer

constexpr byte key[32] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
                          17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
constexpr byte iv[12] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 1, 2};

unsigned errors = 0;  // failed wolfCrypt calls: the benchmark is only valid with 0
void check(int r) {
    errors += r != 0;
}

void header(std::byte* h, std::size_t inner) {
    h[0] = std::byte{23};
    h[1] = h[2] = std::byte{3};
    h[3] = static_cast<std::byte>((inner + 16) >> 8);
    h[4] = static_cast<std::byte>(inner + 16);
}

/// Seal `n` plaintext bytes from `src` into the record at `h` (header written here):
/// the same streaming calls as TlsSession, for both variants.
void seal(std::byte* h, const std::byte* src, std::size_t n) {
    header(h, n + 1);
    ChaChaPoly_Aead aead;
    const byte type = 23;
    auto* out = reinterpret_cast<byte*>(h + 5);
    check(wc_ChaCha20Poly1305_Init(&aead, key, iv, CHACHA20_POLY1305_AEAD_ENCRYPT));
    check(wc_ChaCha20Poly1305_UpdateAad(&aead, reinterpret_cast<byte*>(h), 5));
    check(wc_ChaCha20Poly1305_UpdateData(&aead, reinterpret_cast<const byte*>(src), out, static_cast<word32>(n)));
    check(wc_ChaCha20Poly1305_UpdateData(&aead, &type, out + n, 1));
    check(wc_ChaCha20Poly1305_Final(&aead, out + n + 1));
}

/// Verify + decrypt the record at `h` in place.
void open(std::byte* h, std::size_t n) {
    ChaChaPoly_Aead aead;
    std::array<byte, 16> tag{};
    auto* body = reinterpret_cast<byte*>(h + 5);
    check(wc_ChaCha20Poly1305_Init(&aead, key, iv, CHACHA20_POLY1305_AEAD_DECRYPT));
    check(wc_ChaCha20Poly1305_UpdateAad(&aead, reinterpret_cast<byte*>(h), 5));
    check(wc_ChaCha20Poly1305_UpdateData(&aead, body, body, static_cast<word32>(n + 1)));
    check(wc_ChaCha20Poly1305_Final(&aead, tag.data()));
    check(wc_ChaCha20Poly1305_CheckTag(body + n + 1, tag.data()));
}

/// Zero copy send: `src` encrypted from where it is into the packet (as send_record
/// with a tail; a payload written into the packet beforehand costs the same).
void seal_zero_copy(const std::byte* src, std::size_t n) { seal(packet.data() + 64, src, n); }

/// Old path: plaintext -> record buffer, seal there, record -> packet.
void seal_with_copies(const std::byte* src, std::size_t n) {
    std::byte* r = record_buffer.data();
    std::memcpy(r + 5, src, n);
    seal(r, r + 5, n);
    std::memcpy(packet.data() + 64, r, n + overhead);
}

/// Zero copy receive: verify + decrypt in the packet buffer.
void open_zero_copy(std::size_t n) { open(packet.data() + 64, n); }

/// Old path: packet -> record buffer, decrypt, plaintext -> app, payload -> decoder.
void open_with_copies(std::size_t n) {
    std::memcpy(record_buffer.data(), packet.data() + 64, n + overhead);
    open(record_buffer.data(), n);
    std::memcpy(app_buffer.data(), record_buffer.data() + 5, n);
    std::memcpy(decoder_buffer.data(), app_buffer.data(), std::min(n, decoder_buffer.size()));
}

/// Best of `runs`; `setup` runs before each run, outside the measurement.
template <class F, class S = void (*)()>
std::uint32_t measure(F&& f, S&& setup = [] {}) {
    std::uint32_t best = UINT32_MAX;
    for (std::size_t i = 0; i < runs; ++i) {
        setup();
        const std::uint32_t t0 = CycleCounter::now();
        f();
        best = std::min(best, CycleCounter::now() - t0);
    }
    return best;
}

void row(const char* name, std::uint32_t zero, std::uint32_t copies) {
    board::print(name);
    board::print(" zero copy ");
    board::print_dec(zero);
    board::print(" | with copies ");
    board::print_dec(copies);
    board::print(" | copies cost ");
    board::print_dec(copies - zero);
    board::print(" = ");
    board::print_dec((copies - zero) * 1000 / copies / 10);
    board::print(".");
    board::print_dec((copies - zero) * 1000 / copies % 10);
    board::print(" %\n");
}

}  // namespace

int main() {
    board::init();
    CycleCounter::enable();
    board::print("\nrtoos bench - record layer copies (cycles, best of 32)\n");
    for (std::size_t i = 0; i < sram_source.size(); ++i) {
        sram_source[i] = static_cast<std::byte>(i * 13);
    }
    struct Case {
        const char* name;
        std::size_t size;
        const std::byte* source;
    };
    const Case cases[] = {{"send   39 B status (SRAM):", 39, sram_source.data()},
                          {"send  512 B message (SRAM):", 512, sram_source.data()},
                          {"send 1438 B web app (flash):", max_record, flash_source.data()}};
    for (const Case& c : cases) {
        const std::uint32_t zero = measure([&] { seal_zero_copy(c.source, c.size); });
        const std::uint32_t copies = measure([&] { seal_with_copies(c.source, c.size); });
        row(c.name, zero, copies);
    }
    for (const std::size_t n : {std::size_t{39}, std::size_t{512}, max_record}) {
        // A valid record, restored before every run (decrypting in place consumes it).
        seal_zero_copy(sram_source.data(), n);
        std::array<std::byte, max_record + overhead> received{};
        std::memcpy(received.data(), packet.data() + 64, n + overhead);
        const auto restore = [&] { std::memcpy(packet.data() + 64, received.data(), n + overhead); };
        const std::uint32_t zero = measure([&] { open_zero_copy(n); }, restore);
        const std::uint32_t copies = measure([&] { open_with_copies(n); }, restore);
        board::print(n == 39 ? "receive   39 B:" : n == 512 ? "receive  512 B:" : "receive 1438 B:");
        row("", zero, copies);
    }
    const std::uint32_t copy = measure([] { std::memcpy(record_buffer.data(), flash_source.data(), max_record); });
    board::print("memcpy 1438 B flash -> SRAM: ");
    board::print_dec(copy);
    board::print(" cycles\n");
    board::print(errors == 0 ? "BENCH DONE (all records verified)\n" : "BENCH INVALID: crypto errors\n");
    for (;;) {
        __asm volatile("wfi");
    }
}
