// SPDX-License-Identifier: MIT
//
// The PLC image (docs/plc.md "Image format"): constants, records and the little-endian readers the
// loader uses. The image is produced by the TypeScript compiler of the web UI; the VM keeps a view
// of it while it is loaded (no copy).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

namespace rtos::plc {

inline constexpr std::array<std::uint8_t, 8> image_magic{'F', 'C', 'U', 'P', 'L', 'C', 0, 1};
inline constexpr std::uint16_t header_size = 80;
inline constexpr std::size_t task_record_size = 32;
inline constexpr std::size_t program_record_size = 8;
inline constexpr std::size_t io_record_size = 12;
inline constexpr std::size_t native_record_size = 8;

/// Header field offsets.
namespace hdr {
inline constexpr std::size_t isa_version = 8, header_size = 10, length = 12, crc = 16, isa_hash = 20,
                             io_map_hash = 24, project_hash = 28, compiler_version = 32, code_offset = 36,
                             code_size = 40, data_size = 44, init_offset = 48, init_size = 52, task_offset = 56,
                             task_count = 60, io_offset = 64, io_count = 68, native_offset = 72, native_count = 76;
}  // namespace hdr

enum class LoadError : std::uint8_t {
    none,
    too_short,         ///< shorter than the header, or than the length in the header
    magic,
    header_size,
    crc,
    isa_version,
    isa_hash,          ///< built for another instruction set table
    section,           ///< a section outside the image or not 4-byte aligned
    code_too_large,
    data_too_large,    ///< the data segment does not fit, or is too small as the verifier's scratch
    init_range,        ///< an initial value outside the data segment
    task_range,        ///< no task, too many, a frame area outside the data segment or overlapping another
    program_range,     ///< a program list outside the image, or an instance outside the data segment
    io_range,          ///< an I/O binding outside the data segment, or bad bits
    bad_opcode,
    truncated,         ///< an instruction's operands run past the code
    jump_target,       ///< not the start of an instruction
    call_target,       ///< a CALL, CALL_FB or program entry that is not an ENTER
    data_address,      ///< a G access outside the data segment
    native_unknown,    ///< the image lists a native this VM does not have
    native_signature,  ///< ... or has with another signature
    native_not_listed, ///< a SYSCALL of a native the image does not list
};

[[nodiscard]] constexpr const char* to_string(LoadError e) noexcept {
    constexpr const char* names[] = {"none", "too_short", "magic", "header_size", "crc", "isa_version",
                                     "isa_hash", "section", "code_too_large", "data_too_large", "init_range",
                                     "task_range", "program_range", "io_range", "bad_opcode", "truncated",
                                     "jump_target", "call_target", "data_address", "native_unknown",
                                     "native_signature", "native_not_listed"};
    return names[static_cast<std::size_t>(e)];
}

struct TaskInfo {
    std::uint16_t interval;      ///< in I/O cycles
    std::uint8_t priority;
    std::uint8_t max_overruns;
    std::uint32_t budget;        ///< instructions per cycle
    std::uint32_t frame_base;
    std::uint32_t frame_size;
    std::uint32_t programs;      ///< offset of the program list in the image
    std::uint32_t program_count;
    std::uint32_t io_first;
    std::uint32_t io_count;
};

struct ProgramInfo {
    std::uint32_t entry;     ///< code offset of an ENTER
    std::uint32_t instance;  ///< data address
};

enum class IoArea : std::uint8_t { input = 0, holding = 1 };
enum class IoKind : std::uint8_t { input = 0, output = 1, command = 2, parameter = 3 };

struct IoBinding {
    std::uint16_t reg;
    IoArea area;
    IoKind kind;
    std::uint8_t bits;   ///< 1 (BOOL), 16 or 32
    std::uint8_t flags;  ///< bit 0: signed
    std::uint32_t addr;
};

// ---- little-endian access ---------------------------------------------------------------------
[[nodiscard]] inline std::uint16_t rd16(const std::uint8_t* p) noexcept {
    return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}
[[nodiscard]] inline std::uint32_t rd32(const std::uint8_t* p) noexcept {
    std::uint32_t v;
    std::memcpy(&v, p, 4);  // the targets and the host are little-endian
    return v;
}
[[nodiscard]] inline std::uint64_t rd64(const std::uint8_t* p) noexcept {
    std::uint64_t v;
    std::memcpy(&v, p, 8);
    return v;
}

/// CRC-32 (IEEE 802.3, reflected, as zlib's crc32).
[[nodiscard]] constexpr std::uint32_t crc32(std::span<const std::uint8_t> bytes, std::uint32_t crc = 0) noexcept {
    constexpr auto table = [] {
        std::array<std::uint32_t, 256> t{};
        for (std::uint32_t i = 0; i < 256; ++i) {
            std::uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            t[i] = c;
        }
        return t;
    }();
    crc = ~crc;
    for (const std::uint8_t b : bytes) crc = table[(crc ^ b) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

}  // namespace rtos::plc
