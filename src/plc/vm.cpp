// SPDX-License-Identifier: MIT
// The PLC bytecode VM (docs/plc.md): loader, verifier, interpreter, standard natives.
#include "rtos/plc/vm.hpp"

#include <bit>
#include <cmath>
#include <contracts>
#include <cstring>
#include <limits>

namespace rtos::plc {
namespace {

// ---- slots: raw bits (docs/plc.md "Machine model") ------------------------------------------------
inline std::uint32_t u32(std::uint64_t s) noexcept { return static_cast<std::uint32_t>(s); }
inline std::int32_t s32(std::uint64_t s) noexcept { return static_cast<std::int32_t>(static_cast<std::uint32_t>(s)); }
inline std::int64_t s64(std::uint64_t s) noexcept { return static_cast<std::int64_t>(s); }
inline std::uint64_t of32(std::uint32_t v) noexcept { return v; }
inline std::uint64_t ofs32(std::int32_t v) noexcept { return static_cast<std::uint32_t>(v); }
inline std::uint64_t ofs64(std::int64_t v) noexcept { return static_cast<std::uint64_t>(v); }
inline float f32(std::uint64_t s) noexcept { return std::bit_cast<float>(u32(s)); }
inline double f64(std::uint64_t s) noexcept { return std::bit_cast<double>(s); }
/// Float results: a NaN becomes the canonical quiet NaN (the platforms create different NaN bits).
inline std::uint64_t off32(float f) noexcept { return std::isnan(f) ? 0x7FC0'0000u : std::bit_cast<std::uint32_t>(f); }
inline std::uint64_t off64(double d) noexcept {
    return std::isnan(d) ? 0x7FF8'0000'0000'0000u : std::bit_cast<std::uint64_t>(d);
}
inline std::uint64_t ofb(bool b) noexcept { return b ? 1u : 0u; }

inline std::uint32_t rotl(std::uint32_t x, std::uint32_t n, unsigned width) noexcept {
    const std::uint32_t mask = width == 32 ? 0xFFFF'FFFFu : (1u << width) - 1u;
    x &= mask;
    n %= width;
    return n == 0 ? x : ((x << n) | (x >> (width - n))) & mask;
}
inline std::uint32_t rotr(std::uint32_t x, std::uint32_t n, unsigned width) noexcept {
    n %= width;
    return rotl(x, n == 0 ? 0 : width - n, width);
}

/// Width in bytes of a G access, 0: not a G access.
constexpr std::array<std::uint8_t, 256> g_width = [] {
    std::array<std::uint8_t, 256> w{};
    w[static_cast<std::size_t>(Op::LDG_U8)] = 1;
    w[static_cast<std::size_t>(Op::LDG_S8)] = 1;
    w[static_cast<std::size_t>(Op::LDG_U16)] = 2;
    w[static_cast<std::size_t>(Op::LDG_S16)] = 2;
    w[static_cast<std::size_t>(Op::LDG_32)] = 4;
    w[static_cast<std::size_t>(Op::LDG_64)] = 8;
    w[static_cast<std::size_t>(Op::STG_8)] = 1;
    w[static_cast<std::size_t>(Op::STG_16)] = 2;
    w[static_cast<std::size_t>(Op::STG_32)] = 4;
    w[static_cast<std::size_t>(Op::STG_64)] = 8;
    return w;
}();

/// [lo, lo+size) inside [0, limit)?
inline bool inside(std::uint64_t lo, std::uint64_t size, std::uint64_t limit) noexcept { return lo + size <= limit; }

}  // namespace

// ---- loader and verifier ------------------------------------------------------------------------

LoadError Vm::load(std::span<const std::uint8_t> image) noexcept {
    loaded_ = false;
    if (image.size() < header_size) return LoadError::too_short;
    const std::uint8_t* h = image.data();
    if (std::memcmp(h, image_magic.data(), image_magic.size()) != 0) return LoadError::magic;
    if (rd16(h + hdr::header_size) != header_size) return LoadError::header_size;
    const std::uint32_t length = rd32(h + hdr::length);
    if (length < header_size || length > image.size()) return LoadError::too_short;
    image = image.first(length);
    if (crc32(image.subspan(hdr::crc + 4)) != rd32(h + hdr::crc)) return LoadError::crc;
    if (rd16(h + hdr::isa_version) != isa_version) return LoadError::isa_version;
    if (rd32(h + hdr::isa_hash) != isa_hash) return LoadError::isa_hash;

    const auto section = [&](std::size_t off_field, std::uint64_t bytes) {
        const std::uint32_t off = rd32(h + off_field);
        return off % 4 == 0 && off >= header_size && inside(off, bytes, length);
    };
    const std::uint32_t code_size = rd32(h + hdr::code_size);
    const std::uint32_t data_size = rd32(h + hdr::data_size);
    const std::uint32_t init_size = rd32(h + hdr::init_size);
    const std::uint32_t task_count = rd32(h + hdr::task_count);
    const std::uint32_t io_count = rd32(h + hdr::io_count);
    const std::uint32_t native_count_ = rd32(h + hdr::native_count);
    if (!section(hdr::code_offset, code_size) || !section(hdr::init_offset, init_size) ||
        !section(hdr::task_offset, std::uint64_t{task_count} * task_record_size) ||
        !section(hdr::io_offset, std::uint64_t{io_count} * io_record_size) ||
        !section(hdr::native_offset, std::uint64_t{native_count_} * native_record_size)) {
        return LoadError::section;
    }
    if (data_size > data_.size()) return LoadError::data_too_large;
    if (code_size == 0) return LoadError::truncated;

    image_ = image;
    code_ = image.subspan(rd32(h + hdr::code_offset), code_size);
    data_size_ = data_size;
    task_offset_ = rd32(h + hdr::task_offset);
    task_count_ = task_count;
    io_offset_ = rd32(h + hdr::io_offset);
    io_count_ = io_count;

    // natives: known, same signature
    natives_listed_ = {};
    const std::uint8_t* nat = h + rd32(h + hdr::native_offset);
    for (std::uint32_t i = 0; i < native_count_; ++i, nat += native_record_size) {
        const std::uint16_t id = rd16(nat);
        if (id >= native_count) return LoadError::native_unknown;
        if (rd32(nat + 4) != native_info[id].signature) return LoadError::native_signature;
        natives_listed_[id] = true;
    }

    // initial values
    const std::uint8_t* in = h + rd32(h + hdr::init_offset);
    for (std::uint32_t pos = 0; pos < init_size;) {
        if (init_size - pos < 8) return LoadError::init_range;
        const std::uint32_t addr = rd32(in + pos);
        const std::uint32_t n = rd32(in + pos + 4);
        const std::uint64_t padded = (std::uint64_t{n} + 3) & ~std::uint64_t{3};
        if (!inside(addr, n, data_size) || padded > init_size - pos - 8) return LoadError::init_range;
        pos += 8 + static_cast<std::uint32_t>(padded);
    }

    // tasks: frame areas inside the data segment, not overlapping; program lists inside the image
    if (task_count == 0 || task_count > max_tasks || task_count > contexts_.size()) return LoadError::task_range;
    for (std::size_t i = 0; i < task_count; ++i) {
        const TaskInfo t = task(i);
        if (t.budget == 0 || t.interval == 0 || !inside(t.frame_base, t.frame_size, data_size)) {
            return LoadError::task_range;
        }
        for (std::size_t j = 0; j < i; ++j) {
            const TaskInfo o = task(j);
            if (t.frame_base < o.frame_base + o.frame_size && o.frame_base < t.frame_base + t.frame_size) {
                return LoadError::task_range;
            }
        }
        if (t.program_count == 0 || t.programs % 4 != 0 || t.programs < header_size ||
            !inside(t.programs, std::uint64_t{t.program_count} * program_record_size, length)) {
            return LoadError::program_range;
        }
        for (std::uint32_t p = 0; p < t.program_count; ++p) {
            if (rd32(h + t.programs + p * program_record_size + 4) > data_size) return LoadError::program_range;
        }
        if (!inside(t.io_first, t.io_count, io_count)) return LoadError::io_range;
    }

    // I/O bindings
    for (std::size_t i = 0; i < io_count; ++i) {
        const IoBinding b = io(i);
        const unsigned bytes = b.bits == 1 ? 1 : b.bits / 8;
        if ((b.bits != 1 && b.bits != 16 && b.bits != 32) || static_cast<unsigned>(b.area) > 1 ||
            static_cast<unsigned>(b.kind) > 3 || !inside(b.addr, bytes, data_size)) {
            return LoadError::io_range;
        }
    }

    if (const LoadError e = verify_code(); e != LoadError::none) return e;

    // the data segment (it was the verifier's scratch)
    std::memset(data_.data(), 0, data_size);
    for (std::uint32_t pos = 0; pos < init_size;) {
        const std::uint32_t addr = rd32(in + pos);
        const std::uint32_t n = rd32(in + pos + 4);
        std::memcpy(data_.data() + addr, in + pos + 8, n);
        pos += 8 + ((n + 3) & ~3u);
    }
    loaded_ = true;
    return LoadError::none;
}

LoadError Vm::verify_code() noexcept {
    const std::uint8_t* code = code_.data();
    const std::uint32_t size = static_cast<std::uint32_t>(code_.size());
    // The instruction starts as a bitmap in the data segment (initialized after the verification):
    // no large buffer on the loading thread's stack.
    const std::size_t bitmap_bytes = (size + 7) / 8;
    if (bitmap_bytes > data_.size()) return LoadError::data_too_large;
    std::uint8_t* starts = data_.data();
    std::memset(starts, 0, bitmap_bytes);
    const auto is_start = [&](std::uint64_t pc) { return pc < size && (starts[pc / 8] >> (pc % 8) & 1) != 0; };

    std::uint32_t last = 0;
    for (std::uint32_t pc = 0; pc < size;) {
        const std::uint8_t b = code[pc];
        const std::uint8_t n = op_size[b];
        if (n == 0) return LoadError::bad_opcode;
        if (n > size - pc) return LoadError::truncated;
        starts[pc / 8] = static_cast<std::uint8_t>(starts[pc / 8] | (1u << (pc % 8)));
        const std::uint8_t* arg = code + pc + 1;
        if (g_width[b] != 0 && !inside(rd32(arg), g_width[b], data_size_)) return LoadError::data_address;
        if (static_cast<Op>(b) == Op::ADDR_G && rd32(arg) > data_size_) return LoadError::data_address;
        if (static_cast<Op>(b) == Op::SYSCALL) {
            const std::uint16_t id = rd16(arg);
            if (id >= native_count || !natives_listed_[id]) return LoadError::native_not_listed;
        }
        last = pc;
        pc += n;
    }
    // no falling off the end of the code
    const Op end = static_cast<Op>(code[last]);
    if (end != Op::RET && end != Op::JMP && end != Op::TRAP) return LoadError::truncated;

    const auto is_enter = [&](std::uint64_t pc) { return is_start(pc) && static_cast<Op>(code[pc]) == Op::ENTER; };
    for (std::uint32_t pc = 0; pc < size; pc += op_size[code[pc]]) {
        const Op o = static_cast<Op>(code[pc]);
        const std::uint8_t* arg = code + pc + 1;
        if (o == Op::JMP || o == Op::JZ || o == Op::JNZ) {
            const std::int64_t target = std::int64_t{pc} + op_size[code[pc]] + static_cast<std::int32_t>(rd32(arg));
            if (target < 0 || !is_start(static_cast<std::uint64_t>(target))) return LoadError::jump_target;
        } else if ((o == Op::CALL || o == Op::CALL_FB) && !is_enter(rd32(arg))) {
            return LoadError::call_target;
        }
    }
    for (std::size_t i = 0; i < task_count_; ++i) {
        const TaskInfo t = task(i);
        for (std::uint32_t p = 0; p < t.program_count; ++p) {
            if (!is_enter(rd32(&image_[t.programs + p * program_record_size]))) return LoadError::call_target;
        }
    }
    return LoadError::none;
}

TaskInfo Vm::task(std::size_t i) const noexcept {
    const std::uint8_t* p = &image_[task_offset_ + i * task_record_size];
    return {rd16(p), p[2], p[3], rd32(p + 4), rd32(p + 8), rd32(p + 12), rd32(p + 16), rd32(p + 20), rd32(p + 24),
            rd32(p + 28)};
}

IoBinding Vm::io(std::size_t i) const noexcept {
    const std::uint8_t* p = &image_[io_offset_ + i * io_record_size];
    return {rd16(p), static_cast<IoArea>(p[2]), static_cast<IoKind>(p[3]), p[4], p[5], rd32(p + 8)};
}

// ---- execution ------------------------------------------------------------------------------------

RunResult Vm::run_cycle(std::size_t task_index, std::uint32_t now_ms) noexcept {
    contract_assert(loaded_ && task_index < task_count_);
    const TaskInfo t = task(task_index);
    Context& c = contexts_[task_index];
    c.now_ms = now_ms;
    c.frame_top = t.frame_base;
    c.frame_end = t.frame_base + t.frame_size;
    RunResult total;
    std::uint32_t remaining = t.budget;
    for (std::uint32_t p = 0; p < t.program_count; ++p) {
        const std::uint8_t* rec = &image_[t.programs + p * program_record_size];
        c.sp = 0;
        c.pc = rd32(rec);
        c.self = rd32(rec + 4);
        c.fp = c.frame_top;
        c.calls[0] = {Context::no_return, c.fp, c.self};
        c.depth = 1;
        const RunResult r = execute(c, remaining);
        total.instructions += r.instructions;
        if (r.state == RunState::trapped) {
            total.state = r.state;
            total.trap = r.trap;
            total.trap_code = r.trap_code;
            total.pc = r.pc;
            return total;
        }
        remaining -= r.instructions;
    }
    return total;
}

RunResult Vm::execute(Context& c, std::uint32_t budget) noexcept {
    const std::uint8_t* const code = code_.data();
    std::uint8_t* const data = data_.data();
    const std::uint64_t dsize = data_size_;
    std::uint64_t* const st = c.stack.data();
    std::uint32_t sp = c.sp;
    std::uint32_t pc = c.pc;
    std::uint32_t at = pc;  // the current instruction
    std::uint32_t count = 0;
    RunResult r;

#define PLC_TRAP(t)        \
    do {                   \
        r.trap = (t);      \
        goto trapped;      \
    } while (false)
#define PLC_NEED(k) \
    if (sp < (k)) PLC_TRAP(Trap::STACK_UNDERFLOW)
#define PLC_ROOM(k) \
    if (sp + (k) > operand_stack_slots) PLC_TRAP(Trap::STACK_OVERFLOW)
#define PLC_ADDR(a, w) \
    if (!inside((a), (w), dsize)) PLC_TRAP(Trap::MEMORY)
#define PLC_BINARY(OPC, EXPR)                  \
    case Op::OPC: {                            \
        PLC_NEED(2);                           \
        const std::uint64_t b = st[--sp];      \
        const std::uint64_t a = st[sp - 1];    \
        st[sp - 1] = (EXPR);                   \
        break;                                 \
    }
#define PLC_UNARY(OPC, EXPR)                \
    case Op::OPC: {                         \
        PLC_NEED(1);                        \
        const std::uint64_t a = st[sp - 1]; \
        st[sp - 1] = (EXPR);                \
        break;                              \
    }
#define PLC_LOADS(M, ADDR)                                                                               \
    case Op::LD##M##_U8: { PLC_ROOM(1); const std::uint64_t ad = (ADDR); PLC_ADDR(ad, 1); st[sp++] = data[ad]; break; } \
    case Op::LD##M##_S8: { PLC_ROOM(1); const std::uint64_t ad = (ADDR); PLC_ADDR(ad, 1);                   \
        st[sp++] = ofs32(static_cast<std::int8_t>(data[ad])); break; }                                    \
    case Op::LD##M##_U16: { PLC_ROOM(1); const std::uint64_t ad = (ADDR); PLC_ADDR(ad, 2); st[sp++] = rd16(data + ad); break; } \
    case Op::LD##M##_S16: { PLC_ROOM(1); const std::uint64_t ad = (ADDR); PLC_ADDR(ad, 2);                  \
        st[sp++] = ofs32(static_cast<std::int16_t>(rd16(data + ad))); break; }                            \
    case Op::LD##M##_32: { PLC_ROOM(1); const std::uint64_t ad = (ADDR); PLC_ADDR(ad, 4); st[sp++] = rd32(data + ad); break; } \
    case Op::LD##M##_64: { PLC_ROOM(1); const std::uint64_t ad = (ADDR); PLC_ADDR(ad, 8); st[sp++] = rd64(data + ad); break; }
#define PLC_STORES(M, ADDR)                                                                                \
    case Op::ST##M##_8: { PLC_NEED(1); const std::uint64_t ad = (ADDR); PLC_ADDR(ad, 1);                     \
        data[ad] = static_cast<std::uint8_t>(st[--sp]); break; }                                            \
    case Op::ST##M##_16: { PLC_NEED(1); const std::uint64_t ad = (ADDR); PLC_ADDR(ad, 2);                    \
        const std::uint16_t v = static_cast<std::uint16_t>(st[--sp]); std::memcpy(data + ad, &v, 2); break; } \
    case Op::ST##M##_32: { PLC_NEED(1); const std::uint64_t ad = (ADDR); PLC_ADDR(ad, 4);                    \
        const std::uint32_t v = u32(st[--sp]); std::memcpy(data + ad, &v, 4); break; }                       \
    case Op::ST##M##_64: { PLC_NEED(1); const std::uint64_t ad = (ADDR); PLC_ADDR(ad, 8);                    \
        const std::uint64_t v = st[--sp]; std::memcpy(data + ad, &v, 8); break; }

    for (;;) {
        at = pc;
        if (count == budget) PLC_TRAP(Trap::BUDGET);
        ++count;
        const std::uint8_t opcode = code[pc];
        const std::uint8_t* const arg = code + pc + 1;
        pc += op_size[opcode];
        switch (static_cast<Op>(opcode)) {
            case Op::NOP: break;
            case Op::PUSH_I8: PLC_ROOM(1); st[sp++] = ofs32(static_cast<std::int8_t>(arg[0])); break;
            case Op::PUSH_I32: PLC_ROOM(1); st[sp++] = rd32(arg); break;
            case Op::PUSH_I64: PLC_ROOM(1); st[sp++] = rd64(arg); break;
            case Op::DUP: PLC_NEED(1); PLC_ROOM(1); st[sp] = st[sp - 1]; ++sp; break;
            case Op::DROP: PLC_NEED(1); --sp; break;
            case Op::SWAP: { PLC_NEED(2); const std::uint64_t t = st[sp - 1]; st[sp - 1] = st[sp - 2]; st[sp - 2] = t; break; }

            PLC_LOADS(G, rd32(arg))
            PLC_LOADS(L, std::uint64_t{c.fp} + rd16(arg))
            PLC_LOADS(T, std::uint64_t{c.self} + rd16(arg))
            PLC_STORES(G, rd32(arg))
            PLC_STORES(L, std::uint64_t{c.fp} + rd16(arg))
            PLC_STORES(T, std::uint64_t{c.self} + rd16(arg))

            // indirect: the address is popped first (loads), or lies below the value (stores)
            case Op::LDI_U8: { PLC_NEED(1); const std::uint64_t ad = u32(st[sp - 1]); PLC_ADDR(ad, 1); st[sp - 1] = data[ad]; break; }
            case Op::LDI_S8: { PLC_NEED(1); const std::uint64_t ad = u32(st[sp - 1]); PLC_ADDR(ad, 1);
                st[sp - 1] = ofs32(static_cast<std::int8_t>(data[ad])); break; }
            case Op::LDI_U16: { PLC_NEED(1); const std::uint64_t ad = u32(st[sp - 1]); PLC_ADDR(ad, 2); st[sp - 1] = rd16(data + ad); break; }
            case Op::LDI_S16: { PLC_NEED(1); const std::uint64_t ad = u32(st[sp - 1]); PLC_ADDR(ad, 2);
                st[sp - 1] = ofs32(static_cast<std::int16_t>(rd16(data + ad))); break; }
            case Op::LDI_32: { PLC_NEED(1); const std::uint64_t ad = u32(st[sp - 1]); PLC_ADDR(ad, 4); st[sp - 1] = rd32(data + ad); break; }
            case Op::LDI_64: { PLC_NEED(1); const std::uint64_t ad = u32(st[sp - 1]); PLC_ADDR(ad, 8); st[sp - 1] = rd64(data + ad); break; }
            case Op::STI_8: { PLC_NEED(2); const std::uint64_t ad = u32(st[sp - 2]); PLC_ADDR(ad, 1);
                data[ad] = static_cast<std::uint8_t>(st[sp - 1]); sp -= 2; break; }
            case Op::STI_16: { PLC_NEED(2); const std::uint64_t ad = u32(st[sp - 2]); PLC_ADDR(ad, 2);
                const std::uint16_t v = static_cast<std::uint16_t>(st[sp - 1]); std::memcpy(data + ad, &v, 2); sp -= 2; break; }
            case Op::STI_32: { PLC_NEED(2); const std::uint64_t ad = u32(st[sp - 2]); PLC_ADDR(ad, 4);
                const std::uint32_t v = u32(st[sp - 1]); std::memcpy(data + ad, &v, 4); sp -= 2; break; }
            case Op::STI_64: { PLC_NEED(2); const std::uint64_t ad = u32(st[sp - 2]); PLC_ADDR(ad, 8);
                const std::uint64_t v = st[sp - 1]; std::memcpy(data + ad, &v, 8); sp -= 2; break; }
            case Op::ADDR_G: PLC_ROOM(1); st[sp++] = rd32(arg); break;
            case Op::ADDR_L: PLC_ROOM(1); st[sp++] = of32(c.fp + rd16(arg)); break;
            case Op::ADDR_T: PLC_ROOM(1); st[sp++] = of32(c.self + rd16(arg)); break;
            case Op::MEMCPY: {
                PLC_NEED(2);
                const std::uint32_t n = rd32(arg);
                const std::uint64_t src = u32(st[sp - 1]);
                const std::uint64_t dst = u32(st[sp - 2]);
                PLC_ADDR(src, n);
                PLC_ADDR(dst, n);
                std::memmove(data + dst, data + src, n);
                sp -= 2;
                break;
            }

            // ---- i32 ----
            PLC_BINARY(ADD_I32, of32(u32(a) + u32(b)))
            PLC_BINARY(SUB_I32, of32(u32(a) - u32(b)))
            PLC_BINARY(MUL_I32, of32(u32(a) * u32(b)))
            case Op::DIVS_I32: {
                PLC_NEED(2);
                const std::int32_t b = s32(st[sp - 1]), a = s32(st[sp - 2]);
                if (b == 0) PLC_TRAP(Trap::DIV_BY_ZERO);
                st[sp - 2] = ofs32(b == -1 ? static_cast<std::int32_t>(0u - static_cast<std::uint32_t>(a)) : a / b);
                --sp;
                break;
            }
            case Op::DIVU_I32: {
                PLC_NEED(2);
                const std::uint32_t b = u32(st[sp - 1]), a = u32(st[sp - 2]);
                if (b == 0) PLC_TRAP(Trap::DIV_BY_ZERO);
                st[sp - 2] = of32(a / b);
                --sp;
                break;
            }
            case Op::MODS_I32: {
                PLC_NEED(2);
                const std::int32_t b = s32(st[sp - 1]), a = s32(st[sp - 2]);
                if (b == 0) PLC_TRAP(Trap::DIV_BY_ZERO);
                st[sp - 2] = ofs32(b == -1 ? 0 : a % b);
                --sp;
                break;
            }
            case Op::MODU_I32: {
                PLC_NEED(2);
                const std::uint32_t b = u32(st[sp - 1]), a = u32(st[sp - 2]);
                if (b == 0) PLC_TRAP(Trap::DIV_BY_ZERO);
                st[sp - 2] = of32(a % b);
                --sp;
                break;
            }
            PLC_UNARY(NEG_I32, of32(0u - u32(a)))
            PLC_BINARY(AND_I32, of32(u32(a) & u32(b)))
            PLC_BINARY(OR_I32, of32(u32(a) | u32(b)))
            PLC_BINARY(XOR_I32, of32(u32(a) ^ u32(b)))
            PLC_UNARY(NOT_I32, of32(~u32(a)))
            PLC_BINARY(SHL_I32, of32(u32(b) >= 32 ? 0u : u32(a) << u32(b)))
            PLC_BINARY(SHR_I32, of32(u32(b) >= 32 ? 0u : u32(a) >> u32(b)))

            // ---- i64 ----
            PLC_BINARY(ADD_I64, a + b)
            PLC_BINARY(SUB_I64, a - b)
            PLC_BINARY(MUL_I64, a * b)
            case Op::DIVS_I64: {
                PLC_NEED(2);
                const std::int64_t b = s64(st[sp - 1]), a = s64(st[sp - 2]);
                if (b == 0) PLC_TRAP(Trap::DIV_BY_ZERO);
                st[sp - 2] = b == -1 ? 0u - st[sp - 2] : ofs64(a / b);
                --sp;
                break;
            }
            case Op::DIVU_I64: {
                PLC_NEED(2);
                const std::uint64_t b = st[sp - 1], a = st[sp - 2];
                if (b == 0) PLC_TRAP(Trap::DIV_BY_ZERO);
                st[sp - 2] = a / b;
                --sp;
                break;
            }
            case Op::MODS_I64: {
                PLC_NEED(2);
                const std::int64_t b = s64(st[sp - 1]), a = s64(st[sp - 2]);
                if (b == 0) PLC_TRAP(Trap::DIV_BY_ZERO);
                st[sp - 2] = b == -1 ? 0u : ofs64(a % b);
                --sp;
                break;
            }
            case Op::MODU_I64: {
                PLC_NEED(2);
                const std::uint64_t b = st[sp - 1], a = st[sp - 2];
                if (b == 0) PLC_TRAP(Trap::DIV_BY_ZERO);
                st[sp - 2] = a % b;
                --sp;
                break;
            }
            PLC_UNARY(NEG_I64, 0u - a)
            PLC_BINARY(AND_I64, a & b)
            PLC_BINARY(OR_I64, a | b)
            PLC_BINARY(XOR_I64, a ^ b)
            PLC_UNARY(NOT_I64, ~a)
            PLC_BINARY(SHL_I64, b >= 64 ? 0u : a << b)
            PLC_BINARY(SHR_I64, b >= 64 ? 0u : a >> b)

            // ---- f32, f64 ----
            PLC_BINARY(ADD_F32, off32(f32(a) + f32(b)))
            PLC_BINARY(SUB_F32, off32(f32(a) - f32(b)))
            PLC_BINARY(MUL_F32, off32(f32(a) * f32(b)))
            case Op::DIV_F32: {
                PLC_NEED(2);
                const float b = f32(st[sp - 1]);
                if (b == 0.0f) PLC_TRAP(Trap::DIV_BY_ZERO);
                st[sp - 2] = off32(f32(st[sp - 2]) / b);
                --sp;
                break;
            }
            PLC_UNARY(NEG_F32, off32(-f32(a)))
            PLC_BINARY(ADD_F64, off64(f64(a) + f64(b)))
            PLC_BINARY(SUB_F64, off64(f64(a) - f64(b)))
            PLC_BINARY(MUL_F64, off64(f64(a) * f64(b)))
            case Op::DIV_F64: {
                PLC_NEED(2);
                const double b = f64(st[sp - 1]);
                if (b == 0.0) PLC_TRAP(Trap::DIV_BY_ZERO);
                st[sp - 2] = off64(f64(st[sp - 2]) / b);
                --sp;
                break;
            }
            PLC_UNARY(NEG_F64, off64(-f64(a)))

            // ---- comparisons ----
            PLC_BINARY(EQ_I32, ofb(u32(a) == u32(b)))
            PLC_BINARY(NE_I32, ofb(u32(a) != u32(b)))
            PLC_BINARY(LTS_I32, ofb(s32(a) < s32(b)))
            PLC_BINARY(LTU_I32, ofb(u32(a) < u32(b)))
            PLC_BINARY(LES_I32, ofb(s32(a) <= s32(b)))
            PLC_BINARY(LEU_I32, ofb(u32(a) <= u32(b)))
            PLC_BINARY(GTS_I32, ofb(s32(a) > s32(b)))
            PLC_BINARY(GTU_I32, ofb(u32(a) > u32(b)))
            PLC_BINARY(GES_I32, ofb(s32(a) >= s32(b)))
            PLC_BINARY(GEU_I32, ofb(u32(a) >= u32(b)))
            PLC_BINARY(EQ_I64, ofb(a == b))
            PLC_BINARY(NE_I64, ofb(a != b))
            PLC_BINARY(LTS_I64, ofb(s64(a) < s64(b)))
            PLC_BINARY(LTU_I64, ofb(a < b))
            PLC_BINARY(LES_I64, ofb(s64(a) <= s64(b)))
            PLC_BINARY(LEU_I64, ofb(a <= b))
            PLC_BINARY(GTS_I64, ofb(s64(a) > s64(b)))
            PLC_BINARY(GTU_I64, ofb(a > b))
            PLC_BINARY(GES_I64, ofb(s64(a) >= s64(b)))
            PLC_BINARY(GEU_I64, ofb(a >= b))
            PLC_BINARY(EQ_F32, ofb(f32(a) == f32(b)))
            PLC_BINARY(NE_F32, ofb(f32(a) != f32(b)))
            PLC_BINARY(LT_F32, ofb(f32(a) < f32(b)))
            PLC_BINARY(LE_F32, ofb(f32(a) <= f32(b)))
            PLC_BINARY(GT_F32, ofb(f32(a) > f32(b)))
            PLC_BINARY(GE_F32, ofb(f32(a) >= f32(b)))
            PLC_BINARY(EQ_F64, ofb(f64(a) == f64(b)))
            PLC_BINARY(NE_F64, ofb(f64(a) != f64(b)))
            PLC_BINARY(LT_F64, ofb(f64(a) < f64(b)))
            PLC_BINARY(LE_F64, ofb(f64(a) <= f64(b)))
            PLC_BINARY(GT_F64, ofb(f64(a) > f64(b)))
            PLC_BINARY(GE_F64, ofb(f64(a) >= f64(b)))

            // ---- normalization, conversion ----
            PLC_UNARY(SEXT8, ofs32(static_cast<std::int8_t>(a)))
            PLC_UNARY(SEXT16, ofs32(static_cast<std::int16_t>(a)))
            PLC_UNARY(ZEXT8, a & 0xFFu)
            PLC_UNARY(ZEXT16, a & 0xFFFFu)
            PLC_UNARY(I32S_TO_I64, ofs64(s32(a)))
            PLC_UNARY(I32U_TO_I64, u32(a))
            PLC_UNARY(I64_TO_I32, u32(a))
            PLC_UNARY(I32S_TO_F32, off32(static_cast<float>(s32(a))))
            PLC_UNARY(I32S_TO_F64, off64(static_cast<double>(s32(a))))
            PLC_UNARY(I32U_TO_F32, off32(static_cast<float>(u32(a))))
            PLC_UNARY(I32U_TO_F64, off64(static_cast<double>(u32(a))))
            PLC_UNARY(I64S_TO_F32, off32(static_cast<float>(s64(a))))
            PLC_UNARY(I64S_TO_F64, off64(static_cast<double>(s64(a))))
            PLC_UNARY(I64U_TO_F32, off32(static_cast<float>(a)))
            PLC_UNARY(I64U_TO_F64, off64(static_cast<double>(a)))
            PLC_UNARY(F32_TO_F64, off64(static_cast<double>(f32(a))))
            PLC_UNARY(F64_TO_F32, off32(static_cast<float>(f64(a))))
            case Op::F32_ROUND_I64:
            case Op::F32_TRUNC_I64:
            case Op::F64_ROUND_I64:
            case Op::F64_TRUNC_I64: {
                PLC_NEED(1);
                const auto o = static_cast<Op>(opcode);
                const double x = (o == Op::F32_ROUND_I64 || o == Op::F32_TRUNC_I64) ? f32(st[sp - 1]) : f64(st[sp - 1]);
                // [-2^63, 2^63): every double near 2^63 is an integer, so rounding stays inside
                if (!(x >= -0x1p63 && x < 0x1p63)) PLC_TRAP(Trap::CONVERSION);
                const double v = (o == Op::F32_ROUND_I64 || o == Op::F64_ROUND_I64) ? std::round(x) : std::trunc(x);
                st[sp - 1] = ofs64(static_cast<std::int64_t>(v));
                break;
            }
            case Op::CHK_I32: {
                PLC_NEED(1);
                const std::int32_t v = s32(st[sp - 1]);
                if (v < static_cast<std::int32_t>(rd32(arg)) || v > static_cast<std::int32_t>(rd32(arg + 4))) PLC_TRAP(Trap::RANGE);
                break;
            }
            case Op::CHK_U32: {
                PLC_NEED(1);
                const std::uint32_t v = u32(st[sp - 1]);
                if (v < rd32(arg) || v > rd32(arg + 4)) PLC_TRAP(Trap::RANGE);
                break;
            }
            case Op::CHK_I64: {
                PLC_NEED(1);
                const std::int64_t v = s64(st[sp - 1]);
                if (v < static_cast<std::int64_t>(rd64(arg)) || v > static_cast<std::int64_t>(rd64(arg + 8))) PLC_TRAP(Trap::RANGE);
                break;
            }
            case Op::CHK_U64: {
                PLC_NEED(1);
                const std::uint64_t v = st[sp - 1];
                if (v < rd64(arg) || v > rd64(arg + 8)) PLC_TRAP(Trap::RANGE);
                break;
            }

            // ---- control ----
            case Op::JMP: pc = static_cast<std::uint32_t>(std::int64_t{pc} + static_cast<std::int32_t>(rd32(arg))); break;
            case Op::JZ:
                PLC_NEED(1);
                if (u32(st[--sp]) == 0) pc = static_cast<std::uint32_t>(std::int64_t{pc} + static_cast<std::int32_t>(rd32(arg)));
                break;
            case Op::JNZ:
                PLC_NEED(1);
                if (u32(st[--sp]) != 0) pc = static_cast<std::uint32_t>(std::int64_t{pc} + static_cast<std::int32_t>(rd32(arg)));
                break;
            case Op::CALL:
            case Op::CALL_FB: {
                if (c.depth == call_depth) PLC_TRAP(Trap::CALL_DEPTH);
                std::uint32_t self = c.self;
                if (static_cast<Op>(opcode) == Op::CALL_FB) {
                    PLC_NEED(1);
                    self = u32(st[--sp]);
                }
                c.calls[c.depth++] = {pc, c.fp, c.self};
                c.self = self;
                pc = rd32(arg);
                break;
            }
            case Op::ENTER: {
                const std::uint32_t frame = rd32(arg);
                if (std::uint64_t{c.frame_top} + frame > c.frame_end) PLC_TRAP(Trap::FRAME_OVERFLOW);
                std::memset(data + c.frame_top, 0, frame);
                c.fp = c.frame_top;
                c.frame_top += frame;
                break;
            }
            case Op::RET: {
                if (c.depth == 0) PLC_TRAP(Trap::STACK_UNDERFLOW);
                const Context::Call k = c.calls[--c.depth];
                c.frame_top = c.fp;
                c.fp = k.fp;
                c.self = k.self;
                if (k.return_pc == Context::no_return) {
                    c.sp = sp;
                    c.pc = pc;
                    r.instructions = count;
                    return r;
                }
                pc = k.return_pc;
                break;
            }
            case Op::TRAP:
                r.trap_code = rd16(arg);
                PLC_TRAP(Trap::USER);
            case Op::SYSCALL: {
                const std::uint16_t id = rd16(arg);
                const NativeInfo info = native_info[id];
                PLC_NEED(info.args);
                if (info.results > info.args) PLC_ROOM(info.results - info.args);
                std::uint64_t* const p = st + sp - info.args;  // the arguments, the results replace them
                switch (static_cast<Native>(id)) {
                    case Native::TIME_MS: p[0] = c.now_ms; break;
                    case Native::ABS_I32: p[0] = of32(s32(p[0]) < 0 ? 0u - u32(p[0]) : u32(p[0])); break;
                    case Native::ABS_I64: p[0] = s64(p[0]) < 0 ? 0u - p[0] : p[0]; break;
                    case Native::ABS_F32: p[0] = off32(std::fabs(f32(p[0]))); break;
                    case Native::ABS_F64: p[0] = off64(std::fabs(f64(p[0]))); break;
                    case Native::SQRT_F32: p[0] = off32(std::sqrt(f32(p[0]))); break;
                    case Native::SQRT_F64: p[0] = off64(std::sqrt(f64(p[0]))); break;
                    case Native::ROL_8: p[0] = rotl(u32(p[0]), u32(p[1]), 8); break;
                    case Native::ROL_16: p[0] = rotl(u32(p[0]), u32(p[1]), 16); break;
                    case Native::ROL_32: p[0] = rotl(u32(p[0]), u32(p[1]), 32); break;
                    case Native::ROL_64: p[0] = std::rotl(p[0], static_cast<int>(u32(p[1]) % 64)); break;
                    case Native::ROR_8: p[0] = rotr(u32(p[0]), u32(p[1]), 8); break;
                    case Native::ROR_16: p[0] = rotr(u32(p[0]), u32(p[1]), 16); break;
                    case Native::ROR_32: p[0] = rotr(u32(p[0]), u32(p[1]), 32); break;
                    case Native::ROR_64: p[0] = std::rotr(p[0], static_cast<int>(u32(p[1]) % 64)); break;
                    case Native::LN_F64: p[0] = off64(std::log(f64(p[0]))); break;
                    case Native::LOG_F64: p[0] = off64(std::log10(f64(p[0]))); break;
                    case Native::EXP_F64: p[0] = off64(std::exp(f64(p[0]))); break;
                    case Native::EXPT_F64: p[0] = off64(std::pow(f64(p[0]), f64(p[1]))); break;
                    case Native::SIN_F64: p[0] = off64(std::sin(f64(p[0]))); break;
                    case Native::COS_F64: p[0] = off64(std::cos(f64(p[0]))); break;
                    case Native::TAN_F64: p[0] = off64(std::tan(f64(p[0]))); break;
                    case Native::ASIN_F64: p[0] = off64(std::asin(f64(p[0]))); break;
                    case Native::ACOS_F64: p[0] = off64(std::acos(f64(p[0]))); break;
                    case Native::ATAN_F64: p[0] = off64(std::atan(f64(p[0]))); break;
                }
                sp = sp - info.args + info.results;
                break;
            }
            default: PLC_TRAP(Trap::USER);  // unreachable: the verifier accepts known opcodes only
        }
    }

trapped:
    r.state = RunState::trapped;
    r.pc = at;
    r.instructions = count;
    c.sp = sp;
    c.pc = at;
    return r;

#undef PLC_TRAP
#undef PLC_NEED
#undef PLC_ROOM
#undef PLC_ADDR
#undef PLC_BINARY
#undef PLC_UNARY
#undef PLC_LOADS
#undef PLC_STORES
}

}  // namespace rtos::plc
