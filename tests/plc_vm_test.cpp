// PLC bytecode VM (rtos/plc/vm.hpp, docs/plc.md): semantics of the instructions, traps, calls, and
// the verifier. The images are assembled here byte by byte - independent of the TypeScript compiler.
#include "rtos/plc/vm.hpp"

#include <gtest/gtest.h>

#include <bit>
#include <cstring>
#include <limits>
#include <vector>

namespace {

using namespace rtos::plc;

// ---- a minimal assembler and image builder --------------------------------------------------------

struct Asm {
    std::vector<std::uint8_t> code;

    std::uint32_t here() const { return static_cast<std::uint32_t>(code.size()); }
    Asm& op(Op o) {
        code.push_back(static_cast<std::uint8_t>(o));
        return *this;
    }
    Asm& i8(std::int8_t v) {
        code.push_back(static_cast<std::uint8_t>(v));
        return *this;
    }
    Asm& u16(std::uint16_t v) { return raw(&v, 2); }
    Asm& u32(std::uint32_t v) { return raw(&v, 4); }
    Asm& u64(std::uint64_t v) { return raw(&v, 8); }
    Asm& raw(const void* p, std::size_t n) {
        const auto* b = static_cast<const std::uint8_t*>(p);
        code.insert(code.end(), b, b + n);
        return *this;
    }
    // common forms
    Asm& push(std::int32_t v) { return op(Op::PUSH_I32).u32(static_cast<std::uint32_t>(v)); }
    Asm& push64(std::uint64_t v) { return op(Op::PUSH_I64).u64(v); }
    Asm& pushf(float f) { return op(Op::PUSH_I32).u32(std::bit_cast<std::uint32_t>(f)); }
    Asm& pushd(double d) { return op(Op::PUSH_I64).u64(std::bit_cast<std::uint64_t>(d)); }
    Asm& enter(std::uint32_t frame) { return op(Op::ENTER).u32(frame); }
    Asm& stg32(std::uint32_t a) { return op(Op::STG_32).u32(a); }
    Asm& stg64(std::uint32_t a) { return op(Op::STG_64).u32(a); }
    Asm& ldg32(std::uint32_t a) { return op(Op::LDG_32).u32(a); }
    Asm& sys(Native n) { return op(Op::SYSCALL).u16(static_cast<std::uint16_t>(n)); }
    /// A jump; returns the operand's position for bind().
    std::uint32_t jump(Op o) {
        op(o);
        const std::uint32_t at = here();
        u32(0);
        return at;
    }
    void bind(std::uint32_t operand) { bind_to(operand, here()); }
    void bind_to(std::uint32_t operand, std::uint32_t target) {
        const std::int32_t rel = static_cast<std::int32_t>(target) - static_cast<std::int32_t>(operand + 4);
        std::memcpy(&code[operand], &rel, 4);
    }
};

struct Builder {
    std::vector<std::uint8_t> code;
    std::uint32_t data_size = 256;
    std::vector<std::pair<std::uint32_t, std::vector<std::uint8_t>>> init;
    struct Task {
        std::uint32_t budget = 10'000;
        std::uint32_t frame_base = 128;
        std::uint32_t frame_size = 128;
        std::vector<ProgramInfo> programs{{0, 0}};
    };
    std::vector<Task> tasks{Task{}};
    std::vector<Native> natives;
    std::uint16_t isa = isa_version;
    std::uint32_t hash = isa_hash;

    std::vector<std::uint8_t> build() const {
        std::vector<std::uint8_t> img(header_size, 0);
        auto align = [&] {
            while (img.size() % 4 != 0) img.push_back(0);
        };
        auto put32 = [&](std::size_t at, std::uint32_t v) { std::memcpy(&img[at], &v, 4); };
        auto app = [&](const void* p, std::size_t n) {
            const auto* b = static_cast<const std::uint8_t*>(p);
            img.insert(img.end(), b, b + n);
        };
        auto app32 = [&](std::uint32_t v) { app(&v, 4); };
        auto app16 = [&](std::uint16_t v) { app(&v, 2); };

        put32(hdr::code_offset, static_cast<std::uint32_t>(img.size()));
        put32(hdr::code_size, static_cast<std::uint32_t>(code.size()));
        app(code.data(), code.size());
        align();
        put32(hdr::data_size, data_size);
        const std::size_t init_start = img.size();
        put32(hdr::init_offset, static_cast<std::uint32_t>(init_start));
        for (const auto& [addr, bytes] : init) {
            app32(addr);
            app32(static_cast<std::uint32_t>(bytes.size()));
            app(bytes.data(), bytes.size());
            align();
        }
        put32(hdr::init_size, static_cast<std::uint32_t>(img.size() - init_start));
        // program lists first, then the task records that point at them
        std::vector<std::uint32_t> lists;
        for (const Task& t : tasks) {
            lists.push_back(static_cast<std::uint32_t>(img.size()));
            for (const ProgramInfo& p : t.programs) {
                app32(p.entry);
                app32(p.instance);
            }
        }
        put32(hdr::task_offset, static_cast<std::uint32_t>(img.size()));
        put32(hdr::task_count, static_cast<std::uint32_t>(tasks.size()));
        for (std::size_t i = 0; i < tasks.size(); ++i) {
            const Task& t = tasks[i];
            app16(1);      // interval
            img.push_back(0);  // priority
            img.push_back(3);  // max overruns
            app32(t.budget);
            app32(t.frame_base);
            app32(t.frame_size);
            app32(lists[i]);
            app32(static_cast<std::uint32_t>(t.programs.size()));
            app32(0);
            app32(0);
        }
        put32(hdr::io_offset, static_cast<std::uint32_t>(img.size()));
        put32(hdr::native_offset, static_cast<std::uint32_t>(img.size()));
        put32(hdr::native_count, static_cast<std::uint32_t>(natives.size()));
        for (const Native n : natives) {
            app16(static_cast<std::uint16_t>(n));
            app16(0);
            app32(native_info[static_cast<std::size_t>(n)].signature);
        }
        std::memcpy(img.data(), image_magic.data(), image_magic.size());
        std::uint16_t v = isa;
        std::memcpy(&img[hdr::isa_version], &v, 2);
        v = header_size;
        std::memcpy(&img[hdr::header_size], &v, 2);
        put32(hdr::length, static_cast<std::uint32_t>(img.size()));
        put32(hdr::isa_hash, hash);
        put32(hdr::crc, crc32(std::span(img).subspan(hdr::crc + 4)));
        return img;
    }
};

struct Machine {
    std::array<std::uint8_t, 1024> data{};
    std::array<Context, 2> contexts{};
    Vm vm{data, contexts};
    std::vector<std::uint8_t> image;

    LoadError load(const Builder& b) {
        image = b.build();
        return vm.load(image);
    }
    std::uint32_t g32(std::uint32_t a) const { return rd32(&data[a]); }
    std::uint64_t g64(std::uint32_t a) const { return rd64(&data[a]); }
    float gf(std::uint32_t a) const { return std::bit_cast<float>(g32(a)); }
    double gd(std::uint32_t a) const { return std::bit_cast<double>(g64(a)); }
};

/// One program: ENTER 0, the body, RET.
Builder program(const Asm& body, std::vector<Native> natives = {}) {
    Asm a;
    a.enter(0);
    a.code.insert(a.code.end(), body.code.begin(), body.code.end());
    a.op(Op::RET);
    Builder b;
    b.code = a.code;
    b.natives = std::move(natives);
    return b;
}

/// Runs `body` once; it may store results to the globals 0..127.
RunResult run(Machine& m, const Asm& body, std::vector<Native> natives = {}) {
    EXPECT_EQ(m.load(program(body, std::move(natives))), LoadError::none);
    return m.vm.run_cycle(0, 1234);
}

// ---- integer arithmetic ----------------------------------------------------------------------------

TEST(PlcVm, RunsAProgramAndKeepsGlobalsAcrossCycles) {
    Machine m;
    Asm a;
    a.ldg32(0).push(1).op(Op::ADD_I32).stg32(0);  // counter := counter + 1
    ASSERT_EQ(m.load(program(a)), LoadError::none);
    for (int i = 0; i < 5; ++i) EXPECT_EQ(m.vm.run_cycle(0, 0).state, RunState::done);
    EXPECT_EQ(m.g32(0), 5u);
}

TEST(PlcVm, InitialValuesAndZeroedDataSegment) {
    Machine m;
    m.data.fill(0xAA);
    Builder b = program(Asm{});
    b.init = {{8, {1, 2, 3}}, {16, {0x78, 0x56, 0x34, 0x12}}};
    ASSERT_EQ(m.load(b), LoadError::none);
    EXPECT_EQ(m.data[0], 0);
    EXPECT_EQ(m.data[8], 1);
    EXPECT_EQ(m.data[10], 3);
    EXPECT_EQ(m.data[11], 0);
    EXPECT_EQ(m.g32(16), 0x12345678u);
    EXPECT_EQ(m.data[255], 0);
    EXPECT_EQ(m.data[256], 0xAA);  // beyond the image's data segment: untouched
}

TEST(PlcVm, IntegerOverflowWrapsAndNarrowTypesNormalize) {
    Machine m;
    Asm a;
    a.push(0x7FFFFFFF).push(1).op(Op::ADD_I32).stg32(0);           // DINT: wraps to MIN
    a.push(127).push(1).op(Op::ADD_I32).op(Op::SEXT8).stg32(4);     // SINT 127 + 1 = -128
    a.push(255).push(1).op(Op::ADD_I32).op(Op::ZEXT8).stg32(8);     // USINT 255 + 1 = 0
    a.push(-32768).op(Op::NEG_I32).op(Op::SEXT16).stg32(12);        // INT -(-32768) = -32768
    a.push64(0x7FFFFFFFFFFFFFFF).push64(1).op(Op::ADD_I64).stg64(16);
    EXPECT_EQ(run(m, a).state, RunState::done);
    EXPECT_EQ(m.g32(0), 0x80000000u);
    EXPECT_EQ(static_cast<std::int32_t>(m.g32(4)), -128);
    EXPECT_EQ(m.g32(8), 0u);
    EXPECT_EQ(static_cast<std::int32_t>(m.g32(12)), -32768);
    EXPECT_EQ(m.g64(16), 0x8000000000000000u);
}

TEST(PlcVm, DivisionAndModulo) {
    Machine m;
    Asm a;
    a.push(-7).push(2).op(Op::DIVS_I32).stg32(0);                   // toward zero: -3
    a.push(-7).push(2).op(Op::MODS_I32).stg32(4);                   // sign of the dividend: -1
    a.push(INT32_MIN).push(-1).op(Op::DIVS_I32).stg32(8);           // MIN / -1 = MIN, no trap
    a.push(INT32_MIN).push(-1).op(Op::MODS_I32).stg32(12);          // 0
    a.push(-1).push(2).op(Op::DIVU_I32).stg32(16);                  // 0xFFFFFFFF / 2
    a.push64(0x8000000000000000).push64(~0ull).op(Op::DIVS_I64).stg64(24);
    EXPECT_EQ(run(m, a).state, RunState::done);
    EXPECT_EQ(static_cast<std::int32_t>(m.g32(0)), -3);
    EXPECT_EQ(static_cast<std::int32_t>(m.g32(4)), -1);
    EXPECT_EQ(m.g32(8), 0x80000000u);
    EXPECT_EQ(m.g32(12), 0u);
    EXPECT_EQ(m.g32(16), 0x7FFFFFFFu);
    EXPECT_EQ(m.g64(24), 0x8000000000000000u);
}

TEST(PlcVm, DivisionByZeroTrapsWithThePc) {
    Machine m;
    Asm a;
    a.push(1).push(0);
    const std::uint32_t at = a.here() + 5;  // after ENTER
    a.op(Op::DIVS_I32).stg32(0);
    const RunResult r = run(m, a);
    EXPECT_EQ(r.state, RunState::trapped);
    EXPECT_EQ(r.trap, Trap::DIV_BY_ZERO);
    EXPECT_EQ(r.pc, at);
    Machine f;
    Asm b;
    b.pushf(1.0f).pushf(-0.0f).op(Op::DIV_F32).stg32(0);
    EXPECT_EQ(run(f, b).trap, Trap::DIV_BY_ZERO);
}

TEST(PlcVm, ShiftsByTheWidthOrMoreGiveZero) {
    Machine m;
    Asm a;
    a.push(1).push(31).op(Op::SHL_I32).stg32(0);
    a.push(1).push(32).op(Op::SHL_I32).stg32(4);
    a.push(-1).push(40).op(Op::SHR_I32).stg32(8);
    a.push(-1).push(4).op(Op::SHR_I32).stg32(12);  // logical
    a.push64(1).push64(64).op(Op::SHL_I64).stg64(16);
    EXPECT_EQ(run(m, a).state, RunState::done);
    EXPECT_EQ(m.g32(0), 0x80000000u);
    EXPECT_EQ(m.g32(4), 0u);
    EXPECT_EQ(m.g32(8), 0u);
    EXPECT_EQ(m.g32(12), 0x0FFFFFFFu);
    EXPECT_EQ(m.g64(16), 0u);
}

TEST(PlcVm, SignedAndUnsignedComparisons) {
    Machine m;
    Asm a;
    a.push(-1).push(1).op(Op::LTS_I32).stg32(0);  // 1
    a.push(-1).push(1).op(Op::LTU_I32).stg32(4);  // 0
    a.push64(~0ull).push64(0).op(Op::GTS_I64).stg32(8);  // 0
    a.push64(~0ull).push64(0).op(Op::GTU_I64).stg32(12); // 1
    EXPECT_EQ(run(m, a).state, RunState::done);
    EXPECT_EQ(m.g32(0), 1u);
    EXPECT_EQ(m.g32(4), 0u);
    EXPECT_EQ(m.g32(8), 0u);
    EXPECT_EQ(m.g32(12), 1u);
}

// ---- floating point -----------------------------------------------------------------------------------

TEST(PlcVm, NanIsCanonicalOnEveryPlatform) {
    Machine m;
    Asm a;
    a.pushf(-1.0f).sys(Native::SQRT_F32).stg32(0);
    a.pushd(-1.0).sys(Native::SQRT_F64).stg64(8);
    a.pushf(std::numeric_limits<float>::infinity()).pushf(std::numeric_limits<float>::infinity()).op(Op::SUB_F32).stg32(16);
    a.op(Op::PUSH_I32).u32(0xFFC00001).op(Op::NEG_F32).stg32(20);  // a NaN with other bits stays a NaN, canonical
    EXPECT_EQ(run(m, a, {Native::SQRT_F32, Native::SQRT_F64}).state, RunState::done);
    EXPECT_EQ(m.g32(0), 0x7FC00000u);
    EXPECT_EQ(m.g64(8), 0x7FF8000000000000u);
    EXPECT_EQ(m.g32(16), 0x7FC00000u);
    EXPECT_EQ(m.g32(20), 0x7FC00000u);
}

TEST(PlcVm, SinglePrecisionIsRoundedPerOperation) {
    Machine m;
    Asm a;
    a.pushf(16777216.0f).pushf(1.0f).op(Op::ADD_F32).stg32(0);  // 2^24 + 1 is not representable
    a.pushf(0.1f).op(Op::F32_TO_F64).stg64(8);
    EXPECT_EQ(run(m, a).state, RunState::done);
    EXPECT_EQ(m.gf(0), 16777216.0f);
    EXPECT_EQ(m.gd(8), static_cast<double>(0.1f));
}

TEST(PlcVm, RoundHalfAwayFromZeroAndTruncate) {
    Machine m;
    Asm a;
    a.pushf(2.5f).op(Op::F32_ROUND_I64).stg64(0);
    a.pushf(-2.5f).op(Op::F32_ROUND_I64).stg64(8);
    a.pushd(-2.7).op(Op::F64_TRUNC_I64).stg64(16);
    a.pushd(0.49999999999999994).op(Op::F64_ROUND_I64).stg64(24);
    EXPECT_EQ(run(m, a).state, RunState::done);
    EXPECT_EQ(static_cast<std::int64_t>(m.g64(0)), 3);
    EXPECT_EQ(static_cast<std::int64_t>(m.g64(8)), -3);
    EXPECT_EQ(static_cast<std::int64_t>(m.g64(16)), -2);
    EXPECT_EQ(static_cast<std::int64_t>(m.g64(24)), 0);
}

TEST(PlcVm, ConversionOutOfRangeOrNanTraps) {
    for (const double x : {9.3e18, -9.3e18, std::numeric_limits<double>::quiet_NaN()}) {
        Machine m;
        Asm a;
        a.pushd(x).op(Op::F64_TRUNC_I64).stg64(0);
        EXPECT_EQ(run(m, a).trap, Trap::CONVERSION) << x;
    }
    Machine m;
    Asm a;
    a.pushd(-0x1p63).op(Op::F64_TRUNC_I64).stg64(0);  // exactly MIN: fits
    EXPECT_EQ(run(m, a).state, RunState::done);
    EXPECT_EQ(m.g64(0), 0x8000000000000000u);
}

TEST(PlcVm, RangeChecksKeepTheValue) {
    Machine m;
    Asm a;
    a.push(5).op(Op::CHK_I32).u32(static_cast<std::uint32_t>(-10)).u32(10).stg32(0);
    EXPECT_EQ(run(m, a).state, RunState::done);
    EXPECT_EQ(m.g32(0), 5u);
    Machine n;
    Asm b;
    b.push(70000).op(Op::CHK_I32).u32(static_cast<std::uint32_t>(-32768)).u32(32767).stg32(0);  // DINT_TO_INT(70000)
    EXPECT_EQ(run(n, b).trap, Trap::RANGE);
}

// ---- memory -------------------------------------------------------------------------------------------

TEST(PlcVm, LoadsAndStoresOfEveryWidthAndSign) {
    Machine m;
    Asm a;
    a.push(0x1FF).op(Op::STG_8).u32(0);           // low byte 0xFF
    a.op(Op::LDG_S8).u32(0).stg32(4);              // -1
    a.op(Op::LDG_U8).u32(0).stg32(8);              // 255
    a.push(0x18000).op(Op::STG_16).u32(12);
    a.op(Op::LDG_S16).u32(12).stg32(16);           // -32768
    a.op(Op::ADDR_G).u32(20).push(77).op(Op::STI_32);  // indirect store
    a.op(Op::ADDR_G).u32(20).op(Op::LDI_32).stg32(24);
    EXPECT_EQ(run(m, a).state, RunState::done);
    EXPECT_EQ(static_cast<std::int32_t>(m.g32(4)), -1);
    EXPECT_EQ(m.g32(8), 255u);
    EXPECT_EQ(m.data[14], 0);  // a 16-bit store writes 2 bytes
    EXPECT_EQ(static_cast<std::int32_t>(m.g32(16)), -32768);
    EXPECT_EQ(m.g32(24), 77u);
}

TEST(PlcVm, IndirectAccessOutsideTheDataSegmentTraps) {
    for (const std::uint32_t addr : {253u, 256u, 0xFFFFFFFCu}) {
        Machine m;
        Asm a;
        a.push(static_cast<std::int32_t>(addr)).op(Op::LDI_32).stg32(0);
        EXPECT_EQ(run(m, a).trap, Trap::MEMORY) << addr;  // data_size is 256, the Machine has 1024
    }
    Machine m;
    Asm a;
    a.push(0).push(250).op(Op::MEMCPY).u32(8);
    EXPECT_EQ(run(m, a).trap, Trap::MEMORY);
}

// ---- calls ----------------------------------------------------------------------------------------------

TEST(PlcVm, FunctionCallWithArgumentsAndResult) {
    // FUNCTION SUB2 (a, b : DINT) : DINT  - the arguments are popped into the frame, last first
    Asm a;
    a.enter(0);
    a.push(10).push(3);
    const std::uint32_t call = a.here() + 1;
    a.op(Op::CALL).u32(0);
    a.stg32(0);
    a.op(Op::RET);
    const std::uint32_t fn = a.here();
    a.enter(8).op(Op::STL_32).u16(4).op(Op::STL_32).u16(0);
    a.op(Op::LDL_32).u16(0).op(Op::LDL_32).u16(4).op(Op::SUB_I32).op(Op::RET);
    std::memcpy(&a.code[call], &fn, 4);
    Builder b;
    b.code = a.code;
    Machine m;
    ASSERT_EQ(m.load(b), LoadError::none);
    EXPECT_EQ(m.vm.run_cycle(0, 0).state, RunState::done);
    EXPECT_EQ(m.g32(0), 7u);
}

TEST(PlcVm, FunctionBlockSeesItsInstanceAsThis) {
    // FB: count := count + 1 (count at THIS + 4); two instances at 32 and 48
    Asm a;
    a.enter(0);
    const std::uint32_t c1 = a.here() + 6;
    a.op(Op::ADDR_G).u32(32).op(Op::CALL_FB).u32(0);
    const std::uint32_t c2 = a.here() + 6;
    a.op(Op::ADDR_G).u32(48).op(Op::CALL_FB).u32(0);
    a.op(Op::ADDR_G).u32(48).op(Op::CALL_FB).u32(0);
    a.op(Op::RET);
    const std::uint32_t fb = a.here();
    a.enter(0).op(Op::LDT_32).u16(4).push(1).op(Op::ADD_I32).op(Op::STT_32).u16(4).op(Op::RET);
    std::memcpy(&a.code[c1], &fb, 4);
    std::memcpy(&a.code[c2], &fb, 4);
    std::memcpy(&a.code[c2 + 10], &fb, 4);
    Builder b;
    b.code = a.code;
    Machine m;
    ASSERT_EQ(m.load(b), LoadError::none);
    EXPECT_EQ(m.vm.run_cycle(0, 0).state, RunState::done);
    EXPECT_EQ(m.g32(36), 1u);
    EXPECT_EQ(m.g32(52), 2u);
}

TEST(PlcVm, FramesAreZeroedOnEveryCall) {
    // a function whose local is read before it is written must see 0, not the last call's value
    Asm a;
    a.enter(0);
    const std::uint32_t c1 = a.here() + 1;
    a.op(Op::CALL).u32(0).stg32(0);
    const std::uint32_t c2 = a.here() + 1;
    a.op(Op::CALL).u32(0).stg32(4);
    a.op(Op::RET);
    const std::uint32_t fn = a.here();
    a.enter(4).op(Op::LDL_32).u16(0).op(Op::DUP).push(9).op(Op::ADD_I32).op(Op::STL_32).u16(0).op(Op::RET);
    std::memcpy(&a.code[c1], &fn, 4);
    std::memcpy(&a.code[c2], &fn, 4);
    Builder b;
    b.code = a.code;
    Machine m;
    ASSERT_EQ(m.load(b), LoadError::none);
    EXPECT_EQ(m.vm.run_cycle(0, 0).state, RunState::done);
    EXPECT_EQ(m.g32(0), 0u);
    EXPECT_EQ(m.g32(4), 0u);
}

TEST(PlcVm, EndlessRecursionTrapsCallDepthOrFrameOverflow) {
    Asm a;
    a.enter(0);
    a.op(Op::CALL).u32(0);  // calls itself
    a.op(Op::RET);
    Builder b;
    b.code = a.code;
    Machine m;
    ASSERT_EQ(m.load(b), LoadError::none);
    EXPECT_EQ(m.vm.run_cycle(0, 0).trap, Trap::CALL_DEPTH);

    Asm f;
    f.enter(16).op(Op::CALL).u32(0).op(Op::RET);  // 16 bytes per call, 128 bytes of frame area
    b.code = f.code;
    Machine n;
    ASSERT_EQ(n.load(b), LoadError::none);
    EXPECT_EQ(n.vm.run_cycle(0, 0).trap, Trap::FRAME_OVERFLOW);
}

// ---- budget, natives, TRAP ------------------------------------------------------------------------------

TEST(PlcVm, EndlessLoopStopsAtTheBudget) {
    Asm a;
    a.enter(0);
    const std::uint32_t loop = a.here();
    const std::uint32_t j = a.jump(Op::JMP);
    a.bind_to(j, loop);
    a.op(Op::RET);
    Builder b;
    b.code = a.code;
    b.tasks[0].budget = 1000;
    Machine m;
    ASSERT_EQ(m.load(b), LoadError::none);
    const RunResult r = m.vm.run_cycle(0, 0);
    EXPECT_EQ(r.trap, Trap::BUDGET);
    EXPECT_EQ(r.instructions, 1000u);
}

TEST(PlcVm, BudgetIsSharedByThePrograms) {
    Asm a;
    a.enter(0).op(Op::NOP).op(Op::NOP).op(Op::RET);  // 4 instructions
    Builder b;
    b.code = a.code;
    b.tasks[0].programs = {{0, 0}, {0, 0}};
    b.tasks[0].budget = 8;
    Machine m;
    ASSERT_EQ(m.load(b), LoadError::none);
    EXPECT_EQ(m.vm.run_cycle(0, 0).state, RunState::done);
    b.tasks[0].budget = 7;
    Machine n;
    ASSERT_EQ(n.load(b), LoadError::none);
    EXPECT_EQ(n.vm.run_cycle(0, 0).trap, Trap::BUDGET);
}

TEST(PlcVm, NativesTimeAndRotation) {
    Machine m;
    Asm a;
    a.sys(Native::TIME_MS).stg32(0);
    a.push(0x81).push(1).sys(Native::ROL_8).stg32(4);   // 0x03
    a.push(0x81).push(9).sys(Native::ROR_8).stg32(8);   // 9 mod 8 = 1: 0xC0
    a.push(1).push(0).sys(Native::ROL_32).stg32(12);
    a.push(INT32_MIN).sys(Native::ABS_I32).stg32(16);   // wraps: MIN
    EXPECT_EQ(run(m, a, {Native::TIME_MS, Native::ROL_8, Native::ROR_8, Native::ROL_32, Native::ABS_I32}).state,
              RunState::done);
    EXPECT_EQ(m.g32(0), 1234u);
    EXPECT_EQ(m.g32(4), 0x03u);
    EXPECT_EQ(m.g32(8), 0xC0u);
    EXPECT_EQ(m.g32(12), 1u);
    EXPECT_EQ(m.g32(16), 0x80000000u);
}

TEST(PlcVm, TrapInstructionCarriesItsCode) {
    Machine m;
    Asm a;
    a.op(Op::TRAP).u16(42);
    const RunResult r = run(m, a);
    EXPECT_EQ(r.trap, Trap::USER);
    EXPECT_EQ(r.trap_code, 42);
}

TEST(PlcVm, OperandStackOverflowTraps) {
    Machine m;
    Asm a;
    for (std::size_t i = 0; i <= operand_stack_slots; ++i) a.push(0);
    EXPECT_EQ(run(m, a).trap, Trap::STACK_OVERFLOW);
}

// ---- verifier -------------------------------------------------------------------------------------------

TEST(PlcVerifier, AcceptsOnlyIntactImagesForThisInstructionSet) {
    Builder b = program(Asm{});
    Machine m;
    auto img = b.build();
    img[hdr::code_offset + 0] ^= 0;  // untouched: fine
    EXPECT_EQ(m.vm.load(img), LoadError::none);
    img[img.size() - 1] ^= 1;
    EXPECT_EQ(m.vm.load(img), LoadError::crc);
    EXPECT_FALSE(m.vm.loaded());
    EXPECT_EQ(m.vm.load(std::span(img).first(40)), LoadError::too_short);
    Builder other = b;
    other.hash ^= 1;
    EXPECT_EQ(m.load(other), LoadError::isa_hash);
    other = b;
    other.isa = isa_version + 1;
    EXPECT_EQ(m.load(other), LoadError::isa_version);
}

TEST(PlcVerifier, RejectsBadCode) {
    Machine m;
    Builder b;
    b.code = {static_cast<std::uint8_t>(Op::ENTER), 0, 0, 0, 0, 0xFF};
    EXPECT_EQ(m.load(b), LoadError::bad_opcode);

    b.code = {static_cast<std::uint8_t>(Op::ENTER), 0, 0, 0, 0, static_cast<std::uint8_t>(Op::PUSH_I32), 1, 2};
    EXPECT_EQ(m.load(b), LoadError::truncated);

    b.code = {static_cast<std::uint8_t>(Op::ENTER), 0, 0, 0, 0, static_cast<std::uint8_t>(Op::NOP)};
    EXPECT_EQ(m.load(b), LoadError::truncated);  // falls off the end of the code
}

TEST(PlcVerifier, JumpsMustHitInstructionStartsAndCallsAnEnter) {
    Machine m;
    Asm a;
    a.enter(0);
    a.op(Op::JMP).u32(static_cast<std::uint32_t>(-3));  // into its own operand
    a.op(Op::RET);
    Builder b;
    b.code = a.code;
    EXPECT_EQ(m.load(b), LoadError::jump_target);

    Asm c;
    c.enter(0).op(Op::CALL).u32(5).op(Op::RET);  // 5: the CALL itself, not an ENTER
    b.code = c.code;
    EXPECT_EQ(m.load(b), LoadError::call_target);

    Asm e;
    e.op(Op::NOP).enter(0).op(Op::RET);
    b.code = e.code;  // the program entry 0 is a NOP
    EXPECT_EQ(m.load(b), LoadError::call_target);
}

TEST(PlcVerifier, GlobalAddressesAndNatives) {
    Machine m;
    Asm a;
    a.ldg32(253).op(Op::DROP);  // 253 + 4 > 256
    EXPECT_EQ(m.load(program(a)), LoadError::data_address);

    Asm s;
    s.sys(Native::TIME_MS).op(Op::DROP);
    EXPECT_EQ(m.load(program(s)), LoadError::native_not_listed);
    EXPECT_EQ(m.load(program(s, {Native::TIME_MS})), LoadError::none);

    Builder b = program(Asm{}, {Native::TIME_MS});
    auto img = b.build();
    const std::uint32_t nat = rd32(&img[hdr::native_offset]);
    img[nat + 4] ^= 1;  // signature
    std::uint32_t crc = crc32(std::span(img).subspan(hdr::crc + 4));
    std::memcpy(&img[hdr::crc], &crc, 4);
    EXPECT_EQ(m.vm.load(img), LoadError::native_signature);
    img[nat] = 0xFF;  // id
    crc = crc32(std::span(img).subspan(hdr::crc + 4));
    std::memcpy(&img[hdr::crc], &crc, 4);
    EXPECT_EQ(m.vm.load(img), LoadError::native_unknown);
}

TEST(PlcVerifier, MemoryLayout) {
    Machine m;
    Builder b = program(Asm{});
    b.data_size = 2048;  // the Machine has 1024
    EXPECT_EQ(m.load(b), LoadError::data_too_large);

    b = program(Asm{});
    b.init = {{250, {1, 2, 3, 4, 5, 6, 7}}};
    EXPECT_EQ(m.load(b), LoadError::init_range);

    b = program(Asm{});
    b.tasks.push_back(b.tasks[0]);  // the same frame area twice
    EXPECT_EQ(m.load(b), LoadError::task_range);
    b.tasks[1].frame_base = 0;
    EXPECT_EQ(m.load(b), LoadError::none);

    b.tasks.push_back(b.tasks[1]);  // three tasks, two contexts
    b.tasks[2].frame_base = 64;
    b.tasks[2].frame_size = 32;
    b.tasks[1].frame_size = 64;
    EXPECT_EQ(m.load(b), LoadError::task_range);
}

}  // namespace
