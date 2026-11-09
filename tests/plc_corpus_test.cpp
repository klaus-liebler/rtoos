// Differential tests of the PLC VM (docs/plc.md "Differential tests"): the images in plc_corpus/ were
// compiled by the FCU web UI's TypeScript compiler and run in its TypeScript VM, which wrote the
// trace - the state and the whole data segment after every cycle. The C++ VM must reproduce every
// trace byte for byte. The files come from `npm run plc:corpus` in the firmware's web/ directory.
#include "rtos/plc/vm.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;
using namespace rtos::plc;

std::vector<std::uint8_t> read_file(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

struct Set {
    std::uint32_t cycle;
    std::uint32_t addr;
    std::vector<std::uint8_t> bytes;
};

struct Plan {
    std::uint32_t cycles = 0;
    std::uint32_t dt = 0;
    std::vector<Set> sets;
};

Plan read_plan(const fs::path& p) {
    Plan plan;
    std::ifstream f(p);
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream in(line);
        std::string key;
        in >> key;
        if (key == "cycles") in >> plan.cycles;
        else if (key == "dt") in >> plan.dt;
        else if (key == "set") {
            Set s;
            std::string hex;
            in >> s.cycle >> s.addr >> hex;
            for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
                s.bytes.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
            }
            plan.sets.push_back(std::move(s));
        }
    }
    return plan;
}

std::vector<fs::path> corpus() {
    std::vector<fs::path> images;
    for (const auto& e : fs::directory_iterator(PLC_CORPUS_DIR)) {
        if (e.path().extension() == ".img") images.push_back(e.path());
    }
    std::ranges::sort(images);
    return images;
}

class PlcCorpus : public testing::TestWithParam<fs::path> {};

TEST_P(PlcCorpus, TraceIsByteIdenticalToTheTypeScriptVm) {
    const fs::path img = GetParam();
    const auto image = read_file(img);
    const Plan plan = read_plan(fs::path(img).replace_extension(".run"));
    const auto trace = read_file(fs::path(img).replace_extension(".trace"));
    ASSERT_GE(trace.size(), 8u);

    static std::array<std::uint8_t, 32 * 1024> data;
    static std::array<Context, max_tasks> contexts;
    data.fill(0x5A);  // the loader must initialize everything the program sees
    Vm vm{data, contexts};
    ASSERT_EQ(vm.load(image), LoadError::none) << to_string(vm.load(image));

    const std::uint32_t data_size = rd32(&trace[0]);
    const std::uint32_t cycles = rd32(&trace[4]);
    ASSERT_EQ(data_size, vm.data_size());
    const std::size_t record = 12 + data_size;
    ASSERT_EQ(trace.size(), 8 + std::size_t{cycles} * record);

    for (std::uint32_t k = 0; k < cycles; ++k) {
        for (const Set& s : plan.sets) {
            if (s.cycle == k) std::ranges::copy(s.bytes, data.begin() + s.addr);
        }
        const RunResult r = vm.run_cycle(0, k * plan.dt);
        const std::uint8_t* expected = &trace[8 + k * record];
        ASSERT_EQ(r.state == RunState::done ? 0 : 1, expected[0]) << "cycle " << k;
        ASSERT_EQ(static_cast<unsigned>(r.trap), expected[1]) << "cycle " << k;
        ASSERT_EQ(r.trap_code, rd16(expected + 2)) << "cycle " << k;
        ASSERT_EQ(r.pc, rd32(expected + 4)) << "cycle " << k;
        ASSERT_EQ(r.instructions, rd32(expected + 8)) << "cycle " << k;
        const auto first = std::mismatch(data.begin(), data.begin() + data_size, expected + 12);
        ASSERT_EQ(first.first, data.begin() + data_size)
            << "cycle " << k << ": data differs at address " << (first.first - data.begin()) << ": C++ "
            << int{*first.first} << ", TypeScript " << int{*first.second};
        if (r.state == RunState::trapped) break;
    }
}

INSTANTIATE_TEST_SUITE_P(Corpus, PlcCorpus, testing::ValuesIn(corpus()),
                         [](const testing::TestParamInfo<fs::path>& info) { return info.param.stem().string(); });

TEST(PlcCorpusFiles, Exist) {
    EXPECT_GE(corpus().size(), 50u) << "regenerate with `npm run plc:corpus` in the firmware's web/ directory";
}

}  // namespace
