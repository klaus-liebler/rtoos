// SPDX-License-Identifier: MIT
//
// The main() of every AFL++ harness (docs/fuzzing.md). A harness defines
//
//     void fuzz_one(std::span<const std::uint8_t> input);
//
// and includes this header once. Built with afl-g++-fast (preset linux-afl) it runs in AFL++'s
// persistent mode: one process, the inputs through shared memory. Built with g++ (any other host
// build) it replays the files given on the command line, directories file by file - the corpus,
// a crash AFL++ found, the regression inputs in tests/fuzz/regressions (ctest fuzz_replay_*).
//
// A finding: ASan/UBSan report, contract violation (enforced on the host), a hang (AFL++ timeout).
#pragma once

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <span>
#include <vector>

void fuzz_one(std::span<const std::uint8_t> input);

#ifdef __AFL_FUZZ_TESTCASE_LEN

#include <unistd.h>  // read(): __AFL_FUZZ_TESTCASE_LEN's fallback without shared memory

__AFL_FUZZ_INIT();

int main() {
    __AFL_INIT();
    const unsigned char* buf = __AFL_FUZZ_TESTCASE_BUF;  // after __AFL_INIT, before __AFL_LOOP
    while (__AFL_LOOP(10000)) {
        fuzz_one({buf, static_cast<std::size_t>(__AFL_FUZZ_TESTCASE_LEN)});
    }
    return 0;
}

#else

namespace fuzz_main {

inline int replay(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "cannot read %s\n", p.string().c_str());
        return 1;
    }
    const std::vector<std::uint8_t> data{std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
    fuzz_one(data);
    return 0;
}

}  // namespace fuzz_main

int main(int argc, char** argv) {
    int failures = 0, inputs = 0;
    for (int i = 1; i < argc; ++i) {
        const std::filesystem::path p(argv[i]);
        if (std::filesystem::is_directory(p)) {
            for (const auto& e : std::filesystem::recursive_directory_iterator(p)) {
                if (e.is_regular_file() && e.path().filename().string()[0] != '.') {
                    failures += fuzz_main::replay(e.path());
                    ++inputs;
                }
            }
        } else {
            failures += fuzz_main::replay(p);
            ++inputs;
        }
    }
    std::printf("replayed %d inputs\n", inputs);
    return failures == 0 ? 0 : 1;
}

#endif
