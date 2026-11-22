# SPDX-License-Identifier: MIT
#
# Linux build environment for the sanitizer and fuzzing presets (linux-asan, linux-afl; see
# docs/fuzzing.md): GCC 16 (the project's one compiler) with AddressSanitizer, which MinGW lacks,
# and AFL++ built against GCC 16's plugin API (afl-g++-fast: edge coverage, persistent mode).
#
#   docker build -t rtoos-fuzz rtoos/tools/fuzz
FROM gcc:16.2.0

RUN apt-get update \
 && apt-get install -y --no-install-recommends cmake ninja-build python3 openssl git ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# AFL++: afl-fuzz, its tools and the GCC plugin (no LLVM in this image). SIMPLE_FILES: findings
# named id_000000... instead of id:000000,... - the campaign writes into the Windows checkout, and
# NTFS has no ':' in file names.
ARG AFL_TAG=v4.40c
RUN git clone --depth 1 --branch ${AFL_TAG} https://github.com/AFLplusplus/AFLplusplus.git /tmp/afl \
 && sed -i 's|^// #define SIMPLE_FILES|#define SIMPLE_FILES|' /tmp/afl/include/config.h \
 && grep -q '^#define SIMPLE_FILES' /tmp/afl/include/config.h \
 && make -C /tmp/afl -j"$(nproc)" all NO_PYTHON=1 \
 && make -C /tmp/afl -f GNUmakefile.gcc_plugin -j"$(nproc)" \
 && make -C /tmp/afl install \
 && test -x /usr/local/bin/afl-g++-fast \
 && rm -rf /tmp/afl

WORKDIR /src
