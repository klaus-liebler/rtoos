#!/bin/sh
# SPDX-License-Identifier: MIT
#
# AFL++ campaign for the harnesses of tests/fuzz (docs/fuzzing.md). In the container, rtoos at /src:
#
#   docker run --rm -v <rtoos>:/src -w /src rtoos-fuzz tools/fuzz/run.sh <seconds> [jobs per harness] [harness...]
#
# Builds preset linux-afl, then fuzzes every harness (or the ones given) at the same time for
# <seconds>: one main instance and jobs-1 secondaries each. Findings in
# build/linux-afl/findings/<harness>/*/crashes and */hangs; a summary at the end.
set -eu
seconds=${1:?seconds}
jobs=${2:-1}
shift $(( $# >= 2 ? 2 : 1 ))
harnesses=${*:-"http websocket dns dhcp modbus opcua ethercat plc"}

cmake --preset linux-afl >/dev/null
cmake --build --preset linux-afl -j "$(nproc)" $(for h in $harnesses; do printf ' --target fuzz_%s' "$h"; done) >/dev/null

export AFL_SKIP_CPUFREQ=1 AFL_NO_UI=1 AFL_I_DONT_CARE_ABOUT_MISSING_CRASHES=1 AFL_NO_AFFINITY=1
export ASAN_OPTIONS=abort_on_error=1:symbolize=0:detect_leaks=0
bin=build/linux-afl/tests/fuzz
for h in $harnesses; do
    out=build/linux-afl/findings/$h
    mkdir -p "$out"
    # an existing campaign is resumed (-i -), else it starts from the seed corpus
    in=tests/fuzz/corpus/$h
    [ -d "$out/main/queue" ] && in=-
    afl-fuzz -M main -i "$in" -o "$out" -V "$seconds" -t 1000 -- "$bin/fuzz_$h" >"$out/main.log" 2>&1 &
    i=1
    while [ "$i" -lt "$jobs" ]; do
        afl-fuzz -S "s$i" -i "$in" -o "$out" -V "$seconds" -t 1000 -- "$bin/fuzz_$h" >"$out/s$i.log" 2>&1 &
        i=$((i + 1))
    done
done
wait

for h in $harnesses; do
    out=build/linux-afl/findings/$h
    execs=$(cat "$out"/*/fuzzer_stats 2>/dev/null | awk -F: '/^execs_done/{s+=$2} END{print s+0}')
    edges=$(awk -F: '/^edges_found/{print $2+0}' "$out/main/fuzzer_stats" 2>/dev/null || echo 0)
    crashes=$(find "$out" -path '*/crashes/id:*' | wc -l)
    hangs=$(find "$out" -path '*/hangs/id:*' | wc -l)
    printf '%-10s execs %10s  edges %6s  crashes %3s  hangs %3s\n' "$h" "$execs" "$edges" "$crashes" "$hangs"
done
