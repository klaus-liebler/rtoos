#!/bin/sh
# SPDX-License-Identifier: MIT
# The state of a running or finished campaign (tools/fuzz/run.sh), per harness: executions, edges,
# crashes, hangs. In the container: tools/fuzz/status.sh
cd build/linux-afl/findings || exit 1
for h in *; do
    execs=$(cat "$h"/*/fuzzer_stats 2>/dev/null | awk -F: '/^execs_done/{s+=$2} END{print s+0}')
    speed=$(cat "$h"/*/fuzzer_stats 2>/dev/null | awk -F: '/^execs_per_sec/{s+=$2} END{printf "%d", s}')
    edges=$(awk -F: '/^edges_found/{print $2+0}' "$h/main/fuzzer_stats" 2>/dev/null)
    crashes=$(find "$h" -path '*/crashes/id:*' | wc -l)
    hangs=$(find "$h" -path '*/hangs/id:*' | wc -l)
    printf '%-10s execs %11s (%6s/s)  edges %6s  crashes %3s  hangs %3s\n' "$h" "$execs" "$speed" "${edges:-?}" "$crashes" "$hangs"
done
