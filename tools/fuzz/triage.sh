#!/bin/sh
# SPDX-License-Identifier: MIT
# Replays the crashes (and with --hangs the hangs) of a campaign under the ASan replay build
# (preset linux-asan) and prints each distinct report once: the error line and the first frames in
# rtoos code. In the container: tools/fuzz/triage.sh <harness> [--hangs]
set -u
h=${1:?harness}
kind=crashes
[ "${2:-}" = "--hangs" ] && kind=hangs
cmake --build --preset linux-asan --target "fuzz_$h" >/dev/null || exit 1
bin=build/linux-asan/tests/fuzz/fuzz_$h
export ASAN_OPTIONS=detect_leaks=0
seen=""
for f in $(find "build/linux-afl/findings/$h" -path "*/$kind/id*" | sort); do
    out=$(timeout 20 "$bin" "$f" 2>&1)
    rc=$?
    head=$(printf '%s\n' "$out" | grep -m1 -E 'ERROR: AddressSanitizer|runtime error|contract violation|terminate|Assertion' |
           sed -E 's/==[0-9]+==//; s/0x[0-9a-f]+//g')
    [ -z "$head" ] && head="exit $rc (no report)"
    case "$seen" in *"$head"*) continue ;; esac
    seen="$seen|$head"
    echo "== $f"
    echo "   $head"
    printf '%s\n' "$out" | grep -E '#[0-9]+ .*/src/(include|src|tests)/' | head -6 | sed 's/^/   /'
done
