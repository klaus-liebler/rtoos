#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Catalog of open62541's test suite against the rtoos OPC UA server.

  open62541_catalog.py <open62541/tests> [--check] [--allow-open]

Every test case of open62541 (START_TEST(name) in <open62541/tests>/**/*.c) is either
  * ported: an rtoos test carries the marker  // open62541: <file> <test>
  * not applicable: a rule in tests/open62541/rules.tsv gives the reason
  * open (only with --allow-open): a rule with the reason "OPEN ..." (decided later)
Rules (tab separated: pattern, reason) match a file ("server/check_x.c"), a directory glob
("pubsub/*"), or tests of a file ("server/check_x.c::Name", "server/check_x.c::Prefix*"); the most
specific rule wins. A marker for a test that does not exist, a test without marker or rule, and
a test-specific rule for a ported test are errors.

Writes tests/open62541/catalog.md (unless --check: then it only verifies the catalog is current).
"""
from __future__ import annotations

import fnmatch
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
RULES = REPO / "tests" / "open62541" / "rules.tsv"
CATALOG = REPO / "tests" / "open62541" / "catalog.md"
MARKER = re.compile(r"//\s*open62541:\s*(\S+)\s+(\w+)")
TEST = re.compile(r"^\s*START_TEST\((\w+)\)", re.M)


def collect_tests(root: Path) -> dict[str, list[str]]:
    tests: dict[str, list[str]] = {}
    for path in sorted(root.rglob("*.c")):
        names = TEST.findall(path.read_text(encoding="utf-8", errors="replace"))
        if names:
            tests[path.relative_to(root).as_posix()] = names
    return tests


def collect_markers() -> dict[tuple[str, str], list[str]]:
    markers: dict[tuple[str, str], list[str]] = defaultdict(list)
    for path in sorted((REPO / "tests").glob("*.cpp")):
        lines = path.read_text(encoding="utf-8").splitlines()
        for i, line in enumerate(lines):
            m = MARKER.search(line)
            if not m:
                continue
            # the rtoos test the marker belongs to: the next TEST/TEST_F line
            target = "?"
            for follow in lines[i + 1:i + 40]:
                t = re.match(r"\s*TEST(?:_F|_P)?\((\w+),\s*(\w+)\)", follow)
                if t:
                    target = f"{t.group(1)}.{t.group(2)}"
                    break
            markers[(m.group(1), m.group(2))].append(f"{path.name}: {target}")
    return markers


def load_rules() -> list[tuple[str, str, str, int]]:
    """(file pattern, test pattern or '', reason, specificity)"""
    rules = []
    for n, line in enumerate(RULES.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip() or line.startswith("#"):
            continue
        if "\t" not in line:
            sys.exit(f"{RULES.name}:{n}: pattern and reason must be separated by a tab")
        pattern, reason = line.split("\t", 1)
        file_pattern, _, test_pattern = pattern.partition("::")
        if test_pattern:
            specificity = 3 if "*" not in test_pattern else 2
        else:
            specificity = 1 if "*" not in file_pattern else 0
        rules.append((file_pattern, test_pattern, reason.strip(), specificity))
    return rules


def rule_for(rules, file: str, test: str):
    best = None
    for file_pattern, test_pattern, reason, specificity in rules:
        if not fnmatch.fnmatchcase(file, file_pattern):
            continue
        if test_pattern and not fnmatch.fnmatchcase(test, test_pattern):
            continue
        if best is None or specificity > best[1]:
            best = (reason, specificity)
    return best


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    check = "--check" in sys.argv
    allow_open = "--allow-open" in sys.argv
    if len(args) != 1:
        print(__doc__)
        return 2
    root = Path(args[0])
    tests = collect_tests(root)
    markers = collect_markers()
    rules = load_rules()
    try:
        commit = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True,
                                check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        commit = "unknown"

    errors = []
    known = {(f, t) for f, names in tests.items() for t in names}
    for key, where in markers.items():
        if key not in known:
            errors.append(f"marker for a test that does not exist: {key[0]} {key[1]} ({', '.join(where)})")

    status: dict[str, list[tuple[str, str, str]]] = {}
    totals = defaultdict(int)
    for file, names in tests.items():
        rows = []
        for test in names:
            rule = rule_for(rules, file, test)
            if (file, test) in markers:
                if rule and rule[1] >= 2:
                    errors.append(f"ported test also has a test rule: {file} {test}")
                rows.append(("ported", test, "; ".join(markers[(file, test)])))
                totals["ported"] += 1
            elif rule is None:
                errors.append(f"neither ported nor justified: {file} {test}")
                rows.append(("missing", test, ""))
            elif rule[0].startswith("OPEN"):
                if not allow_open:
                    errors.append(f"still open: {file} {test}")
                rows.append(("open", test, rule[0]))
                totals["open"] += 1
            else:
                rows.append(("n/a", test, rule[0]))
                totals["n/a"] += 1
        status[file] = rows

    out = [
        "# open62541 tests and the rtoos OPC UA server",
        "",
        "Generated by `tools/open62541_catalog.py` - do not edit; reasons live in `rules.tsv`, ports are",
        "the tests marked `// open62541: <file> <test>` in `tests/*.cpp`.",
        "",
        f"open62541 commit `{commit}`: {sum(len(v) for v in tests.values())} test cases in {len(tests)} files -",
        f"ported {totals['ported']}, not applicable {totals['n/a']}" + (f", open {totals['open']}" if totals["open"] else "") + ".",
        "",
        "| File | Tests | Ported | Not applicable |",
        "|---|---:|---:|---:|",
    ]
    for file, rows in status.items():
        ported = sum(1 for r in rows if r[0] == "ported")
        na = sum(1 for r in rows if r[0] == "n/a")
        out.append(f"| `{file}` | {len(rows)} | {ported} | {na} |")
    out.append("")
    for file, rows in status.items():
        reasons = {r[2] for r in rows if r[0] == "n/a"}
        if all(r[0] == "n/a" for r in rows) and len(reasons) == 1:
            out += [f"## `{file}`", "", f"Not applicable ({len(rows)} tests): {next(iter(reasons))}", ""]
            continue
        out += [f"## `{file}`", "", "| Test | Status |", "|---|---|"]
        for kind, test, detail in rows:
            text = {"ported": f"ported: {detail}", "n/a": f"not applicable: {detail}", "open": f"open: {detail}",
                    "missing": "MISSING"}[kind]
            out.append(f"| `{test}` | {text} |")
        out.append("")
    text = "\n".join(out)

    for e in errors:
        print("error:", e, file=sys.stderr)
    if check:
        if not CATALOG.exists() or CATALOG.read_text(encoding="utf-8") != text:
            print("error: tests/open62541/catalog.md is not current", file=sys.stderr)
            return 1
    else:
        CATALOG.write_text(text, encoding="utf-8", newline="\n")
    print(f"{totals['ported']} ported, {totals['n/a']} not applicable, {totals['open']} open, {len(errors)} errors")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
