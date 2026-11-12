"""No start value other than 0 in the DMA sections of a firmware image.

The DMA sections (.dma_buffer, .sdmmc_buffer: RTOS_DMA_BUFFER, RTOS_SDMMC_BUFFER) are NOLOAD - on
the G431 part of .bss - and only zeroed at start-up (port/cortex_m/src/startup.cpp): nothing is
copied from flash. An object the compiler constant-initializes with a value other than 0 (a default
member initializer, a constexpr constructor) therefore starts with 0 instead. On FCU revision 1
the section was not even zeroed, and a stepper read SRAM garbage at boot (locked up); its non-zero
defaults would have been lost either way. Emulators start with zeroed RAM and hide both.

So after the link: every input section of these names (listed in the linker map, contents from the
object files) holds only zeros. Run by rtos_add_firmware() (port/cortex_m/CMakeLists.txt).

    python check_noload.py <objdump> <image.map> <directory the link ran in>
"""
import re
import subprocess
import sys
from pathlib import Path

SECTIONS = (".dma_buffer", ".sdmmc_buffer")
# " .dma_buffer    0x20090040      0x390 CMakeFiles/x.dir/src/io.cpp.obj" (the name may stand
# alone on its line when it is long; address, size and file follow on the next)
INPUT = re.compile(r"^ (\.(?:dma_buffer|sdmmc_buffer)\S*)(?:\s+0x[0-9a-f]+\s+0x([0-9a-f]+)\s+(\S.*))?$")
NEXT = re.compile(r"^\s+0x[0-9a-f]+\s+0x([0-9a-f]+)\s+(\S.*)$")
MEMBER = re.compile(r"^(.+)\((.+)\)$")  # lib.a(member.obj)


def contributions(map_file: str) -> list[tuple[str, str]]:
    """(section, file) of every non-empty input section of the DMA sections that was linked."""
    lines = Path(map_file).read_text(encoding="utf-8", errors="replace").splitlines()
    out = []
    start = next((i for i, l in enumerate(lines) if l.startswith("Linker script and memory map")), 0)
    i = start
    while i < len(lines):
        m = INPUT.match(lines[i])
        if m:
            section, size, path = m.group(1), m.group(2), m.group(3)
            if size is None and i + 1 < len(lines) and (n := NEXT.match(lines[i + 1])):
                size, path = n.group(1), n.group(2)
                i += 1
            if size and int(size, 16) > 0 and path:
                out.append((section, path.strip()))
        i += 1
    return out


def non_zero(objdump: str, section: str, path: str, cwd: Path) -> bool:
    archive, member = (m.group(1), m.group(2)) if (m := MEMBER.match(path)) else (path, None)
    result = subprocess.run([objdump, "-s", "-j", section, str(cwd / archive)], capture_output=True, text=True)
    current = None
    for line in result.stdout.splitlines():
        if (h := re.match(r"^(\S+):\s+file format", line)):
            current = h.group(1)
            continue
        if member is not None and current != member:
            continue
        # " 0260 00000000 00000000 00000000 64000000  ............d..."
        if (d := re.match(r"^ [0-9a-f]+ ((?:[0-9a-f]{2,8} ?){1,4})", line)):
            if d.group(1).replace(" ", "").strip("0"):
                return True
    return False


def main() -> int:
    objdump, map_file, cwd = sys.argv[1], sys.argv[2], Path(sys.argv[3])
    bad = sorted({f"{path} ({section})" for section, path in contributions(map_file) if non_zero(objdump, section, path, cwd)})
    for b in bad:
        print(f"check_noload: {b} gives an object in a DMA section a start value other than 0 - it would "
              f"start at 0 (the section is zeroed, not loaded): give its members 0 as default", file=sys.stderr)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
