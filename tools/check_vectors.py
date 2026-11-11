"""Every interrupt handler of a firmware image is in its vector table.

A handler is found by its name only: `extern "C" void USB_DRD_FS_IRQHandler()` replaces the weak
alias of that name in the chip's vector table (port/<chip>/src/vectors.cpp). Spelled differently
from the table - another name for the same peripheral, a typo - it links without a word, and the
interrupt goes to Default_Handler (a breakpoint: without a debugger the chip locks up). That is what
happened to the H563's USB (the table said USB_FS_IRQHandler): the board hung as soon as a PC was
plugged in.

So after the link:
- no section of a handler (.text.<name>_Handler / _IRQHandler, -ffunction-sections) is among the
  linker map's discarded input sections: nothing referenced it, so the table does not know its name
  (that is how the misspelled one shows - --gc-sections removes it from the image);
- every strongly defined handler left in the image is the target of an entry of .isr_vector.
Run by rtos_add_firmware() (port/cortex_m/CMakeLists.txt).

    python check_vectors.py <nm> <objcopy> <image.elf> <image.map>
"""
import re
import subprocess
import sys
import tempfile
from pathlib import Path

HANDLER = re.compile(r"^\w+_(IRQ)?Handler$")


def strong_handlers(nm: str, elf: str) -> dict[str, int]:
    out = subprocess.run([nm, "--defined-only", elf], check=True, capture_output=True, text=True).stdout
    handlers = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) != 3:
            continue
        address, kind, name = parts
        # T/t: defined in the text section; W/w (the table's weak aliases) are not chosen by anyone
        if kind in ("T", "t") and HANDLER.match(name):
            handlers[name] = int(address, 16) & ~1
    return handlers


def vector_targets(objcopy: str, elf: str) -> set[int]:
    with tempfile.TemporaryDirectory() as tmp:
        raw = Path(tmp) / "vectors.bin"
        subprocess.run([objcopy, "-O", "binary", "-j", ".isr_vector", elf, str(raw)], check=True)
        data = raw.read_bytes()
    if not data:
        sys.exit(f"check_vectors: {elf} has no .isr_vector section")
    words = [int.from_bytes(data[i:i + 4], "little") for i in range(0, len(data) - 3, 4)]
    return {w & ~1 for w in words[1:]}  # word 0: the initial stack pointer


DISCARDED = re.compile(r"^ \.text\.(\w+_(?:IRQ)?Handler)\s*$")


def discarded_handlers(map_file: str) -> set[str]:
    names = set()
    inside = False
    for line in Path(map_file).read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("Discarded input sections"):
            inside = True
        elif line.startswith("Memory Configuration"):
            break
        elif inside and (m := DISCARDED.match(line)):
            names.add(m.group(1))
    return names


def main() -> int:
    nm, objcopy, elf, map_file = sys.argv[1:5]
    targets = vector_targets(objcopy, elf)
    missing = sorted({name for name, address in strong_handlers(nm, elf).items() if address not in targets}
                     | discarded_handlers(map_file))
    for name in missing:
        print(f"check_vectors: {name} is defined but not in the vector table of {Path(elf).name} "
              f"(misspelled? the names are in port/<chip>/src/vectors.cpp)", file=sys.stderr)
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
