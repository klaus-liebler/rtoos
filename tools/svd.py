"""Look up peripherals, registers and bit fields in an SVD file.

Hardware facts come from the SVD (CLAUDE.md), not from memory. This prints them in the form
the drivers need:

  python tools/svd.py STM32H563.svd I2C1                 # base address, all registers
  python tools/svd.py STM32H563.svd I2C1 CR1             # one register with its fields
  python tools/svd.py STM32H563.svd RCC APB1LENR I2C     # fields whose name contains "I2C"
  python tools/svd.py STM32H563.svd --irq I2C            # interrupt numbers matching "I2C"

Derived peripherals (derivedFrom) show the registers of their base. Addresses are the
non-secure aliases as given in the SVD.

SVD source: https://github.com/modm-io/cmsis-svd-stm32 (stm32h5/STM32H563.svd, v1.8).
"""
import argparse
import sys
import xml.etree.ElementTree as ET


def num(text):
    text = (text or "0").strip().lower()
    return int(text, 16) if text.startswith("0x") else int(text)


def field_range(f):
    if f.find("bitOffset") is not None:
        lo = num(f.findtext("bitOffset"))
        return lo, lo + num(f.findtext("bitWidth")) - 1
    if f.find("lsb") is not None:
        return num(f.findtext("lsb")), num(f.findtext("msb"))
    hi, lo = f.findtext("bitRange").strip("[]").split(":")
    return int(lo), int(hi)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("svd")
    ap.add_argument("peripheral", nargs="?")
    ap.add_argument("register", nargs="?")
    ap.add_argument("field", nargs="?", help="substring of the field names to show")
    ap.add_argument("--irq", metavar="NAME", help="list interrupts whose name contains NAME")
    args = ap.parse_args()

    root = ET.parse(args.svd).getroot()
    peripherals = {p.findtext("name"): p for p in root.iter("peripheral")}

    if args.irq:
        seen = set()
        for p in peripherals.values():
            for i in p.findall("interrupt"):
                name, value = i.findtext("name"), num(i.findtext("value"))
                if args.irq.upper() in name.upper() and (name, value) not in seen:
                    seen.add((name, value))
                    print(f"{value:4d}  {name}")
        return

    if not args.peripheral:
        for name, p in sorted(peripherals.items(), key=lambda kv: num(kv[1].findtext("baseAddress"))):
            print(f"0x{num(p.findtext('baseAddress')):08X}  {name}")
        return

    p = peripherals.get(args.peripheral)
    if p is None:
        sys.exit(f"no peripheral {args.peripheral}")
    base = num(p.findtext("baseAddress"))
    src = peripherals[p.get("derivedFrom")] if p.get("derivedFrom") else p
    print(f"{args.peripheral} base 0x{base:08X}" + (f" (registers of {p.get('derivedFrom')})" if src is not p else ""))
    for r in src.iter("register"):
        name = r.findtext("name")
        if args.register and args.register.upper() not in (name.upper(), name.upper().split("_", 1)[-1]):
            continue
        offset = num(r.findtext("addressOffset"))
        reset = r.findtext("resetValue")
        print(f"  +0x{offset:03X}  {name:<12} reset {reset}  {(r.findtext('description') or '').split(chr(10))[0].strip()}")
        if args.register:
            for f in sorted(r.iter("field"), key=lambda f: field_range(f)[0]):
                fname = f.findtext("name")
                if args.field and args.field.upper() not in fname.upper():
                    continue
                lo, hi = field_range(f)
                bits = f"{lo}" if lo == hi else f"{hi}:{lo}"
                desc = " ".join((f.findtext("description") or "").split())[:90]
                print(f"      [{bits:>5}] {fname:<14} {desc}")
                if args.field:  # values only for the fields asked for
                    for e in f.iter("enumeratedValue"):
                        edesc = " ".join((e.findtext("description") or "").split())[:70]
                        print(f"               {e.findtext('value'):>6} {e.findtext('name'):<14} {edesc}")


if __name__ == "__main__":
    main()
