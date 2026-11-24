#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Generate a chip's pin_functions.hpp from ST's pin database.

    python tools/gen_pin_functions.py "<STM32CubeMX>/db/mcu" h5 > port/stm32h563/include/rtos/port/stm32h5/pin_functions.hpp
    python tools/gen_pin_functions.py "<STM32CubeMX>/db/mcu" g4 > port/stm32g431/include/rtos/port/stm32g4/pin_functions.hpp
    python tools/gen_pin_functions.py "<STM32CubeMX>/db/mcu" h7 > port/stm32h743/include/rtos/port/stm32h7/pin_functions.hpp
    python tools/gen_pin_functions.py "<STM32CubeMX>/db/mcu" f1 > port/stm32f103/include/rtos/port/stm32f1/pin_functions.hpp

Sources (STM32CubeMX database, the data behind ST's open pin data):
  <part>.xml                 the pins of a package and their signals
  IP/GPIO-<family>_Modes.xml the alternate function number of each pin signal

A signal with an AF number is an alternate function (MODER = AF, AFR = the number); a signal
without one is an additional function (analog: ADC inputs, UCPD CC lines, USB DP/DM, ...).

A chip with one package (h5, h7: the FCU's LQFP100) gets plain tables. A chip whose packages the
board chooses (g4) gets the union of all packages, every entry with a mask of the packages
that have it: smaller packages lack pins and, on some pins, functions (G431 LQFP48: no UART4_CTS
on PB7). gpio.hpp of the chip filters the tables for the selected package at compile time.

The STM32F1 (f1) has no AF numbers: every pin has fixed functions, and some peripherals can move
their pins as a group by a field in AFIO_MAPR (RM0008 "AF remap and debug I/O configuration").
IP/GPIO-<part>_Modes.xml lists, per pin and signal, the remap blocks that put the signal there:
the default block (field value 0) or a block with ST's HAL macro (__HAL_AFIO_REMAP_TIM3_PARTIAL ->
TIM3_REMAP = 10b). The f1 table gives each function its AFIO_MAPR field and the set of field
values that put the signal on that pin; stm32f1/gpio.hpp intersects them over a pin list.
"""
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

CHIPS = {
    "h5": {
        "ns": "stm32h5",
        "modes": "IP/GPIO-STM32H56x_gpio_v1_0_Modes.xml",
        "packages": [("LQFP100", "STM32H563VITx")],
    },
    "h7": {
        "ns": "stm32h7",
        # the H743's GPIO IP version in STM32H743VITx.xml: STM32H747_gpio_v1_0
        "modes": "IP/GPIO-STM32H747_gpio_v1_0_Modes.xml",
        "packages": [("LQFP100", "STM32H743VITx")],
    },
    "g4": {
        "ns": "stm32g4",
        "modes": "IP/GPIO-STM32G43x_gpio_v1_0_Modes.xml",
        # (enum name, CubeMX part file); LQFP32 = UFQFPN32 and LQFP64 = UFBGA64 pin for pin,
        # the "...TxZ" order codes are left out.
        "packages": [
            ("LQFP32", "STM32G431K(6-8-B)Tx"),
            ("UFQFPN32", "STM32G431K(6-8-B)Ux"),
            ("LQFP48", "STM32G431C(6-8-B)Tx"),
            ("UFQFPN48", "STM32G431C(6-8-B)Ux"),
            ("WLCSP49", "STM32G431CBYx"),
            ("LQFP64", "STM32G431R(6-8-B)Tx"),
            ("UFBGA64", "STM32G431R(6-8-B)Ix"),
            ("LQFP80", "STM32G431M(6-8-B)Tx"),
            ("LQFP100", "STM32G431V(6-8-B)Tx"),
        ],
    },
    "f1": {
        "ns": "stm32f1",
        # the STM32F103x8/xB in the LQFP48 (the UFQFPN48 "CBUx" has the same pins)
        "modes": "IP/GPIO-STM32F103x8_gpio_v1_0_Modes.xml",
        "packages": [("LQFP48", "STM32F103C(8-B)Tx")],
        "remap": True,
    },
}

# AFIO_MAPR fields (RM0008 rev 21, 9.4.2) by the prefix of CubeMX's remap block name, and the field
# value each HAL remap macro writes (stm32f1xx_hal_gpio_ex.h).
F1_REMAP_FIELDS = ["SPI1", "I2C1", "USART1", "USART2", "USART3", "TIM1", "TIM2", "TIM3", "TIM4", "CAN", "PD01"]
F1_REMAP_MACROS = {
    "SPI1_ENABLE": ("SPI1", 1),
    "I2C1_ENABLE": ("I2C1", 1),
    "USART1_ENABLE": ("USART1", 1),
    "USART2_ENABLE": ("USART2", 1),
    "USART3_PARTIAL": ("USART3", 1),
    "USART3_ENABLE": ("USART3", 3),
    "TIM1_PARTIAL": ("TIM1", 1),
    "TIM1_ENABLE": ("TIM1", 3),
    "TIM2_PARTIAL_1": ("TIM2", 1),
    "TIM2_PARTIAL_2": ("TIM2", 2),
    "TIM2_ENABLE": ("TIM2", 3),
    "TIM3_PARTIAL": ("TIM3", 2),
    "TIM3_ENABLE": ("TIM3", 3),
    "TIM4_ENABLE": ("TIM4", 1),
    "CAN1_2": ("CAN", 2),
    "CAN1_3": ("CAN", 3),
    "PD01_ENABLE": ("PD01", 1),
}


def f1_field_ident(field):
    return field.capitalize() if field != "PD01" else "Pd01"


def read_f1_remaps(db, modes):
    """(pin, signal) -> (field, mask of the allowed field values); pins without remap are absent."""
    remaps = {}
    for e in ET.parse(db / modes).getroot().iter():
        if strip(e.tag) != "GPIO_Pin":
            continue
        pin = re.match(r"P[A-I]\d+", e.get("Name"))
        if not pin:
            continue
        for sig in e:
            if strip(sig.tag) != "PinSignal":
                continue
            field, mask = None, 0
            for block in sig:
                if strip(block.tag) != "RemapBlock":
                    continue
                m = re.match(r"(.+)_REMAP\d+$", block.get("Name"))
                if not m:
                    raise SystemExit(f"unknown remap block {block.get('Name')}")
                block_field = m.group(1)
                if block.get("DefaultRemap") == "true":
                    value = 0
                else:
                    macro = next((v.text for v in block.iter() if strip(v.tag) == "PossibleValue" and v.text), None)
                    if macro is None or not macro.startswith("__HAL_AFIO_REMAP_"):
                        raise SystemExit(f"{pin.group(0)} {sig.get('Name')}: no remap macro in {block.get('Name')}")
                    block_field, value = F1_REMAP_MACROS[macro[len("__HAL_AFIO_REMAP_"):]]
                if field not in (None, block_field):
                    raise SystemExit(f"{pin.group(0)} {sig.get('Name')}: two remap fields")
                field = block_field
                mask |= 1 << value
            if field is not None:
                remaps[(pin.group(0), sig.get("Name"))] = (field, mask)
    return remaps


def main_f1(db, chip):
    ns = chip["ns"]
    (package, part), = chip["packages"]
    package_pins = read_package(db, part)
    remaps = read_f1_remaps(db, chip["modes"])
    entries = sorted(((p, s) for p, signals in package_pins.items() for s in signals),
                     key=lambda k: (pin_key(k[0]), k[1]))
    signals = sorted({s for _, s in entries})
    pin_list = sorted(package_pins, key=pin_key)
    out = sys.stdout
    out.write(f"""// SPDX-License-Identifier: MIT
//
// GENERATED by tools/gen_pin_functions.py from ST's pin database (STM32CubeMX db: {part}.xml,
// {chip["modes"]}) - do not edit. The functions of every I/O pin of the {part} ({package}):
// the AFIO_MAPR field that moves the signal (None: the signal is fixed to its pin) and the set of
// field values that put it on this pin (bit v = field value v). Analog functions (ADC inputs)
// have field None and analog = true. gpio.hpp looks pins up here at compile time.
#pragma once

#include <array>
#include <cstdint>
#include <utility>

#include "rtos/port/stm32/rcc.hpp"

namespace rtos::port::{ns}::gpio {{

using stm32::GpioPort;

/// Peripheral signals as ST names them (datasheet, reference manual).
enum class Signal : std::uint16_t {{
""")
    for s in signals:
        out.write(f"    {ident(s)},\n")
    out.write("};\n\n/// The remap fields of AFIO_MAPR (RM0008 9.4.2).\n")
    out.write("enum class Remap : std::uint8_t {\n    None,\n")
    for f in F1_REMAP_FIELDS:
        out.write(f"    {f1_field_ident(f)},\n")
    out.write("""};

struct PinFunction {
    GpioPort port;
    std::uint8_t number;
    Signal signal;
    Remap remap;
    std::uint8_t values;  ///< bit v: the signal is on this pin when the remap field is v
    bool analog;          ///< an ADC input: the pin in analog mode
};

""")
    out.write(f"inline constexpr std::array<PinFunction, {len(entries)}> pin_functions{{{{\n")
    for pin, s in entries:
        field, mask = remaps.get((pin, s), (None, 0))
        remap = f1_field_ident(field) if field else "None"
        analog = "true" if re.match(r"ADC\d+_IN\d+$", s) else "false"
        out.write(f"    {{GpioPort::{pin[1]}, {int(pin[2:])}, Signal::{ident(s)}, Remap::{remap}, 0x{mask:X}, {analog}}},\n")
    out.write("}};\n\n")
    out.write(f"/// Every I/O pin of the package ({len(pin_list)}).\n")
    out.write(f"inline constexpr std::array<std::pair<GpioPort, std::uint8_t>, {len(pin_list)}> package_pins{{{{\n")
    for pin in pin_list:
        out.write(f"    {{GpioPort::{pin[1]}, {int(pin[2:])}}},\n")
    out.write("}};\n")
    out.write(f"\n}}  // namespace rtos::port::{ns}::gpio\n")


def ident(signal):
    """C++ name of a signal: DEBUG_JTCK-SWCLK -> DEBUG_JTCK_SWCLK."""
    return re.sub(r"[^A-Za-z0-9_]", "_", signal)


def strip(tag):
    return tag.split('}', 1)[-1]


def pin_key(name):
    return (name[1], int(name[2:]))


def read_package(db, part):
    """{"PA0": [signals]} of the I/O pins of one package."""
    pins = {}
    for e in ET.parse(db / f"{part}.xml").getroot():
        if strip(e.tag) != "Pin":
            continue
        m = re.match(r"P([A-I])(\d+)", e.get("Name"))
        if not m or e.get("Type") != "I/O":
            continue
        name = f"P{m.group(1)}{m.group(2)}"
        pins[name] = [s.get("Name") for s in e if strip(s.tag) == "Signal" and s.get("Name") != "GPIO"]
    return pins


def read_af(db, modes):
    af = {}  # (pin, signal) -> number
    for e in ET.parse(db / modes).getroot().iter():
        if strip(e.tag) != "GPIO_Pin":
            continue
        pin = re.match(r"P[A-I]\d+", e.get("Name"))
        if not pin:
            continue
        for sig in e:
            if strip(sig.tag) != "PinSignal":
                continue
            for v in sig.iter():
                if strip(v.tag) == "PossibleValue" and v.text and v.text.startswith("GPIO_AF"):
                    af[(pin.group(0), sig.get("Name"))] = int(re.match(r"GPIO_AF(\d+)_", v.text).group(1))
    return af


def main():
    sys.stdout.reconfigure(newline="\n")  # LF on Windows too (.gitattributes)
    db = Path(sys.argv[1])
    chip = CHIPS[sys.argv[2]]
    if chip.get("remap"):
        main_f1(db, chip)
        return
    ns = chip["ns"]
    packages = [(name, part, read_package(db, part)) for name, part in chip["packages"]]
    af = read_af(db, chip["modes"])
    single = len(packages) == 1

    functions = {}  # (pin, signal) -> package mask
    pins = {}  # pin -> package mask
    for bit, (_, _, package_pins) in enumerate(packages):
        for pin, signals in package_pins.items():
            pins[pin] = pins.get(pin, 0) | (1 << bit)
            for s in signals:
                functions[(pin, s)] = functions.get((pin, s), 0) | (1 << bit)
    signals = sorted({s for _, s in functions})
    entries = sorted(functions, key=lambda k: (pin_key(k[0]), k[1]))
    pin_list = sorted(pins, key=pin_key)

    out = sys.stdout
    sources = ", ".join(f"{part}.xml" for _, part, _ in packages)
    if single:
        what = f"The functions of every I/O pin of the {packages[0][1]} ({packages[0][0]})"
    else:
        what = "The functions of every I/O pin of every package (mask: the packages that have it)"
    out.write(f"""// SPDX-License-Identifier: MIT
//
// GENERATED by tools/gen_pin_functions.py from ST's pin database (STM32CubeMX db: {sources},
// {chip["modes"]}) - do not edit. {what}:
// alternate functions with their AF number, additional functions (analog: ADC inputs, UCPD,
// USB, ...) with af == additional. gpio.hpp looks pins up here at compile time.
#pragma once

#include <array>
#include <cstdint>
#include <utility>

#include "rtos/port/stm32/gpio.hpp"

namespace rtos::port::{ns}::gpio {{

using stm32::GpioPort;
using stm32::gpio::additional;

/// Peripheral signals as ST names them (datasheet, reference manual).
enum class Signal : std::uint16_t {{
""")
    for s in signals:
        out.write(f"    {ident(s)},\n")
    out.write("};\n\nusing PinFunction = stm32::gpio::PinFunction<Signal>;\n\n")

    if single:
        out.write(f"inline constexpr std::array<PinFunction, {len(entries)}> pin_functions{{{{\n")
        for pin, s in entries:
            a = af.get((pin, s), 0xFF)
            out.write(f"    {{GpioPort::{pin[1]}, {int(pin[2:])}, Signal::{ident(s)}, {'additional' if a == 0xFF else a}}},\n")
        out.write("}};\n\n")
        out.write(f"/// Every I/O pin of the package ({len(pin_list)}).\n")
        out.write(f"inline constexpr std::array<std::pair<GpioPort, std::uint8_t>, {len(pin_list)}> package_pins{{{{\n")
        for pin in pin_list:
            out.write(f"    {{GpioPort::{pin[1]}, {int(pin[2:])}}},\n")
        out.write("}};\n")
    else:
        out.write("/// The packages, as bits of the masks below.\n")
        out.write("enum class Package : std::uint8_t {\n")
        for bit, (name, part, package_pins) in enumerate(packages):
            out.write(f"    {name} = {bit},  ///< {part}: {len(package_pins)} I/O pins\n")
        out.write("};\n\n")
        out.write("""struct PackagedPinFunction {
    PinFunction function;
    std::uint16_t packages;  ///< bit Package: this package has the function on this pin
};

struct PackagedPin {
    GpioPort port;
    std::uint8_t number;
    std::uint16_t packages;
};

""")
        out.write(f"inline constexpr std::array<PackagedPinFunction, {len(entries)}> all_pin_functions{{{{\n")
        for pin, s in entries:
            a = af.get((pin, s), 0xFF)
            out.write(f"    {{{{GpioPort::{pin[1]}, {int(pin[2:])}, Signal::{ident(s)}, {'additional' if a == 0xFF else a}}}, "
                      f"0x{functions[(pin, s)]:03X}}},\n")
        out.write("}};\n\n")
        out.write(f"/// Every I/O pin of any package ({len(pin_list)}).\n")
        out.write(f"inline constexpr std::array<PackagedPin, {len(pin_list)}> all_pins{{{{\n")
        for pin in pin_list:
            out.write(f"    {{GpioPort::{pin[1]}, {int(pin[2:])}, 0x{pins[pin]:03X}}},\n")
        out.write("}};\n")
    out.write(f"\n}}  // namespace rtos::port::{ns}::gpio\n")


if __name__ == "__main__":
    main()
