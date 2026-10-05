#!/usr/bin/env python3
"""Pin-map and peripheral-ownership check for the maze-solver firmware.

Several people add drivers in parallel, and two PRs that each pass CI alone can
still collide once merged: the same pin number under two names, or two drivers
claiming the same timer. Git sees different lines, so there is no merge
conflict. This script finds those collisions from the code itself, so there is
no second pin table to keep in sync.

Checks (stdlib only, ~100 ms):
  pins      every PIN_* in include/RobotConfig.h resolves to a real Mega pin
            (literal or A0-A15), no two names share a pin, reserved pins
            (0/1 USB serial, 20/21 I2C) are used only by their owner. An alias
            (`constexpr uint8_t PIN_X = PIN_Y;`) is allowed - it is a rename,
            not a second claim.
  notes     port / INTn / OCnX / PCINTn notes in a trailing `// ...` comment
            ("// PE4, INT4") agree with the pin number.
  asserts   every src/hal module that uses a PIN_* constant has a
            static_assert on it (the convention in encoders.cpp/motors.cpp).
  owners    each peripheral (timerN, intN, pcintN, usartN, adc, twi, spi) is
            driven by one src/ module. Timer0 is reserved for the Arduino core.
            Deliberate sharing: put `// pin-check: shared timer4 - <reason>`
            in every module involved.

Usage: scripts/check-pin-map.py [--root DIR] [--core-header PATH]
  --core-header  also verify the built-in Mega pin table against the Arduino
                 core's pins_arduino.h (CI does this where the core is installed).
Exit status: 0 clean, 1 findings, 2 usage / unreadable input.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from collections import defaultdict
from pathlib import Path

from cpplex import strip_comments

# Arduino Mega 2560 digital pin number -> AVR port bit. Generated from the
# core's digital_pin_to_port_PGM / digital_pin_to_bit_mask_PGM; --core-header
# re-verifies it. A0-A15 are pins 54-69.
MEGA_PINS = (
    "PE0 PE1 PE4 PE5 PG5 PE3 PH3 PH4 PH5 PH6 PB4 PB5 PB6 PB7 PJ1 PJ0 PH1 PH0 PD3 PD2 "
    "PD1 PD0 PA0 PA1 PA2 PA3 PA4 PA5 PA6 PA7 PC7 PC6 PC5 PC4 PC3 PC2 PC1 PC0 PD7 PG2 "
    "PG1 PG0 PL7 PL6 PL5 PL4 PL3 PL2 PL1 PL0 PB3 PB2 PB1 PB0 PF0 PF1 PF2 PF3 PF4 PF5 "
    "PF6 PF7 PK0 PK1 PK2 PK3 PK4 PK5 PK6 PK7"
).split()
NUM_PINS = len(MEGA_PINS)  # 70
ANALOG_BASE = 54

# Datasheet alternate functions: name -> port bit.
ALT_FUNCTIONS = {
    **{f"INT{n}": p for n, p in enumerate(["PD0", "PD1", "PD2", "PD3", "PE4", "PE5", "PE6", "PE7"])},
    **{f"PCINT{n}": f"PB{n}" for n in range(8)},
    "PCINT8": "PE0",
    **{f"PCINT{9 + n}": f"PJ{n}" for n in range(7)},
    **{f"PCINT{16 + n}": f"PK{n}" for n in range(8)},
    "OC0A": "PB7", "OC0B": "PG5", "OC1A": "PB5", "OC1B": "PB6", "OC1C": "PB7",
    "OC2A": "PB4", "OC2B": "PH6", "OC3A": "PE3", "OC3B": "PE4", "OC3C": "PE5",
    "OC4A": "PH3", "OC4B": "PH4", "OC4C": "PH5", "OC5A": "PL3", "OC5B": "PL4",
    "OC5C": "PL5", "ICP4": "PL0", "ICP5": "PL1",
    **{f"ADC{n}": f"PF{n}" for n in range(8)},
    **{f"ADC{8 + n}": f"PK{n}" for n in range(8)},
}  # fmt: skip

# Pin number -> who owns it. The owners are the only names allowed on these.
RESERVED_PINS = {
    0: ("USB serial RX (Serial0)", None),
    1: ("USB serial TX (Serial0)", None),
    20: ("I2C SDA", "PIN_I2C_SDA"),
    21: ("I2C SCL", "PIN_I2C_SCL"),
}

# Peripherals the Arduino core uses for itself; no src/ module may touch them.
RESERVED_RESOURCES = {"timer0": "the Arduino core (millis()/delay())"}

# Register / vector tokens -> peripheral. First group is the instance number
# where there is one. Applied to comment- and string-stripped code.
RESOURCE_PATTERNS = [
    (re.compile(r"\b(?:TCCR|TCNT|OCR|ICR|TIMSK|TIFR)([0-5])[A-C]?\b"), "timer{0}"),
    (re.compile(r"\bISR\s*\(\s*TIMER([0-5])_\w+_vect"), "timer{0}"),
    (re.compile(r"\bISR\s*\(\s*INT([0-7])_vect"), "int{0}"),
    (re.compile(r"\b(?:INT|INTF)([0-7])\b|\bISC([0-7])[01]\b"), "int{0}{1}"),
    (re.compile(r"\bISR\s*\(\s*PCINT([0-2])_vect"), "pcint{0}"),
    (re.compile(r"\b(?:PCMSK|PCIE|PCIF)([0-2])\b"), "pcint{0}"),
    (re.compile(r"\b(?:UCSR|UBRR|UDR)([0-3])[A-CHL]?\b"), "usart{0}"),
    (re.compile(r"\bISR\s*\(\s*USART([0-3])_\w+_vect"), "usart{0}"),
    (re.compile(r"\b(?:ADMUX|ADCSRA|ADCSRB|ADCH|ADCL|ADCW)\b|\bISR\s*\(\s*ADC_vect"), "adc"),
    (re.compile(r"\b(?:TWCR|TWSR|TWBR|TWDR|TWAR|TWAMR)\b|\bISR\s*\(\s*TWI_vect"), "twi"),
    (re.compile(r"\b(?:SPCR|SPSR|SPDR)\b|\bISR\s*\(\s*SPI_STC_vect"), "spi"),
]

SHARED_MARKER = re.compile(r"pin-check:\s*shared\s+([a-z0-9_,\s]+?)(?:\s+-|\s*$)", re.M)


class Finding:
    def __init__(self, path: str, line: int, rule: str, message: str):
        self.path, self.line, self.rule, self.message = path, line, rule, message

    def __str__(self) -> str:
        return f"{self.path}:{self.line}: [{self.rule}] {self.message}"


def resolve_value(token: str, known: dict[str, int]) -> int | None:
    token = token.strip()
    if re.fullmatch(r"\d+", token):
        return int(token)
    m = re.fullmatch(r"A(\d+)", token)
    if m:
        return ANALOG_BASE + int(m.group(1))
    return known.get(token)


def code_only(text: str) -> str:
    """`text` with every line's trailing // comment removed, newlines kept."""
    return "\n".join(line.split("//")[0] for line in text.splitlines())


def parse_pins(text: str):
    """Return (pins, aliases, errors). pins: [(name, number, line, note)]."""
    pins, errors = [], []
    known: dict[str, int] = {}
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        if not re.match(r"\s*constexpr\s+\w+\s+PIN_\w+", lines[i]):
            i += 1
            continue
        start = i
        stmt = lines[i]
        # Strip the comments per line, not once over the whole accumulated
        # statement: `PIN_IR[8] = {  // left to right` has its terminating ';'
        # on a later line, and splitting the joined text on the FIRST '//'
        # would hide it - the loop then swallowed the rest of the file.
        while ";" not in code_only(stmt) and i + 1 < len(lines):
            i += 1
            stmt += "\n" + lines[i]
        i += 1
        code = code_only(stmt)
        note = " ".join(s.split("//", 1)[1] for s in stmt.splitlines() if "//" in s)
        m = re.match(r"\s*constexpr\s+\w+\s+(PIN_\w+)\s*(\[\s*\d*\s*\])?\s*=\s*(.*?);", code, re.S)
        if not m:
            errors.append((start + 1, f"cannot parse pin declaration: {lines[start].strip()}"))
            continue
        name, is_array, rhs = m.group(1), m.group(2), m.group(3).strip()
        if is_array:
            body = re.fullmatch(r"\{(.*)\}", rhs, re.S)
            if not body:
                errors.append((start + 1, f"{name}: expected a {{...}} initializer"))
                continue
            elements = [e for e in body.group(1).split(",") if e.strip()]
            for idx, el in enumerate(elements):
                value = resolve_value(el, known)
                if value is None:
                    errors.append((start + 1, f"{name}[{idx}] = '{el.strip()}': use a number or A0-A15 so the pin check can read it"))
                else:
                    pins.append((f"{name}[{idx}]", value, start + 1, note))
        else:
            value = resolve_value(rhs, known)
            if value is None:
                errors.append((start + 1, f"{name} = '{rhs}': use a number, A0-A15 or another PIN_* name so the pin check can read it"))
                continue
            known[name] = value
            if rhs in known and rhs.startswith("PIN_"):
                continue  # alias: a rename of an existing claim, not a new one
            pins.append((name, value, start + 1, note))
    return pins, errors


def check_pins(root: Path, findings: list[Finding]):
    rel = "include/RobotConfig.h"
    path = root / rel
    if not path.exists():
        return []
    pins, errors = parse_pins(path.read_text())
    for line, message in errors:
        findings.append(Finding(rel, line, "pins", message))

    by_pin = defaultdict(list)
    for name, number, line, note in pins:
        if not 0 <= number < NUM_PINS:
            findings.append(Finding(rel, line, "pins", f"{name} = {number}: the Mega has pins 0-{NUM_PINS - 1} (A0-A15 are {ANALOG_BASE}-{NUM_PINS - 1})"))
            continue
        by_pin[number].append((name, line))
        if number in RESERVED_PINS:
            what, owner = RESERVED_PINS[number]
            if name != owner:
                findings.append(Finding(rel, line, "pins", f"{name} uses pin {number}, reserved for {what}"))
        for owned_number, (what, owner) in RESERVED_PINS.items():
            if owner == name and number != owned_number:
                findings.append(Finding(rel, line, "pins", f"{name} must be pin {owned_number} ({what} is fixed in hardware), found {number}"))
        check_note(rel, name, number, line, note, findings)

    for number, claimants in sorted(by_pin.items()):
        if len(claimants) > 1:
            names = ", ".join(f"{n} (line {ln})" for n, ln in claimants)
            findings.append(Finding(rel, claimants[-1][1], "pins", f"pin {number} ({MEGA_PINS[number]}) is claimed twice: {names}"))
    return pins


def check_note(rel, name, number, line, note, findings):
    if not note:
        return
    port_bit = MEGA_PINS[number]
    for token in re.findall(r"\b(?:P[A-L][0-7]|[A-Z]+[0-9]+[A-C]?)\b", note):
        if re.fullmatch(r"P[A-L][0-7]", token):
            expected = token
        elif token in ALT_FUNCTIONS:
            expected = ALT_FUNCTIONS[token]
        else:
            continue  # not a hardware note ("PWM6", "D13", ...)
        if expected != port_bit:
            findings.append(Finding(rel, line, "notes", f"{name} = {number} is {port_bit}, but the comment says {token} ({expected})"))


def code_files(root: Path, sub: str, exts=(".c", ".cpp", ".h", ".hpp")):
    base = root / sub
    if not base.is_dir():
        return []
    return sorted(p for p in base.rglob("*") if p.suffix in exts)


def module_of(root: Path, path: Path) -> str:
    return str(path.relative_to(root).with_suffix(""))


def static_assert_text(code: str) -> str:
    """Concatenated text of every static_assert(...) in `code`."""
    chunks = []
    for m in re.finditer(r"\bstatic_assert\s*\(", code):
        depth, j = 1, m.end()
        while j < len(code) and depth:
            depth += {"(": 1, ")": -1}.get(code[j], 0)
            j += 1
        chunks.append(code[m.end() : j])
    return "\n".join(chunks)


def check_hal(root: Path, findings: list[Finding]):
    modules = defaultdict(list)
    for path in code_files(root, "src/hal"):
        modules[module_of(root, path)].append(path)
    for module, paths in sorted(modules.items()):
        used, asserted = {}, ""
        for path in paths:
            code = strip_comments(path.read_text())
            asserted += static_assert_text(code)
            for ln, text in enumerate(code.splitlines(), 1):
                for name in re.findall(r"\bPIN_[A-Z0-9_]+\b", text):
                    used.setdefault(name, (path, ln))
        covered = set(re.findall(r"\bPIN_[A-Z0-9_]+\b", asserted))
        for name, (path, ln) in sorted(used.items()):
            if name not in covered:
                findings.append(Finding(str(path.relative_to(root)), ln, "asserts", f"{name} is used here but no static_assert in {module}.* ties it to the register code"))


def check_owners(root: Path, findings: list[Finding]):
    claims = defaultdict(dict)  # resource -> {module: (path, line)}
    shared = defaultdict(set)  # module -> resources declared shared
    for path in code_files(root, "src"):
        raw = path.read_text()
        module = module_of(root, path)
        for m in SHARED_MARKER.finditer(raw):
            shared[module].update(r.strip() for r in re.split(r"[,\s]+", m.group(1)) if r.strip())
        code = strip_comments(raw)
        for pattern, template in RESOURCE_PATTERNS:
            for m in pattern.finditer(code):
                resource = template.format(*[g or "" for g in m.groups()])
                if resource == "int" or not resource:
                    continue
                line = code.count("\n", 0, m.start()) + 1
                claims[resource].setdefault(module, (path, line))

    for resource, owners in sorted(claims.items()):
        if resource in RESERVED_RESOURCES:
            for module, (path, line) in owners.items():
                findings.append(Finding(str(path.relative_to(root)), line, "owners", f"{resource} is reserved for {RESERVED_RESOURCES[resource]}"))
            continue
        if len(owners) < 2:
            continue
        undeclared = [m for m in owners if resource not in shared[m]]
        if undeclared:
            where = ", ".join(f"{m} (line {owners[m][1]})" for m in sorted(owners))
            path, line = owners[sorted(owners)[-1]]
            findings.append(Finding(str(path.relative_to(root)), line, "owners", f"{resource} is driven by more than one module: {where}. If that is deliberate, add '// pin-check: shared {resource} - <reason>' to each"))


def verify_core_table(header: Path) -> list[str]:
    text = header.read_text()

    def array(name):
        body = re.search(name + r"\[\]\s*=\s*\{(.*?)\};", text, re.S).group(1)
        return [x.strip() for x in re.sub(r"//.*", "", body).split(",") if x.strip()]

    bit = re.compile(r"_BV\(\s*(\d)\s*\)")
    ports, masks = array("digital_pin_to_port_PGM"), array("digital_pin_to_bit_mask_PGM")
    table = [f"{p}{bit.search(m).group(1)}" for p, m in zip(ports, masks)]
    if table != MEGA_PINS:
        diff = [f"pin {i}: built-in {a}, core {b}" for i, (a, b) in enumerate(zip(MEGA_PINS, table)) if a != b]
        return diff or [f"table length differs: built-in {len(MEGA_PINS)}, core {len(table)}"]
    return []


def check(root: Path) -> list[Finding]:
    findings: list[Finding] = []
    check_pins(root, findings)
    check_hal(root, findings)
    check_owners(root, findings)
    return findings


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--core-header", type=Path)
    args = parser.parse_args(argv)

    if not (args.root / "include").is_dir() and not (args.root / "src").is_dir():
        print(f"{args.root}: no include/ or src/ - wrong --root?", file=sys.stderr)
        return 2

    if args.core_header:
        try:
            problems = verify_core_table(args.core_header)
        except (OSError, AttributeError) as exc:
            print(f"cannot read the Arduino core pin table from {args.core_header}: {exc}", file=sys.stderr)
            return 2
        if problems:
            print("built-in Mega pin table disagrees with the Arduino core:")
            print("\n".join(problems))
            return 1

    findings = check(args.root)
    on_github = os.environ.get("GITHUB_ACTIONS") == "true"
    for f in findings:
        print(f)
        if on_github:
            print(f"::error file={f.path},line={f.line},title=pin-map ({f.rule})::{f.message}")
    if findings:
        print(f"\npin-map: {len(findings)} problem(s)")
        return 1
    print("pin-map: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
