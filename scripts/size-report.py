#!/usr/bin/env python3
"""Markdown report of firmware flash / static-SRAM use, and the change against a
baseline build (the PR's base commit). Posted on PRs by ci.yml; also readable
locally:

    scripts/size-report.py --head-elf .pio/build/mega/firmware.elf \\
        --base-elf /path/to/other/firmware.elf --base-label "main (abc1234)"

Flash is avr-size's "Program" (.text + .data), static SRAM its "Data"
(.data + .bss + .noinit) - the same numbers scripts/check-*-budget.sh enforce.
The symbol table is for finding *what* grew: avr-nm types T/t/W/w/R/r count as
flash, D/d/B/b/V/v as SRAM (initialised data also takes flash for its image,
which this per-symbol view leaves out).

Without --base-elf (or if it does not exist) only the current sizes are shown.
Exit status: 0 on success, 2 on unreadable input.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

FLASH_TYPES = set("TtWwRr")
RAM_TYPES = set("DdBbVv")
TOP_SYMBOLS = 10


def parse_avr_size(text: str) -> dict[str, int]:
    sizes = {}
    for key, label in (("flash", "Program"), ("ram", "Data")):
        m = re.search(rf"^{label}:\s+(\d+)\s+bytes", text, re.M)
        if not m:
            raise ValueError(f"no '{label}:' line in avr-size output")
        sizes[key] = int(m.group(1))
    return sizes


def parse_nm(text: str) -> dict[str, tuple[str, int]]:
    """avr-nm --size-sort -S --radix=d output -> {symbol: (flash|ram, bytes)}."""
    symbols: dict[str, tuple[str, int]] = {}
    for line in text.splitlines():
        fields = line.split(None, 3)
        if len(fields) != 4 or not fields[1].isdigit() or len(fields[2]) != 1:
            continue
        kind = "flash" if fields[2] in FLASH_TYPES else "ram" if fields[2] in RAM_TYPES else None
        if kind:
            _, size = symbols.get(fields[3], (kind, 0))
            symbols[fields[3]] = (kind, size + int(fields[1]))
    return symbols


def read_budgets(root: Path) -> dict[str, int]:
    budgets = {}
    for key, script, var in (("flash", "check-flash-budget.sh", "TOTAL_FLASH"), ("ram", "check-ram-budget.sh", "TOTAL_SRAM")):
        m = re.search(rf"^{var}=(\d+)", (root / "scripts" / script).read_text(), re.M)
        if not m:
            raise ValueError(f"{var} not found in scripts/{script}")
        budgets[key] = int(m.group(1))
    return budgets


def delta(new: int, old: int) -> str:
    d = new - old
    return "no change" if d == 0 else f"{d:+,} B"


def cell(used: int, total: int) -> str:
    return f"{used:,} B ({used * 100 / total:.1f}%)"


def render(head, base, budgets, head_syms=None, base_syms=None, base_label="base") -> str:
    out = ["### Firmware size (`env:mega`)", ""]
    if base:
        out += [f"| | {base_label} | This PR | Change |", "|---|---:|---:|---:|"]
    else:
        out += ["| | This PR |", "|---|---:|"]
    for key, label in (("flash", "Flash"), ("ram", "Static SRAM")):
        row = f"| {label} | "
        if base:
            row += f"{cell(base[key], budgets[key])} | {cell(head[key], budgets[key])} | **{delta(head[key], base[key])}** |"
        else:
            row += f"{cell(head[key], budgets[key])} |"
        out.append(row)
    if not base:
        out += ["", "_No baseline build, so no change is shown._"]
    elif head_syms is not None and base_syms is not None:
        rows = []
        for name in set(head_syms) | set(base_syms):
            kind = (head_syms.get(name) or base_syms[name])[0]
            old, new = base_syms.get(name, (kind, 0))[1], head_syms.get(name, (kind, 0))[1]
            if new != old:
                note = " (new)" if name not in base_syms else " (removed)" if name not in head_syms else ""
                rows.append((abs(new - old), name, kind, old, new, note))
        if rows:
            rows.sort(reverse=True)
            out += ["", f"<details><summary>Largest symbol changes ({min(len(rows), TOP_SYMBOLS)} of {len(rows)})</summary>", "", "| Symbol | Where | Before | After | Change |", "|---|---|---:|---:|---:|"]
            for _, name, kind, old, new, note in rows[:TOP_SYMBOLS]:
                out.append(f"| `{name.replace('|', chr(92) + '|')}`{note} | {'flash' if kind == 'flash' else 'SRAM'} | {old:,} | {new:,} | {new - old:+,} |")
            out += ["", "</details>"]
    out += ["", f"_Budgets: flash {budgets['flash']:,} B, SRAM {budgets['ram']:,} B. Compared with {base_label}._" if base else f"_Budgets: flash {budgets['flash']:,} B, SRAM {budgets['ram']:,} B._"]
    return "\n".join(out) + "\n"


def find_tool(name: str, avr_bin: Path | None) -> str:
    if avr_bin and (avr_bin / name).exists():
        return str(avr_bin / name)
    found = shutil.which(name) or next(iter(Path.home().glob(f".platformio/packages/toolchain-atmelavr/bin/{name}")), None)
    if not found:
        raise ValueError(f"{name} not found (expected in the PlatformIO AVR toolchain; see --avr-bin)")
    return str(found)


def measure(elf: Path, avr_bin: Path | None):
    size = subprocess.run([find_tool("avr-size", avr_bin), "--format=avr", "--mcu=atmega2560", str(elf)], capture_output=True, text=True, check=True).stdout
    nm = subprocess.run([find_tool("avr-nm", avr_bin), "--size-sort", "-S", "--radix=d", "--demangle", str(elf)], capture_output=True, text=True, check=True).stdout
    return parse_avr_size(size), parse_nm(nm)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--head-elf", type=Path, required=True)
    parser.add_argument("--base-elf", type=Path)
    parser.add_argument("--base-label", default="base")
    parser.add_argument("--avr-bin", type=Path, help="directory holding avr-size and avr-nm")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent.parent)
    args = parser.parse_args(argv)
    try:
        budgets = read_budgets(args.root)
        head, head_syms = measure(args.head_elf, args.avr_bin)
        base = base_syms = None
        if args.base_elf and args.base_elf.exists():
            base, base_syms = measure(args.base_elf, args.avr_bin)
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(f"size-report: {exc}", file=sys.stderr)
        return 2
    sys.stdout.write(render(head, base, budgets, head_syms, base_syms, args.base_label))
    return 0


if __name__ == "__main__":
    sys.exit(main())
