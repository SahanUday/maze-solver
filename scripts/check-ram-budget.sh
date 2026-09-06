#!/usr/bin/env bash
# Enforces an SRAM headroom budget on the built .elf. ATmega2560 has 8192 bytes
# of SRAM total and no heap (malloc is banned project-wide, see
# check-banned-patterns.sh) — the only two consumers are static .data+.bss and
# the call stack. There's no stack-overflow protection on AVR and several ISRs
# (encoder/timer/ADC) can nest on top of whatever the main loop's call depth is
# at that instant, so we reserve a fixed 25% of total SRAM for stack headroom
# no matter how large the static usage grows, rather than letting the budget
# creep toward 100% as features get added. See
# .claude/skills/avr-c-memory-efficiency for the full reasoning.
#
# Usage: scripts/check-ram-budget.sh <path-to-firmware.elf>
set -euo pipefail

ELF="${1:?usage: check-ram-budget.sh <path-to-firmware.elf>}"
TOTAL_SRAM=8192
WARN_PCT=60
FAIL_PCT=75

AVR_SIZE=$(command -v avr-size || find "${HOME}/.platformio" -maxdepth 4 -iname 'avr-size' 2>/dev/null | head -1 || true)
if [ -z "$AVR_SIZE" ]; then
    echo "avr-size not found (expected in the PlatformIO AVR toolchain)" >&2
    exit 2
fi

# `Data: N bytes (X.X% Full)` is .data + .bss + .noinit — the actual runtime
# SRAM footprint before the stack is even considered.
DATA_BYTES=$("$AVR_SIZE" --format=avr --mcu=atmega2560 "$ELF" | awk '/^Data:/ {print $2}')

if [ -z "$DATA_BYTES" ]; then
    echo "Could not parse avr-size output for $ELF" >&2
    exit 2
fi

PCT=$(( DATA_BYTES * 100 / TOTAL_SRAM ))

echo "SRAM (.data+.bss+.noinit): ${DATA_BYTES} / ${TOTAL_SRAM} bytes (${PCT}%)"
echo "Reserved for stack: $(( TOTAL_SRAM * (100 - FAIL_PCT) / 100 )) bytes minimum (hard-fail threshold)"

if [ "$PCT" -ge "$FAIL_PCT" ]; then
    echo "FAIL: static RAM usage is ${PCT}% of total SRAM (threshold ${FAIL_PCT}%)." >&2
    echo "Fewer than $(( TOTAL_SRAM * (100 - FAIL_PCT) / 100 )) bytes would be left for the stack, which is too tight given ISR nesting (encoder/timer/ADC) on top of the main loop's own call depth." >&2
    echo "See .claude/skills/avr-c-memory-efficiency for how to reduce static usage (PROGMEM tables, smaller types, avoid unnecessary globals)." >&2
    exit 1
elif [ "$PCT" -ge "$WARN_PCT" ]; then
    echo "WARNING: static RAM usage is ${PCT}% of total SRAM (soft threshold ${WARN_PCT}%). Not failing the build, but keep an eye on it."
fi

exit 0
