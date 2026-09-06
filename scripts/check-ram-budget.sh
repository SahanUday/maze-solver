#!/usr/bin/env bash
# Enforces an SRAM headroom budget on the built .elf. No heap (malloc is
# banned project-wide), so the only consumers are static .data+.bss and the
# stack - and AVR has no stack-overflow protection, with several ISRs able to
# nest on top of the main loop's call depth. A fixed 25% of SRAM is reserved
# for stack no matter how large static usage grows.
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

# `Data: N bytes` is .data + .bss + .noinit - the actual runtime SRAM
# footprint before the stack is even considered.
DATA_BYTES=$("$AVR_SIZE" --format=avr --mcu=atmega2560 "$ELF" | awk '/^Data:/ {print $2}')

if [ -z "$DATA_BYTES" ]; then
    echo "Could not parse avr-size output for $ELF" >&2
    exit 2
fi

PCT=$(( DATA_BYTES * 100 / TOTAL_SRAM ))

echo "SRAM (.data+.bss+.noinit): ${DATA_BYTES} / ${TOTAL_SRAM} bytes (${PCT}%)"

if [ "$PCT" -ge "$FAIL_PCT" ]; then
    echo "FAIL: static RAM usage is ${PCT}% of total SRAM (threshold ${FAIL_PCT}%)." >&2
    echo "Reduce static usage: PROGMEM tables, smaller types, fewer globals." >&2
    exit 1
elif [ "$PCT" -ge "$WARN_PCT" ]; then
    echo "WARNING: static RAM usage is ${PCT}% of total SRAM (soft threshold ${WARN_PCT}%)."
fi

exit 0
