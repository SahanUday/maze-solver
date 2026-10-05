#!/usr/bin/env bash
# Checks program flash (.text + .data) against the ATmega2560 budget.
# TOTAL_FLASH is PlatformIO's megaatmega2560 maximum (256 KB minus the 8 KB
# stk500v2 bootloader). The thresholds are assumptions, not measurements - flash
# is the roomy resource here; the check exists to catch a runaway table or an
# accidentally linked library, not to police a few hundred bytes.
#
# Usage: scripts/check-flash-budget.sh <path-to-firmware.elf>
set -euo pipefail

ELF="${1:?usage: check-flash-budget.sh <path-to-firmware.elf>}"
TOTAL_FLASH=253952
WARN_PCT=60
FAIL_PCT=85

AVR_SIZE=$(command -v avr-size || find "${HOME}/.platformio" -maxdepth 4 -iname 'avr-size' 2>/dev/null | head -1 || true)
if [ -z "$AVR_SIZE" ]; then
    echo "avr-size not found (expected in the PlatformIO AVR toolchain)" >&2
    exit 2
fi

PROGRAM_BYTES=$("$AVR_SIZE" --format=avr --mcu=atmega2560 "$ELF" | awk '/^Program:/ {print $2}')

if [ -z "$PROGRAM_BYTES" ]; then
    echo "Could not parse avr-size output for $ELF" >&2
    exit 2
fi

PCT=$(( PROGRAM_BYTES * 100 / TOTAL_FLASH ))

echo "Flash (.text+.data): ${PROGRAM_BYTES} / ${TOTAL_FLASH} bytes (${PCT}%)"

if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    echo "| Flash | ${PROGRAM_BYTES} | ${TOTAL_FLASH} | ${PCT}% |" >> "$GITHUB_STEP_SUMMARY"
fi

if [ "$PCT" -ge "$FAIL_PCT" ]; then
    echo "FAIL: flash usage is ${PCT}% of the ${TOTAL_FLASH}-byte budget (threshold ${FAIL_PCT}%)." >&2
    echo "Check the .map file for large symbols; keep tables in PROGMEM and avoid heavy libraries." >&2
    exit 1
elif [ "$PCT" -ge "$WARN_PCT" ]; then
    echo "WARNING: flash usage is ${PCT}% of the budget (soft threshold ${WARN_PCT}%)."
fi

exit 0
