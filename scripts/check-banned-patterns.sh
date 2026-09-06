#!/usr/bin/env bash
# Hard bans pulled directly from .claude/skills/avr-c-memory-efficiency and
# avr-c-speed-optimization: no dynamic allocation on an 8-bit MCU with no
# heap worth the flash cost, and no float-formatting printf (pulls in a large
# soft-float routine). These are deterministic, zero-judgment checks, safe to
# hard-fail on. Run from the repo root; used by both the pre-commit hook and CI.
set -euo pipefail

SEARCH_DIRS=()
for d in src include; do
    [ -d "$d" ] && SEARCH_DIRS+=("$d")
done

if [ ${#SEARCH_DIRS[@]} -eq 0 ]; then
    exit 0
fi

fail=0

check() {
    local pattern="$1"
    local message="$2"
    local matches
    matches=$(grep -rnE "$pattern" "${SEARCH_DIRS[@]}" --include='*.c' --include='*.h' || true)
    if [ -n "$matches" ]; then
        echo "BANNED PATTERN: $message"
        echo "$matches"
        echo
        fail=1
    fi
}

check '\b(malloc|calloc|realloc|free)\s*\(' \
    "dynamic allocation is not allowed in this codebase (no heap on an 8-bit MCU with 8KB SRAM) — use a fixed-size static/global array instead"

check '\bnew\b' \
    "'new' is not allowed (this is C, and dynamic allocation is banned project-wide anyway)"

check '%[-+ 0#]*[0-9]*\.?[0-9]*[fFeEgG]' \
    "float-formatting printf/sprintf specifier found — pulls in avr-libc's soft-float formatting routine; format as fixed-point integers instead (see avr-c-speed-optimization)"

if [ "$fail" -ne 0 ]; then
    echo "One or more banned patterns found. See .claude/skills/avr-c-memory-efficiency and avr-c-speed-optimization for the reasoning." >&2
    exit 1
fi

exit 0
