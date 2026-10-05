#!/usr/bin/env bash
# Hard bans: no dynamic allocation, no STL containers, no virtual dispatch, no
# float-formatting printf. Exceptions/RTTI are rejected by the compiler itself
# (-fno-exceptions -fno-rtti in [common] build_src_flags and lib/maze's
# library.json, so src/ and lib/maze in every env), not grepped for here. Also
# enforces the HAL access policy (src/hal/ goes through AVR registers, not the
# Arduino GPIO API; see docs/architecture/decisions/0001). Used by pre-commit
# and CI.
set -euo pipefail

SEARCH_DIRS=()
for d in src include lib; do
    [ -d "$d" ] && SEARCH_DIRS+=("$d")
done

if [ ${#SEARCH_DIRS[@]} -eq 0 ]; then
    exit 0
fi

EXTS=(--include='*.c' --include='*.h' --include='*.cpp' --include='*.hpp' --include='*.cc')

fail=0

check() {
    local pattern="$1"
    local message="$2"
    local matches
    matches=$(grep -rnE "$pattern" "${SEARCH_DIRS[@]}" "${EXTS[@]}" || true)
    if [ -n "$matches" ]; then
        echo "BANNED PATTERN: $message"
        echo "$matches"
        echo
        fail=1
    fi
}

check '\b(malloc|calloc|realloc|free)\s*\(' \
    "dynamic allocation banned (no heap on an 8-bit MCU) - use a fixed-size array"

# Contextual, not a bare \bnew\b/\bdelete\b, to avoid matching English prose
# ("a new sensor", "delete the old approach").
check '[=(,]\s*new\s+[A-Za-z_][A-Za-z0-9_:]*\s*[[(]' \
    "'new' banned - dynamic allocation is not allowed"

# NOTE: `]` is only a literal inside a POSIX bracket expression as its first
# member - anywhere else it closes the class early. `[]A-Za-z0-9_.[>-]` is
# deliberate: `]` first, so it (and `[`) are treated as literal members
# instead of silently truncating the class.
check '\bdelete(\[\])?\s+[A-Za-z_][]A-Za-z0-9_.[>-]*\s*;' \
    "'delete' banned - dynamic allocation is not allowed"

check '#include\s*<(vector|map|unordered_map|unordered_set|set|list|deque|string|memory|forward_list)>' \
    "STL container header banned (implies heap allocation) - use a fixed-size array"

# A declaration shaped like `virtual <type> name(`, not a bare \bvirtual\b.
check '\bvirtual\s+[A-Za-z_][A-Za-z0-9_:<>* ]*\s+[A-Za-z_][A-Za-z0-9_]*\s*\(' \
    "virtual function banned - no runtime polymorphism needed (fixed hardware config)"

check '%[-+ 0#]*[0-9]*\.?[0-9]*[fFeEgG]' \
    "float-formatting printf/sprintf specifier banned - format as fixed-point integers"

# HAL policy (ADR 0001): drivers touch registers directly. Scoped to src/hal/ -
# main.cpp and other glue may use the Arduino framework. Comment-only mentions
# are ignored so a driver can explain why it avoids digitalWrite().
#
# The comment is removed from each line BEFORE matching. Filtering out whole
# lines that mention the API after a '//' let a real call hide behind a comment
# that happened to name it ("digitalWrite(13, 1); // digitalWrite() is fine").
HAL_API='\b(pinMode|digitalWrite|digitalRead|analogRead|analogWrite|analogReference|attachInterrupt|detachInterrupt|pulseIn|shiftIn|shiftOut|tone|noTone)\s*\('
if [ -d src/hal ]; then
    hal_matches=$(grep -rn '' src/hal "${EXTS[@]}" \
        | grep -vE '^[^:]+:[0-9]+:[[:space:]]*(\*|/\*)' \
        | sed -E 's|//.*$||' \
        | grep -E "$HAL_API" \
        || true)
    if [ -n "$hal_matches" ]; then
        echo "BANNED PATTERN: Arduino GPIO/ADC/interrupt API inside src/hal/ - HAL drivers must use AVR registers directly (ADR 0001)"
        echo "$hal_matches"
        echo
        fail=1
    fi
fi

if [ "$fail" -ne 0 ]; then
    exit 1
fi

exit 0
