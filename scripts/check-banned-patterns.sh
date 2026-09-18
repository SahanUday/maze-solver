#!/usr/bin/env bash
# Hard bans: no dynamic allocation, no STL containers, no virtual dispatch, no
# float-formatting printf. Exceptions/RTTI are rejected by the compiler itself
# (-fno-exceptions -fno-rtti), not grepped for here. Used by pre-commit and CI.
set -euo pipefail

SEARCH_DIRS=()
for d in src include; do
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

if [ "$fail" -ne 0 ]; then
    exit 1
fi

exit 0
