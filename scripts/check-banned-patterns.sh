#!/usr/bin/env bash
# Hard bans pulled from .claude/skills/avr-c-memory-efficiency,
# avr-c-speed-optimization, and embedded-code-architecture: no dynamic
# allocation on an 8-bit MCU with no heap worth the flash cost, no
# float-formatting printf (pulls in a large soft-float routine), no virtual
# dispatch (no runtime polymorphism this project needs), no STL containers
# (imply heap allocation). Exceptions and RTTI are enforced by the compiler
# itself (-fno-exceptions -fno-rtti in platformio.ini) rather than grepped for
# here — a compile error is more reliable than a regex, and grepping for
# `try`/`throw` would false-positive on ordinary English comments.
#
# These are deterministic, zero-judgment checks, safe to hard-fail on. Run
# from the repo root; used by both the pre-commit hook and CI.
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
    "dynamic allocation is not allowed in this codebase (no heap on an 8-bit MCU with 8KB SRAM) — use a fixed-size static/global array instead"

# Contextual, not a bare `\bnew\b`/`\bdelete\b` — those would false-positive on
# ordinary English comments ("a new sensor", "delete the old approach"). Real
# `new`/`delete` expressions are shaped like `= new Type(` / `new Type[` /
# `delete ptr` / `delete[] arr`.
check '[=(,]\s*new\s+[A-Za-z_][A-Za-z0-9_:]*\s*[[(]' \
    "'new' is not allowed — dynamic allocation is banned project-wide (no heap on an 8-bit MCU)"

# Real `delete`/`delete[]` expressions are a pointer expression terminated by
# `;` (`delete ptr;`, `delete[] arr;`, `delete obj.member;`) — requiring the
# semicolon avoids matching English prose like "delete the old approach".
# NOTE on the character class below: a `]` is only literal in a POSIX bracket
# expression as its very first member (right after `[` or `[^`) — anywhere
# else it closes the class early. Putting `]` first (`[].[...`) is what makes
# `]`, `[`, `.`, `>`, `-` all be treated as literal members here instead of
# silently truncating the class (this bit the very first version of this
# check: it matched nothing, ever, under real GNU grep).
check '\bdelete(\[\])?\s+[A-Za-z_][]A-Za-z0-9_.[>-]*\s*;' \
    "'delete'/'delete[]' found, implying 'new' was used somewhere — dynamic allocation is banned project-wide"

# STL containers imply heap allocation we can't afford; fixed-size arrays only.
check '#include\s*<(vector|map|unordered_map|unordered_set|set|list|deque|string|memory|forward_list)>' \
    "STL container/heap-backed header included — not allowed on an 8-bit MCU with no heap; use a fixed-size array sized for the known maze/sensor dimensions instead"

# Virtual dispatch: a function *declaration* shaped like `virtual <type> name(`,
# not a bare `\bvirtual\b` (lower false-positive rate against comments).
check '\bvirtual\s+[A-Za-z_][A-Za-z0-9_:<>* ]*\s+[A-Za-z_][A-Za-z0-9_]*\s*\(' \
    "virtual function found — this project has one fixed hardware configuration known at compile time, so runtime polymorphism/vtables buy nothing and only cost flash + an indirection; use a plain function or, if genuinely needed, a template instead"

check '%[-+ 0#]*[0-9]*\.?[0-9]*[fFeEgG]' \
    "float-formatting printf/sprintf specifier found — pulls in avr-libc's soft-float formatting routine; format as fixed-point integers instead (see avr-c-speed-optimization)"

if [ "$fail" -ne 0 ]; then
    echo "One or more banned patterns found. See .claude/skills/avr-c-memory-efficiency, avr-c-speed-optimization, and embedded-code-architecture for the reasoning." >&2
    exit 1
fi

exit 0
