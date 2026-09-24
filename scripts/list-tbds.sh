#!/usr/bin/env bash
# Lists every <<TBD HW-01>> / <<TBD CAL-01>> placeholder still outstanding.
#
# A comment on a placeholder constant only works as team communication if
# someone happens to scroll past it. This makes "what don't we actually know
# yet" a standing, greppable answer instead of something you have to
# remember or stumble onto - useful both day-to-day and at viva ("why is
# this zero" should point here, not to a half-remembered conversation).
#
# Informational only - does not fail the build. It is expected and correct
# for this to report matches until HW-01/CAL-01 land; it becomes interesting
# once a phase gate says a specific file should be TBD-free.
set -euo pipefail

SEARCH_DIRS=()
for d in src include lib; do
    [ -d "$d" ] && SEARCH_DIRS+=("$d")
done

if [ ${#SEARCH_DIRS[@]} -eq 0 ]; then
    echo "No source directories found."
    exit 0
fi

EXTS=(--include='*.c' --include='*.h' --include='*.cpp' --include='*.hpp' --include='*.cc')

matches=$(grep -rnE '<<TBD [A-Z0-9-]+>>' "${SEARCH_DIRS[@]}" "${EXTS[@]}" || true)

if [ -z "$matches" ]; then
    echo "No outstanding TBD placeholders."
    exit 0
fi

count=$(echo "$matches" | wc -l | tr -d ' ')
echo "$count outstanding TBD placeholder(s):"
echo
echo "$matches"
