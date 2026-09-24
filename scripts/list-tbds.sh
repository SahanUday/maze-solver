#!/usr/bin/env bash
# Lists every outstanding <<TBD ...>> placeholder. Informational only.
set -euo pipefail

SEARCH_DIRS=()
for d in src include lib scripts; do
    [ -d "$d" ] && SEARCH_DIRS+=("$d")
done

if [ ${#SEARCH_DIRS[@]} -eq 0 ]; then
    echo "No source directories found."
    exit 0
fi

EXTS=(--include='*.c' --include='*.h' --include='*.cpp' --include='*.hpp' --include='*.cc' --include='*.sh')

matches=$(grep -rnE '<<TBD [A-Z0-9-]+>>' "${SEARCH_DIRS[@]}" "${EXTS[@]}" || true)

if [ -z "$matches" ]; then
    echo "No outstanding TBD placeholders."
    exit 0
fi

count=$(echo "$matches" | wc -l | tr -d ' ')
echo "$count outstanding TBD placeholder(s):"
echo
echo "$matches"
