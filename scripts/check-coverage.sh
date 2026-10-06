#!/usr/bin/env bash
# Line-coverage floor for the host-testable code (include/ and lib/).
# Run after `pio test -e native_cov`. A header that no test compiles is not
# in the report at all, so the floor alone guards against shrinking coverage of
# tested code, not against code nobody wrote a test for. check-untested-headers.py
# closes that: a header with logic in it must appear in the report.
#
# Usage: scripts/check-coverage.sh      (COVERAGE_FLOOR overrides the floor)
set -euo pipefail

FLOOR="${COVERAGE_FLOOR:-90}"
BUILD_DIR=".pio/build/native_cov"

if [ ! -d "$BUILD_DIR" ]; then
    echo "no $BUILD_DIR - run 'pio test -e native_cov' first" >&2
    exit 2
fi

REPORT="$BUILD_DIR/coverage-summary.json"
status=0

# Both checks always run, so one failure does not hide the other.
gcovr --root . --object-directory "$BUILD_DIR" \
    --filter 'include/' --filter 'lib/' \
    --txt --json-summary "$REPORT" --fail-under-line "$FLOOR" || status=$?

python3 "$(dirname "$0")/check-untested-headers.py" --root . --report "$REPORT" || status=$?

exit "$status"
