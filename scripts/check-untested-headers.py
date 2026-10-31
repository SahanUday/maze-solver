#!/usr/bin/env python3
"""Fail when a header with logic in it is missing from the coverage report.

The coverage floor only measures files some test compiles. A new header with
functions in it that no test includes is not in the report at all, so the floor
can stay at 100% while that logic has no test. This finds those headers.

A header under include/ or lib/ "has logic" when it contains a function body (a
`) {`), which constants, structs and declarations do not. If it has logic it
must appear in the gcovr report. A header that really cannot be built on the
host (it needs Arduino.h, say) says so with
`// coverage-exempt: <reason>`; the reason is required.

The detection is textual, not a compiler pass. Comments, preprocessor lines
(macro bodies) and `__attribute__((...))` are ignored; it still errs toward
reporting, and the exemption marker is the escape hatch for a false positive.

Usage: scripts/check-untested-headers.py --report FILE [--root DIR]
  --report FILE  gcovr --json-summary output (scripts/check-coverage.sh makes it)
Exit status: 0 clean, 1 findings, 2 usage / unreadable input.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path

from cpplex import strip_comments

HEADER_DIRS = ("include", "lib")
HEADER_SUFFIXES = (".h", ".hpp")
# `) {`, `) const {`, `) noexcept {`, `) -> T {`: the opening of a function body.
FUNCTION_BODY = re.compile(r"\)\s*(?:const\s*)?(?:noexcept\s*)?(?:->\s*[\w:<>\s*&]+)?\{")
EXEMPT = re.compile(r"coverage-exempt:[ \t]*(\S.*)")
ATTRIBUTE = re.compile(r"__attribute__\s*\(\(.*?\)\)", re.S)


def blank(text: str) -> str:
    """`text` with everything but newlines replaced by spaces."""
    return re.sub(r"[^\n]", " ", text)


def logic_text(raw: str) -> str:
    """The header with comments, strings, preprocessor lines and attributes blanked
    (newlines kept, so a match's line number is the line in the file)."""
    lines, continued = [], False
    for line in strip_comments(raw).split("\n"):
        if continued or line.lstrip().startswith("#"):
            continued = line.rstrip().endswith("\\")
            line = blank(line)
        lines.append(line)
    return ATTRIBUTE.sub(lambda m: blank(m.group()), "\n".join(lines))


class Finding:
    def __init__(self, path: str, line: int, message: str):
        self.path, self.line, self.message = path, line, message

    def __str__(self) -> str:
        return f"{self.path}:{self.line}: [untested-header] {self.message}"


def headers(root: Path):
    for sub in HEADER_DIRS:
        base = root / sub
        if base.is_dir():
            yield from sorted(p for p in base.rglob("*") if p.suffix in HEADER_SUFFIXES)


def covered_files(report: dict) -> set[str]:
    return {Path(f["filename"]).as_posix() for f in report.get("files", [])}


def check(root: Path, report: dict) -> list[Finding]:
    covered = covered_files(report)
    findings: list[Finding] = []
    for path in headers(root):
        rel = path.relative_to(root).as_posix()
        raw = path.read_text()
        code = logic_text(raw)
        match = FUNCTION_BODY.search(code)
        if not match or rel in covered:
            continue
        if EXEMPT.search(raw):
            continue
        line = code.count("\n", 0, match.start()) + 1
        bare = re.search(r"coverage-exempt:[ \t]*$", raw, re.M)
        hint = "'// coverage-exempt:' needs a reason after the colon" if bare else (
            "write a test/test_<name>/ that includes it, or add '// coverage-exempt: <reason>' "
            "if it cannot be built on the host or this is a false positive"
        )
        findings.append(Finding(rel, line, f"has logic but no test compiles it, so the coverage report does not include it; {hint}"))
    return findings


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)

    try:
        report = json.loads(args.report.read_text())
    except (OSError, ValueError) as exc:
        print(f"cannot read the coverage report {args.report}: {exc}", file=sys.stderr)
        return 2

    findings = check(args.root, report)
    on_github = os.environ.get("GITHUB_ACTIONS") == "true"
    for f in findings:
        print(f)
        if on_github:
            print(f"::error file={f.path},line={f.line},title=untested header::{f.message}")
    if findings:
        print(f"\nuntested-headers: {len(findings)} problem(s)")
        return 1
    print("untested-headers: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
