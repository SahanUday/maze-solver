#!/usr/bin/env python3
"""PR title check: `type(scope): Subject`, e.g. `feat(hal): Add IR array driver`.

PRs are squash-merged, so the PR title becomes the commit on main. This keeps
that history uniform across contributors (it is the convention every merged PR
so far follows).

  type      feat fix docs refactor perf test build ci chore revert
  scope     optional, in parentheses: letters, digits, . _ / -
  subject   starts with an upper-case letter or digit, no trailing period
  length    at most 72 characters (GitHub adds " (#123)" when squashing)

Usage: scripts/check-pr-title.py "<title>"     (or PR_TITLE in the environment)
Exit status: 0 ok, 1 invalid, 2 no title given.
"""

from __future__ import annotations

import os
import re
import sys

TYPES = ("feat", "fix", "docs", "refactor", "perf", "test", "build", "ci", "chore", "revert")
MAX_LENGTH = 72
PATTERN = re.compile(rf"^(?P<type>{'|'.join(TYPES)})(?:\((?P<scope>[A-Za-z0-9._/-]+)\))?(?P<bang>!)?: (?P<subject>.+)$")


def problems(title: str) -> list[str]:
    title = title.strip()
    m = PATTERN.match(title)
    if not m:
        if re.match(r"^\w+(\(.*\))?!?:", title):
            return [f"unknown type or malformed scope; type must be one of: {', '.join(TYPES)}"]
        return ["expected 'type(scope): Subject' - for example 'feat(hal): Add IR array driver'"]
    found = []
    subject = m.group("subject")
    if not (subject[0].isupper() or subject[0].isdigit()):
        found.append("the subject should start with an upper-case letter (the style of the merged PRs)")
    if subject.endswith("."):
        found.append("no trailing period")
    if len(title) > MAX_LENGTH:
        found.append(f"{len(title)} characters; keep it to {MAX_LENGTH} (squash-merge adds ' (#N)')")
    return found


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    title = argv[0] if argv else os.environ.get("PR_TITLE")
    if not title:
        print("usage: check-pr-title.py '<title>' (or set PR_TITLE)", file=sys.stderr)
        return 2
    found = problems(title)
    if not found:
        print(f"PR title ok: {title}")
        return 0
    print(f"PR title: {title}")
    for p in found:
        print(f"  - {p}")
        if os.environ.get("GITHUB_ACTIONS") == "true":
            print(f"::error title=PR title::{p}")
    print("\nexamples: 'feat(hal): Add IR array driver', 'fix(encoders): Handle counter wrap', 'ci: Pin the AVR toolchain', 'docs: Describe the pin map'")
    return 1


if __name__ == "__main__":
    sys.exit(main())
