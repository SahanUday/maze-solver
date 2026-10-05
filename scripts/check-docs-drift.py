#!/usr/bin/env python3
"""Keep docs/architecture/ honest (rules from CLAUDE.md).

  modules     every src/hal/<name>.cpp has docs/architecture/modules/<name>.md
              ("added in the same PR that introduces the module's first working
              code").
  adr-names   ADR files are decisions/NNNN-title.md, the NNNN is unique, the
              '# NNNN: Title' heading matches the filename, and a Status line
              exists. Two contributors writing "the next ADR" in parallel both
              pick the same number; git sees two different files, so only a
              check notices.
  testing-doc every check/report script in scripts/ and every `[env:...]` in
              platformio.ini is described in docs/testing.md, so the testing
              guide cannot silently fall behind what CI actually runs.
  adr-frozen  (with --base REF) an ADR that was `Accepted` on the base is
              immutable: the only allowed edit is its Status line moving to
              Superseded/Deprecated. A changed mind gets a new ADR.

Usage: scripts/check-docs-drift.py [--root DIR] [--base REF]
  --base REF  compare against REF (CI passes HEAD^1 on the PR merge commit).
              Without it only the static checks run (pre-commit).
Exit status: 0 clean, 1 findings, 2 usage / git error.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

ADR_DIR = "docs/architecture/decisions"
MODULE_DIR = "docs/architecture/modules"
ADR_NAME = re.compile(r"^(\d{4})-[a-z0-9]+(?:-[a-z0-9]+)*\.md$")
STATUS = re.compile(r"^\s*[-*]?\s*\*\*Status:?\*\*:?\s*(.+?)\s*$", re.M)
HEADING = re.compile(r"^#\s+(\d{4}):\s+\S", re.M)
FROZEN_STATUS = re.compile(r"^accepted\b", re.I)
RETIRED_STATUS = re.compile(r"^(superseded|deprecated)\b", re.I)


class Finding:
    def __init__(self, path: str, line: int, rule: str, message: str):
        self.path, self.line, self.rule, self.message = path, line, rule, message

    def __str__(self) -> str:
        return f"{self.path}:{self.line}: [{self.rule}] {self.message}"


def check_modules(root: Path, findings: list[Finding]):
    hal = root / "src/hal"
    if not hal.is_dir():
        return
    for path in sorted(hal.glob("*.cpp")):
        doc = root / MODULE_DIR / f"{path.stem}.md"
        if not doc.exists():
            findings.append(Finding(str(path.relative_to(root)), 1, "modules", f"no {MODULE_DIR}/{path.stem}.md - a HAL module's doc is added in the same PR as its first working code"))


def check_adr_names(root: Path, findings: list[Finding]):
    base = root / ADR_DIR
    if not base.is_dir():
        return
    by_number = defaultdict(list)
    for path in sorted(base.glob("*.md")):
        rel = str(path.relative_to(root))
        m = ADR_NAME.match(path.name)
        if not m:
            findings.append(Finding(rel, 1, "adr-names", "ADR files are named NNNN-short-title.md (four digits, lower-case, hyphens)"))
            continue
        number = m.group(1)
        by_number[number].append(rel)
        text = path.read_text()
        heading = HEADING.search(text)
        if not heading or heading.group(1) != number:
            findings.append(Finding(rel, 1, "adr-names", f"the first heading should be '# {number}: <title>' to match the filename"))
        if not STATUS.search(text):
            findings.append(Finding(rel, 1, "adr-names", "no '- **Status:** ...' line"))
    for number, files in sorted(by_number.items()):
        if len(files) > 1:
            findings.append(Finding(files[-1], 1, "adr-names", f"ADR number {number} is used by more than one file: {', '.join(files)}. Renumber the newer one"))


# Helpers other scripts import; not something a developer runs or needs explained.
INTERNAL_SCRIPTS = {"cpplex.py"}
TESTING_DOC = "docs/testing.md"


def check_testing_doc(root: Path, findings: list[Finding]):
    scripts = root / "scripts"
    names = sorted(p.name for p in scripts.glob("*") if p.is_file() and p.suffix in (".py", ".sh") and p.name not in INTERNAL_SCRIPTS) if scripts.is_dir() else []
    ini = root / "platformio.ini"
    envs = re.findall(r"^\[env:([A-Za-z0-9_]+)\]", ini.read_text(), re.M) if ini.exists() else []
    if not names and not envs:
        return
    doc = root / TESTING_DOC
    if not doc.exists():
        findings.append(Finding(TESTING_DOC, 1, "testing-doc", f"{TESTING_DOC} is missing; it must describe every check script and PlatformIO env"))
        return
    text = doc.read_text()
    for name in names:
        if name not in text:
            findings.append(Finding(f"scripts/{name}", 1, "testing-doc", f"scripts/{name} is not described in {TESTING_DOC}"))
    for env in envs:
        # \b so that a documented `env:native_san` does not also satisfy the
        # undocumented `env:native` that is a prefix of it.
        if not re.search(rf"env:{re.escape(env)}\b", text):
            findings.append(Finding("platformio.ini", 1, "testing-doc", f"env:{env} is not described in {TESTING_DOC}"))


def git(root: Path, *args: str) -> str:
    result = subprocess.run(["git", *args], cwd=root, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)}: {result.stderr.strip()}")
    return result.stdout


def without_status(text: str) -> str:
    return STATUS.sub("", text)


def check_adr_frozen(root: Path, base: str, findings: list[Finding]):
    changes = git(root, "diff", "--name-status", "-M", base, "HEAD", "--", ADR_DIR)
    for row in changes.splitlines():
        fields = row.split("\t")
        code, old = fields[0][0], fields[1]
        if code == "A":
            continue
        try:
            old_text = git(root, "show", f"{base}:{old}")
        except RuntimeError:
            continue
        m = STATUS.search(old_text)
        if not m or not FROZEN_STATUS.match(m.group(1)):
            continue  # Proposed (or unknown): still editable
        if code in ("D", "R"):
            what = "deleted" if code == "D" else f"renamed to {fields[2]}"
            findings.append(Finding(old, 1, "adr-frozen", f"{old} is Accepted and was {what}. Accepted ADRs are immutable - supersede it with a new ADR instead"))
            continue
        new_text = (root / old).read_text()
        new_status = STATUS.search(new_text)
        retired = bool(new_status and RETIRED_STATUS.match(new_status.group(1)))
        if not (retired and without_status(new_text) == without_status(old_text)):
            findings.append(Finding(old, 1, "adr-frozen", f"{old} is Accepted and was edited. Accepted ADRs are immutable (CLAUDE.md): write a new ADR that supersedes it - the only allowed edit is its Status line becoming 'Superseded by NNNN'"))


def check(root: Path, base: str | None = None) -> list[Finding]:
    findings: list[Finding] = []
    check_modules(root, findings)
    check_adr_names(root, findings)
    check_testing_doc(root, findings)
    if base:
        check_adr_frozen(root, base, findings)
    return findings


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--base")
    args = parser.parse_args(argv)
    try:
        findings = check(args.root, args.base)
    except RuntimeError as exc:
        print(f"docs-drift: {exc}", file=sys.stderr)
        return 2

    on_github = os.environ.get("GITHUB_ACTIONS") == "true"
    for f in findings:
        print(f)
        if on_github:
            print(f"::error file={f.path},line={f.line},title=docs drift ({f.rule})::{f.message}")
    if findings:
        print(f"\ndocs-drift: {len(findings)} problem(s)")
        return 1
    print("docs-drift: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
