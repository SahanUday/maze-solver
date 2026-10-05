#!/usr/bin/env python3
"""Markdown reminder of the open `<<TBD ...>>` placeholders: values that are named
but not yet measured (HARDWARE), tuned (CALIBRATION) or otherwise still to do.

Built on scripts/list-tbds.sh (the single definition of where to look). Posted
on every PR next to the firmware size, so nobody has to remember to run it.
Informational: it never fails.

  scripts/tbd-report.py [--base-root DIR] [--repo OWNER/NAME --sha SHA]

  --base-root  a checkout of the PR's base commit; adds what this PR added and
               resolved (compared by file + kind + description, not line number)
  --repo/--sha make each entry a link to that line on GitHub

Lines that only *describe* the marker (it sits inside backticks, as in the
legend at the top of RobotSpec.h) are not placeholders and are skipped.
Exit status: 0, or 2 if list-tbds.sh cannot be run.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from collections import Counter
from pathlib import Path

ENTRY = re.compile(r"^(?P<file>[^\s:][^:]*):(?P<line>\d+):(?P<text>.*)$")
MARKER = re.compile(r"<<TBD (?P<kind>[A-Z0-9-]+)>>")
COLLAPSE_AFTER = 12


def parse(listing: str) -> list[dict]:
    """list-tbds.sh output -> [{file, line, kind, what}], legend lines dropped."""
    items = []
    for row in listing.splitlines():
        m = ENTRY.match(row)
        if not m:
            continue  # the "N outstanding..." header, blanks, "No outstanding..."
        text = m["text"]
        marker = MARKER.search(text)
        if not marker or "`" + marker.group(0) in text:
            continue
        what = MARKER.sub("", text).strip(" /#*-:\t")
        items.append({"file": m["file"], "line": int(m["line"]), "kind": marker["kind"], "what": what or "(no description)"})
    return sorted(items, key=lambda i: (i["kind"], i["file"], i["line"]))


def collect(root: Path, script: Path) -> list[dict]:
    result = subprocess.run(["bash", str(script)], cwd=root, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"list-tbds.sh failed in {root}: {result.stderr.strip()}")
    return parse(result.stdout)


def identity(item: dict) -> tuple:
    return (item["file"], item["kind"], item["what"])


def render(items: list[dict], base: list[dict] | None = None, repo: str | None = None, sha: str | None = None) -> str:
    out = ["### Open placeholders (`<<TBD …>>`)", ""]
    if not items:
        out.append("None - every placeholder is resolved.")
        added: set = set()
    else:
        kinds = Counter(i["kind"] for i in items)
        summary = " · ".join(f"{n} {k}" for k, n in sorted(kinds.items()))
        out.append(f"**{len(items)} open**: {summary}")
        added = set()
        if base is not None:
            now, before = Counter(identity(i) for i in items), Counter(identity(i) for i in base)
            added = set((now - before).keys())
            resolved = sorted((before - now).keys())
            out.append(f"This PR: **{sum((now - before).values())} added**, **{sum((before - now).values())} resolved**.")
            if resolved:
                out += ["", "Resolved here: " + "; ".join(f"{what} (`{file}`)" for file, _kind, what in resolved)]
        table = ["| Kind | Where | What |", "|---|---|---|"]
        for i in items:
            where = f"{i['file']}:{i['line']}"
            if repo and sha:
                where = f"[{where}](https://github.com/{repo}/blob/{sha}/{i['file']}#L{i['line']})"
            else:
                where = f"`{where}`"
            new = " (new)" if identity(i) in added else ""
            table.append(f"| {i['kind']} | {where} | {i['what'].replace('|', chr(92) + '|')}{new} |")
        if len(items) > COLLAPSE_AFTER:
            out += ["", f"<details><summary>Show all {len(items)}</summary>", ""] + table + ["", "</details>"]
        else:
            out += [""] + table
    out += ["", "_Values named but not yet measured (HARDWARE) or tuned (CALIBRATION). Reminder only; list them locally with `scripts/list-tbds.sh`._"]
    return "\n".join(out) + "\n"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--base-root", type=Path)
    parser.add_argument("--repo")
    parser.add_argument("--sha")
    args = parser.parse_args(argv)
    script = args.root / "scripts" / "list-tbds.sh"
    try:
        items = collect(args.root, script)
        base = collect(args.base_root, script) if args.base_root and args.base_root.is_dir() else None
    except (RuntimeError, OSError) as exc:
        print(f"tbd-report: {exc}", file=sys.stderr)
        return 2
    sys.stdout.write(render(items, base, args.repo, args.sha))
    return 0


if __name__ == "__main__":
    sys.exit(main())
