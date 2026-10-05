#!/usr/bin/env python3
"""ISR / main-loop shared-data check for the maze-solver firmware.

On an 8-bit AVR a 16- or 32-bit load or store is several instructions. If an
interrupt fires in the middle, the main loop sees half of the old value and
half of the new one. The classic case is an encoder count read while its ISR
runs: it works for months, then a torn read shows up as a one-off jump of 256
counts. The fix (an ATOMIC_BLOCK around the access) is easy to forget, so this
script makes forgetting it a CI failure.

Rule: a `volatile` variable wider than one byte that an ISR touches must only
be accessed
  - inside an ISR(...) body,
  - inside an ATOMIC_BLOCK(...) body, or
  - inside a `static` function whose every call site is one of the above
    (encoders.cpp's stepLeft() is called only from ISRs).
A multi-byte volatile with external linkage (extern, or a non-static global)
is rejected outright: other files' accesses cannot be checked, so expose it
through an accessor that does the ATOMIC_BLOCK.

A type whose width cannot be read off the declaration - a typedef, a struct, a
scoped enum - is assumed to be multi-byte. Only `char`, `bool`, `int8_t` and
`uint8_t` are treated as safe to access unguarded.

Escape hatch for a reviewed exception: `// isr-safe: <reason>` on the access
line or the line above it.

Lexical, not a compiler pass: it understands braces, ISR/ATOMIC_BLOCK and
function bodies well enough for this codebase and errs toward reporting.

Usage: scripts/check-isr-atomicity.py [--root DIR]
Exit status: 0 clean, 1 findings, 2 usage error.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

from cpplex import strip_comments

QUALIFIERS = r"(?:static|extern|inline|const|constexpr|register|volatile)"
TYPE_WORDS = r"(?:unsigned|signed|short|long|int|char|float|double|bool|u?int(?:8|16|32|64)_t|u?intptr_t|size_t|ptrdiff_t)"
TYPE_SEQ = rf"(?:{TYPE_WORDS}\b\s*)+"
DECL = re.compile(
    rf"^\s*(?P<pre>(?:{QUALIFIERS}\b\s+)*)(?P<type>{TYPE_SEQ})(?P<mid>(?:{QUALIFIERS}\b\s*)*)(?P<rest>.+)$",
    re.S,
)
DECLARATOR = re.compile(r"^\s*(?P<ptr>\*?)\s*(?:const\s*)?(?P<vol>volatile\s*)?(?P<name>[A-Za-z_]\w*)\s*(?P<arr>\[[^\]]*\])?\s*(?:=.*)?$", re.S)
# A declaration whose type is a single identifier this script does not know -
# a typedef, a struct/union/enum tag, a scoped enum. Assumed multi-byte.
UNKNOWN_DECL = re.compile(
    rf"^\s*(?P<pre>(?:{QUALIFIERS}\b\s+)*)(?:(?:struct|union|enum|class)\s+)?"
    r"(?P<type>[A-Za-z_][\w:]*)\s+(?P<mid>(?:volatile|const)\b\s*)*(?P<rest>[^;]+)$",
    re.S,
)
NOT_FUNCTIONS = {"if", "for", "while", "switch", "catch", "ISR", "ATOMIC_BLOCK", "NONATOMIC_BLOCK", "__attribute__", "noexcept", "decltype", "sizeof", "alignas"}
TOKEN = re.compile(r"[{};]|[A-Za-z_]\w*")
EXEMPT = re.compile(r"isr-safe:\s*\S")


class Finding:
    def __init__(self, path: str, line: int, message: str):
        self.path, self.line, self.message = path, line, message

    def __str__(self) -> str:
        return f"{self.path}:{self.line}: [isr] {self.message}"


def is_multibyte(type_text: str) -> bool:
    words = type_text.split()
    return not any(w in ("char", "bool", "int8_t", "uint8_t") for w in words)


def statement_end(code: str, pos: int) -> int:
    """Offset of the ';' that ends the statement containing `pos`, ignoring the
    ';'s inside a braced type definition (`volatile struct { uint16_t a; } s;`)."""
    depth = 0
    for j in range(pos, len(code)):
        if code[j] == "{":
            depth += 1
        elif code[j] == "}":
            depth -= 1
        elif code[j] == ";" and depth <= 0:
            return j
    return -1


def declaration_parts(stmt: str, volatile_at: int):
    """(pre, mid, rest, multibyte) for a volatile declaration, or None.

    A type this script does not recognise - a typedef, a struct, a scoped enum -
    cannot have its width read off the declaration, so it is assumed MULTI-BYTE
    rather than skipped. Skipping was a silent pass on exactly the declarations
    an ISR state machine uses (`static volatile UsState g_state;`), which is the
    wrong way for this check to fail.
    """
    parts = DECL.match(stmt)
    if parts and "volatile" in parts["pre"] + parts["mid"]:
        return parts["pre"], parts["mid"], parts["rest"], is_multibyte(parts["type"])

    # Not a declaration at all: a parameter list (`void f(volatile uint8_t *p)`)
    # or a cast (`*(volatile uint8_t *)addr`) has a '(' ahead of the volatile.
    if "(" in stmt[:volatile_at]:
        return None

    if "{" in stmt:  # an inline struct/union/enum definition; declarator follows '}'
        if not re.search(r"\bvolatile\b\s*(?:struct|union|enum|class)\b", stmt):
            return None
        rest = stmt[stmt.rfind("}") + 1 :]
        if not rest.strip():
            return None
        return stmt[:volatile_at], "", rest, True

    parts = UNKNOWN_DECL.match(stmt)
    if not parts or "volatile" not in parts["pre"] + (parts["mid"] or ""):
        return None
    return parts["pre"], parts["mid"] or "", parts["rest"], True


def find_volatiles(code: str):
    """Multi-byte volatile variable declarations: [(name, offset, external)]."""
    found = []
    for m in re.finditer(r"\bvolatile\b", code):
        start = max(code.rfind(c, 0, m.start()) for c in ";{}") + 1
        end = statement_end(code, m.end())
        if end == -1:
            continue
        stmt = code[start:end]
        parsed = declaration_parts(stmt, m.start() - start)
        if parsed is None:
            continue
        pre, mid, rest, multibyte = parsed
        if not multibyte:
            continue
        offset = start + len(stmt) - len(rest)
        is_static = "static" in pre
        is_extern = "extern" in pre
        pos = 0
        for decl in re.split(r",(?![^(]*\))", rest):
            d = DECLARATOR.match(decl)
            if d and (not d["ptr"] or d["vol"]):
                name_at = offset + rest.index(d["name"], pos)
                found.append((d["name"], name_at, is_extern, is_static, start))
            pos += len(decl) + 1
    return found


def classify(header: str, stack: list) -> tuple[str, str, bool]:
    """(kind, name, is_static) for the block opened by a '{' after `header`."""
    header = re.sub(r"__attribute__\s*\(\(.*?\)\)", " ", header, flags=re.S).strip()
    if re.match(r"ISR\s*\(", header):
        return "isr", "", False
    if re.match(r"NONATOMIC_BLOCK\s*\(", header):
        return "nonatomic", "", False
    if re.match(r"ATOMIC_BLOCK\s*\(", header):
        return "atomic", "", False
    m = re.match(r"(?:inline\s+)?namespace\b\s*([A-Za-z_]\w*)?\s*$", header)
    if m:
        return ("ns" if m.group(1) else "ns_anon"), m.group(1) or "", False
    if re.match(r'extern\s*$', header):
        return "ns", "", False
    if re.match(r"(?:typedef\s+)?(?:struct|class|union|enum)\b", header):
        return "type", "", False
    if all(kind in ("ns", "ns_anon", "type") for kind, _, _ in stack) and header.endswith((")", "const", "noexcept", "override", "final")):
        names = [n for n in re.findall(r"([A-Za-z_]\w*)\s*\(", header) if n not in NOT_FUNCTIONS]
        if names:
            return "func", names[0], bool(re.search(r"\bstatic\b", header))
    return "other", "", False


def events(code: str):
    """Walk the code once. Yields (token, pos, stack snapshot, next_is_call)."""
    stack: list[tuple[str, str, bool]] = []
    boundary = 0
    for m in TOKEN.finditer(code):
        tok = m.group()
        if tok == "{":
            stack.append(classify(code[boundary : m.start()], stack))
            boundary = m.end()
        elif tok == "}":
            if stack:
                stack.pop()
            boundary = m.end()
        elif tok == ";":
            boundary = m.end()
        else:
            nxt = code[m.end() : m.end() + 40].lstrip()
            yield tok, m.start(), tuple(stack), nxt.startswith("(")


def local_context(stack):
    """Walk out from the innermost block to the function boundary.
    Returns ('isr'|'atomic'|'unsafe'|'func', function_name)."""
    for kind, name, _ in reversed(stack):
        if kind in ("isr", "atomic"):
            return kind, ""
        if kind == "nonatomic":
            return "unsafe", ""
        if kind == "func":
            return "func", name
    return "unsafe", ""


def check_file(rel: str, raw: str) -> list[Finding]:
    code = strip_comments(raw)
    code = re.sub(r"^[ \t]*#.*$", lambda m: " " * len(m.group()), code, flags=re.M)
    raw_lines = raw.splitlines()
    findings: list[Finding] = []

    def line_of(pos: int) -> int:
        return code.count("\n", 0, pos) + 1

    def exempt(line: int) -> bool:
        return any(EXEMPT.search(raw_lines[i]) for i in (line - 1, line - 2) if 0 <= i < len(raw_lines))

    volatiles = find_volatiles(code)
    if not volatiles:
        return findings
    names = {n for n, *_ in volatiles}
    decl_offsets = {off for _, off, *_ in volatiles}

    evs = list(events(code))
    static_funcs = {}
    for _tok, _pos, stack, _call in evs:
        for kind, name, is_static in stack:
            if kind == "func":
                static_funcs[name] = static_funcs.get(name, True) and is_static

    # callee -> [(local ctx, caller)]. A name followed by '(' outside every
    # function body is a prototype or the definition itself, not a call.
    calls: dict[str, list[tuple[str, str]]] = {}
    for tok, _pos, stack, is_call in evs:
        if is_call and tok in static_funcs and any(k in ("func", "isr", "atomic", "other") for k, _n, _s in stack):
            calls.setdefault(tok, []).append(local_context(stack))

    # Functions only ever called from ISR / ATOMIC contexts (least fixpoint).
    safe: set[str] = set()
    reached_from_isr: set[str] = set()
    changed = True
    while changed:
        changed = False
        for fn, sites in calls.items():
            if static_funcs[fn] and fn not in safe and all(ctx in ("isr", "atomic") or (ctx == "func" and caller in safe) for ctx, caller in sites):
                safe.add(fn)
                changed = True
            if fn not in reached_from_isr and any(ctx == "isr" or (ctx == "func" and caller in reached_from_isr) for ctx, caller in sites):
                reached_from_isr.add(fn)
                changed = True

    def access_state(stack):
        ctx, func = local_context(stack)
        in_isr = ctx == "isr" or (ctx == "func" and func in reached_from_isr)
        ok = ctx in ("isr", "atomic") or (ctx == "func" and func in safe)
        return ok, in_isr

    accesses: dict[str, list[tuple[int, bool, bool]]] = {n: [] for n in names}
    for tok, pos, stack, _call in evs:
        if tok in names and pos not in decl_offsets:
            ok, in_isr = access_state(stack)
            accesses[tok].append((line_of(pos), ok, in_isr))

    decl_stack = {pos: stack for _tok, pos, stack, _call in evs if pos in decl_offsets}
    for name, offset, is_extern, is_static, _start in volatiles:
        decl_line = line_of(offset)
        scope = [k for k, _n, _s in decl_stack.get(offset, ())]
        top_level = all(k in ("ns", "type") for k in scope)  # an anonymous namespace has internal linkage
        if exempt(decl_line):
            continue
        if is_extern or (not is_static and top_level):
            findings.append(Finding(rel, decl_line, f"'{name}' is a multi-byte volatile with external linkage - other files' accesses can't be checked. Make it file-static and expose an accessor that reads it inside ATOMIC_BLOCK"))
            continue
        if not any(in_isr for _l, _ok, in_isr in accesses[name]):
            continue  # not shared with an interrupt
        for line in sorted({ln for ln, ok, _i in accesses[name] if not ok}):
            if not exempt(line):
                findings.append(Finding(rel, line, f"'{name}' is a multi-byte volatile used by an ISR; this access is not inside an ISR or ATOMIC_BLOCK, so an interrupt can tear it on AVR. Wrap it in ATOMIC_BLOCK(ATOMIC_RESTORESTATE), or add '// isr-safe: <reason>' if it is reviewed"))
    return findings


def check(root: Path) -> list[Finding]:
    findings = []
    src = root / "src"
    if src.is_dir():
        for path in sorted(p for p in src.rglob("*") if p.suffix in (".c", ".cpp", ".h", ".hpp")):
            findings += check_file(str(path.relative_to(root)), path.read_text())
    return findings


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent.parent)
    args = parser.parse_args(argv)
    if not args.root.is_dir():
        print(f"{args.root}: not a directory", file=sys.stderr)
        return 2

    findings = check(args.root)
    on_github = os.environ.get("GITHUB_ACTIONS") == "true"
    for f in findings:
        print(f)
        if on_github:
            print(f"::error file={f.path},line={f.line},title=ISR atomicity::{f.message}")
    if findings:
        print(f"\nisr-atomicity: {len(findings)} problem(s)")
        return 1
    print("isr-atomicity: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
