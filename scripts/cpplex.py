"""Tiny C/C++ lexing helper shared by the check-*.py scripts."""

from __future__ import annotations


def strip_comments(text: str) -> str:
    """Blank out comments and string/char literals, keeping every newline so
    line numbers stay valid."""
    out, i, n = [], 0, len(text)
    while i < n:
        two = text[i : i + 2]
        if two == "//":
            while i < n and text[i] != "\n":
                out.append(" ")
                i += 1
        elif two == "/*":
            end = text.find("*/", i + 2)
            end = n if end == -1 else end + 2
            out.append("".join("\n" if c == "\n" else " " for c in text[i:end]))
            i = end
        elif text[i] in "\"'":
            quote = text[i]
            out.append(" ")
            i += 1
            while i < n and text[i] != quote:
                if text[i] == "\\" and i + 1 < n:
                    out.append(" ")
                    i += 1
                out.append("\n" if text[i] == "\n" else " ")
                i += 1
            out.append(" ")
            i += 1
        else:
            out.append(text[i])
            i += 1
    return "".join(out)
