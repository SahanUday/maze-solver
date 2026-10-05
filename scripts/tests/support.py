"""Shared helpers for the script self-tests (stdlib unittest, no pip deps)."""

import importlib.util
import tempfile
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent.parent


def load_script(filename: str):
    """Import scripts/<filename> (hyphens in the name rule out a plain import)."""
    path = SCRIPTS / filename
    spec = importlib.util.spec_from_file_location(path.stem.replace("-", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Tree:
    """A throwaway project tree: Tree({"src/a.cpp": "..."}).root."""

    def __init__(self, files: dict[str, str]):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        for rel, text in files.items():
            path = self.root / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)

    def close(self):
        self._tmp.cleanup()
