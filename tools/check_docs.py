#!/usr/bin/env python3
"""Check Markdown internal links and code-fence balance. Python stdlib only.

This is a lightweight integrity check, NOT a Mermaid parser or proof of
academic correctness. CI runs it with the controller host-side tests.
"""
from __future__ import annotations

import pathlib
import re
import sys
from urllib.parse import unquote, urlsplit

ROOT = pathlib.Path(__file__).resolve().parents[1]
LINK_RE = re.compile(r"(?<!!)\[[^\]]+\]\(([^)]+)\)")
CODE_RE = re.compile(r"^\s*\x60\x60\x60", re.MULTILINE)


def check_markdown(path: pathlib.Path) -> list[str]:
    errors: list[str] = []
    rel = path.relative_to(ROOT)
    text = path.read_text(encoding="utf-8")
    fences = CODE_RE.findall(text)
    if len(fences) % 2:
        errors.append(f"{rel}: odd number of Markdown code fences")
    for match in LINK_RE.finditer(text):
        raw = match.group(1).strip()
        target = raw.split(" ", 1)[0].strip("<>")
        parsed = urlsplit(target)
        if parsed.scheme or target.startswith(("#", "//")):
            continue
        if not parsed.path:  # local anchor
            continue
        dest = (path.parent / unquote(parsed.path)).resolve()
        if not dest.is_relative_to(ROOT):
            errors.append(f"{rel}: link escapes repository: {target}")
            continue
        if not dest.exists():
            errors.append(f"{rel}: broken relative link: {target}")
    return errors


def main() -> int:
    files = sorted(ROOT.rglob("*.md"))
    if not files:
        print("No Markdown files found", file=sys.stderr)
        return 1
    errors: list[str] = []
    for path in files:
        if ".git" in path.parts:
            continue
        errors.extend(check_markdown(path))
    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    print(f"PASS: {len(files)} Markdown files, internal links and fence counts")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
