#!/usr/bin/env python3
"""
Extract code snippets from firmware source and expand them in LaTeX files.

Firmware source files use markers:
    // @@snippet: name
    ... code ...
    // @@end-snippet: name

LaTeX files use placeholders:
    %%SNIPPET:relative/path/to/file.cpp:name%%

The script reads the LaTeX source, resolves all %%SNIPPET%% placeholders,
and writes expanded files to a build directory. The original .tex files
are never modified.

Usage:
    python extract_snippets.py --check     # Verify all snippets resolve
    python extract_snippets.py --expand    # Write expanded files to build dir
"""

import argparse
import re
import sys
from pathlib import Path

# Paths relative to repository root
REPO_ROOT = Path(__file__).resolve().parent.parent
LESSONS_DIR = REPO_ROOT / "firmware" / "workshop" / "lessons"
CHAPTERS_DIR = REPO_ROOT / "docs" / "events" / "stampfly_workshop" / "slides" / "chapters"
BUILD_DIR = REPO_ROOT / "docs" / "events" / "stampfly_workshop" / "slides" / ".build_chapters"

# Regex patterns
SNIPPET_MARKER_START = re.compile(r'^\s*//\s*@@snippet:\s*(\S+)\s*$')
SNIPPET_MARKER_END = re.compile(r'^\s*//\s*@@end-snippet:\s*(\S+)\s*$')
PLACEHOLDER = re.compile(r'%%SNIPPET:([^%]+)%%')


def extract_snippets_from_file(filepath: Path) -> dict[str, str]:
    """Extract all named snippets from a source file.

    Returns dict mapping snippet name -> code text (without markers).
    """
    snippets: dict[str, list[str]] = {}
    active: dict[str, list[str]] = {}

    for line in filepath.read_text(encoding="utf-8").splitlines():
        # Check for start marker
        m = SNIPPET_MARKER_START.match(line)
        if m:
            name = m.group(1)
            active[name] = []
            continue

        # Check for end marker
        m = SNIPPET_MARKER_END.match(line)
        if m:
            name = m.group(1)
            if name in active:
                snippets[name] = active.pop(name)
            continue

        # Collect lines for all active snippets
        for lines_list in active.values():
            lines_list.append(line)

    # Warn about unclosed snippets
    for name in active:
        print(f"  WARNING: unclosed snippet '{name}' in {filepath}", file=sys.stderr)

    return {name: "\n".join(lines) for name, lines in snippets.items()}


def build_snippet_cache() -> dict[str, dict[str, str]]:
    """Build cache of all snippets from all lesson source files.

    Returns dict mapping "relative/path:name" -> code text.
    """
    cache: dict[str, dict[str, str]] = {}

    for lesson_dir in sorted(LESSONS_DIR.iterdir()):
        if not lesson_dir.is_dir():
            continue
        for cpp_file in sorted(lesson_dir.glob("*.cpp")):
            rel = f"{lesson_dir.name}/{cpp_file.name}"
            snippets = extract_snippets_from_file(cpp_file)
            if snippets:
                cache[rel] = snippets

    return cache


def resolve_placeholder(placeholder: str, cache: dict[str, dict[str, str]]) -> str | None:
    """Resolve a %%SNIPPET:path:name%% placeholder.

    placeholder format: "relative/path/to/file.cpp:snippet_name"
    Multiple names can be joined with '+': "path:name1+name2"
    """
    # Split path and snippet name(s)
    parts = placeholder.rsplit(":", 1)
    if len(parts) != 2:
        return None

    file_rel, names_str = parts
    names = names_str.split("+")

    file_snippets = cache.get(file_rel, {})
    if not file_snippets:
        return None

    resolved = []
    for name in names:
        if name not in file_snippets:
            return None
        resolved.append(file_snippets[name])

    return "\n\n".join(resolved)


def expand_file(tex_path: Path, cache: dict[str, dict[str, str]]) -> tuple[str, list[str]]:
    """Expand all %%SNIPPET%% placeholders in a .tex file.

    Returns (expanded_text, list_of_errors).
    """
    text = tex_path.read_text(encoding="utf-8")
    errors = []

    def replacer(match):
        placeholder = match.group(1)
        resolved = resolve_placeholder(placeholder, cache)
        if resolved is None:
            errors.append(f"Unresolved: %%SNIPPET:{placeholder}%%")
            return match.group(0)  # Leave placeholder as-is
        return resolved

    expanded = PLACEHOLDER.sub(replacer, text)
    return expanded, errors


def cmd_check(args):
    """Check all snippets resolve without writing files."""
    cache = build_snippet_cache()

    total_snippets = sum(len(v) for v in cache.values())
    print(f"Snippet cache: {total_snippets} snippets from {len(cache)} files")

    # Show available snippets
    if args.verbose:
        for file_rel, snippets in sorted(cache.items()):
            for name in sorted(snippets):
                print(f"  {file_rel}:{name}")

    # Check all chapter files
    all_ok = True
    for tex_path in sorted(CHAPTERS_DIR.glob("*.tex")):
        _, errors = expand_file(tex_path, cache)
        if errors:
            all_ok = False
            print(f"\n{tex_path.name}:")
            for err in errors:
                print(f"  ERROR: {err}")

    if all_ok:
        # Check if any chapters use snippets
        has_snippets = False
        for tex_path in sorted(CHAPTERS_DIR.glob("*.tex")):
            if PLACEHOLDER.search(tex_path.read_text(encoding="utf-8")):
                has_snippets = True
                break
        if has_snippets:
            print("\nAll snippets resolved successfully.")
        else:
            print("\nNo snippet placeholders found in chapter files.")
    else:
        print("\nSome snippets could not be resolved.")
        return 1

    return 0


def cmd_expand(args):
    """Expand snippets and write to build directory."""
    cache = build_snippet_cache()

    BUILD_DIR.mkdir(parents=True, exist_ok=True)

    has_errors = False
    expanded_count = 0

    for tex_path in sorted(CHAPTERS_DIR.glob("*.tex")):
        text = tex_path.read_text(encoding="utf-8")

        if not PLACEHOLDER.search(text):
            # No placeholders — copy as-is (symlink for efficiency)
            dst = BUILD_DIR / tex_path.name
            dst.write_text(text, encoding="utf-8")
            continue

        expanded, errors = expand_file(tex_path, cache)
        if errors:
            has_errors = True
            print(f"{tex_path.name}:")
            for err in errors:
                print(f"  ERROR: {err}")

        dst = BUILD_DIR / tex_path.name
        dst.write_text(expanded, encoding="utf-8")
        expanded_count += 1

    print(f"Expanded {expanded_count} files to {BUILD_DIR}")

    if has_errors:
        print("WARNING: Some snippets could not be resolved.")
        return 1

    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command")

    check_p = sub.add_parser("check", help="Verify all snippets resolve")
    check_p.add_argument("-v", "--verbose", action="store_true",
                         help="Show all available snippets")
    check_p.set_defaults(func=cmd_check)

    expand_p = sub.add_parser("expand", help="Write expanded files to build dir")
    expand_p.set_defaults(func=cmd_expand)

    args = parser.parse_args()
    if not hasattr(args, "func"):
        parser.print_help()
        return 0

    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
