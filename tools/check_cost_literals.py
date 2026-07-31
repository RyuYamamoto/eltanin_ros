#!/usr/bin/env python3
"""Keep the cost sentinel values and the visualization coefficients in cost_conversion.

Design decision D-20 says the cost range conversion exists in exactly one place. Nothing in the
build enforces that, so this hook is the fence: the sentinels 253 / 254 / 255 and the visualization
coefficients 97 / 251 may appear only in eltanin_ros_common/{include,src}/.../cost_conversion.*.

Only these five numbers are checked. 100, -1, 65 and 25 appear all over ordinary code, and a hook
with false positives is a hook someone disables. Tests are not checked: pinning the table by writing
its values down is what a test is for.
"""

import re
import sys
from pathlib import Path

LITERALS = re.compile(r"(?<![\w.])(253|254|255|97|251)(?![\w.])")
ALLOWED_STEMS = {"cost_conversion"}


def offending_lines(path):
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return []
    found = []
    for number, line in enumerate(text.splitlines(), start=1):
        match = LITERALS.search(line)
        if match:
            found.append((number, match.group(1), line.strip()))
    return found


def main(argv):
    failures = []
    for name in argv:
        path = Path(name)
        if path.stem in ALLOWED_STEMS:
            continue
        parts = path.parts
        if "include" not in parts and "src" not in parts:
            continue
        if "test" in parts:
            continue
        for number, literal, line in offending_lines(path):
            failures.append(
                f"{path}:{number}: literal {literal} belongs in cost_conversion: {line}"
            )

    if failures:
        print("\n".join(failures))
        print(
            "Cost values are converted in eltanin_ros_common/cost_conversion only (D-20). "
            "Use eltanin::map's named constants, or call the conversion."
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
