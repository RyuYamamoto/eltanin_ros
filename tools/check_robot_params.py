#!/usr/bin/env python3
"""Keep robot.* and frames.* declared and read in eltanin_ros_common's robot_profile.

N-6 and F-14 record what happens otherwise: navyu declared the same footprint in four nodes, under
node-name keys that had drifted apart, with values that no longer matched. declare_robot_profile()
is the single declaration and validation path, so a node that reads robot.max_linear_vel itself has
bypassed the validation even when the value it gets happens to be right.

Unlike the acceptance condition, which names declare_parameter only, get_parameter is checked too:
there is no legitimate reason to read those keys outside robot_profile, and the false-positive risk
is nil. Tests are exempt. Like the cost literal hook this is a fence, not a proof - another spelling
of the same call gets through.
"""

import re
import sys
from pathlib import Path

PARAMETER_CALL = re.compile(
    r"(?:declare|get)_parameters?\s*(?:<[^<>()]*>)?\s*\(\s*\"(robot|frames)\."
)
ALLOWED_STEMS = {"robot_profile"}


def offending_lines(path):
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return []
    found = []
    for number, line in enumerate(text.splitlines(), start=1):
        match = PARAMETER_CALL.search(line)
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
        for number, prefix, line in offending_lines(path):
            failures.append(f"{path}:{number}: {prefix}.* belongs in robot_profile: {line}")

    if failures:
        print("\n".join(failures))
        print(
            "robot.* and frames.* are declared, read and validated by "
            "eltanin_ros_common's declare_robot_profile() alone (F-14 / N-6). "
            "Take the values from the RobotProfile it returns."
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
