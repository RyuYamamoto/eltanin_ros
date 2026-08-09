# Copyright 2026 RyuYamamoto.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License

"""Keep rviz/eltanin.rviz pointed at topics that exist, in both directions.

A display reads a topic, so something in the stack has to publish it; some of those publishers are
switched on by a parameter, so the check is against config/navigation.yaml rather than a fixed
list. A tool writes a topic, so the same launch file has to start something that reads it —
otherwise the button is there and pressing it does nothing.
"""

import importlib.util
from pathlib import Path

import yaml

from launch.substitutions import TextSubstitution

PACKAGE = Path(__file__).resolve().parents[1]
RVIZ = PACKAGE / "rviz" / "eltanin.rviz"
NAVIGATION = PACKAGE / "config" / "navigation.yaml"
LAUNCH_FILE = PACKAGE / "launch" / "eltanin_bringup.launch.py"

# Topic -> the parameter that has to be true for a publisher to exist. None means always.
PUBLISHERS = {
    "/map": None,
    "/global_costmap/global_costmap_visual": ("global_costmap", "publish_visualization"),
    "/global_path_planner/global_path": None,
    "/global_path_planner/global_path_raw": ("global_path_planner", "publish_raw_path"),
    "/global_path_planner/footprint_path": ("global_path_planner", "publish_footprint_path"),
}

# RViz's Map display derives an Update Topic and offers no way to remove it.
SCHEMA_ARTIFACTS = {topic + "_updates" for topic in PUBLISHERS}


def topics_under(section):
    """Every topic name reachable from one section of the configuration."""
    document = yaml.safe_load(RVIZ.read_text(encoding="utf-8"))
    found = set()

    def walk(node):
        if isinstance(node, dict):
            for key, value in node.items():
                if key == "Value" and isinstance(value, str) and value.startswith("/"):
                    found.add(value)
                else:
                    walk(value)
        elif isinstance(node, list):
            for value in node:
                walk(value)

    walk(document["Visualization Manager"][section])
    return found


def subscribed_topics():
    return topics_under("Displays")


def tool_topics():
    return topics_under("Tools")


def text(value):
    """Flatten a normalized substitution list back to its literal text."""
    if isinstance(value, TextSubstitution):
        return value.text
    if isinstance(value, (list, tuple)):
        return "".join(text(item) for item in value)
    return str(value)


def remapped_destinations():
    """The topic names the launch file remaps its own nodes onto."""
    spec = importlib.util.spec_from_file_location("eltanin_bringup_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    found = set()
    for entity in module.generate_launch_description().entities:
        for _, destination in getattr(entity, "_Node__remappings", None) or []:
            found.add(text(destination))
    return found


def enabled(gate):
    node, parameter = gate
    document = yaml.safe_load(NAVIGATION.read_text(encoding="utf-8"))
    return document.get(node, {}).get("ros__parameters", {}).get(parameter) is True


def test_every_display_has_a_publisher():
    unknown = sorted(subscribed_topics() - set(PUBLISHERS) - SCHEMA_ARTIFACTS)
    assert not unknown, (
        f"{unknown} is displayed but nothing in this stack publishes it (F-13 / N-13). "
        "Either drop the display or add the publisher to PUBLISHERS with its gating parameter."
    )


def test_every_gated_publisher_is_switched_on():
    off = sorted(
        topic
        for topic in subscribed_topics() & set(PUBLISHERS)
        if PUBLISHERS[topic] is not None and not enabled(PUBLISHERS[topic])
    )
    assert not off, (
        f"{off} is displayed, but the parameter that creates its publisher is off in "
        "config/navigation.yaml. A display whose publisher never exists is the navyu defect."
    )


def test_fixed_frame_is_the_map_frame():
    document = yaml.safe_load(RVIZ.read_text(encoding="utf-8"))
    options = document["Visualization Manager"]["Global Options"]
    # With no tf at all nothing draws: tf2 answers canTransform for two unknown frames false.
    assert options["Fixed Frame"] == "map"


def test_the_costmap_uses_the_costmap_colour_scheme():
    document = yaml.safe_load(RVIZ.read_text(encoding="utf-8"))
    displays = document["Visualization Manager"]["Displays"]
    costmap = [
        display
        for display in displays
        if display.get("Topic", {}).get("Value") == "/global_costmap/global_costmap_visual"
    ]
    assert len(costmap) == 1
    # The map scheme draws 1..98 as one shade, hiding the inflation this display exists for.
    assert costmap[0]["Color Scheme"] == "costmap"


def test_no_tool_publishes_a_topic_nobody_reads():
    # The mirror of the display check: a button whose topic no launched node reads is the same
    # defect seen from the other side. /goal_pose is read by goal_pose_relay, remapped onto it.
    unread = sorted(tool_topics() - remapped_destinations())
    assert not unread, (
        f"{unread} is published by an rviz tool, but eltanin_bringup.launch.py starts nothing "
        "that reads it. Either drop the tool or launch its subscriber in the same commit."
    )


def test_the_goal_tool_reaches_the_planner():
    document = yaml.safe_load(RVIZ.read_text(encoding="utf-8"))
    tools = {tool["Class"] for tool in document["Visualization Manager"]["Tools"]}
    # The whole point of goal_pose_relay: 2D Goal Pose has to be able to start a plan.
    assert "rviz_default_plugins/SetGoal" in tools
    assert "/goal_pose" in tool_topics()
