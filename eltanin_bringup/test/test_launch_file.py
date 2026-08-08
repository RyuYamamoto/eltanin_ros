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

"""Keep every launch argument declared and both start-up paths carrying the same parameters."""

import ast
import importlib.util
from pathlib import Path

import pytest

from launch import Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch_ros.actions import ComposableNodeContainer, Node

LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "eltanin_bringup.launch.py"

# autostart_output is absent on purpose: its consumer is collision_predictor (task 12).
EXPECTED_ARGUMENTS = {
    "robot_profile",
    "use_sim_time",
    "use_composition",
    "use_rviz",
    "use_goal_pose_relay",
    "map",
    "params_file",
    "rviz_config",
}

# Loaded as a component and started as its own process.
ELTANIN_NODES = {"global_costmap", "global_path_planner"}


def launch_module():
    spec = importlib.util.spec_from_file_location("eltanin_bringup_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def declared_and_referenced():
    """Read the source, not the built description: a reference can hide behind a branch."""
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    declared = set()
    referenced = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id == "DeclareLaunchArgument":
            name = node.args[0] if node.args else None
            if isinstance(name, ast.Constant):
                declared.add(name.value)
        elif node.func.id == "LaunchConfiguration":
            name = node.args[0] if node.args else None
            if isinstance(name, ast.Constant):
                referenced.add(name.value)
    return declared, referenced


def test_every_referenced_argument_is_declared():
    declared, referenced = declared_and_referenced()
    assert not referenced - declared, f"{sorted(referenced - declared)} is read but never declared"


def test_the_declared_arguments_are_the_documented_ones():
    declared, _ = declared_and_referenced()
    assert declared == EXPECTED_ARGUMENTS


def test_every_declared_argument_is_used():
    # An argument nobody reads is an argument that does nothing when passed.
    declared, referenced = declared_and_referenced()
    assert not declared - referenced, f"{sorted(declared - referenced)} is declared but never read"


def test_the_description_builds_and_declares_its_arguments():
    entities = launch_module().generate_launch_description().entities
    arguments = [entity for entity in entities if isinstance(entity, DeclareLaunchArgument)]
    assert {argument.name for argument in arguments} == EXPECTED_ARGUMENTS
    for argument in arguments:
        # --show-args prints these.
        assert argument.description


def text(value):
    """Flatten a normalized substitution list back to its literal text."""
    if isinstance(value, TextSubstitution):
        return value.text
    if isinstance(value, (list, tuple)):
        return "".join(text(item) for item in value)
    return str(value)


def describe(value):
    """A canonical form for substitutions, using describe(), which needs no LaunchContext."""
    if isinstance(value, Substitution):
        return value.describe()
    if isinstance(value, (list, tuple)):
        return [describe(item) for item in value]
    if isinstance(value, dict):
        return {text(key): describe(item) for key, item in value.items()}
    return str(value)


def container_and_nodes():
    entities = launch_module().generate_launch_description().entities
    # ComposableNodeContainer is itself a Node, so the plain nodes need it filtered out.
    containers = [e for e in entities if isinstance(e, ComposableNodeContainer)]
    assert len(containers) == 1
    nodes = [
        e for e in entities if isinstance(e, Node) and not isinstance(e, ComposableNodeContainer)
    ]
    descriptions = containers[0]._ComposableNodeContainer__composable_node_descriptions
    return containers[0], descriptions, nodes


def test_one_multi_threaded_container_holds_both_nodes():
    container, descriptions, _ = container_and_nodes()
    # The single-threaded container would let a long computation stop the subscriptions.
    assert text(container.node_executable) == "component_container_mt"
    assert {text(d.node_name) for d in descriptions} == ELTANIN_NODES


def test_the_components_share_their_process_without_copying():
    _, descriptions, _ = container_and_nodes()
    for description in descriptions:
        extra = {}
        for argument in description.extra_arguments or []:
            extra.update({text(key): value for key, value in argument.items()})
        # 16 MB of costmap crosses this boundary on every update.
        assert extra.get("use_intra_process_comms") is True, (
            f"{text(description.node_name)} copies every message it shares; "
            "use_intra_process_comms is what makes composition worth having here"
        )


@pytest.mark.parametrize("name", sorted(ELTANIN_NODES))
def test_both_start_up_paths_pass_the_same_parameters(name):
    _, descriptions, nodes = container_and_nodes()
    composed = next(d for d in descriptions if text(d.node_name) == name)
    separate = next(n for n in nodes if text(n.node_executable) == name)
    # Different machine profiles here is what makes inflation and the Free boundary disagree.
    assert describe(composed.parameters) == describe(separate._Node__parameters)


def test_the_map_server_is_not_given_the_machine_profile():
    _, _, nodes = container_and_nodes()
    map_server = next(n for n in nodes if text(n.node_executable) == "map_server")
    rendered = str(describe(map_server._Node__parameters))
    # Harmless if it were, but the wildcard file is about the machine the eltanin nodes drive.
    assert "config/robot" not in rendered and "robot_profile" not in rendered
