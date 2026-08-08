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

"""Keep config/navigation.yaml node-specific, keyed by the real node names, and correctly typed."""

from pathlib import Path

import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / "config" / "navigation.yaml"

# Node names are fixed in the constructors, so a key that is not one of these reaches nothing.
EXPECTED_NODES = {"global_costmap", "global_path_planner"}

# What declare_robot_profile() owns, by exact name: publish_footprint_path is not one of them.
MACHINE_GROUPS = {"robot", "frames"}
MACHINE_KEYS = {
    "footprint",
    "inflation_radius",
    "cost_scaling_factor",
    "max_linear_vel",
    "max_angular_vel",
    "max_accel",
    "max_decel",
}

# The type each declaration gives the parameter; a nested dict is a parameter group. Both sides
# of the comparison are exact, so adding a parameter to a node means editing this table.
EXPECTED = {
    "global_costmap": {
        "inflate_unknown": bool,
        "occupied_threshold": int,
        "free_threshold": int,
        "publish_visualization": bool,
    },
    "global_path_planner": {
        "planner_type": str,
        "start_search_radius_cells": int,
        "weight_data": float,
        "weight_smooth": float,
        "smoother_tolerance": float,
        "smoother_max_iterations": int,
        "publish_raw_path": bool,
        "publish_footprint_path": bool,
        "footprint_marker_stride": int,
        "unknown_is_free": bool,
        "tf_lookup_timeout": float,
        "hybrid": {
            "heading_bins": int,
            "minimum_turning_radius": float,
            "motion_step": float,
            "collision_check_step": float,
            "dubins_expansion_distance": float,
            "analytic_expansion_ratio": float,
            "motion_model": str,
            "heuristic_weight": float,
            "steering_penalty": float,
            "steering_change_penalty": float,
            "max_expansions": int,
            "corridor_margin_cells": int,
            "max_states": int,
        },
    },
}


def document():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def parameters(node):
    return document()[node]["ros__parameters"]


def test_top_level_keys_are_the_real_node_names():
    assert set(document()) == EXPECTED_NODES


def check_keys(expected, actual, where):
    # A missing key cannot be edited without reading the source; an extra one is dropped silently.
    assert set(actual) == set(expected), where
    for key, expected_type in expected.items():
        if isinstance(expected_type, dict):
            check_keys(expected_type, actual[key], f"{where}.{key}")


def check_types(expected, actual, where):
    for key, expected_type in expected.items():
        value = actual[key]
        name = f"{where}.{key}"
        if isinstance(expected_type, dict):
            check_types(expected_type, value, name)
        elif expected_type is bool:
            assert isinstance(value, bool), name
        elif expected_type is int:
            # bool is a subclass of int in Python; a true/false here would be the wrong type.
            assert isinstance(value, int) and not isinstance(value, bool), name
        elif expected_type is str:
            assert isinstance(value, str), name
        else:
            # 1e-4 is a string to some YAML readers. 0.0001 and 1.0e-4 are not.
            assert isinstance(value, float), name


@pytest.mark.parametrize("node", sorted(EXPECTED))
def test_node_declares_every_key_and_no_others(node):
    check_keys(EXPECTED[node], parameters(node), node)


@pytest.mark.parametrize("node", sorted(EXPECTED))
def test_values_have_the_type_their_declaration_gives_them(node):
    check_types(EXPECTED[node], parameters(node), node)


def test_machine_values_are_not_repeated_here():
    # Under a node name these would shadow the wildcard for that node alone. Both spellings.
    def walk(node, where):
        if isinstance(node, dict):
            for key, value in node.items():
                assert key not in MACHINE_GROUPS, f"{where}.{key}.* belongs in config/robot/*.yaml"
                assert key not in MACHINE_KEYS, f"{where}.{key} belongs in config/robot/*.yaml"
                for group in MACHINE_GROUPS:
                    assert not key.startswith(group + "."), f"{where}.{key} is a machine value"
                walk(value, f"{where}.{key}")
        elif isinstance(node, list):
            for index, value in enumerate(node):
                walk(value, f"{where}[{index}]")

    walk(document(), "navigation.yaml")


def test_smoother_weights_converge():
    # Below 2, or the smoother diverges and the node refuses to start.
    planner = parameters("global_path_planner")
    assert planner["weight_data"] + 4.0 * planner["weight_smooth"] < 2.0


def test_thresholds_are_ordered_and_in_range():
    costmap = parameters("global_costmap")
    assert 0 <= costmap["free_threshold"] < costmap["occupied_threshold"] <= 100
