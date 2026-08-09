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

"""Keep config/robot/*.yaml a complete machine profile: declare_robot_profile() has no fallbacks.

A key spelled wrongly here stops the node with that key named, which is the point. What this file
guards is the other direction: a profile that is missing a key, or carries one nobody declares.
"""

from pathlib import Path

import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / "config" / "robot"

# declare_robot_profile() declares exactly these and nothing else.
ROBOT_KEYS = {
    "footprint",
    "inflation_radius",
    "cost_scaling_factor",
    "max_linear_vel",
    "max_angular_vel",
    "max_accel",
    "max_decel",
}
FRAME_KEYS = {"map", "odom", "base"}

# The kachaka collision box of design section 2.6, in base_link.
KACHAKA_FOOTPRINT = [-0.150, -0.120, 0.237, -0.120, 0.237, 0.120, -0.150, 0.120]

# The simulator models kachaka, so the geometry is shared and only the limits may differ.
SHARED_KEYS = ["footprint", "inflation_radius", "cost_scaling_factor"]

PROFILES = ["kachaka", "sim_robot"]


def profile(name):
    document = yaml.safe_load((CONFIG / f"{name}.yaml").read_text(encoding="utf-8"))
    assert list(document) == ["/**"], "the profile has to reach every node through the wildcard"
    return document["/**"]["ros__parameters"]


@pytest.mark.parametrize("name", PROFILES)
def test_profile_declares_every_key_and_no_others(name):
    parameters = profile(name)
    assert set(parameters) == {"robot", "frames"}
    assert set(parameters["robot"]) == ROBOT_KEYS
    assert set(parameters["frames"]) == FRAME_KEYS


@pytest.mark.parametrize("name", PROFILES)
def test_footprint_is_eight_floats(name):
    footprint = profile(name)["robot"]["footprint"]
    assert len(footprint) == 8
    # An integer array is a different parameter type and would be rejected.
    for value in footprint:
        assert isinstance(value, float) and not isinstance(value, bool)


@pytest.mark.parametrize("name", PROFILES)
def test_limits_are_positive_floats(name):
    robot = profile(name)["robot"]
    for key in ("inflation_radius", "cost_scaling_factor"):
        assert isinstance(robot[key], float)
    for key in ("max_linear_vel", "max_angular_vel", "max_accel", "max_decel"):
        assert isinstance(robot[key], float)
        # validate_positive() in robot_profile.cpp refuses zero and negatives.
        assert robot[key] > 0.0


def test_kachaka_footprint_is_the_collision_box_not_a_square():
    footprint = profile("kachaka")["robot"]["footprint"]
    xs = footprint[0::2]
    ys = footprint[1::2]
    # kachaka's collision box in base_link, not the 0.6 m square navyu and eltanin default to.
    assert (min(xs), max(xs)) == pytest.approx((-0.150, 0.237))
    assert (min(ys), max(ys)) == pytest.approx((-0.120, 0.120))


def test_profiles_share_their_geometry():
    kachaka = profile("kachaka")
    sim = profile("sim_robot")
    for key in SHARED_KEYS:
        assert sim["robot"][key] == pytest.approx(kachaka["robot"][key]), key
    assert sim["frames"] == kachaka["frames"]
