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

"""Bring up the global half of the stack: a map, global_costmap and global_path_planner."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        "robot_profile",
        default_value="kachaka",
        description="Name of the config/robot/<name>.yaml the whole stack reads.",
    ),
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Follow /clock. The tf lookup timeout only expires while that clock runs.",
    ),
    DeclareLaunchArgument(
        "use_composition",
        default_value="true",
        description="Load the nodes into one component_container_mt instead of one process each.",
    ),
    DeclareLaunchArgument("use_rviz", default_value="true", description="Start RViz."),
    DeclareLaunchArgument(
        "map",
        default_value="",
        description="Map yaml for nav2_map_server. Empty means no map_server: /map then comes "
        "from whoever else publishes it.",
    ),
    DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("eltanin_bringup"), "config", "navigation.yaml"]
        ),
        description="Node-specific parameters, keyed by node name.",
    ),
    DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("eltanin_bringup"), "rviz", "eltanin.rviz"]
        ),
        description="RViz configuration.",
    ),
]


def have_map():
    """Whether map names a file. One instance per action, since a Condition is not shared."""
    return IfCondition(PythonExpression(['"', LaunchConfiguration("map"), '" != ""']))


def generate_launch_description():
    use_composition = LaunchConfiguration("use_composition")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_params = PathJoinSubstitution(
        [
            FindPackageShare("eltanin_bringup"),
            "config",
            "robot",
            [LaunchConfiguration("robot_profile"), ".yaml"],
        ]
    )
    # The machine profile arrives through /**, the node file by node name. No key is in both.
    parameters = [robot_params, LaunchConfiguration("params_file"), {"use_sim_time": use_sim_time}]

    # 16 MB of costmap crosses this boundary on every update, and nothing needs a copy of it.
    intra_process = [{"use_intra_process_comms": True}]

    components = [
        ComposableNode(
            package="eltanin_costmap",
            plugin="eltanin_costmap::GlobalCostmap",
            name="global_costmap",
            parameters=parameters,
            extra_arguments=intra_process,
        ),
        ComposableNode(
            package="eltanin_planner",
            plugin="eltanin_planner::GlobalPathPlanner",
            name="global_path_planner",
            parameters=parameters,
            extra_arguments=intra_process,
        ),
    ]

    # Never the single-threaded container: a long computation would stop the subscriptions.
    container = ComposableNodeContainer(
        name="eltanin_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=components,
        output="screen",
        condition=IfCondition(use_composition),
    )

    # The same registered components as separate processes, so both paths fail the same way.
    separate_nodes = [
        Node(
            package="eltanin_costmap",
            executable="global_costmap",
            name="global_costmap",
            parameters=parameters,
            output="screen",
            condition=UnlessCondition(use_composition),
        ),
        Node(
            package="eltanin_planner",
            executable="global_path_planner",
            name="global_path_planner",
            parameters=parameters,
            output="screen",
            condition=UnlessCondition(use_composition),
        ),
    ]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[
            {"yaml_filename": LaunchConfiguration("map")},
            {"frame_id": "map"},
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        condition=have_map(),
    )

    # map_server is a lifecycle node and nav2_lifecycle_manager is not part of this install.
    activate_map_server = ExecuteProcess(
        cmd=["ros2", "run", "nav2_util", "lifecycle_bringup", "map_server"],
        output="screen",
        condition=have_map(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            *ARGUMENTS,
            map_server,
            activate_map_server,
            container,
            *separate_nodes,
            rviz,
        ]
    )
