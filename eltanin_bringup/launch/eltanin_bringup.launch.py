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

"""Bring up the global half of the stack: a map, global_costmap and global_path_planner.

Every argument is declared. navyu's localization.launch.py read an argument it never declared
(design section 2.4-4), which is why the value silently stayed at its default.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
        "use_map_server",
        default_value="true",
        description="Start nav2_map_server. Turn off when something else publishes /map.",
    ),
    DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution(
            [FindPackageShare("navyu_navigation"), "map", "map.yaml"]
        ),
        description="Map yaml for nav2_map_server. The default is navyu's map, which is not a "
        "declared dependency: pass map:=<yaml> when navyu is not in the workspace (task 22).",
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
    DeclareLaunchArgument(
        "use_static_robot_tf",
        default_value="false",
        description="Publish a fixed map -> base frame. For driving the planner by hand only; "
        "localization or the simulator owns this transform otherwise.",
    ),
    DeclareLaunchArgument(
        "static_robot_frame",
        default_value="base_footprint",
        description="Child frame of the fixed transform; must match frames.base.",
    ),
    DeclareLaunchArgument("static_robot_x", default_value="0.0", description="Its x in map [m]."),
    DeclareLaunchArgument("static_robot_y", default_value="0.0", description="Its y in map [m]."),
]


def generate_launch_description():
    use_composition = LaunchConfiguration("use_composition")
    use_map_server = LaunchConfiguration("use_map_server")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_params = PathJoinSubstitution(
        [
            FindPackageShare("eltanin_bringup"),
            "config",
            "robot",
            [LaunchConfiguration("robot_profile"), ".yaml"],
        ]
    )
    # Order matters: the machine profile arrives through /**, the node file by node name, and the
    # dict last. No key appears in more than one of them (N-6).
    parameters = [robot_params, LaunchConfiguration("params_file"), {"use_sim_time": use_sim_time}]

    components = [
        ComposableNode(
            package="eltanin_costmap",
            plugin="eltanin_costmap::GlobalCostmap",
            name="global_costmap",
            parameters=parameters,
        ),
        ComposableNode(
            package="eltanin_planner",
            plugin="eltanin_planner::GlobalPathPlanner",
            name="global_path_planner",
            parameters=parameters,
        ),
    ]

    # component_container_mt, never the single-threaded one: both nodes rely on a multi-threaded
    # executor to keep their subscriptions alive while a long computation runs (R-P6-3).
    container = ComposableNodeContainer(
        name="eltanin_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=components,
        output="screen",
        condition=IfCondition(use_composition),
    )

    # The same nodes as separate processes. The generated executables run the same registered
    # component behind the same executor, so this path fails the same way the composed one does.
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
        condition=IfCondition(use_map_server),
    )

    # map_server is a lifecycle node and nav2_lifecycle_manager is not part of this install, so the
    # two transitions are driven directly. lifecycle_bringup waits for the node to appear.
    activate_map_server = ExecuteProcess(
        cmd=["ros2", "run", "nav2_util", "lifecycle_bringup", "map_server"],
        output="screen",
        condition=IfCondition(use_map_server),
    )

    static_robot_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_robot_tf",
        arguments=[
            "--frame-id",
            "map",
            "--child-frame-id",
            LaunchConfiguration("static_robot_frame"),
            "--x",
            LaunchConfiguration("static_robot_x"),
            "--y",
            LaunchConfiguration("static_robot_y"),
        ],
        condition=IfCondition(LaunchConfiguration("use_static_robot_tf")),
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
            static_robot_tf,
            rviz,
        ]
    )
