"""
igvc_simulation — sim.launch.py — AVL IGVC 2026

Launches the full Nav2 pipeline with mock hardware for desktop visualization.
No Teensy or real sensors required.

Usage:
  ros2 launch igvc_simulation sim.launch.py
  ros2 launch igvc_simulation sim.launch.py map:=/path/to/custom_map.yaml

To test:
  1. In RViz, click "2D Goal Pose" to send a navigation goal
  2. Watch the planner compute a path and the controller track it
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    sim_pkg = get_package_share_directory("igvc_simulation")
    control_pkg = get_package_share_directory("igvc_control")

    # ── Launch arguments ──────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(sim_pkg, "maps", "blank_map.yaml"),
        description="Path to map YAML file",
    )

    # ── Config paths ──────────────────────────────────────────────────────────
    nav2_params = os.path.join(sim_pkg, "config", "nav2_params_sim.yaml")
    rviz_config = os.path.join(sim_pkg, "rviz", "nav2_sim.rviz")
    controllers_yaml = os.path.join(control_pkg, "config", "controllers.yaml")
    map_yaml = LaunchConfiguration("map")
    sim_param = {"use_sim_time": False}

    # ── ros2_control with mock hardware URDF ─────────────────────────────────
    xacro_file = os.path.join(sim_pkg, "config", "igvc_sim.urdf.xacro")
    robot_description = Command(["xacro ", xacro_file])

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    ddc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen",
    )

    delay_ddc = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[ddc_spawner],
        )
    )

    static_tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id", "map",
            "--child-frame-id", "odom",
            "--x", "0", "--y", "0", "--z", "0",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    # ── Nav2 nodes ────────────────────────────────────────────────────────────
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen",
        parameters=[nav2_params, sim_param, {"yaml_filename": map_yaml}],
    )

    controller_server = Node(
        package="nav2_controller", executable="controller_server",
        name="controller_server", output="screen",
        parameters=[nav2_params, sim_param],
        remappings=[
            ("cmd_vel", "cmd_vel_nav"),
            ("odom", "/diff_drive_controller/odom"),
        ],
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother", executable="velocity_smoother",
        name="velocity_smoother", output="screen",
        parameters=[nav2_params, sim_param],
        remappings=[
            ("cmd_vel",          "cmd_vel_nav"),
            ("cmd_vel_smoothed", "/diff_drive_controller/cmd_vel_unstamped"),
            ("odom",             "/diff_drive_controller/odom"),
        ],
    )

    smoother_server = Node(
        package="nav2_smoother", executable="smoother_server",
        name="smoother_server", output="screen",
        parameters=[nav2_params, sim_param],
    )

    planner_server = Node(
        package="nav2_planner", executable="planner_server",
        name="planner_server", output="screen",
        parameters=[nav2_params, sim_param],
    )

    behavior_server = Node(
        package="nav2_behaviors", executable="behavior_server",
        name="behavior_server", output="screen",
        parameters=[nav2_params, sim_param],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator", executable="bt_navigator",
        name="bt_navigator", output="screen",
        parameters=[nav2_params, sim_param],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower", executable="waypoint_follower",
        name="waypoint_follower", output="screen",
        parameters=[nav2_params, sim_param],
    )

    # ── Lifecycle managers ────────────────────────────────────────────────────
    lifecycle_map = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_map", output="screen",
        parameters=[sim_param, {"autostart": True},
                    {"node_names": ["map_server"]}],
    )

    lifecycle_navigation = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation", output="screen",
        parameters=[sim_param, {"autostart": True},
                    {"node_names": [
                        "controller_server", "smoother_server", "planner_server",
                        "behavior_server", "bt_navigator", "waypoint_follower",
                        "velocity_smoother",
                    ]}],
    )

    return LaunchDescription([
        map_arg,
        # ros2_control (mock hardware)
        robot_state_pub,
        controller_manager,
        jsb_spawner,
        delay_ddc,
        static_tf_map_odom,
        # Nav2 nodes + lifecycle managers (all at once, managers wait internally)
        map_server,
        controller_server, velocity_smoother, smoother_server,
        planner_server, behavior_server, bt_navigator, waypoint_follower,
        lifecycle_map, lifecycle_navigation,
        # Visualization
        rviz,
    ])
