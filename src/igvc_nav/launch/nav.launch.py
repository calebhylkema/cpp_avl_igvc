"""
igvc_nav — nav.launch.py — AVL IGVC 2026

Launches the full autonomous navigation stack:
  - map_server + AMCL         (localisation)
  - SMAC 2D planner           (global path planning)
  - MPPI controller            (local trajectory control)
  - smoother_server           (path smoothing)
  - behavior_server           (recovery behaviours)
  - bt_navigator              (mission-level BT)
  - waypoint_follower
  - velocity_smoother
  - igvc_control              (ros2_control + diff_drive_controller + Teensy serial)

Launch arguments:
  map           Path to map YAML file        (default: empty)
  use_sim_time  true/false                   (default: false)
  params_file   Path to nav2_params.yaml     (default: package config)
  serial_port   Teensy USB device            (default: /dev/ttyACM0)

Example:
  ros2 launch igvc_nav nav.launch.py map:=/path/to/course.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    nav_pkg     = get_package_share_directory("igvc_nav")
    control_pkg = get_package_share_directory("igvc_control")

    # ── Launch arguments ──────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        "map", default_value="",
        description="Absolute path to map YAML file",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(nav_pkg, "config", "nav2_params.yaml"),
    )
    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyACM0",
        description="Teensy USB serial device",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file  = LaunchConfiguration("params_file")
    map_yaml     = LaunchConfiguration("map")
    sim_param    = {"use_sim_time": use_sim_time}

    # ── igvc_control (ros2_control + diff_drive_controller + Teensy) ──────────
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_pkg, "launch", "control.launch.py")
        ),
        launch_arguments={"serial_port": LaunchConfiguration("serial_port")}.items(),
    )

    # ── Nav2 nodes ────────────────────────────────────────────────────────────
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen",
        parameters=[params_file, sim_param, {"yaml_filename": map_yaml}],
    )
    amcl = Node(
        package="nav2_amcl", executable="amcl", name="amcl",
        output="screen",
        parameters=[params_file, sim_param],
        remappings=[("odom", "/diff_drive_controller/odom")],
    )
    controller_server = Node(
        package="nav2_controller", executable="controller_server",
        name="controller_server", output="screen",
        parameters=[params_file, sim_param],
        remappings=[
            ("cmd_vel", "cmd_vel_nav"),
            ("odom", "/diff_drive_controller/odom"),
        ],
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother", executable="velocity_smoother",
        name="velocity_smoother", output="screen",
        parameters=[params_file, sim_param],
        remappings=[
            ("cmd_vel",          "cmd_vel_nav"),
            ("cmd_vel_smoothed", "/diff_drive_controller/cmd_vel_unstamped"),
            ("odom",             "/diff_drive_controller/odom"),
        ],
    )
    smoother_server = Node(
        package="nav2_smoother", executable="smoother_server",
        name="smoother_server", output="screen",
        parameters=[params_file, sim_param],
    )
    planner_server = Node(
        package="nav2_planner", executable="planner_server",
        name="planner_server", output="screen",
        parameters=[params_file, sim_param],
    )
    behavior_server = Node(
        package="nav2_behaviors", executable="behavior_server",
        name="behavior_server", output="screen",
        parameters=[params_file, sim_param],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator", executable="bt_navigator",
        name="bt_navigator", output="screen",
        parameters=[params_file, sim_param],
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower", executable="waypoint_follower",
        name="waypoint_follower", output="screen",
        parameters=[params_file, sim_param],
    )

    # ── Lifecycle managers ────────────────────────────────────────────────────
    lifecycle_localization = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_localization", output="screen",
        parameters=[sim_param, {"autostart": True},
                    {"node_names": ["map_server", "amcl"]}],
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
        map_arg, use_sim_time_arg, params_file_arg, serial_port_arg,
        control_launch,
        map_server, amcl,
        controller_server, velocity_smoother, smoother_server,
        planner_server, behavior_server, bt_navigator, waypoint_follower,
        lifecycle_localization, lifecycle_navigation,
    ])
