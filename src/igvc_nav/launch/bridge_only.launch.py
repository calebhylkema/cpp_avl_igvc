"""
igvc_nav — bridge_only.launch.py — AVL IGVC 2026

Launches cmd_vel_bridge + serial_bridge.
Use when Nav2 is running separately and you just need the
/cmd_vel → /igvc/motor_cmd → Teensy serial chain.

  ros2 launch igvc_nav bridge_only.launch.py
  ros2 launch igvc_nav bridge_only.launch.py serial_port:=/dev/ttyUSB0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    nav_pkg = get_package_share_directory("igvc_nav")
    hw_pkg  = get_package_share_directory("igvc_raw_control_old")

    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyACM0",
        description="Teensy USB serial device",
    )

    cmd_vel_bridge = Node(
        package="igvc_nav", executable="cmd_vel_bridge",
        name="igvc_cmd_vel_bridge", output="screen",
        parameters=[os.path.join(nav_pkg, "config", "bridge_params.yaml")],
    )

    serial_bridge = Node(
        package="igvc_raw_control_old", executable="serial_bridge",
        name="igvc_serial_bridge", output="screen",
        parameters=[
            os.path.join(hw_pkg, "config", "serial_bridge_params.yaml"),
            {"serial_port": LaunchConfiguration("serial_port")},
        ],
    )

    return LaunchDescription([
        serial_port_arg,
        cmd_vel_bridge,
        serial_bridge,
    ])
