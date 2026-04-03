"""
igvc_nav — bridge_only.launch.py — AVL IGVC 2026

Launches just the igvc_control stack (ros2_control + diff_drive_controller).
Use when Nav2 is running separately and you just need the motor controller.

  ros2 launch igvc_nav bridge_only.launch.py
  ros2 launch igvc_nav bridge_only.launch.py serial_port:=/dev/ttyACM1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    control_pkg = get_package_share_directory("igvc_control")

    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyACM0",
        description="Teensy USB serial device",
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_pkg, "launch", "control.launch.py")
        ),
        launch_arguments={"serial_port": LaunchConfiguration("serial_port")}.items(),
    )

    return LaunchDescription([
        serial_port_arg,
        control_launch,
    ])
