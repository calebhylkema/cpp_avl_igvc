"""
igvc_raw_control_old — serial_bridge.launch.py

Launches only the serial bridge (standalone).
Normally included automatically by igvc_teleop and igvc_nav launch files.
Use this if you want to run the bridge independently.

  ros2 launch igvc_raw_control_old serial_bridge.launch.py
  ros2 launch igvc_raw_control_old serial_bridge.launch.py serial_port:=/dev/ttyUSB0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory("igvc_raw_control_old")

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
        description="Teensy USB serial device",
    )

    serial_bridge = Node(
        package="igvc_raw_control_old",
        executable="serial_bridge",
        name="igvc_serial_bridge",
        output="screen",
        parameters=[
            os.path.join(pkg_dir, "config", "serial_bridge_params.yaml"),
            {"serial_port": LaunchConfiguration("serial_port")},
        ],
    )

    return LaunchDescription([serial_port_arg, serial_bridge])
