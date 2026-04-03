import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    teleop_config = os.path.join(
        get_package_share_directory("igvc_teleop"),
        "config", "teleop_params.yaml"
    )

    control_launch = os.path.join(
        get_package_share_directory("igvc_control"),
        "launch", "control.launch.py"
    )

    # Keyboard → /diff_drive_controller/cmd_vel_unstamped
    teleop_node = Node(
        package="igvc_teleop",
        executable="teleop_node",
        name="igvc_teleop_node",
        output="screen",
        parameters=[teleop_config],
        emulate_tty=True,
    )

    # ros2_control stack (hardware interface + diff_drive_controller)
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch),
    )

    return LaunchDescription([
        control,
        teleop_node,
    ])
