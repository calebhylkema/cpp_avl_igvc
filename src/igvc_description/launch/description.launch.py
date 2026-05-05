import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('igvc_description')
    xacro_file = os.path.join(pkg, 'urdf', 'igvc.urdf.xacro')

    robot_description = Command(['xacro ', xacro_file])

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    return LaunchDescription([
        robot_state_pub,
    ])
