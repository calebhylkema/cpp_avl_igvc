import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('igvc_control')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Teensy USB serial port')

    # Process xacro → URDF, passing serial_port arg through
    xacro_file = os.path.join(pkg, 'config', 'igvc.urdf.xacro')
    serial_port = LaunchConfiguration('serial_port')
    robot_description = Command([
        'xacro ', xacro_file, ' serial_port:=', serial_port
    ])

    controllers_yaml = os.path.join(pkg, 'config', 'controllers.yaml')

    # Robot state publisher (TF from URDF)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # Controller manager (loads hardware interface + controllers)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        output='screen',
    )

    # Spawn joint_state_broadcaster after controller_manager starts
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # Spawn diff_drive_controller after joint_state_broadcaster is up
    ddc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    # Chain: controller_manager → jsb → ddc
    delay_ddc = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[ddc_spawner],
        )
    )

    return LaunchDescription([
        serial_port_arg,
        robot_state_pub,
        controller_manager,
        jsb_spawner,
        delay_ddc,
    ])
