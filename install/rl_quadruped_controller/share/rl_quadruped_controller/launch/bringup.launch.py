import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. robot_description from xacro
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('reddog_description'),
            'xacro',
            'robot.xacro'
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 2. RL controller yaml path
    controller_yaml = PathJoinSubstitution([
        FindPackageShare('rl_quadruped_controller'),
        'config',
        'rl_controller.yaml'
    ])

    # 3. Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_yaml],
        remappings=[('~/robot_description', '/robot_description')],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-type', 'joint_state_broadcaster/JointStateBroadcaster'],
        output='screen'
    )

    imu_sensor_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_broadcaster', '--controller-type', 'imu_sensor_broadcaster/IMUSensorBroadcaster'],
        output='screen'
    )

    rl_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rl_quadruped_controller', '--controller-type', 'rl_quadruped_controller::RLQuadrupedController'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster,

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[imu_sensor_broadcaster]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=imu_sensor_broadcaster,
                on_exit=[rl_controller_spawner]
            )
        )
    ])
