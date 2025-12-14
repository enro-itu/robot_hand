import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'three_finger_hand'

    pkg_share = get_package_share_directory(pkg_name)
    default_model_path = os.path.join(
        pkg_share,
        'urdf',
        'three_finger_hand.urdf.xacro'
    )
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'),

        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
