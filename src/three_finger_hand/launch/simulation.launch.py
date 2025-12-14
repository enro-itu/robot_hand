import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'three_finger_hand'
    pkg_share = get_package_share_directory(pkg_name)

    install_dir = os.path.dirname(pkg_share)

    gz_plugin_path = os.path.join(
        get_package_share_directory('gz_ros2_control'), 'lib')

    set_res_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_dir
    )

    set_plugin_path = AppendEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=gz_plugin_path
    )

    xacro_file = os.path.join(pkg_share, 'urdf', 'three_finger_hand.urdf.xacro')  # Xacro -> URDF
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # Run Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'three_finger_hand',
                   '-z', '0.5'],
        output='screen'
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Bridge (ROS2 <-> Gazebo)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],  # [ yenine @ olması gerekebilir, çift taraflı köprü için
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    load_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller"],
    )

    return LaunchDescription([
        set_res_path,
        set_plugin_path,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        bridge,
        load_joint_state_broadcaster,
        load_position_controller
    ])
