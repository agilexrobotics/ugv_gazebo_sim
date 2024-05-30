#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'fishbot.world'
    world = os.path.join(get_package_share_directory('ranger_mini'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('ranger_mini'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_four_ws_control = get_package_share_directory('four_ws_control')

    gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        )

    gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        )

    controller = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_four_ws_control, 'launch', 'four_ws_control.launch.py')
            ),
        )

    robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/ranger_mini_v2_display.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

    forward_position_controller = ExecuteProcess( 
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_position_controller'], output='screen'
        )

    forward_velocity_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller'], output='screen'
        )

    joint_state_broadcaster = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], output='screen'
        )
        
    joy_node = Node(
        package = "joy",
        executable = "joy_node"
        )

    nodes = [
        gzserver,
        gzclient, 
        robot_state_publisher,
        joy_node,
        controller,
        forward_position_controller,
        forward_velocity_controller,
        joint_state_broadcaster
    ]

    return LaunchDescription(nodes)