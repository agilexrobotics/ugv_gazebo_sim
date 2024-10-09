import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions
import launch_ros
import launch


def generate_launch_description():
    pkg_share = FindPackageShare(package='scout_description').find('scout_description')
    urdf_file = os.path.join(pkg_share, 'urdf/scout_v2.xacro')

    SetEnvironmentVariable('GAZEBO_MODEL_PATH', pkg_share)

    return LaunchDescription([
        DeclareLaunchArgument('start_x', default_value='0.0',
                              description='X coordinate of starting position'),
        DeclareLaunchArgument('start_y', default_value='0.0',
                              description='Y coordinate of starting position'),
        DeclareLaunchArgument('start_z', default_value='0.0',
                              description='Z coordinate of starting position'),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='Yaw angle of starting orientation'),
        DeclareLaunchArgument('robot_name', default_value='',
                              description='Name and prefix for this robot'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            # parameters=[{
            #     'use_sim_time': True,
            #     'robot_description': Command(
            #         [f'xacro {urdf_file}', ' robot_name:=', LaunchConfiguration('robot_name')])
            parameters=[{'robot_description': launch_ros.descriptions.ParameterValue( launch.substitutions.Command([
                        'xacro ',os.path.join(pkg_share,urdf_file)]), value_type=str)  }]
            ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', LaunchConfiguration('robot_name'),
                '-topic', 'robot_description',
                '-x', LaunchConfiguration('start_x'),
                '-y', LaunchConfiguration('start_y'),
                '-z', LaunchConfiguration('start_z'),
                '-Y', LaunchConfiguration('start_yaw'),
                '-timeout', '1000'
            ],
            output='screen')
    ])
