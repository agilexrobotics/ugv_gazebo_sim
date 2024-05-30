import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    gazebo_world_path = os.path.join(
        get_package_share_directory('hunter_se_gazebo'), 'world', 'house.world')

    gazebo_options_dict = {
        'world': gazebo_world_path,
        'verbose': 'true'
    }

    gazebo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments=gazebo_options_dict.items()
    )

    car_sim_options = {
        'start_x': '0',
        'start_y': '0',
        'start_z': '0',
        'start_yaw': '0',
        'pub_tf': 'true',
        'tf_freq': '100.0',
        'blue': 'false'
    }

    spawn_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                         get_package_share_directory('hunter_se_gazebo'),
                         'launch', 'hunter_se_spawn.launch.py')
        ]),
        launch_arguments=car_sim_options.items()
    )

    return LaunchDescription([
        gazebo_simulator,
        spawn_car,
    ])
