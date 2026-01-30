#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    # Get package paths
    pkg_path = get_package_share_path('goat_robo_discription')
    pkg_path_nav2 = get_package_share_path('nav2_bringup')
    
    # YAML file paths
    map_yaml = os.path.join(pkg_path, 'map', 'my_map.yaml')
    nav2_params_path = os.path.join(pkg_path, 'yaml_config', 'nav2_params.yaml')

    # Launch file paths
    goat_robo_sim_path = os.path.join(pkg_path, 'launch', 'goat_robo_sim.launch.py')
    # nav2_bringup_path = os.path.join(pkg_path_nav2, 'launch', 'bringup_launch.py') 
    nav2_bringup_path = os.path.join(pkg_path, 'launch', 'goat_nav2_bringup_launch.py')

    # nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')


    # Include robot sim launch
    goat_robo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(goat_robo_sim_path)
    )

    # Include nav2 bringup with namespace
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_path),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'True',
            'params_file': nav2_params_path,
            # 'use_namespace': 'True'
        }.items()
)



    return LaunchDescription([
        goat_robo_sim,
        nav2_launch
    ])
