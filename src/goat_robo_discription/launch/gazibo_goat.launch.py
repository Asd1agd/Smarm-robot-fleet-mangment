#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path
import xacro



def generate_launch_description():
    pkg_path = get_package_share_path('goat_robo_discription')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'robot_config2.rviz')
    gz_launch_path = os.path.join(get_package_share_path('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    my_world_path = os.path.join(pkg_path, 'world', 'tugbot_warehouse.sdf')

    map_yaml = os.path.join(pkg_path, 'map', 'my_map.yaml')

    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml}, {'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'), ('/map', 'map')]
    )

    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': True}, {'autostart': True}, {'node_names': ['map_server']}]
    )

    # Gazebo simulator with custom world
    gz_sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': f'{my_world_path} -r'
        }.items()
    )

    # # Gazebo simulator with custom world
    # gz_sim_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(gz_launch_path),
    #     launch_arguments={
    #         'gz_args': f'empty.sdf -r'
    #     }.items()
    # )

    # Clock bridge
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )


    return LaunchDescription([
        gz_sim_include,
        clock_bridge_node,
        rviz_node,
        map_server_node,
        TimerAction(period=5.0, actions=[lifecycle_manager]),
    ])
