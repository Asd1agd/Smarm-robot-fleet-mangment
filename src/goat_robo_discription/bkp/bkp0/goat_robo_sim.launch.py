#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_path
import xacro


def create_robot_group(name: str, x: float, y: float, yaw: float):
    pkg_path = get_package_share_path('goat_robo_discription')
    urdf_path = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' robot_name:=', name,
            ' tf_prefix:=', name # + '_'  Remove trailing underscore
        ]),
        value_type=str
    )

     # Add static transform publisher for map->odom
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_tf_{name}',
        arguments=[str(x), str(y), '0', '0', '0', str(yaw), 'map', f'{name}_odom'],
        output='screen'
    )

    return GroupAction([
        PushRosNamespace(name),
        static_tf_node,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                # 'frame_prefix': name + '_',
                'use_sim_time': True
            }],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            name=f'gz_create_{name}',
            arguments=[
                '-name', name,
                '-topic', f'/{name}/robot_description',
                '-x', str(x),
                '-y', str(y),
                '-Y', str(yaw)
            ],
            output='screen'
        )
    ])


def generate_launch_description():
    pkg_path = get_package_share_path('goat_robo_discription')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'robot_config.rviz')
    gazebo_config_path = os.path.join(pkg_path, 'config', 'gz_bridge_config.yaml')
    gz_launch_path = os.path.join(get_package_share_path('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    my_world_path = os.path.join(pkg_path, 'world', 'tugbot_warehouse.sdf')

    # Gazebo simulation launch
    gz_sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': f'{my_world_path} -r'
        }.items()
    )

    # Bridge
    parameter_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{'config_file': gazebo_config_path}],
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

    # Robot groups with unique names and positions
    robot1 = create_robot_group(name='robot1', x=0.0, y=0.0, yaw=0.0)
    robot2 = create_robot_group(name='robot2', x=2.0, y=2.0, yaw=1.57)

    return LaunchDescription([
        gz_sim_include,
        parameter_bridge_node,
        rviz_node,
        robot1,
        robot2
    ])