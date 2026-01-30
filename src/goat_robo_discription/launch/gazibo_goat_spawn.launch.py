#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path


def print_robot_info(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    print("✅ Resolved robot_name:", robot_name)
    print("✅ Resolved robot_name with _:", f"{robot_name}_")
    print(f"✅ Position: x={x}, y={y}, yaw={yaw}")
    return []


def create_robot_spawn(context):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    pkg_path = get_package_share_path('goat_robo_discription')
    urdf_path = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' robot_name:=', robot_name,
            ' tf_prefix:=', f"{robot_name}_"
        ]),
        value_type=str
    )

    return [GroupAction([
        PushRosNamespace(robot_name),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            output='screen'
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name=f"gz_create_{robot_name}",
            arguments=[
                '-name', robot_name,
                '-topic', f"/{robot_name}/robot_description",
                '-x', x,
                '-y', y,
                '-Y', yaw
            ],
            output='screen'
        ),
    ])]


def create_robot_bridge(context):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    pkg_path = get_package_share_path('goat_robo_discription')
    config_path = os.path.join(pkg_path, 'config', f'{robot_name}_bridge_config_template.yaml')

    return [GroupAction([
        PushRosNamespace(robot_name),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{robot_name}_bridge',
            parameters=[{'config_file': config_path}],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f"static_tf_{robot_name}",
            arguments=[
                x, y, '0', yaw, '0', '0', 'map',
                f"{robot_name}_/odom"
            ],
            output='screen'
        ),
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot0', description='Robot namespace'),
        DeclareLaunchArgument('x', default_value='0.0', description='X position'),
        DeclareLaunchArgument('y', default_value='0.0', description='Y position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw in radians'),

        # Print resolved config
        OpaqueFunction(function=print_robot_info),

        # Spawn after delay
        TimerAction(period=5.0, actions=[
            OpaqueFunction(function=create_robot_spawn)
        ]),

        # Bridge after delay
        TimerAction(period=6.5, actions=[
            OpaqueFunction(function=create_robot_bridge)
        ])
    ])
