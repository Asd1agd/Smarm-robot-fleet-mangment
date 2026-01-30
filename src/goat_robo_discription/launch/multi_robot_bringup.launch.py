from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to goat_robo_discription's launch file
    goat_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('goat_robo_discription'),
                'launch',
                'gazibo_goat.launch.py'  # Ensure correct name/spelling
            )
        )
    )

    # Run server_gui node
    server_gui_node = ExecuteProcess(
        cmd=['ros2', 'run', 'multi_robo_simulator', 'server_gui'],
        output='screen'
    )

    # Run robot_spawner node
    robot_spawner_node = ExecuteProcess(
        cmd=['ros2', 'run', 'multi_robo_simulator', 'robot_spawner'],
        output='screen'
    )

    return LaunchDescription([
        goat_description_launch,
        server_gui_node,
        robot_spawner_node
    ])

