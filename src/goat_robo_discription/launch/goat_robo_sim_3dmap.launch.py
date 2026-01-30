from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get absolute path to your YAML file in the package

    config_path = os.path.join(
    get_package_share_directory('goat_robo_discription'),
    'config',  # your copied folder
    'glim_config.json'
)

    return LaunchDescription([
        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name='glim_rosnode',
            parameters=[config_path],
            output='screen'
        )
    ])
