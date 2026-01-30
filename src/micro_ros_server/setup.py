from setuptools import setup

package_name = 'micro_ros_server'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',  'rclpy', 'geometry_msgs', 'nav_msgs','tf2_ros', 'std_msgs', 'micro_ros_custom_interfaces'],
    zip_safe=True,
    maintainer='asd',
    maintainer_email='a93054223@gmail.com',
    description='Python ROS 2 micro server',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Odom_generator = micro_ros_server.Odom_generator:main',
            'qr_array_server = micro_ros_server.qr_array_server:main',
            'qr_array_proceser = micro_ros_server.qr_array_proceser:main',
            'odom_to_pose_publisher = micro_ros_server.odom_to_pose_publisher:main',
            'cmd_vel_republisher = micro_ros_server.cmd_vel_republisher:main',
            'virtual_qr_publisher = micro_ros_server.virtual_qr_publisher:main'
        ],
    },
)

