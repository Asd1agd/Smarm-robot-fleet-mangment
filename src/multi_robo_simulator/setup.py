from setuptools import find_packages, setup

package_name = 'multi_robo_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asd',
    maintainer_email='a93054223@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_spawner = multi_robo_simulator.robot_spawner:main',
            'server_gui = multi_robo_simulator.server_gui:main',
            'esp_client_fake_1 = multi_robo_simulator.esp_client_fake_1:main',
            'esp_client_fake_2 = multi_robo_simulator.esp_client_fake_2:main',
            'esp_client_fake_3 = multi_robo_simulator.esp_client_fake_3:main',
            'esp_client_fake_4 = multi_robo_simulator.esp_client_fake_4:main',
            'esp_client_fake_5 = multi_robo_simulator.esp_client_fake_5:main',
            'esp_client_fake_6 = multi_robo_simulator.esp_client_fake_6:main',
        ],
    },
)
