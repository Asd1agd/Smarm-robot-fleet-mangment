from setuptools import find_packages, setup

package_name = 'Qr_action'

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
            'Qr_server = Qr_action.Qr_server:main',
            'Qr_client = Qr_action.Qr_client:main',
        ],
    },
)
