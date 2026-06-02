from setuptools import find_packages, setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/slam.launch.py',
             'launch/navigation.launch.py',
             'launch/patrol.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brandon407',
    maintainer_email='brandon407@github.com',
    description='Navigation and patrol for industrial inspection robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_server = robot_navigation.patrol_server:main',
            'waypoint_patrol = robot_navigation.waypoint_patrol:main',
            'slam_manager = robot_navigation.slam_manager:main',
        ],
    },
)
