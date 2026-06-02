from setuptools import find_packages, setup

package_name = 'robot_inspection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/inspection.launch.py']),
        ('share/' + package_name + '/config',
            ['config/detectors.yaml',
             'config/inspection_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brandon407',
    maintainer_email='brandon407@github.com',
    description='Visual inspection for industrial robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspection_orchestrator = robot_inspection.orchestrator:main',
            'defect_detector = robot_inspection.detectors.defect_detector:main',
            'meter_reader = robot_inspection.detectors.meter_reader:main',
            'safety_checker = robot_inspection.detectors.safety_checker:main',
        ],
    },
)
