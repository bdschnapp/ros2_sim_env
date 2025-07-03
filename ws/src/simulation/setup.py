import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files from the launch directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Include all rviz files from the rviz directory
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        # Include all map files
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ben.schnapp@cplaneai.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_simulator = simulation.vehicle_simulator:main',
            'pcd_map_publisher = simulation.pcd_map_publisher:main',
            'lidar_simulator = simulation.lidar_simulator:main',
        ],
    },
)
