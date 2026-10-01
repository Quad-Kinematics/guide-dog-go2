import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'unitree_go2_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install the launch file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Install the custom world
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),

        # Install Rviz configs
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        # Install the arena map, the Nav2 parameters and the behavior tree they point to
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config', 'nav2'),
         glob('config/nav2/*.yaml') + glob('config/nav2/*.xml')),

        # Install the CHAMP quadruped configuration files
        (os.path.join('share', package_name, 'config', 'gait'),
         glob('config/gait/*.yaml')),
        (os.path.join('share', package_name, 'config',
         'joints'), glob('config/joints/*.yaml')),
        (os.path.join('share', package_name, 'config',
         'links'), glob('config/links/*.yaml')),
        (os.path.join('share', package_name, 'config',
         'ros_control'), glob('config/ros_control/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Janith Chamikara',
    maintainer_email='janithchamikara13@gmail.com',
    description='Custom SLAM mapping and simulation package for Unitree Go2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'body_tilt_tf = unitree_go2_slam.body_tilt_tf:main',
            'pose_logger = unitree_go2_slam.pose_logger:main',
            'plot_pose_log = unitree_go2_slam.plot_pose_log:main',
        ],
    },
)
