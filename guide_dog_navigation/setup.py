from glob import glob

from setuptools import find_packages, setup

package_name = 'guide_dog_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@todo.todo',
    description='Go2 localization, Nav2 and SLAM: launch files, params, maps and helper nodes '
                '(moved from ~/SLAM go2_mapping)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_to_tf = guide_dog_navigation.odom_to_tf:main',
            'lowstate_to_joint_states = guide_dog_navigation.lowstate_to_joint_states:main',
            'goal_pose_relay = guide_dog_navigation.goal_pose_relay:main',
            'rviz_click_logger = guide_dog_navigation.rviz_click_logger:main',
            'save_map = guide_dog_navigation.save_map:main',
        ],
    },
)
