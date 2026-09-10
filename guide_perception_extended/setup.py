import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'guide_perception_extended'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the standalone launch file and its config so
        # `ros2 launch guide_perception_extended face_recognition.launch.py` can find them.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oxy',
    maintainer_email='claudecodeshared67@gmail.com',
    description=(
        'Face recognition for the Unitree Go2 guide robot pipeline: detects and '
        'identifies the person the camera is looking at and publishes the result '
        'on /recognized_person.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_recognition_node = guide_perception_extended.face_recognition_node:main', 'yolo_vision_node = guide_perception_extended.yolo_vision_node:main',
            'enroll_faces = guide_perception_extended.enroll_faces:main',
        ],
    },
)
