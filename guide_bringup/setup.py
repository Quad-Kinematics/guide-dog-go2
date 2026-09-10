import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'guide_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oxy',
    maintainer_email='claudecodeshared67@gmail.com',
    description=(
        'Single launch file that starts the guide dog packages together, '
        'with each subsystem switchable on or off.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
