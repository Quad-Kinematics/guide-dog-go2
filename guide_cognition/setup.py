import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'guide_cognition'

# Install every config file EXCEPT the secrets one. That file holds a real
# API key, so it must not be copied into the build output either.
config_files = []
for config_path in glob('config/*.yaml'):
    if not config_path.endswith('_secrets.yaml'):
        config_files.append(config_path)

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), config_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oxy',
    maintainer_email='claudecodeshared67@gmail.com',
    description=(
        'Speech decisions for the Unitree Go2 guide robot: turns visit events '
        'into a short spoken greeting and publishes it on /speech_out.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'greeting_node = guide_cognition.greeting_node:main',
        ],
    },
)
