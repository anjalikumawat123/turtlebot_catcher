from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot_catcher'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ament to find the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='competitor',
    maintainer_email='competitor@example.com',
    description='TurtleBot 4 Pursuit-and-Evasion Catcher',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run turtlebot_catcher catcher_node
            'catcher_node = turtlebot_catcher.catcher_node:main',
        ],
    },
)
