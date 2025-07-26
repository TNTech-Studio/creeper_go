from setuptools import setup
import os
from glob import glob

package_name = 'creeper_go'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 如果有launch文件，取消下面的注释
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 如果有配置文件，取消下面的注释
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for creeper robot tracking and following using YOLO detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_node = creeper_go.tracker_node:main',
            'creeper_sound_node = creeper_go.creeper_sound_node:main',
        ],
    },
)