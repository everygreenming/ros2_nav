import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'waypoint'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/config", ['config/waypoints.yaml']),
        ('share/' + package_name+"/launch", ['launch/pose_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='everygreen',
    maintainer_email='2245281871@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "init_bot_pose=waypoint.init_bot_pose:main", 
            "get_pose=waypoint.get_pose:main",
            "go_to_pose=waypoint.go_to_pose:main",
            "mutil_pose=waypoint.mutil_pose:main",
            

                

        ],
    },
)
