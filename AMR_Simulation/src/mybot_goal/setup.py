import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mybot_goal'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nam',
    maintainer_email='nam@todo.todo',
    description='Waypoint and Goal Management package for AMR Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'create_goal = mybot_goal.create_goal:main',
            'update_goal = mybot_goal.update_goal:main',
            'delete_goal = mybot_goal.delete_goal:main',
            'get_goals = mybot_goal.get_goals:main',
            'go_goal = mybot_goal.go_goal:main',
            'goal_manager = mybot_goal.goal_manager:main',
        ],
    },
)
