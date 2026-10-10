import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mybot_task_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nam',
    maintainer_email='nam@todo.todo',
    description='Package handling task control commands (PAUSE, RESUME, CANCEL) from UI/Tablet via Rosbridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_controller_node = mybot_task_controller.task_controller_node:main',
        ],
    },
)
