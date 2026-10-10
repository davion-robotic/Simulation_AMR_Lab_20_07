import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mybot_localization'

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
    description='Localization lifecycle and pose management package for AMR Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start_localization = mybot_localization.start_localization:main',
            'stop_localization = mybot_localization.stop_localization:main',
            'initial_pose = mybot_localization.initial_pose:main',
            'reset_pose = mybot_localization.reset_pose:main',
        ],
    },
)
