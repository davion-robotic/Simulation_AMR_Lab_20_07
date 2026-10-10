from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mybot_rmf'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'),   glob('maps/*')),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amr',
    maintainer_email='warlord999190502@gmail.com',
    description='Open-RMF fleet adapter for mybot',
    license='Apache-2.0',
)
