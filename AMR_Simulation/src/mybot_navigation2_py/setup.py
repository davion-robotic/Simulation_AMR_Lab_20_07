from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mybot_navigation2_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            [f for f in glob('launch/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'param'),
            [f for f in glob('param/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'fleet_adapter'),
            [f for f in glob('fleet_adapter/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='warlord',
    maintainer_email='warlord@todo.todo',
    description='Robot UART bridge and fleet adapter',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ControlRobotOdom = mybot_navigation2_py.ControlRobotOdom:main',
            'ScanFilter = mybot_navigation2_py.ScanFilter:main',
            'pcFilter = mybot_navigation2_py.pc_to_Scan:main',
            'wait_for_odom = mybot_navigation2_py.wait_for_odom:main',
            'nav2_starter = mybot_navigation2_py.nav2_starter:main',
            'uartipad = mybot_navigation2_py.uartipad:main',
            'PointGo = mybot_navigation2_py.PointGo:main',
            'Status = mybot_navigation2_py.Status:main',
            'PointSaver = mybot_navigation2_py.PointSaver:main',
            'stop_timer_relay = mybot_navigation2_py.stop_timer_relay:main',
            'fleet_adapter = mybot_navigation2_py.fleet_adapter.fleet_adapter:main',
            'fleet_manager = mybot_navigation2_py.fleet_adapter.fleet_manager:main',
        ],
    },
)