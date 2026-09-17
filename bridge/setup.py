import os
from glob import glob
from setuptools import setup

package_name = 'mower_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='mower',
    maintainer_email='todo@example.com',
    description='割草机底盘串口桥：收发全包，mower-host 独占这个串口',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'chassis_bridge = mower_bridge.node:main',
        ],
    },
)
