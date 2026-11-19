from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rtk_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farhangnaderi',
    maintainer_email='farhang.nba@gmail.com',
    description='RTK base station driver: configures the receiver, forwards RTCM '
                'corrections (UDP, TCP, serial) and publishes NavSatFix and raw NMEA',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtk_base_node = rtk_base.base_node:main',
            'rtk_base_relay = rtk_base.tools.relay:main',
        ],
    },
)
