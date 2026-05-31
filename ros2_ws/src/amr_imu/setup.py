from setuptools import setup
import os
from glob import glob

package_name = 'amr_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'imu_sensor_node = amr_imu.imu_sensor_node:main',
            'twist_to_reference = amr_imu.twist_to_reference:main',
            'cmd_vel_safe_relay = amr_imu.cmd_vel_safe_relay:main',
        ],
    },
)
