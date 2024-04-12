from setuptools import setup
from glob import glob
import os

package_name = 'riptide_hardware2'

setup(
    name=package_name,
    version='0.0.17',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch*')),
        (os.path.join('share', package_name, 'cfg'), glob('cfg/*')),
        (os.path.join('share', package_name, 'weights'), glob('weights/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Robert Pafford',
    maintainer='OSU UWRT',
    maintainer_email='osu.uwrt@gmail.com',
    description='Hardware abstraction layer for OSU\'s Riptide AUV.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'electrical_monitor = riptide_hardware2.electrical_monitor:ElectricalMonitor.main',
            'firmware_monitor = riptide_hardware2.firmware_monitor:FirmwareMonitor.main',
            'voltage_monitor = riptide_hardware2.voltage_monitor:VoltageMonitor.main',
            'sensor_monitor = riptide_hardware2.sensor_monitor:main',
            'computer_monitor = riptide_hardware2.computer_monitor:main',
            'depth_converter = riptide_hardware2.depth_converter:main',
            'fake_ekf = riptide_hardware2.fake_ekf:main',
            'fake_dvl = riptide_hardware2.fake_dvl:main',
            'pose_converter = riptide_hardware2.pose_converter:main',
            'rpm_echo = riptide_hardware2.rpm_echo:main',
            'tag_odom = riptide_hardware2.tag_odom:main',
            'alternate_thruster = riptide_hardware2.alternateThruster:main ',
            'imu_power_cycle = riptide_hardware2.imu_power_cycle:main',
            'simple_actuator_interface = riptide_hardware2.simple_actuator_interface:main'
        ],
    },
)
