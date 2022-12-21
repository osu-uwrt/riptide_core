from setuptools import setup
import os
from glob import glob

package_name = 'riptide_localization2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mitch',
    maintainer_email='mitch@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_converter = riptide_localization2.depth_converter:main',
            'dvl_converter = riptide_localization2.dvl_converter:main',
            'fake_ekf = riptide_localization2.fake_ekf:main'
        ],
    },
)
