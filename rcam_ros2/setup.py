from setuptools import setup
from glob import glob
import os
package_name = 'rcam_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'example_map'), glob('example_map/*.yaml')),
        (os.path.join('share', package_name, 'example_map'), glob('example_map/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rodion',
    maintainer_email='rodion_anisimov@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_odom = rcam_ros2.static_odom_pub:main',
            'chassis = rcam_ros2.chassis:main',
        ],
    },
)
