from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'self_balancing_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shivam Thareja',
    maintainer_email='tharejashivam07@gmail.com',
    description='Self Balancing Two-Wheeled Robot using LQR Control - ROS 2 Jazzy + Gazebo Harmonic',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lqr_controller = self_balancing_robot.lqr_controller:main',
            'teleop_key = self_balancing_robot.teleop_key:main',
        ],
    },
)
