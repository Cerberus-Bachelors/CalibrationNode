from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motorcalibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'odrive'],
    zip_safe=True,
    maintainer='maggebag',
    maintainer_email='magnus.mortensen@hotmail.no',
    description='Python ROS 2 package for calibrating 12 motors using ODrive boards',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate_motors = motorcalibration.motor_calibrator:main',
        ],
    },
)
