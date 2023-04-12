#!/usr/bin/env python
import os
from setuptools import setup

package_name = 'hand_eye_flexbe_behaviors'

setup(
    name=package_name,
    version='1.3.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis',
    maintainer_email='errrr0501done@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'automatic_hand_eye_calibration_sm = hand_eye_flexbe_behaviors.automatic_hand_eye_calibration_sm',
            'camera_calibration_sm = hand_eye_flexbe_behaviors.camera_calibration_sm',
            'hand_eye_calibration_sm = hand_eye_flexbe_behaviors.hand_eye_calibration_sm',
            'manual_hand_eye_calibration_sm = hand_eye_flexbe_behaviors.manual_hand_eye_calibration_sm',
            'verify_calibraion_sm = hand_eye_flexbe_behaviors.verify_calibraion_sm',
        ],
    },
)