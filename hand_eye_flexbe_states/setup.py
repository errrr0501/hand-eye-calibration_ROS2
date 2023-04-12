#!/usr/bin/env python
from setuptools import setup
from setuptools import find_packages

package_name = 'hand_eye_flexbe_states'

setup(
    name=package_name,
    version='1.3.1',
    packages=find_packages(),
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
            'automatic_move_robot = hand_eye_flexbe_states.automatic_move_robot',
            'charuco_camera_calibration = hand_eye_flexbe_states.charuco_camera_calibration',
            'compute_calib = hand_eye_flexbe_states.compute_calib',
            'execute_traj = hand_eye_flexbe_states.execute_traj',
            'find_charuco = hand_eye_flexbe_states.find_charuco',
            'generate_hand_eye_point = hand_eye_flexbe_states.generate_hand_eye_point',
            'get_ar_marker = hand_eye_flexbe_states.get_ar_marker',
            'get_calib_pose = hand_eye_flexbe_states.get_calib_pose',
            'initial_pose = hand_eye_flexbe_states.initial_pose',
            'joints_plan = hand_eye_flexbe_states.joints_plan',
            'move_charuco_center = hand_eye_flexbe_states.move_charuco_center',
            'moveit_hand_eye_excute = hand_eye_flexbe_states.moveit_hand_eye_excute',
            'moveit_plan_excute = hand_eye_flexbe_states.moveit_plan_excute',
            'move_robot_manually = hand_eye_flexbe_states.move_robot_manually',
            'obj_trans_to_arm = hand_eye_flexbe_states.obj_trans_to_arm',
            'pose_plan = hand_eye_flexbe_states.pose_plan',
            'take_picture = hand_eye_flexbe_states.take_picture',
        ],
    },
)
