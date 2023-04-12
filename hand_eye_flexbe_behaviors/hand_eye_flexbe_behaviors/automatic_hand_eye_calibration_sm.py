#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hand_eye_flexbe_states.compute_calib import ComputeCalibState
from hand_eye_flexbe_states.find_charuco import FindCharucoState
from hand_eye_flexbe_states.generate_hand_eye_point import GenerateHandEyePointState
from hand_eye_flexbe_states.initial_pose import InitialPoseState
from hand_eye_flexbe_states.move_robot_manually import MoveRobotManuallyState
from hand_eye_flexbe_states.moveit_hand_eye_excute import MoveitHandEyeExecuteState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Nov 14 2021
@author: Luis Tsai
'''
class AutomaticHandEyeCalibrationSM(Behavior):
	'''
	Execute hand eye calibration by manual
	'''


	def __init__(self, node):
		super(AutomaticHandEyeCalibrationSM, self).__init__()
		self.name = 'Automatic Hand Eye Calibration'
		self.node = node

		# parameters of this behavior
		ComputeCalibState.initialize_ros(node)
		FindCharucoState.initialize_ros(node)
		GenerateHandEyePointState.initialize_ros(node)
		InitialPoseState.initialize_ros(node)
		MoveRobotManuallyState.initialize_ros(node)
		MoveitHandEyeExecuteState.initialize_ros(node)
		OperatableStateMachine.initialize_ros(node)
		Logger.initialize(node)
		self.add_parameter('eye_in_hand_mode', False)
		self.add_parameter('customize_file', False)
		self.add_parameter('calib_pose_num', 0)
		self.add_parameter('base_link', '/base_link')
		self.add_parameter('tip_link', '/tool0_controller')
		self.add_parameter('calibration_file_name', 'hand_eye_calibration.ini')
		self.add_parameter('move_distance', 0.10)
		self.add_parameter('reference_frame', 'base_link')
		self.add_parameter('group_name', 'manipulator')
		self.add_parameter('axis', 'x-y-z')
		self.add_parameter('points_num', 24)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:275 y:503, x:182 y:417
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:133 y:27
			OperatableStateMachine.add('Move_Robot_Manually',
										MoveRobotManuallyState(wait_time=2, pose_num=self.calib_pose_num),
										transitions={'done': 'Find_First_Charuco'},
										autonomy={'done': Autonomy.Off},
										remapping={'result_compute': 'result_compute'})

			# x:564 y:364
			OperatableStateMachine.add('Calibration_Computation',
										ComputeCalibState(eye_in_hand_mode=self.eye_in_hand_mode, calibration_file_name=self.calibration_file_name, customize_file=self.customize_file),
										transitions={'finish': 'Back_to_Initial_Pose'},
										autonomy={'finish': Autonomy.Off},
										remapping={'base_h_tool': 'base_h_tool', 'camera_h_charuco': 'camera_h_charuco'})

			# x:734 y:248
			OperatableStateMachine.add('Find_Charuco',
										FindCharucoState(base_link=self.base_link, tip_link=self.tip_link, eye_in_hand_mode=self.eye_in_hand_mode),
										transitions={'done': 'Moveit_Execute_Points', 'go_compute': 'Calibration_Computation'},
										autonomy={'done': Autonomy.Off, 'go_compute': Autonomy.Off},
										remapping={'result_compute': 'result_compute', 'base_h_tool': 'base_h_tool', 'camera_h_charuco': 'camera_h_charuco'})

			# x:144 y:121
			OperatableStateMachine.add('Find_First_Charuco',
										FindCharucoState(base_link=self.base_link, tip_link=self.tip_link, eye_in_hand_mode=self.eye_in_hand_mode),
										transitions={'done': 'Find_First_Charuco', 'go_compute': 'Generate_Points'},
										autonomy={'done': Autonomy.Off, 'go_compute': Autonomy.Off},
										remapping={'result_compute': 'result_compute', 'base_h_tool': 'base_h_tool', 'camera_h_charuco': 'camera_h_charuco'})

			# x:83 y:243
			OperatableStateMachine.add('Generate_Points',
										GenerateHandEyePointState(eye_in_hand_mode=self.eye_in_hand_mode, base_link=self.base_link, tip_link=self.tip_link, move_distance=self.move_distance, group_name=self.group_name, reference_frame=self.reference_frame, points_num=self.points_num, axis=self.axis),
										transitions={'done': 'Moveit_Execute_Points', 'failed': 'failed', 'correct_pose': 'Moveit_Execute_Points', 'recheck': 'recheck_Charuco'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'correct_pose': Autonomy.Off, 'recheck': Autonomy.Off},
										remapping={'camera_h_charuco': 'camera_h_charuco', 'hand_eye_points': 'hand_eye_points', 'motion_state': 'motion_state', 'result_compute': 'result_compute'})

			# x:468 y:112
			OperatableStateMachine.add('Moveit_Execute_Points',
										MoveitHandEyeExecuteState(group_name=self.group_name, reference_frame=self.reference_frame, points_num=self.points_num),
										transitions={'received': 'Find_Charuco', 'done': 'Find_Charuco', 'finish_correction': 'Generate_Points', 'collision': 'failed'},
										autonomy={'received': Autonomy.Off, 'done': Autonomy.Off, 'finish_correction': Autonomy.Off, 'collision': Autonomy.Off},
										remapping={'hand_eye_points': 'hand_eye_points', 'motion_state': 'motion_state', 'result_compute': 'result_compute'})

			# x:29 y:458
			OperatableStateMachine.add('recheck_Charuco',
										FindCharucoState(base_link=self.base_link, tip_link=self.tip_link, eye_in_hand_mode=self.eye_in_hand_mode),
										transitions={'done': 'recheck_Charuco', 'go_compute': 'Generate_Points'},
										autonomy={'done': Autonomy.Off, 'go_compute': Autonomy.Off},
										remapping={'result_compute': 'result_compute', 'base_h_tool': 'base_h_tool', 'camera_h_charuco': 'camera_h_charuco'})

			# x:356 y:325
			OperatableStateMachine.add('Back_to_Initial_Pose',
										InitialPoseState(group_name=self.group_name, reference_frame=self.reference_frame),
										transitions={'done': 'finished', 'collision': 'failed'},
										autonomy={'done': Autonomy.Off, 'collision': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
