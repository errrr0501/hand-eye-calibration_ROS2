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
from hand_eye_flexbe_states.move_robot_manually import MoveRobotManuallyState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Nov 14 2021
@author: Andy Chien
'''
class ManualHandEyeCalibrationSM(Behavior):
	'''
	Execute hand eye calibration by manual
	'''


	def __init__(self, node):
		super(ManualHandEyeCalibrationSM, self).__init__()
		self.name = 'Manual Hand Eye Calibration'

		# parameters of this behavior
		self.add_parameter('eye_in_hand', False)
		self.add_parameter('calib_pose_num', 10)
		self.add_parameter('base_link', 'base_link')
		self.add_parameter('tip_link', 'tool0')
		self.add_parameter('calibration_file_name', 'hand_eye_calibration.ini')
		self.add_parameter('customize_file', False)

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)
		ComputeCalibState.initialize_ros(node)
		FindCharucoState.initialize_ros(node)
		MoveRobotManuallyState.initialize_ros(node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:308 y:369, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:71 y:102
			OperatableStateMachine.add('Move_Robot_Manually',
										MoveRobotManuallyState(wait_time=2, pose_num=self.calib_pose_num),
										transitions={'done': 'Find_Charuco'},
										autonomy={'done': Autonomy.Low},
										remapping={'result_compute': 'result_compute'})

			# x:301 y:92
			OperatableStateMachine.add('Find_Charuco',
										FindCharucoState(base_link=self.base_link, tip_link=self.tip_link, eye_in_hand_mode=self.eye_in_hand),
										transitions={'done': 'Move_Robot_Manually', 'go_compute': 'Calibration_Computation'},
										autonomy={'done': Autonomy.Off, 'go_compute': Autonomy.Off},
										remapping={'result_compute': 'result_compute', 'base_h_tool': 'base_h_tool', 'camera_h_charuco': 'camera_h_charuco'})

			# x:302 y:228
			OperatableStateMachine.add('Calibration_Computation',
										ComputeCalibState(eye_in_hand_mode=self.eye_in_hand, calibration_file_name=self.calibration_file_name, customize_file=self.customize_file),
										transitions={'finish': 'finished'},
										autonomy={'finish': Autonomy.Off},
										remapping={'base_h_tool': 'base_h_tool', 'camera_h_charuco': 'camera_h_charuco'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
