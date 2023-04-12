#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hand_eye_flexbe_states.charuco_camera_calibration import CharucoCameraCalibrationState
from hand_eye_flexbe_states.take_picture import TakePictureState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Nov 15 2022
@author: Luis
'''
class camera_calibrationSM(Behavior):
	'''
	camera_calibration
	'''


	def __init__(self, node):
		super(camera_calibrationSM, self).__init__()
		self.name = 'camera_calibration'
		self.node = node

		# parameters of this behavior
		CharucoCameraCalibrationState.initialize_ros(node)
		TakePictureState.initialize_ros(node)
		OperatableStateMachine.initialize_ros(node)
		Logger.initialize(node)
		self.add_parameter('pic_num', 30)
		self.add_parameter('square_size', 0.0200)
		self.add_parameter('marker_size', 0.0150)
		self.add_parameter('col_count', 10)
		self.add_parameter('row_count', 14)
		self.add_parameter('camera_type', 'realsense')
		self.add_parameter('save_file_name', 'camera_calibration.ini')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:570 y:120, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:149 y:88
			OperatableStateMachine.add('take_camera_cali_pic',
										TakePictureState(pic_num=self.pic_num, camera_type=self.camera_type),
										transitions={'done': 'camera_calibration', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:348 y:87
			OperatableStateMachine.add('camera_calibration',
										CharucoCameraCalibrationState(square_size=self.square_size, marker_size=self.marker_size, col_count=self.col_count, row_count=self.row_count, save_file_name=self.save_file_name),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
