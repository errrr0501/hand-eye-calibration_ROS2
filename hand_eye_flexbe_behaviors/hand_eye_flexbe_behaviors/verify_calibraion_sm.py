#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hand_eye_flexbe_states.get_ar_marker import GetArMarkerState
from hand_eye_flexbe_states.initial_pose import InitialPoseState
from hand_eye_flexbe_states.moveit_plan_excute import MoveitPlanExecuteState
from hand_eye_flexbe_states.obj_trans_to_arm import ObjTransToArmState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Nov 17 2022
@author: Luis
'''
class verify_calibraionSM(Behavior):
	'''
	verify_calibraion
	'''


	def __init__(self, node):
		super(verify_calibraionSM, self).__init__()
		self.name = 'verify_calibraion'
		self.node = node

		# parameters of this behavior
		GetArMarkerState.initialize_ros(node)
		InitialPoseState.initialize_ros(node)
		MoveitPlanExecuteState.initialize_ros(node)
		ObjTransToArmState.initialize_ros(node)
		OperatableStateMachine.initialize_ros(node)
		Logger.initialize(node)
		self.add_parameter('eye_in_hand_mode', False)
		self.add_parameter('base_link', '/base_link')
		self.add_parameter('tip_link', '/tool0_controller')
		self.add_parameter('group_name', 'manipulator')
		self.add_parameter('reference_frame', 'base_link')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:48 y:501, x:58 y:229
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:250 y:76
			OperatableStateMachine.add('get_obj_position',
										GetArMarkerState(eye_in_hand_mode=self.eye_in_hand_mode),
										transitions={'done': 'obj_to_arm_base', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'camera_h_charuco': 'camera_h_charuco'})

			# x:551 y:180
			OperatableStateMachine.add('excute_moveit_plan',
										MoveitPlanExecuteState(group_name=self.group_name, reference_frame=self.reference_frame),
										transitions={'received': 'excute_moveit_plan', 'done': 'back_home', 'collision': 'failed'},
										autonomy={'received': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off},
										remapping={'excute_position': 'excute_position', 'result_compute': 'result_compute'})

			# x:564 y:83
			OperatableStateMachine.add('obj_to_arm_base',
										ObjTransToArmState(eye_in_hand_mode=self.eye_in_hand_mode, base_link=self.base_link, tip_link=self.tip_link),
										transitions={'done': 'excute_moveit_plan', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'camera_h_charuco': 'camera_h_charuco', 'excute_position': 'excute_position'})

			# x:294 y:315
			OperatableStateMachine.add('back_home',
										InitialPoseState(group_name=self.group_name, reference_frame=self.reference_frame),
										transitions={'done': 'finished', 'collision': 'failed'},
										autonomy={'done': Autonomy.Off, 'collision': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
