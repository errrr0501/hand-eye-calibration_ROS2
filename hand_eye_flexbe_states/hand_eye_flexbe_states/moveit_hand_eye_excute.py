#!/usr/bin/env python3

import numpy as np
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from math import pi, radians
from std_msgs.msg import String
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from tf import transformations as tf

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from tf.transformations import quaternion_from_euler, euler_from_quaternion


# from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes
# from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, JointTrajectoryControllerState, FollowJointTrajectoryResult

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class MoveitHandEyeExecuteState(EventState):
	'''
	Move robot by planned trajectory.

	-- group_name         string      move group name

	># joint_trajectory             JointTrajectory  planned trajectory

	<= done 						Robot move done.
	<= failed 						Robot move failed.
	<= collision 				    Robot during collision.
	'''


	def __init__(self, group_name,reference_frame, points_num):
		'''
		Constructor
		'''
		super(MoveitHandEyeExecuteState, self).__init__(outcomes=['received','done', 'finish_correction', 'collision'],
											input_keys=['hand_eye_points','motion_state'],
											output_keys=['result_compute'])
		# group_name = ""
		self._group_name = group_name
		self._reference_frame = reference_frame
		self._move_group = moveit_commander.MoveGroupCommander(self._group_name)
		self._result = MoveItErrorCodes.FAILURE
		self._move_group.set_pose_reference_frame(self._reference_frame)
		self._end_effector_link = self._move_group.get_end_effector_link()
		self._move_group.set_end_effector_link(self._end_effector_link)
		self._move_group.set_max_acceleration_scaling_factor(0.1)
		self._move_group.set_max_velocity_scaling_factor(0.1)
		self._first_joints= self._move_group.get_current_joint_values()
		self.points_num  = points_num
		self._execute_times = 0
		self.plan, self.fraction =0,0
		self.centralize = False
		self.correct_rotation = False

	def on_start(self):

		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''

		print(np.size(userdata.hand_eye_points))
		print(self._execute_times)
		self.plan = userdata.hand_eye_points[self._execute_times]
		# input()
		# excute planing path
		self._result = self._move_group.execute(self.plan, wait=True)

		userdata.result_compute = self._execute_times >= self.points_num -1 


		if userdata.result_compute:
			return 'done'

		if self._result == MoveItErrorCodes.SUCCESS:
			self._execute_times += 1
			if not self.centralize or not self.correct_rotation:
				self._first_joints= self._move_group.get_current_joint_values()
				self._execute_times = 0
				if userdata.motion_state == 'centralize':
					self.centralize = True
					return 'finish_correction'
				elif userdata.motion_state == 'correct_rotation':
					self.correct_rotation = True
					return 'finish_correction'
			return 'received'
			
		elif self._result == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
			return 'collision'


	def on_enter(self, userdata):
		# back to center pose
		self._result = self._move_group.go(self._first_joints, wait=True)
		self._move_group.stop()
		self._move_group.clear_pose_targets()
		pass

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		# self.on_enter(userdata)
		pass
