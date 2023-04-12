#!/usr/bin/env python

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

# from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes
# from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, JointTrajectoryControllerState, FollowJointTrajectoryResult

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class InitialPoseState(EventState):
	'''
	Move robot by planned trajectory.

	-- group_name         string      move group name

	># joint_trajectory             JointTrajectory  planned trajectory

	<= done 						Robot move done.
	<= failed 						Robot move failed.
	<= collision 				    Robot during collision.
	'''


	def __init__(self, group_name,reference_frame):
		'''
		Constructor
		'''
		super(InitialPoseState, self).__init__(outcomes=['done', 'collision'])
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

	def stop(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''
		joint_goal= self._move_group.get_current_joint_values()
		joint_goal[0] =  pi * 0.5
		joint_goal[1] = -pi * 0.5
		joint_goal[2] =  pi * 0.5
		joint_goal[3] = -pi * 0.5
		joint_goal[4] = -pi * 0.5
		joint_goal[5] = -pi * 0.5   
		input() 
		self._result = self._move_group.go(joint_goal, wait=True)
		self._move_group.stop()
		self._move_group.clear_pose_targets()


		if self._result == MoveItErrorCodes.SUCCESS:
			return 'done'
			
		elif self._result == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
			return 'collision'


	def on_enter(self, userdata):
		pass

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)
