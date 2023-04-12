#!/usr/bin/env python3

import numpy as np
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from math import pi, radians
from std_msgs.msg import String
import geometry_msgs.msg
from tf import transformations as tf

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

# from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes
# from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, JointTrajectoryControllerState, FollowJointTrajectoryResult

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class MoveitPlanExecuteState(EventState):
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
		super(MoveitPlanExecuteState, self).__init__(outcomes=['done', 'collision'],
											input_keys=['excute_position'],
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
		self._first_pose = self._move_group.get_current_pose()
		self.points_num  = 0
		self._execute_times = 0
		self.plan, self.fraction =0,0

	def on_start(self):
		# back to first pose
		# input()
		# self._result = self._move_group.go(self._first_joints, wait=True)
		# self._move_group.stop()
		# self._move_group.clear_pose_targets()
		# self._execute_times += 1
		# self.points_num = np.size(userdata.hand_eye_points)
		# print(np.size(userdata.hand_eye_points))
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''

		new_pose = self._move_group.get_current_pose()
		
		pose_goal = geometry_msgs.msg.Pose()

		if np.size(userdata.excute_position["x"]) == 0:
			pose_goal.position.x = self._first_pose.pose.position.x
			pose_goal.position.y = self._first_pose.pose.position.y
			pose_goal.position.z = self._first_pose.pose.position.z

		else:
			pose_goal.position.x = userdata.excute_position["x"][self._execute_times]
			pose_goal.position.y = userdata.excute_position["y"][self._execute_times]
			pose_goal.position.z = userdata.excute_position["z"][self._execute_times]


		pose_goal.orientation.x = userdata.excute_position["qx"][self._execute_times]
		pose_goal.orientation.y = userdata.excute_position["qy"][self._execute_times]
		pose_goal.orientation.z = userdata.excute_position["qz"][self._execute_times]
		pose_goal.orientation.w = userdata.excute_position["qw"][self._execute_times]





		print(pose_goal)
		input()
		self._move_group.set_pose_target(pose_goal, self._end_effector_link)

		self._result = self._move_group.go(wait=True)
		self._move_group.stop()
		self._move_group.clear_pose_targets()

		userdata.result_compute = self._execute_times >= self.points_num



		if self._result == MoveItErrorCodes.SUCCESS:
			self._execute_times += 1
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
		pass
