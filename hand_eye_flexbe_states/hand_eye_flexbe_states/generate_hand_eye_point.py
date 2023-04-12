#!/usr/bin/env python3

import copy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from visp_hand2eye_calibration.msg import TransformArray
from math import pi, radians
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from moveit_commander.conversions import pose_to_list
import tf
# from tf import transformations
from visp_hand2eye_calibration.msg import TransformArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from flexbe_core import EventState, Logger
import numpy as np
from flexbe_core.proxy import ProxyActionClient

# from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes
# from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, JointTrajectoryControllerState, FollowJointTrajectoryResult

# class correct_point(dict):

#     def __init__(self):
#         self['x']       = [] 
#         self['y']       = []
#         self['z']       = []
#         self['qw']      = []
#         self['qx']      = []
#         self['qy']      = []
#         self['qz']      = []

class GenerateHandEyePointState(EventState):
	"""
	Output a fixed pose to move.

	<= done									   points has been created.
	<= fail									   create points fail.

	"""


	def __init__(self, eye_in_hand_mode, base_link, tip_link, move_distance, group_name, reference_frame, points_num, axis):
		'''
		Constructor
		'''
		super(GenerateHandEyePointState, self).__init__(outcomes=['done', 'failed','correct_pose','recheck'],
											input_keys=['camera_h_charuco'],
											output_keys=['hand_eye_points','motion_state','result_compute'])
		self.move_distance = float(move_distance)
		self.eye_in_hand_mode = eye_in_hand_mode
		self.base_link = base_link
		self.tip_link = tip_link
		self.tf_listener = tf.TransformListener()
		# self.tool_h_base = TransformArray()
		self.waypoints = []
		self.plan_path = []
		self.execute_num = 0
		self.points_num = points_num
		self.loop_size = int(self.points_num/4)
		self.loop_num = 0
		self.First_charuco_array = []
		self._group_name = group_name
		self._reference_frame = reference_frame
		self._move_group = moveit_commander.MoveGroupCommander(self._group_name)
		self._result = MoveItErrorCodes.FAILURE
		self._move_group.set_pose_reference_frame(self._reference_frame)
		self._end_effector_link = self._move_group.get_end_effector_link()
		self._first_pose = self._move_group.get_current_pose()
		self._first_joints= self._move_group.get_current_joint_values()
		self.wpose = self._move_group.get_current_pose().pose
		self._origin_euler  = [0, 0, 0]
		self._base_end_axis = axis
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.correct_degree = []
		self.cen_delta_x = 0.000
		self.cen_delta_y = 0.000
		self.cen_delta_z = 0.000
		self.rotation_correct = False
		self.centralized = False
		self.recheck = False
		self.degree_direction = 1


	def centralize_pose(self):
		try:
			(tool_trans_base, tool_rot_base) = self.tf_listener.lookupTransform(self.tip_link, self.base_link, GenerateHandEyePointState._node.get_clock().now())
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			Logger.logwarn('lookupTransform for robot failed!, ' + self.tip_link + ', ' + self.base_link)
			return

		tool_base_cen_m = self.tf_listener.fromTranslationRotation((tool_trans_base[0]+self.cen_delta_x,
																tool_trans_base[1]+self.cen_delta_y,
																tool_trans_base[2]+self.cen_delta_z,)
																,(tool_rot_base[0],
																tool_rot_base[1],
																tool_rot_base[2],
																tool_rot_base[3]))

		base_move = np.linalg.inv(tool_base_cen_m)

		base_move_pos = np.array(base_move[0:3, 3]).reshape(-1)

		base_move_pos[0] = (self.wpose.position.x - base_move_pos[0])
		base_move_pos[1] = (self.wpose.position.y - base_move_pos[1])
		base_move_pos[2] = (self.wpose.position.z - base_move_pos[2])

		print("delta_pos",base_move_pos)
		input()
		
		return base_move_pos



	def generate_points(self, roll_detla, pitch_delta, yaw_delta, x_distance, y_distance, z_distance):

		print(x_distance)
		# input()

		try:
			(tool_trans_base, tool_rot_base) = self.tf_listener.lookupTransform(self.tip_link, self.base_link, self.get_clock().now())
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			Logger.logwarn('lookupTransform for robot failed!, ' + self.tip_link + ', ' + self.base_link)
			return

		# tool_rotation = list(euler_from_quaternion([tool_rot_base[0],tool_rot_base[1],tool_rot_base[2],tool_rot_base[3]]))

		# quaternion =  quaternion_from_euler(tool_rotation[0]+np.radians(roll_detla),
		# 									tool_rotation[1]+np.radians(pitch_delta), 
		# 									tool_rotation[2]+np.radians(yaw_delta))  	
		tool_base_m = self.tf_listener.fromTranslationRotation((tool_trans_base[0]+x_distance,
																tool_trans_base[1]+y_distance,
																tool_trans_base[2]+z_distance,)
																,(tool_rot_base[0],
																tool_rot_base[1],
																tool_rot_base[2],
																tool_rot_base[3]))

		base_move_delta = np.linalg.inv(tool_base_m)

		delta_pos = np.array(base_move_delta[0:3, 3]).reshape(-1)


		delta_pos[0] = (self.wpose.position.x - delta_pos[0])
		delta_pos[1] = (self.wpose.position.y - delta_pos[1])
		delta_pos[2] = (self.wpose.position.z - delta_pos[2])

		self.wpose.position.x += delta_pos[0]
		self.wpose.position.y += delta_pos[1]
		self.wpose.position.z += delta_pos[2]

		# print("delta_pos",delta_pos)
		
		# return delta_pos
		return




	def on_start(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''

		if self.move_distance <= 0.11:
			quaternion =  quaternion_from_euler(np.radians(self.roll),
												np.radians(self.pitch), 
												np.radians(self.yaw))  
			if not self.rotation_correct:
				self.wpose.orientation.x, self.wpose.orientation.y, self.wpose.orientation.z, self.wpose.orientation.w = (quaternion[0],
																										quaternion[1],
																										quaternion[2],
																										quaternion[3])
				self.waypoints.append(copy.deepcopy(self.wpose))
				(plan, fraction) = self._move_group.compute_cartesian_path(self.waypoints,   # waypoints to follow
            	   																0.01,        # eef_step
            	   																0.0)         # jump_threshold
				self.plan_path.append(plan)
				userdata.hand_eye_points = self.plan_path
				userdata.motion_state = 'correct_rotation'
				self.rotation_correct = True
				return 'correct_pose'

			self.waypoints.clear()
			self.plan_path.clear()
			##############need to decide endeffector axis#################################
			if self.eye_in_hand_mode:
				#####move axis from camera
				if not self.recheck:
					self.recheck = True
					userdata.result_compute = True
					return 'recheck'
				self.cen_delta_x = 0.1000-userdata.camera_h_charuco.transforms[0].translation.y
				self.cen_delta_y = -(0.1425-userdata.camera_h_charuco.transforms[0].translation.x)	
				self.cen_delta_z = -(0.55-userdata.camera_h_charuco.transforms[0].translation.z)
				self.degree_direction = 1
				print(self.cen_delta_x, self.cen_delta_y, self.cen_delta_z)
			else:
				#####move axis from board
				if not self.recheck:
					self.recheck = True
					userdata.result_compute = True
					return 'recheck'
				self.cen_delta_x = 0.1000-userdata.camera_h_charuco.transforms[0].translation.x
				self.cen_delta_y = -(0.1425-userdata.camera_h_charuco.transforms[0].translation.y)
				self.cen_delta_z = 0.55-userdata.camera_h_charuco.transforms[0].translation.z
				self.degree_direction = -1
				# print(self.cen_delta_x, self.cen_delta_y, self.cen_delta_z)
			if not self.centralized:
				self.wpose = self._move_group.get_current_pose().pose
				cen_pose = self.centralize_pose()
				self.wpose.position.x += cen_pose[0]
				self.wpose.position.y += cen_pose[1]
				self.wpose.position.z += cen_pose[2]
				correct_degree = euler_from_quaternion([self.wpose.orientation.x, self.wpose.orientation.y, self.wpose.orientation.z, self.wpose.orientation.w])
				self.correct_degree.append(correct_degree[0]/3.14*180.0)
				self.correct_degree.append(correct_degree[1]/3.14*180.0)
				self.correct_degree.append(correct_degree[2]/3.14*180.0)
				# print("self.wpose",self.wpose.position.x, self.wpose.position.y, self.wpose.position.z)
				self.waypoints.append(copy.deepcopy(self.wpose))
				(plan, fraction) = self._move_group.compute_cartesian_path(self.waypoints,   # waypoints to follow
            	   																0.01,        # eef_step
            	   																0.0)         # jump_threshold
				self.plan_path.append(plan)
				userdata.hand_eye_points = self.plan_path
				userdata.motion_state = 'centralize'
				self.centralized = True
				return 'correct_pose'
		
			self.waypoints.clear()
			self.plan_path.clear()	

			
			quaternion =  quaternion_from_euler(np.radians(self.correct_degree[0]+15*self.degree_direction),
												np.radians(self.correct_degree[1]), 
												np.radians(self.correct_degree[2]))  
			##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
			for self.loop_num in range(self.loop_size):
				self.wpose = self._move_group.get_current_pose().pose
				delta_distance = -self.move_distance+self.loop_num*(self.move_distance*2/self.loop_size)
				self.generate_points(15*self.degree_direction,0 ,0, delta_distance, self.move_distance, 0)
				self.wpose.orientation.x, self.wpose.orientation.y, self.wpose.orientation.z, self.wpose.orientation.w = (quaternion[0],
																									quaternion[1],
																									quaternion[2],
																									quaternion[3])
				self.waypoints.append(copy.deepcopy(self.wpose))
				
				(plan, fraction) = self._move_group.compute_cartesian_path(self.waypoints,   # waypoints to follow
               																0.01,        # eef_step
               																0.0)         # jump_threshold
				self.plan_path.append(plan)
				self.waypoints.clear()
				self.loop_num += 1
			quaternion =  quaternion_from_euler(np.radians(self.correct_degree[0]),
												np.radians(self.correct_degree[1]+15*self.degree_direction), 
												np.radians(self.correct_degree[2]))  	
			##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
			
			for self.loop_num in range(self.loop_size,self.loop_size*2):
				self.wpose = self._move_group.get_current_pose().pose
				delta_distance = self.move_distance-(self.loop_num-self.loop_size)*(self.move_distance*2/self.loop_size)
				self.generate_points(0,15*self.degree_direction ,0 , self.move_distance, delta_distance, 0)
				self.wpose.orientation.x, self.wpose.orientation.y, self.wpose.orientation.z, self.wpose.orientation.w = (quaternion[0],
																									quaternion[1],
																									quaternion[2],
																									quaternion[3])
				self.waypoints.append(copy.deepcopy(self.wpose))
				(plan, fraction) = self._move_group.compute_cartesian_path(self.waypoints,   # waypoints to follow
               																0.01,        # eef_step
               																0.0)         # jump_threshold
				self.plan_path.append(plan)
				self.waypoints.clear()
				self.loop_num += 1
			quaternion =  quaternion_from_euler(np.radians(self.correct_degree[0]-15*self.degree_direction),
												np.radians(self.correct_degree[1]), 
												np.radians(self.correct_degree[2]))  		
			##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
			for self.loop_num in range(self.loop_size*2,self.loop_size*3):
				self.wpose = self._move_group.get_current_pose().pose
				delta_distance = self.move_distance-(self.loop_num-self.loop_size*2)*(self.move_distance*2/self.loop_size)
				self.generate_points(-15*self.degree_direction,0 ,0, delta_distance, -self.move_distance, 0)
				self.wpose.orientation.x, self.wpose.orientation.y, self.wpose.orientation.z, self.wpose.orientation.w = (quaternion[0],
																									quaternion[1],
																									quaternion[2],
																									quaternion[3])
				self.waypoints.append(copy.deepcopy(self.wpose))
				(plan, fraction) = self._move_group.compute_cartesian_path(self.waypoints,   # waypoints to follow
               																0.01,        # eef_step
               																0.0)         # jump_threshold
				self.plan_path.append(plan)
				self.waypoints.clear()
				self.loop_num += 1
			quaternion =  quaternion_from_euler(np.radians(self.correct_degree[0]),
												np.radians(self.correct_degree[1]-15*self.degree_direction), 
												np.radians(self.correct_degree[2])) 
			##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
			for self.loop_num in range(self.loop_size*3,self.points_num):
				self.wpose = self._move_group.get_current_pose().pose
				delta_distance = -self.move_distance+(self.loop_num-self.loop_size*3)*(self.move_distance*2/self.loop_size)
				self.generate_points(0,-15*self.degree_direction ,0 , -self.move_distance, delta_distance, 0)
				self.wpose.orientation.x, self.wpose.orientation.y, self.wpose.orientation.z, self.wpose.orientation.w = (quaternion[0],
																									quaternion[1],
																									quaternion[2],
																									quaternion[3])
				self.waypoints.append(copy.deepcopy(self.wpose))
				(plan, fraction) = self._move_group.compute_cartesian_path(self.waypoints,   # waypoints to follow
               																0.01,        # eef_step
               																0.0)         # jump_threshold
				self.plan_path.append(plan)
				self.waypoints.clear()
				self.loop_num += 1
			
			# self.plan_path.append(self.plan_path[0])
			userdata.hand_eye_points = self.plan_path
			userdata.motion_state = 'start_calibration'
		return 'done'

	def on_enter(self, userdata):

		origindegree = euler_from_quaternion([self._first_pose.pose.orientation.x, self._first_pose.pose.orientation.y, self._first_pose.pose.orientation.z, self._first_pose.pose.orientation.w])
		self._origin_euler[0] = origindegree[0]/3.14*180.0
		self._origin_euler[1] = origindegree[1]/3.14*180.0
		self._origin_euler[2] = origindegree[2]/3.14*180.0

		# print(self.move_distance)
		# print("=====================================================================")

		# print("first aruco")
		print(userdata.camera_h_charuco.transforms[0].translation.x)
		print(userdata.camera_h_charuco.transforms[0].translation.y)
		print(userdata.camera_h_charuco.transforms[0].translation.z)
		aruco_rotation = list(euler_from_quaternion([userdata.camera_h_charuco.transforms[0].rotation.x, 
												userdata.camera_h_charuco.transforms[0].rotation.y, 
												userdata.camera_h_charuco.transforms[0].rotation.z, 
												userdata.camera_h_charuco.transforms[0].rotation.w]))
		aruco_rotation[0] = aruco_rotation[0]/3.14*180
		aruco_rotation[1] = aruco_rotation[1]/3.14*180
		aruco_rotation[2] = aruco_rotation[2]/3.14*180
		# print("===============================================================")
		# print(aruco_rotation)
		# print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++")	
	

		if self._base_end_axis == 'xyz':
			self.roll  = self._origin_euler[0]+1*(180+aruco_rotation[0])
			self.pitch = self._origin_euler[1]+1*(0+aruco_rotation[1])
			self.yaw   = self._origin_euler[2]+1*(90+aruco_rotation[2])
			pass
		elif self._base_end_axis == 'xy-z':
			self.roll  = self._origin_euler[0]+1*(180+aruco_rotation[0])
			self.pitch = self._origin_euler[1]+1*(0+aruco_rotation[1])
			self.yaw   = self._origin_euler[2]+-1*(90+aruco_rotation[2])
			pass
		elif self._base_end_axis == 'x-yz':
			self.roll  = self._origin_euler[0]+1*(180+aruco_rotation[0])
			self.pitch = self._origin_euler[1]+-1*(0+aruco_rotation[1])
			self.yaw   = self._origin_euler[2]+1*(90+aruco_rotation[2])
			pass
		elif self._base_end_axis == 'x-y-z':
			self.roll  = self._origin_euler[0]+1*(180+aruco_rotation[0])
			self.pitch = self._origin_euler[1]+-1*(0+aruco_rotation[1])
			self.yaw   = self._origin_euler[2]+-1*(90+aruco_rotation[2])
			pass
		elif self._base_end_axis == '-xyz':
			self.roll  = self._origin_euler[0]+-1*(180+aruco_rotation[0])
			self.pitch = self._origin_euler[1]+1*(0+aruco_rotation[1])
			self.yaw   = self._origin_euler[2]+1*(90+aruco_rotation[2])
			pass
		elif self._base_end_axis == '-xy-z':
			self.roll  = self._origin_euler[0]+-1*(180+aruco_rotation[0])
			self.pitch = self._origin_euler[1]+1*(0+aruco_rotation[1])
			self.yaw   = self._origin_euler[2]+-1*(90+aruco_rotation[2])				
			pass
		elif self._base_end_axis == '-x-yz':
			self.roll  = self._origin_euler[0]+-1*(180+aruco_rotation[0])
			self.pitch = self._origin_euler[1]+-1*(0+aruco_rotation[1])
			self.yaw   = self._origin_euler[2]+1*(90+aruco_rotation[2])
			pass
		elif self._base_end_axis == '-x-y-z':
			self.roll  = self._origin_euler[0]+-1*(180+aruco_rotation[0])
			self.pitch = self._origin_euler[1]+-1*(0+aruco_rotation[1])
			self.yaw   = self._origin_euler[2]+-1*(90+aruco_rotation[2])
			pass
		elif self._base_end_axis == 'yxz':
			self.roll  = self._origin_euler[0]+1*(0+aruco_rotation[1])
			self.pitch = self._origin_euler[1]+1*(180+aruco_rotation[0])
			self.yaw   = self._origin_euler[2]+1*(90+aruco_rotation[2])
			pass
		elif self._base_end_axis == 'yx-z':
			self.roll  = self._origin_euler[0]+1*(0+aruco_rotation[1])
			self.pitch = self._origin_euler[1]+1*(180+aruco_rotation[0])
			self.yaw   = self._origin_euler[2]+-1*(90+aruco_rotation[2])
			pass
		elif self._base_end_axis == 'y-xz':
			self.roll  = self._origin_euler[0]+1*(0+aruco_rotation[1])
			self.pitch = self._origin_euler[1]+-1*(180+aruco_rotation[0])
			self.yaw   = self._origin_euler[2]+1*(90+aruco_rotation[2])
			pass
		elif self._base_end_axis == 'y-x-z':
			self.roll  = self._origin_euler[0]+1*(0+aruco_rotation[1])
			self.pitch = self._origin_euler[1]+-1*(180+aruco_rotation[0])
			self.yaw   = self._origin_euler[2]+-1*(90+aruco_rotation[2])
			pass
		elif self._base_end_axis == '-yxz':
			self.roll  = self._origin_euler[0]+-1*(0+aruco_rotation[1])
			self.pitch = self._origin_euler[1]+1*(180+aruco_rotation[0])
			self.yaw   = self._origin_euler[2]+1*(90+aruco_rotation[2])
			pass
		elif self._base_end_axis == '-yx-z':
			self.roll  = self._origin_euler[0]+-1*(0+aruco_rotation[1])
			self.pitch = self._origin_euler[1]+1*(180+aruco_rotation[0])
			self.yaw   = self._origin_euler[2]+-1*(90+aruco_rotation[2])
			pass
		elif self._base_end_axis == '-y-xz':
			self.roll  = self._origin_euler[0]+-1*(0+aruco_rotation[1])
			self.pitch = self._origin_euler[1]+-1*(180+aruco_rotation[0])
			self.yaw   = self._origin_euler[2]+1*(90+aruco_rotation[2])
			pass
		elif self._base_end_axis == '-y-x-z':
			self.roll  = self._origin_euler[0]+-1*(0+aruco_rotation[1])
			self.pitch = self._origin_euler[1]+-1*(180+aruco_rotation[0])
			self.yaw   = self._origin_euler[2]+-1*(90+aruco_rotation[2])
			pass
##################eye in hand is x-y-z   cam to base(axis)
#################eye to hand is -yxz   board to base(axis) 
			print("=====================================================================")

		pass

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		# self.on_enter(userdata)
		pass
