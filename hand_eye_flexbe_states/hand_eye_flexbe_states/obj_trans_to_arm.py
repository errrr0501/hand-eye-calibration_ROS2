#!/usr/bin/env python3

import copy, os
import configparser
import numpy as np
import math as m
from visp_hand2eye_calibration.msg import TransformArray
from math import pi, radians
from std_msgs.msg import String
from geometry_msgs.msg import Transform
import tf
# from tf import transformations
from visp_hand2eye_calibration.msg import TransformArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from flexbe_core.proxy import ProxyServiceCaller

from charuco_detector.srv import eye2base, eye2baseRequest, eye2baseResponse
# from charuco_detector import HandEyeTrans



class obj_info(dict):

    def __init__(self):
        self['x']       = [] 
        self['y']       = []
        self['z']       = []
        self['qx']      = []
        self['qy']      = []
        self['qz']      = []
        self['qw']      = []


class ObjTransToArmState(EventState):
	"""
	Output obj pose for arm.

	<= done									   points has been created.
	<= failed								   create points fail.

	"""


	def __init__(self, eye_in_hand_mode, base_link, tip_link):
		'''
		Constructor
		'''
		super(ObjTransToArmState, self).__init__(outcomes=['done', 'failed'],
											input_keys=['camera_h_charuco'],
											output_keys=['excute_position'])
		# if eye_in_hand_mode:
		# 	self.eye_in_hand_mode = 1
		# else:
		# 	self.eye_in_hand_mode = -1
		self.eye_in_hand_mode = eye_in_hand_mode
		self.base_link = base_link
		self.tip_link = tip_link
		self.tf_listener = tf.TransformListener()
		# self.tool_h_base = TransformArray()
		self.First_charuco_array = []
		self._origin_euler  = [0, 0, 0]

		self.save_pwd = os.path.join(os.path.dirname(__file__), '..','..','..','charuco_detector/','config/','hand_eye_calibration/')

		self.trans2robot_service = '/eye_trans2base'
		self.trans2robot_client = ProxyServiceCaller({self.trans2robot_service: eye2base})
		self.pos2robot_service = '/eye2base'
		self.pos2robot_client = ProxyServiceCaller({self.pos2robot_service: eye2base})
		self.pix2robot_service = '/pix2base'
		self.pix2robot_client = ProxyServiceCaller({self.pix2robot_service: eye2base})


		self.quaternion = []
		self.excute_pos = []




	def on_start(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''

		userdata.excute_position  = obj_info()
		userdata.excute_position['x'].append(self.excute_pos[0])
		userdata.excute_position['y'].append(self.excute_pos[1])
		userdata.excute_position['z'].append(self.excute_pos[2]+0.20265)
		userdata.excute_position['qx'].append(self.quaternion[0])
		userdata.excute_position['qy'].append(self.quaternion[1])
		userdata.excute_position['qz'].append(self.quaternion[2])
		userdata.excute_position['qw'].append(self.quaternion[3])


		print(userdata.excute_position)
		return 'done'

	def on_enter(self, userdata):


		print(userdata.camera_h_charuco.transforms[0].translation)

		origindegree = list(euler_from_quaternion([userdata.camera_h_charuco.transforms[0].rotation.x, 
													userdata.camera_h_charuco.transforms[0].rotation.y, 
													userdata.camera_h_charuco.transforms[0].rotation.z, 
													userdata.camera_h_charuco.transforms[0].rotation.w]))
		origindegree[0] = origindegree[0]/3.14*180.0
		origindegree[1] = origindegree[1]/3.14*180.0
		origindegree[2] = origindegree[2]/3.14*180.0
		# print(origindegree)


		obj_pose = self.tf_listener.fromTranslationRotation((userdata.camera_h_charuco.transforms[0].translation.x,
															userdata.camera_h_charuco.transforms[0].translation.y,
															userdata.camera_h_charuco.transforms[0].translation.z)
															,(userdata.camera_h_charuco.transforms[0].rotation.x,
															userdata.camera_h_charuco.transforms[0].rotation.y,
															userdata.camera_h_charuco.transforms[0].rotation.z,
															userdata.camera_h_charuco.transforms[0].rotation.w))



##################################################################################		
		thetax = np.radians(180)
		thetay = np.radians(180)
		thetaz = np.radians(-90)
		Rx = np.matrix([[ 1, 0           , 0             ,0],
		                [ 0, m.cos(thetax),-m.sin(thetax),0],
		                [ 0, m.sin(thetax), m.cos(thetax),0],
						[ 0, 0           , 0             ,1]])

		Ry = np.matrix([[ m.cos(thetay), 0, m.sin(thetay) ,0],
		                [ 0            , 1, 0             ,0],
		                [-m.sin(thetay), 0, m.cos(thetay) ,0],
						[ 0            , 0,0              ,1]])

		Rz = np.matrix([[ m.cos(thetaz), -m.sin(thetaz), 0 ,0],
		                [ m.sin(thetaz), m.cos(thetaz) , 0 ,0],
		                [ 0           , 0              , 1 ,0],
						[ 0           , 0              , 0 ,1]])

###############################################################################

		if self.eye_in_hand_mode:

			obj_pose = obj_pose * Rx
			# 
		else:
			obj_pose = obj_pose * Ry

		obj_pose = list(obj_pose.flat)
		print(obj_pose)




		req = eye2baseRequest()
		req.ini_pose = obj_pose

		try:
			res = self.trans2robot_client.call(self.trans2robot_service, req)
		except rospy.ServiceException as e:
			Logger.loginfo("Service call failed: %s" % e)
			return 'failed'

		self.excute_pos = res.pos
		
		self.quaternion = [res.quat[0], res.quat[1], res.quat[2], res.quat[3]]


		origindegree = list(euler_from_quaternion(self.quaternion))
		origindegree[0] = origindegree[0]/3.14*180.0
		origindegree[1] = origindegree[1]/3.14*180.0
		origindegree[2] = origindegree[2]/3.14*180.0

		print(origindegree)


		
		pass

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		# self.on_enter(userdata)
		pass
