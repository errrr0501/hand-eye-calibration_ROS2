#!/usr/bin/env python

import configparser,os
import numpy as np
import tf
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from ament_index_python.packages import get_package_share_directory
from visp_hand2eye_calibration.srv import ComputeEffectorCameraQuick


class ComputeCalibState(EventState):
	"""
	Output a fixed pose to move.

	<= done									   Pose has been published.
	<= fail									   Task fail and finished

	"""
	
	def __init__(self, eye_in_hand_mode, calibration_file_name, customize_file):
		"""Constructor"""
		super(ComputeCalibState, self).__init__(outcomes=['finish'], input_keys=['base_h_tool', 'camera_h_charuco'])
		self.eye_in_hand_mode = eye_in_hand_mode
		self.camera_object_list = TransformArray()
		self.world_effector_list = TransformArray()
		ProxyServiceCaller._initialize(ComputeCalibState._node)
		self.calib_compute_client = ProxyServiceCaller({'/compute_effector_camera_quick':ComputeEffectorCameraQuick})
		self.save_pwd = get_package_share_directory('charuco_detector') + '/config/hand_eye_calibration/'
		self.config = configparser.ConfigParser()
		self.config.optionxform = str #reference: http://docs.python.org/library/configparser.html

		if customize_file:
			self.calibration_file_name = str(calibration_file_name)
			with open(self.save_pwd+ self.calibration_file_name, 'w') as file:
				self.config.write(file)
		else:
			if eye_in_hand_mode:
				self.calibration_file_name = "eye_in_hand_calibration.ini"
			else:
				self.calibration_file_name = "eye_to_hand_calibration.ini"


	
	def execute(self, userdata):
		req = ComputeEffectorCameraQuick.Request()
		req.camera_object = self.camera_object_list
		req.world_effector = self.world_effector_list
		print ("========================================================================================================")
		res = self.calib_compute_client.call('/compute_effector_camera_quick', req)
		
		print('x = '  + str(res.effector_camera.translation.x))
		print('y = '  + str(res.effector_camera.translation.y))
		print('z = '  + str(res.effector_camera.translation.z))
		print('qx = ' + str(res.effector_camera.rotation.x))
		print('qy = ' + str(res.effector_camera.rotation.y))
		print('qz = ' + str(res.effector_camera.rotation.z))
		print('qw = ' + str(res.effector_camera.rotation.w))

		# config = configparser.ConfigParser()
		# config.optionxform = str #reference: http://docs.python.org/library/configparser.html
		self.config.read(self.save_pwd + self.calibration_file_name)
		# config.read(curr_path + '/config/hand_eye_calibration/'+ self.calibration_file_name)

		if self.config.get("hand_eye_calibration" ,"x") != None:
			pass
		else:
			self.config.add_section("hand_eye_calibration")
		self.config.set("hand_eye_calibration", "x",  str(res.effector_camera.translation.x))
		self.config.set("hand_eye_calibration", "y",  str(res.effector_camera.translation.y))
		self.config.set("hand_eye_calibration", "z",  str(res.effector_camera.translation.z))
		self.config.set("hand_eye_calibration", "qx", str(res.effector_camera.rotation.x))
		self.config.set("hand_eye_calibration", "qy", str(res.effector_camera.rotation.y))
		self.config.set("hand_eye_calibration", "qz", str(res.effector_camera.rotation.z))
		self.config.set("hand_eye_calibration", "qw", str(res.effector_camera.rotation.w))

		with open(self.save_pwd+ self.calibration_file_name, 'w') as file:
			self.config.write(file)
		return 'finish'
	
	def on_enter(self, userdata):
		if self.eye_in_hand_mode:
			print("------------------------------------------------------------------")
			self.world_effector_list = userdata.base_h_tool
			self.camera_object_list = userdata.camera_h_charuco
			# print(self.camera_object_list)
			# print(self.world_effector_list)
		else:
			self.world_effector_list.header = userdata.base_h_tool.header
			print ("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
			print (userdata.base_h_tool)

			self.camera_object_list.header = userdata.camera_h_charuco.header

			for transform in userdata.base_h_tool.transforms:
				trans = tf.transformations.quaternion_matrix([transform.rotation.x, transform.rotation.y,
															  transform.rotation.z, transform.rotation.w])
				trans[0:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
				trans = tf.transformations.inverse_matrix(trans)
				trans_B = Transform()
				trans_B.translation.x, trans_B.translation.y, trans_B.translation.z = trans[:3, 3]
				trans_B.rotation.x, trans_B.rotation.y, trans_B.rotation.z, \
					trans_B.rotation.w = tf.transformations.quaternion_from_matrix(trans)
				self.world_effector_list.transforms.append(trans_B)

			for transform in userdata.camera_h_charuco.transforms:
				trans = tf.transformations.quaternion_matrix([transform.rotation.x, transform.rotation.y,
															  transform.rotation.z, transform.rotation.w])
				trans[0:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
				trans = tf.transformations.inverse_matrix(trans)
				trans_C = Transform()
				trans_C.translation.x, trans_C.translation.y, trans_C.translation.z = trans[:3, 3]
				trans_C.rotation.x, trans_C.rotation.y, trans_C.rotation.z, \
					trans_C.rotation.w = tf.transformations.quaternion_from_matrix(trans)
				self.camera_object_list.transforms.append(trans_C)

