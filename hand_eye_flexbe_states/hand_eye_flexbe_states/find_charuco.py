#!/usr/bin/env python3
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
import time
import rclpy
from rclpy.clock import Clock, ClockType
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# import tf

class FindCharucoState(EventState):
	"""
	Output a fixed pose to move.

	<= done									   Charuco pose has been received.
	<= go_compute							   Ready to compute the result.

	"""
	
	def __init__(self, base_link, tip_link, eye_in_hand_mode):
		"""Constructor"""
		super(FindCharucoState, self).__init__(outcomes=['done', 'go_compute'], input_keys=['result_compute'], 
											   output_keys=['base_h_tool', 'camera_h_charuco'])
		self.base_link = base_link
		self.tip_link = tip_link
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self._node)
		self.base_h_tool = TransformArray()
		self.camera_h_charuco = TransformArray()
		self.base_h_tool.header.frame_id = self.base_link
		self.camera_h_charuco.header.frame_id = 'calib_camera'
		self.enter_time = Clock(clock_type=ClockType.ROS_TIME).now() 
		self.eye_in_hand_mode = eye_in_hand_mode

	def execute(self, userdata):
		time.sleep(1.5)
		if (Clock(clock_type=ClockType.ROS_TIME).now()  - self.enter_time).nanoseconds / 1000000000 > 2:
			Logger.logwarn('Can not get charuco board pose, abandon this position')
			return 'done'


		if self.eye_in_hand_mode:
			try:
				camera_trans_charuco = self.tf_buffer.lookup_transform(
					'calib_camera',
					'calib_charuco',
					rclpy.time.Time())
				print(camera_trans_charuco)
			except TransformException as ex:
				Logger.logwarn('lookupTransform for charuco failed!')
				return
		else:
			try:
				camera_trans_charuco = self.tf_buffer.lookup_transform(
					'calib_charuco',
					'calib_camera',
					rclpy.time.Time())
			except TransformException as ex:
				Logger.logwarn('lookupTransform for charuco failed!')
				return

		try:
			base_trans_tool = self.tf_buffer.lookup_transform(
				self.base_link,
				self.tip_link,
				rclpy.time.Time())
		except TransformException as ex:
			Logger.logwarn('lookupTransform for robot failed!')
			return


		trans = Transform()
		trans.translation.x = camera_trans_charuco.transform.translation.x
		trans.translation.y = camera_trans_charuco.transform.translation.y
		trans.translation.z = camera_trans_charuco.transform.translation.z
		trans.rotation.x = camera_trans_charuco.transform.rotation.x
		trans.rotation.y = camera_trans_charuco.transform.rotation.y
		trans.rotation.z = camera_trans_charuco.transform.rotation.z
		trans.rotation.w = camera_trans_charuco.transform.rotation.w
		self.camera_h_charuco.transforms.append(trans)
		# print(self.camera_h_charuco.transforms)
		# print(self.camera_h_charuco)
		# print(self.camera_h_charuco.transforms[0].translation.x)
		# print(self.camera_h_charuco.transforms[0].translation.y)
		# print(self.camera_h_charuco.transforms[0].translation.z)
		print("============================")

		trans = Transform()
		trans.translation.x = base_trans_tool.transform.translation.x
		trans.translation.y = base_trans_tool.transform.translation.y
		trans.translation.z = base_trans_tool.transform.translation.z
		trans.rotation.x = base_trans_tool.transform.rotation.x
		trans.rotation.y = base_trans_tool.transform.rotation.y
		trans.rotation.z = base_trans_tool.transform.rotation.z
		trans.rotation.w = base_trans_tool.transform.rotation.w
		self.base_h_tool.transforms.append(trans)
		# print(self.base_h_tool.transforms)
		print("============================")


		if userdata.result_compute:
			userdata.base_h_tool = self.base_h_tool
			userdata.camera_h_charuco = self.camera_h_charuco
			return 'go_compute'
		else:
			return 'done'
			
	def on_enter(self, userdata):
		self.enter_time = Clock(clock_type=ClockType.ROS_TIME).now() 
