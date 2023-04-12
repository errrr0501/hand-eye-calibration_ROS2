#!/usr/bin/env python3
import tf
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
import time

class GetArMarkerState(EventState):
	"""
	Output a fixed pose to move.

	<= done									   Charuco pose has been received.
	<= go_compute							   Ready to compute the result.

	"""
	
	def __init__(self, eye_in_hand_mode):
		"""Constructor"""
		super(GetArMarkerState, self).__init__(outcomes=['done', 'failed'], 
											   output_keys=['camera_h_charuco'])
		# self.base_link = base_link
		# self.tip_link = tip_link
		self.eye_in_hand_mode = eye_in_hand_mode
		self.tf_listener = tf.TransformListener()
		# self.base_h_tool = TransformArray()
		self.camera_h_charuco = TransformArray()
		# self.base_h_tool.header.frame_id = self.base_link
		self.camera_h_charuco.header.frame_id = 'calib_camera'
		self.enter_time = GetArMarkerState._node.get_clock().now()

	def execute(self, userdata):
		time.sleep(1)
		if (GetArMarkerState._node.get_clock().now() - self.enter_time).to_sec() > 2:
			Logger.logwarn('Can not get charuco board pose, abandon this position')
			return 'failed'


		try:
			(camera_trans_charuco, camera_rot_charuco) = self.tf_listener.lookupTransform('/calib_camera', '/calib_charuco', self.get_clock().now())
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			Logger.logwarn('lookupTransform for charuco failed!')
			return

		# try:
		# 	(base_trans_tool, base_rot_tool) = self.tf_listener.lookupTransform(self.base_link, self.tip_link, rospy.Time(0))
		# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		# 	self.get_logger().warn('lookupTransform for robot failed!, ' + self.base_link + ', ' + self.tip_link)
		# 	return


		trans = Transform()
		trans.translation.x = camera_trans_charuco[0]
		trans.translation.y = camera_trans_charuco[1]
		trans.translation.z = camera_trans_charuco[2]
		trans.rotation.x = camera_rot_charuco[0]
		trans.rotation.y = camera_rot_charuco[1]
		trans.rotation.z = camera_rot_charuco[2]
		trans.rotation.w = camera_rot_charuco[3]
		self.camera_h_charuco.transforms.append(trans)
		print(self.camera_h_charuco.transforms)
		# print(self.camera_h_charuco)
		# print(self.camera_h_charuco.transforms[0].translation.x)
		# print(self.camera_h_charuco.transforms[0].translation.y)
		# print(self.camera_h_charuco.transforms[0].translation.z)
		print("============================")

		# trans = Transform()
		# trans.translation.x = base_trans_tool[0]
		# trans.translation.y = base_trans_tool[1]
		# trans.translation.z = base_trans_tool[2]
		# trans.rotation.x = base_rot_tool[0]
		# trans.rotation.y = base_rot_tool[1]
		# trans.rotation.z = base_rot_tool[2]
		# trans.rotation.w = base_rot_tool[3]
		# self.base_h_tool.transforms.append(trans)
		# print(self.base_h_tool.transforms)
		# print("============================")
		userdata.camera_h_charuco = self.camera_h_charuco
		return 'done'

		# if userdata.result_compute:
		# 	# userdata.base_h_tool = self.base_h_tool
		# 	userdata.camera_h_charuco = self.camera_h_charuco
		# 	return 'done'
		# else:
		# 	return 'done'
			
	def on_enter(self, userdata):
		self.enter_time = GetArMarkerState._node.get_clock().now()
