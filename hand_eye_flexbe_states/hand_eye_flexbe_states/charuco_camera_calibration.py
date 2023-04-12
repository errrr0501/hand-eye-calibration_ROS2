#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import time
import numpy
import cv2
from cv2 import aruco
import pickle
import glob,os
import configparser
from sensor_msgs.msg import CameraInfo
from ament_index_python.packages import get_package_share_directory

class CharucoCameraCalibrationState(EventState):
	"""
	Output a fixed pose to move.

	<= done									   Charuco pose has been received.
	<= go_compute							   Ready to compute the result.

	"""
	
	def __init__(self, square_size, marker_size, col_count, row_count, save_file_name):
		"""Constructor"""
		super(CharucoCameraCalibrationState, self).__init__(outcomes=['done', 'failed'])
		self.camera_calibration_file = save_file_name
		
		self.CHARUCOBOARD_COLCOUNT = col_count
		self.CHARUCOBOARD_ROWCOUNT = row_count	
		self.ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_100)
		self.squareLength = square_size
		self.markerLength = marker_size

		# Create constants to be passed into OpenCV and Aruco methods
		self.CHARUCO_BOARD = aruco.CharucoBoard_create(
								squaresX=self.CHARUCOBOARD_COLCOUNT,
								squaresY=self.CHARUCOBOARD_ROWCOUNT,
								squareLength=self.squareLength,
								markerLength=self.markerLength,
								dictionary=self.ARUCO_DICT)

		# Create the arrays and variables we'll use to store info like corners and IDs from images processed
		self.corners_all = [] # Corners discovered in all images processed
		self.ids_all = [] # Aruco ids corresponding to corners discovered
		self.image_size = None # Determined at runtime
		# self.save_pwd = os.path.join(os.path.dirname(__file__), '..','..','..',',','charuco_detector_ROS2/','charuco_detector/','config/','camera_calibration/')
		self.save_pwd = get_package_share_directory('charuco_detector') + '/config/camera_calibration/'

		self.images = glob.glob(self.save_pwd + 'pic/camera-pic-of-charucoboard-*.jpg')


	def on_start(self):
		pass
	def execute(self, userdata):
		for iname in self.images:
			# Open the image
			img = cv2.imread(iname)
			# Grayscale the image
			gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

			# Find aruco markers in the query image
			corners, ids, _ = aruco.detectMarkers(
					image=gray,
					dictionary=self.ARUCO_DICT)

			# Outline the aruco markers found in our query image
			img = aruco.drawDetectedMarkers(
					image=img, 
					corners=corners)

			# Get charuco corners and ids from detected aruco markers
			response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
					markerCorners=corners,
					markerIds=ids,
					image=gray,
					board=self.CHARUCO_BOARD)

			# If a Charuco board was found, let's collect image/corner points
			# Requiring at least 20 squares
			if response > 20:
				# Add these corners and ids to our calibration arrays
				self.corners_all.append(charuco_corners)
				self.ids_all.append(charuco_ids)

				# Draw the Charuco board we've detected to show our calibrator the board was properly detected
				img = aruco.drawDetectedCornersCharuco(
						image=img,
						charucoCorners=charuco_corners,
						charucoIds=charuco_ids)

				# If our image size is unknown, set it now
				if not self.image_size:
					self.image_size = gray.shape[::-1]

				# Reproportion the image, maxing width or height at 1000
				proportion = max(img.shape) / 1920.0
				img = cv2.resize(img, (int(img.shape[1]/proportion), int(img.shape[0]/proportion)))
				# Pause to display each image, waiting for key press
				cv2.imshow('Charuco board', img)
				cv2.waitKey(0)
			else:
				Logger.logwarn("Not able to detect a charuco board in image: {}".format(iname))

		# Destroy any open CV windows
		cv2.destroyAllWindows()
		Logger.logwarn("==========================================================================")
		# Make sure at least one image was found
		if len(self.images) < 1:
			# Calibration failed because there were no images, warn the user
			print("Calibration was unsuccessful. No images of charucoboards were found. Add images of charucoboards and use or alter the naming conventions used in this file.")
			# Exit for failure
			return "failed"

		# Make sure we were able to calibrate on at least one charucoboard by checking
		# if we ever determined the image size
		if not self.image_size:
			# Calibration failed because we didn't see any charucoboards of the PatternSize used
			Logger.logwarn("Calibration was unsuccessful. We couldn't detect charucoboards in any of the images supplied. Try changing the patternSize passed into Charucoboard_create(), or try different pictures of charucoboards.")
			# Exit for failure
			return "failed"

		# Now that we've seen all of our images, perform the camera calibration
		# based on the set of points we've discovered
		# print("==========================================================================")
		calibration, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
				charucoCorners=self.corners_all,
				charucoIds=self.ids_all,
				board=self.CHARUCO_BOARD,
				imageSize=self.image_size,
				cameraMatrix=None,
				distCoeffs=None)

		# Print matrix and distortion coefficient to the console
		# print("==========================================================================")

		print("-----------------------------------------------------")
		print(cameraMatrix)
		print(distCoeffs)
		print("-----------------------------------------------------")
		config = configparser.ConfigParser()
		config.optionxform = str 
		#reference: http://docs.python.org/library/configparser.html
		# rospack = rospkg.RosPack()
		# # curr_path = rospack.get_path('charuco_detector')
		# # config.read(curr_path + '/config/ '+ self.camera_calibration_file)
		
		config.add_section("Distortion")
		config.set("Distortion", "k1",  str(distCoeffs[0][0]))
		config.set("Distortion", "k2",  str(distCoeffs[0][1]))
		config.set("Distortion", "t1",  str(distCoeffs[0][2]))
		config.set("Distortion", "t2",  str(distCoeffs[0][3]))
		config.set("Distortion", "k3",  str(distCoeffs[0][4]))
		config.add_section("Intrinsic")
		config.set("Intrinsic", "0_0", str(cameraMatrix[0][0]))
		config.set("Intrinsic", "0_1", str(cameraMatrix[0][1]))
		config.set("Intrinsic", "0_2", str(cameraMatrix[0][2]))
		config.set("Intrinsic", "1_0", str(cameraMatrix[1][0]))
		config.set("Intrinsic", "1_1", str(cameraMatrix[1][1]))
		config.set("Intrinsic", "1_2", str(cameraMatrix[1][2]))
		config.set("Intrinsic", "2_0", str(cameraMatrix[2][0]))
		config.set("Intrinsic", "2_1", str(cameraMatrix[2][1]))
		config.set("Intrinsic", "2_2", str(cameraMatrix[2][2]))
		
		with open(self.save_pwd+ self.camera_calibration_file, 'w') as file:
			config.write(file)
		# Save values to be used where matrix+dist is required, for instance for posture estimation
		# I save files in a pickle file, but you can use yaml or whatever works for you
		# f = open(self.save_pwd+'/calibration.pckl', 'wb')
		# pickle.dump((cameraMatrix, distCoeffs, rvecs, tvecs), f)
		# f.close()
		# f = open(self.save_pwd+'/calibration.yaml', 'wb')
		# pickle.dump((cameraMatrix, distCoeffs, rvecs, tvecs), f)
		# f.close()


		# Print to console our success
		print('Calibration successful. Calibration file used: {}'.format('calibration.yaml'))


		return 'done'
			
	def on_enter(self, userdata):
		self.enter_time = CharucoCameraCalibrationState._node.get_clock().now()

		pass
