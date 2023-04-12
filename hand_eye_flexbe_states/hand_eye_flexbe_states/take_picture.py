#!/usr/bin/env python3
import rospy
from flexbe_core import EventState
import cv2
import pyrealsense2 as rs
import numpy as np
import os
from pathlib import Path



class TakePictureState(EventState):
    """
    Output a fixed pose to move.

    <= done									   Charuco pose has been received.
    <= failed							       Failed to get result.

    """
    
    def __init__(self, pic_num, camera_type):
        """Constructor"""
        super(TakePictureState, self).__init__(outcomes=['done', 'failed'])
        self.excu_num = 1
        self.pic_num = pic_num
        if camera_type == 'realsense':
            # Configure depth and color streams
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            # self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

            # Start streaming
            self.cfg = self.pipeline.start(self.config)
            # self.pipeline.poll_for_frames
            # self.dev = cfg.get_device()
            # self.depth_sensor = dev.first_depth_sensor()
            # self.depth_sensor.set_option(rs.option.visual_preset, 4)
        
        self.iteration = 0
        self.preset = 0
        self.preset_name = ""
        self.capture = cv2.VideoCapture(1)

        self.save_pwd = os.path.join(os.path.dirname(__file__), '..','..','..','charuco_detector/','config/','camera_calibration/','pic/')
    


    def on_start(self):
        
        pass
    def execute(self, userdata):

  
        while(True):
        
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()

            #depth_frame = frames.get_depth_frame()

            color_frame = frames.get_color_frame()
            #iteration = iteration + 1
            #if iteration > 100:
                #preset = preset + 1
                #iteration = 0
                #range = depth_sensor.get_option_range(rs.option.visual_preset)
                #preset = preset % range.max
                #depth_sensor.set_option(rs.option.visual_preset, preset)
                #preset_name = depth_sensor.get_option_value_description(rs.option.visual_preset, preset)
            # Convert images to numpy arrays
            #depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, None, 0.5, 0), cv2.COLORMAP_JET)
            # Stack both images horizontally
            #images = np.hstack((color_image, depth_colormap))
            images = color_image
            #font = cv2.FONT_HERSHEY_SIMPLEX
            #cv2.putText(images, preset_name,(720,1300), font, 4,(255,255,255),2,cv2.LINE_AA)
            # Show images
            cv2.namedWindow('preview', cv2.WINDOW_AUTOSIZE)
            ret, RealSense = self.capture.read()
            cv2.imshow('preview', images)
            #cv2.imshow('Real', images)
            key = cv2.waitKey(1)
            if key  == 13 : #enter
                print("----------------------------------------------")
                # cv2.imwrite("./../../../config/pic/camera-pic-of-charucoboard-"+str(self.excu_num)+".jpg",images)
                cv2.imwrite(self.save_pwd+"camera-pic-of-charucoboard-"+str(self.excu_num)+".jpg",images)
                self.excu_num += 1
            #cv2.imshow('frame', frame)
            elif key & 0xFF == ord('q')or key == 27: # q or esc
                print("==========================================")
                self.excu_num = self.pic_num
                return "failed"

            elif self.excu_num >= self.pic_num+1: 
                self.capture.release()
                cv2.destroyAllWindows()
                # Stop streaming
                self.pipeline.stop()

                return 'done'
            
    def on_enter(self, userdata):
        self.enter_time = rospy.Time.now()
        pass


