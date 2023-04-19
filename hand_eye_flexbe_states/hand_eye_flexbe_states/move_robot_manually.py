#!/usr/bin/env python
from flexbe_core import EventState, Logger
from rclpy.clock import Clock, ClockType



class MoveRobotManuallyState(EventState):
    '''
    Implements a state that can be used to wait on timed process.

    -- wait_time 	float	Amount of time to wait in seconds.
    -- pose_num     int     Number of pose to calibrate

    #> result_compute bool  Ready to compute the result

    <= done					Indicates that the wait time has elapsed.
    '''

    def __init__(self, wait_time, pose_num):
        super(MoveRobotManuallyState, self).__init__(outcomes=['done'], output_keys=['result_compute'])
        self._wait = wait_time
        self._pose_num = pose_num
        self._pose_count = 0

    def execute(self, userdata):
        # elapsed = MoveRobotManuallyState._node.get_clock().now().seconds() - self._start_time
        elapsed = Clock(clock_type=ClockType.ROS_TIME).now() - self._start_time
        userdata.result_compute = self._pose_count >= self._pose_num
        if elapsed.nanoseconds / 1000000000 > self._wait:
            return 'done'

    def on_enter(self, userdata):
        '''Upon entering the state, save the current time and start waiting.'''
        self._start_time = Clock(clock_type=ClockType.ROS_TIME).now()   
        print(self._start_time) 
        self._pose_count += 1
