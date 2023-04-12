#!/usr/bin/env python
from flexbe_core import EventState


class AutomaticMoveRobot(EventState):
    '''
    Implements a state that can be used to wait on timed process.

    -- wait_time 	float	Amount of time to wait in seconds.
    -- pose_num     int     Number of pose to calibrate

    #> result_compute bool  Ready to compute the result

    <= done					Indicates that the wait time has elapsed.
    '''

    def __init__(self, wait_time, pose_num):
        super(AutomaticMoveRobot, self).__init__(outcomes=['done'], output_keys=['result_compute'])
        self._wait = wait_time
        self._pose_num = pose_num
        self._pose_count = 0

    def execute(self, userdata):
        elapsed = AutomaticMoveRobot._node.get_clock().now() - self._start_time
        userdata.result_compute = self._pose_count >= self._pose_num
        if elapsed.to_sec() > self._wait:
            return 'done'

    def on_enter(self, userdata):
        '''Upon entering the state, save the current time and start waiting.'''
        self._start_time = AutomaticMoveRobot._node.get_clock().now()
        self._pose_count += 1