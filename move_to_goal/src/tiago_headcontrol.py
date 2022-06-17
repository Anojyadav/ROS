#!/usr/bin/python

import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal


class TiagoHeadMovement:

    def __init__(self):

        self.client_head = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
        self.client_head.wait_for_server()

    def head_goal(self, x, y, z):

        head_goal = PointHeadGoal()
        head_goal.target.header.frame_id = 'map'
        head_goal.target.point.x = x
        head_goal.target.point.y = y
        head_goal.target.point.z = z
        head_goal.pointing_axis.x = 1
        head_goal.pointing_axis.y = 0
        head_goal.pointing_axis.z = 0
        head_goal.pointing_frame = 'xtion_rgb_frame'
        self.client_head.send_goal(head_goal)
        self.client_head.wait_for_result()
        return
