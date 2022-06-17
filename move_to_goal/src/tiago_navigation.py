#!/usr/bin/python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from positions import Location
from tiago_headcontrol import TiagoHeadMovement
from math import sin, cos, sqrt, atan2


class Navigation:

    def __init__(self):

        self.client_nav = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client_nav.wait_for_server()
        self.human_loc = Location()
        self.head_movement = TiagoHeadMovement()
        self.store_goals = []

    def navigate(self, init_x, init_y):

        cur_goal = {'x' : init_x, 'y' : init_y} # dict of previous goal point visited by human
        k = 0
        while True:
            hum_pos = self.human_loc.human_location
            if sqrt((cur_goal['x'] - hum_pos['x'])**2 + (cur_goal['y'] - hum_pos['y'])**2) <= 0.5: # condition to check if the human has reached his destination
                k = k + 1
                if k == 20:
                    r = input('Destination reached? (y or n)')
                    if r == 'y':
                        return # ending navigation if human has reached his destination
                    else:
                        k = 0
                rospy.sleep(1)
            else:
                # transforming all world frame co-ordinates to map frame before passing the goal points to move_base and head_controller
                # passing z-co-ordinate as 1.13 for robot's head to be parallel to the ground
                self.head_movement.head_goal(hum_pos['x'] - init_x, hum_pos['y'] - init_y, 1.13) # looking at the human
                self.head_movement.head_goal(cur_goal['x'] - init_x, cur_goal['y'] - init_y, 1.13) # looking at the current goal
                robot_goal = MoveBaseGoal()
                robot_goal.target_pose.header.frame_id = 'map'
                robot_goal.target_pose.header.stamp = rospy.Time.now()
                robot_goal.target_pose.pose.position.x = cur_goal['x'] - init_x
                robot_goal.target_pose.pose.position.y = cur_goal['y'] - init_y
                robot_goal.target_pose.pose.position.z = 0
                tiago_position = self.human_loc.tiago_position
                # calculating yaw angle of the current goal and converting it into quaternions to orient the robot in the direction of the goal point
                theta = atan2(cur_goal['y'] - init_y - tiago_position['y'], cur_goal['x'] - init_x - tiago_position['x'])
                robot_goal.target_pose.pose.orientation.w = cos(theta/2) #
                robot_goal.target_pose.pose.orientation.x = 0
                robot_goal.target_pose.pose.orientation.y = 0
                robot_goal.target_pose.pose.orientation.z = sin(theta/2)
                self.client_nav.send_goal(robot_goal)
                self.client_nav.wait_for_result()
                self.head_movement.head_goal(hum_pos['x'] - init_x, hum_pos['y'] - init_y, 1.13)
                self.store_goals.append(cur_goal) # storing the visited goal points to plan the return of the robot to the initial position
                cur_goal = hum_pos # updating to human location


    def return_to_init_pos(self, init_x, init_y):

        self.store_goals.reverse() # reversing the stored goals to plan the return of the robot to the initial position
        for i in range(len(self.store_goals)):
            self.head_movement.head_goal(self.store_goals[i]['x'] - init_x, self.store_goals[i]['y'] - init_y, 1.13)
            robot_goal = MoveBaseGoal()
            robot_goal.target_pose.header.frame_id = 'map'
            robot_goal.target_pose.header.stamp = rospy.Time.now()
            robot_goal.target_pose.pose.position.x = self.store_goals[i]['x'] - init_x
            robot_goal.target_pose.pose.position.y = self.store_goals[i]['y'] - init_y
            robot_goal.target_pose.pose.position.z = 0
            tiago_position = self.human_loc.tiago_position
            theta = atan2(self.store_goals[i]['y'] - init_y - tiago_position['y'], self.store_goals[i]['x'] - init_x - tiago_position['x'])
            robot_goal.target_pose.pose.orientation.w = cos(theta/2)
            robot_goal.target_pose.pose.orientation.x = 0
            robot_goal.target_pose.pose.orientation.y = 0
            robot_goal.target_pose.pose.orientation.z = sin(theta/2)
            self.client_nav.send_goal(robot_goal)
            self.client_nav.wait_for_result()
        return

