#!/usr/bin/python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from math import sin, cos


class Location:

    def __init__(self):

        self.human_location = None
        self.luggage_location = None
        self.tiago_location = None
        self.tiago_position = None
        self.yaw = None
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_pose)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.tiago_amcl_pose)

    def gazebo_pose(self, gaz) :

        self.tiago_location = {'x' : gaz.pose[15].position.x, 'y' : gaz.pose[15].position.y}
        self.human_location = {'x' : gaz.pose[13].position.x, 'y' : gaz.pose[13].position.y}
        self.luggage_location = {'x' : gaz.pose[16].position.x, 'y' : gaz.pose[16].position.y}

    def tiago_amcl_pose(self, tiago_real_pose) :

        self.tiago_position = {'x' : tiago_real_pose.pose.pose.position.x, 'y' : tiago_real_pose.pose.pose.position.y, 'q_w' : tiago_real_pose.pose.pose.orientation.w, 'q_x' : tiago_real_pose.pose.pose.orientation.x, 'q_y' : tiago_real_pose.pose.pose.orientation.y, 'q_z' : tiago_real_pose.pose.pose.orientation.z}

        (r, p, self.yaw) = euler_from_quaternion([self.tiago_position['q_x'], self.tiago_position['q_y'], self.tiago_position['q_z'], self.tiago_position['q_w']])

    def base_to_map(self, x, y) : # transforming the dropping point of the luggage from the base co-ordinate to the map co-ordinate system

        return {'x' : x * cos(self.yaw) - y * sin(self.yaw) + self.tiago_position['x'], 'y' : x * sin(self.yaw) + y * cos(self.yaw) + self.tiago_position['y']}

