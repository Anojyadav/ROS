#!/usr/bin/python

import rospy
from moveit_commander import RobotCommander
from geometry_msgs.msg import Pose
from math import sqrt


class TiagoArmMovement :

    def __init__(self):

        self.robot = RobotCommander()
        self.torso_group = self.robot.get_group('arm_torso')
        self.gripper_group = self.robot.get_group('gripper')
        self.init_goal = None

    def go_to_pose_goal(self, lug_x, lug_y, a, b):

        self.init_goal = self.torso_group.get_current_joint_values()
        pose_goal = Pose()
        # rotating the gripper 90 degrees about the y-axis
        pose_goal.orientation.w = 1/sqrt(2)
        pose_goal.orientation.x = 0
        pose_goal.orientation.y = 1/sqrt(2)
        pose_goal.orientation.z = 0
        pose_goal.position.x = lug_x
        pose_goal.position.y = lug_y
        pose_goal.position.z = 0.25 # tolerance for avoiding the gripper to collide with the ground
        self.torso_group.set_pose_target(pose_goal)
        self.torso_group.go(wait = True)
        self.gripper_group.go([a, b], wait=True)
        self.gripper_group.stop()
        rospy.sleep(1)
        joint_goal = self.torso_group.get_current_joint_values()
        joint_goal[0] = 0.3 # tolerance for torso
        self.torso_group.go(joint_goal, wait = True)
        self.torso_group.go(self.init_goal, wait = True)
        self.torso_group.stop()
        self.torso_group.clear_pose_targets()
        rospy.sleep(1)
        return

