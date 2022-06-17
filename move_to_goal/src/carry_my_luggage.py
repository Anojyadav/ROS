#!/usr/bin/python

import rospy
from positions import Location
from tiago_headcontrol import TiagoHeadMovement
from tiago_arm import TiagoArmMovement
from tiago_navigation import Navigation


class CarryFollow(object) :

    def __init__(self):

        rospy.init_node('tiago')
        self.robot_location = Location()
        self.robot_head = TiagoHeadMovement()
        self.robot_arm = TiagoArmMovement()
        self.robot_navigation = Navigation()

    def all_task(self):

        tiago_loc = self.robot_location.tiago_location
        human_loc = self.robot_location.human_location
        rospy.loginfo('Locating person')
        # transforming all world frame co-ordinates to map frame before passing the goal to head_controller
        self.robot_head.head_goal(human_loc['x'] - tiago_loc['x'], human_loc['y'] - tiago_loc['y'], 1.13) # passing z-co-ordinate as 1.13 for robot's head to be parallel to the ground
        rospy.loginfo('Person located')
        rospy.loginfo('Locating luggage')
        lug_pos = self.robot_location.luggage_location
        self.robot_head.head_goal(lug_pos['x'] - tiago_loc['x'], lug_pos['y'] - tiago_loc['y'], 0.3) # setting z to 0.3 to look at the luggage
        rospy.loginfo('Luggage located')
        rospy.loginfo('Picking up luggage')
        # giving positions in base frame to tiago arm
        self.robot_arm.go_to_pose_goal(lug_pos['x'] - tiago_loc['x'], lug_pos['y'] - tiago_loc['y'], 0.001, 0.001) # passing 0.001 to close the gripper to grip the luggage
        rospy.loginfo('Luggage gripped')
        rospy.loginfo('Following person')
        self.robot_navigation.navigate(tiago_loc['x'], tiago_loc['y'])
        rospy.loginfo('Destination reached')
        rospy.loginfo('Dropping luggage')
        b_m = self.robot_location.base_to_map(lug_pos['x'] - tiago_loc['x'], lug_pos['y'] - tiago_loc['y'])
        self.robot_head.head_goal(b_m['x'], b_m['y'], 0.3)
        self.robot_arm.go_to_pose_goal(lug_pos['x'] - tiago_loc['x'], lug_pos['y'] - tiago_loc['y'], 0.04, 0.04) # passing 0.04 to open the gripper to release the luggage
        rospy.loginfo('Luggage dropped')
        rospy.loginfo('Going to initial position')
        self.robot_navigation.return_to_init_pos(tiago_loc['x'], tiago_loc['y'])
        rospy.loginfo('Reached initial position')


if __name__ == '__main__':

    obj = CarryFollow()
    obj.all_task()
		
