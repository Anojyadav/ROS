#!/usr/bin/python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import sin, cos, sqrt, atan2
from visualization_msgs.msg import Marker

class RobotGoals():

    def __init__(self):

        rospy.init_node('goal_nav')
        self.goal_points=[]
        self.goal_return=[]
        self.pub = rospy.Publisher("/visualization_marker",Marker, queue_size = 10)
        self.client_nav = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client_nav.wait_for_server()

# list of goals
    def goal_list(self): #random goal points through obstacles based on environment
		
        self.goal_points=[{'x' :- 1 ,'y' : 3, 'Q_w' : 1/sqrt(2), 'Q_z' : 1/sqrt(2)},{'x': 1 , 'y' : 3, 'Q_w' : 1, 'Q_z' : 0},{'x' :5 ,'y' : 0, 'Q_w' : 1/sqrt(2), 'Q_z' : -1/sqrt(2)},{'x': 2, 'y' : 1, 'Q_w' :  1/sqrt(2), 'Q_z' :  -1/sqrt(2)},{'x' : 2, 'y' : -3, 'Q_w' :  1/sqrt(2), 'Q_z' :  -1/sqrt(2)},{'x' : -1, 'y' : -3, 'Q_w' : 0, 'Q_z' : 1},{'x' : -3, 'y' : -3, 'Q_w' : 0, 'Q_z' : 1},{'x' : -3, 'y' : -8, 'Q_w' : 0, 'Q_z' : 1}]

# marking goal on rviz
    def goal_marker(self,goal):

            marker_goal = Marker()
            marker_goal.id = 0
            marker_goal.type = marker_goal.SPHERE
            marker_goal.action = marker_goal.ADD
            marker_goal.header.frame_id = 'map'
            marker_goal.header.stamp = rospy.Time.now()
            marker_goal.pose.position.x = goal['x']
            marker_goal.pose.position.y = goal['y']
            marker_goal.pose.orientation.w = goal['Q_w']
            marker_goal.pose.orientation.x = 0
            marker_goal.pose.orientation.y = 0
            marker_goal.pose.orientation.z = goal['Q_z']
            marker_goal.scale.x = 1
            marker_goal.scale.y = 0.1
            marker_goal.scale.z = 0.1
            marker_goal.color.r = 1
            marker_goal.color.g = 0
            marker_goal.color.b = 0
            marker_goal.color.a = 1
            self.pub.publish(marker_goal)

# sending goal to rviz
    def movebase(self,a):

        for i in range(len(a)):

            rospy.loginfo("visualizing the goal on rviz")
            self.goal_marker(a[i])
            robot_goal = MoveBaseGoal()
            robot_goal.target_pose.header.frame_id = 'map'
            robot_goal.target_pose.header.stamp = rospy.Time.now()
            rospy.loginfo("moving towards the goal point")
            t_1 = rospy.get_rostime()
            robot_goal.target_pose.pose.position.x = a[i]['x']
            robot_goal.target_pose.pose.position.y = a[i]['y']
            robot_goal.target_pose.pose.position.z = 0
            robot_goal.target_pose.pose.orientation.w = a[i]['Q_w']
            robot_goal.target_pose.pose.orientation.x = 0
            robot_goal.target_pose.pose.orientation.y = 0
            robot_goal.target_pose.pose.orientation.z = a[i]['Q_z']
            self.client_nav.send_goal(robot_goal)
            self.client_nav.wait_for_result()
            t_2=rospy.get_rostime()
            time_needed= (t_2 - t_1)*pow(10,-9)
            if self.client_nav.get_state() == 3:
                rospy.loginfo('Goal reached x: {}, y: {}'.format(a[i]['x'], a[i]['y']))
                rospy.loginfo('Time needed to drive route in seconds {}'.format(time_needed))
            if self.client_nav.get_state() == 2:
                rospy.loginfo("Goal cancelled moving to starting position")
                self.goal_return=[{'x' : 0, 'y' : 0, 'Q_w' : 1, 'Q_z':0 }]
                self.movebase(self.goal_return)
                return


if __name__ == '__main__':

    obj = RobotGoals()
    obj.goal_list()
    obj.movebase(obj.goal_points)
