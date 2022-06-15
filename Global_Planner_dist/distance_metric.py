#!/usr/bin/env python

import rospy
import nav_msgs.srv
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def path_distance(path):

    first_time = True
    prev_x = 0.0
    prev_y = 0.0
    total_distance = 0.0
    print(len(path.plan.poses))
    if len(path.plan.poses) > 0:
        for i, vol in enumerate(path.plan.poses):
            #print(i)
            x = path.plan.poses[i].pose.position.x
            y = path.plan.poses[i].pose.position.y
            if not first_time:
                total_distance += math.hypot(prev_x - x, prev_y - y)
            else:
                first_time = False
            prev_x = x
            prev_y = y
        return total_distance

def path_plan(a,b):

    start = PoseStamped()
    start.header.seq = 0
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time(0)
    start.pose.position.x = a[0]
    start.pose.position.y = a[1]

    Goal = PoseStamped()
    Goal.header.seq = 0
    Goal.header.frame_id = "map"
    Goal.header.stamp = rospy.Time(0)
    Goal.pose.position.x = b[0]
    Goal.pose.position.y = b[1]

    service_get_plan = rospy.ServiceProxy('/move_base_node/make_plan', nav_msgs.srv.GetPlan)
    plan_path = nav_msgs.srv.GetPlan()
    plan_path.start = start
    plan_path.goal = Goal
    plan_path.tolerance = 100
    service_plan_path = service_get_plan(plan_path.start, plan_path.goal, plan_path.tolerance)
    #print(service_plan_path)
    distance_to_point = path_distance(service_plan_path)
    #return distance_to_point

if __name__ == "__main__":
    c1=[1,0.77]
    c2 = [3.1, -3.5]
    path_plan(c1,c2)
