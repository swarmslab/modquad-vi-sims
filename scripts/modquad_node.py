#!/usr/bin/env python

import rospy
from modquad.srv import *
from modquad.msg import *
from modquad_init import modquad
from geometry_msgs.msg import Vector3,PoseStamped
from mavros_msgs.msg import AttitudeTarget, Thrust, State
import numpy as np

def waypoint(msg):
    des_waypoint.x = msg.x
    des_waypoint.y = msg.y
    des_waypoint.z = msg.z
    des_waypoint.updated = True

def pose_cb(msg):
    curr_pose = msg

def vel_cb(msg):
    curr_vel = msg

if __name__ == "__main__":
    rospy.init_node('modquad1_traj_generator')
    rospy.Subscriber('/modquad1/waypoint', Vector3, callback=waypoint)
    rospy.Subscriber('/mavros1/local_position/pose', PoseStamped, callback=pose_cb)

    modquad_func_lib = modquad()
    des_waypoint = Waypoint()
    start_point = Waypoint()
    curr_pose = PoseStamped()
    time_init = 0.0
    yaw_des = 0.0
    rate = rospy.Rate(100)  # Hz
    rate.sleep()
    while not rospy.is_shutdown():

        if des_waypoint.updated:
            start_point.x = curr_pose.pose.position.x
            start_point.y = curr_pose.pose.position.y
            start_point.z = curr_pose.pose.position.z
            start_point.updated = False
            print start_point
            modquad_func_lib.two_pts_trajectory_init(start_point,des_waypoint)
            des_waypoint.updated = False
            time_init = rospy.get_time()

        t = rospy.get_time() - time_init
        curr_pose
        des_trajectory_point = modquad_func_lib.two_pts_trajectory_generator(t,des_waypoint)
        modquad_func_lib.pos_control(des_trajectory_point, curr_pose, curr_vel, yaw_des)

        rate.sleep()
