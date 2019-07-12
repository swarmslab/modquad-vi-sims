#!/usr/bin/env python
"""
Author: Jnaneshwar Das <jnaneshwar.das@gmail.com> 
testing looping behavior across a set of waypoints using offboard control
"""
import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray
import math
import numpy as np
import sys
import tf
import pdb


class TestLoop:
    curr_pose = PoseStamped()
    waypointIndex = 0
    distThreshold = 0.01
    sim_ctr = 1
    des_pose = PoseStamped()
    tag_orientation = Quaternion()
    isReadyToFly = False

    def __init__(self, X,Y,Z,uav_prefix,yaw):
        print yaw
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
 #       self.locations = numpy.matrix([[H, H, V, q[0], q[1], q[2], q[3]],
 #                                      [-H, H, V, q[0], q[1], q[2], q[3]],
 #                                      [-H, -H, V, q[0], q[1], q[2], q[3]],
 #                                      [H, -H, V, q[0], q[1], q[2], q[3]],
 #                                      ])
        self.locations = np.matrix([[X, Y, Z, q[0], q[1], q[2], q[3]]])
 #                                      [-H, 0, V, q[0], q[1], q[2], q[3]]])
        print self.locations
        print '/mavros'+ uav_prefix + '/setpoint_position/local'	
        rospy.init_node('offboard_test', anonymous=True)
        pose_pub = rospy.Publisher('/mavros'+ uav_prefix + '/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.Subscriber('/mavros'+ uav_prefix + '/local_position/pose', PoseStamped, callback=self.mocap_cb)
        rospy.Subscriber('/mavros'+ uav_prefix + '/state', State, callback=self.state_cb)
        rospy.Subscriber('/whycon/poses', PoseArray, callback=self.tag_cb)

        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_pose)
        shape = self.locations.shape

        while not rospy.is_shutdown():
            print self.sim_ctr, shape[0], self.waypointIndex
            if self.waypointIndex is shape[0]:
                self.waypointIndex = 0
                self.sim_ctr += 1

            if self.isReadyToFly:
                des_x = self.locations[self.waypointIndex, 0]
                des_y = self.locations[self.waypointIndex, 1]
                des_z = self.locations[self.waypointIndex, 2]
                self.des_pose.pose.position.x = self.locations[self.waypointIndex, 0]
                self.des_pose.pose.position.y = self.locations[self.waypointIndex, 1]
                self.des_pose.pose.position.z = self.locations[self.waypointIndex, 2]
                self.des_pose.pose.orientation.x = self.locations[self.waypointIndex, 3]
                self.des_pose.pose.orientation.y = self.locations[self.waypointIndex, 4]
                self.des_pose.pose.orientation.z = self.locations[self.waypointIndex, 5]
                self.des_pose.pose.orientation.w = self.locations[self.waypointIndex, 6]

                curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z
                
                dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
                if dist < self.distThreshold:
                    self.waypointIndex += 1

            pose_pub.publish(self.des_pose)
            #pdb.set_trace()
            rate.sleep()
 
    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def mocap_cb(self, msg):
        # print msg
        self.curr_pose = msg

    def tag_cb(self, msg):
        # print msg
        self.tag_orientation = msg.poses[0].orientation
        self.euler_angle = tf.transformations.euler_from_quaternion([self.tag_orientation.x,self.tag_orientation.y,self.tag_orientation.z,self.tag_orientation.w], axes='szyx')
        self.Rot = tf.transformations.euler_matrix(0.0,self.euler_angle[1],self.euler_angle[2],'szyx')
        yaw = np.arctan2(-self.Rot[2][0],self.Rot[0][0])*180/np.pi
        yaw1 = self.euler_angle[2]*180/np.pi
        #print 'yaw1 %5.3f' % yaw
        #print 'yaw2 %5.3f' % yaw1
    def state_cb(self,msg):
        #print msg.mode
        if(msg.mode=='OFFBOARD'):
            self.isReadyToFly = True
            print "readyToFly"

if __name__ == "__main__":
     TestLoop(float(-1.7),float(0),float(0.2), str(1), float(0))
#    TestLoop(float(sys.argv[1]), float(sys.argv[2]), sys.argv[3], float(sys.argv[4]))
