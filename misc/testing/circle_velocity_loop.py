#!/usr/bin/env python
"""
Author: Jnaneshwar Das <jnaneshwar.das@gmail.com> 
testing looping behavior across a set of waypoints using offboard control
"""
import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TwistStamped, Vector3, Vector3Stamped
import math
import numpy as np
import sys
import tf
import pdb
from numpy import linalg as LA


class TestLoop:
    curr_pose = PoseStamped()
    curr_vel = TwistStamped()
    waypointIndex = 0
    distThreshold = 0.01
    sim_ctr = 1
    des_pose = Vector3()
    des_vel = Vector3()
    des_acc = Vector3()
    Kp = Vector3()
    Kd = Vector3()
    accel_setpoint = Vector3Stamped()
    isReadyToFly = False

    def __init__(self, radius,height,uav_prefix,yaw):
        print yaw
        self.Kp_z = 1.0
        self.n = 0.0;
        self.Radius = radius
        self.des_z = height
        self.z_initial = np.matrix([[0,1,0,0,0,0]]).transpose()
        self.theta_initial = np.matrix([[0,2*np.pi,0,0,0,0]]).transpose()
        
        self.t0 = 0
        self.tf = 0.1
        self.Kp.x = 0.4
	self.Kp.y = 0.4
	self.Kp.z = 5.0
        self.Kd.x = 0.05
	self.Kd.y = 0.05
	self.Kd.z = 3.0    
        self.m = 1.5
        self.g = 9.81



        self.A = np.matrix([[1, self.t0, self.t0**2, self.t0**3, self.t0**4, self.t0**5],
                            [1, self.tf, self.tf**2, self.tf**3, self.tf**4, self.tf**5],
                            [0, 1,       2*self.t0,  3*self.t0**2, 4*self.t0**3, 5*self.t0**4],
                            [0, 1,       2*self.tf,  3*self.tf**2, 4*self.tf**3, 5*self.tf**4],
                            [0, 0,       2,          6*self.t0,  12*self.t0**2, 20*self.t0**3],
                            [0, 0,       2,          6*self.tf,  12*self.tf**2, 20*self.tf**3]])
        self.zcoeff = np.linalg.inv(self.A)*self.z_initial
        self.thetacoeff = np.linalg.inv(self.A)*self.theta_initial
        

        q = tf.transformations.quaternion_from_euler(0, 0, yaw)

        print '/mavros'+ uav_prefix + '/setpoint_position/local'	
        rospy.init_node('offboard_test', anonymous=True)
        #pose_pub = rospy.Publisher('/mavros'+ uav_prefix + '/setpoint_position/local', PoseStamped, queue_size=10)
        #vel_pub = rospy.Publisher('/mavros'+ uav_prefix + '/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        acc_pub = rospy.Publisher('/mavros'+ uav_prefix + '/setpoint_accel/accel', Vector3Stamped, queue_size=10)
        rospy.Subscriber('/mavros'+ uav_prefix + '/local_position/pose', PoseStamped, callback=self.mocap_cb)
        rospy.Subscriber('/mavros'+ uav_prefix + '/local_position/velocity', TwistStamped, callback=self.vel_cb)
        rospy.Subscriber('/mavros'+ uav_prefix + '/state', State, callback=self.state_cb)

        rate = rospy.Rate(100)  # Hz
        rate.sleep()
        #self.des_pose = self.copy_pose(self.curr_pose)
        self.t_init = rospy.get_time()
        

        while not rospy.is_shutdown():

            self.trajectory_generator()
        
            if self.isReadyToFly:
                
            	self.accel_setpoint.vector.x = self.des_acc.x + self.Kp.x*(self.des_pose.x-self.curr_pose.pose.position.x)+self.Kd.x*(self.des_vel.x-self.curr_vel.twist.linear.x)
            	self.accel_setpoint.vector.y = self.des_acc.y + self.Kp.y*(self.des_pose.y-self.curr_pose.pose.position.y)+self.Kd.y*(self.des_vel.y-self.curr_vel.twist.linear.y)
            	self.accel_setpoint.vector.z = self.des_acc.z + self.Kp.z*(self.des_pose.z-self.curr_pose.pose.position.z)+self.Kd.z*(self.des_vel.z-self.curr_vel.twist.linear.z)
            #pdb.set_trace()	

                Rot_des = np.matrix(np.zeros((3,3)))
                curr_quad = [self.curr_pose.pose.orientation.x,self.curr_pose.pose.orientation.y,self.curr_pose.pose.orientation.z,self.curr_pose.pose.orientation.w]
                H_curr = tf.transformations.quaternion_matrix(curr_quad)
                Rot_curr = np.matrix(H_curr[:3,:3])
                Force_des = np.matrix([[0], [0], [self.m*self.g]]) + self.m*np.matrix([[self.accel_setpoint.vector.x],[self.accel_setpoint.vector.y],[self.accel_setpoint.vector.z]])
                Force_des_body = Rot_curr*Force_des
                thrust = Force_des_body[2]
                

                print str(thrust)
                
            #
            acc_pub.publish(self.accel_setpoint)
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

    def vel_cb(self, msg):
        # print msg
        self.curr_vel = msg

    def state_cb(self,msg):
        print msg.mode
        if(msg.mode=='OFFBOARD'):
            self.isReadyToFly = True
            print "readyToFly"

    def trajectory_generator(self,):
        self.t = rospy.get_time() - self.t_init
        #print 'time now = ' + str(self.t)
        if self.t < self.tf:
           polynominal = np.matrix([[1,self.t,self.t**2,self.t**3,self.t**4,self.t**5],
                                    [0,1,2*self.t,3*self.t**2,4*self.t**3,5*self.t**4],
                                    [0,0,2,6*self.t,12*self.t**2,20*self.t**3]])
           theta_d = polynominal*self.thetacoeff
           z_d = polynominal*self.zcoeff
           
           self.des_pose.x = -self.Radius + self.Radius * np.cos(theta_d[0])
           self.des_pose.y = self.Radius * np.sin(theta_d[0])
           self.des_pose.z = z_d[0]
           self.des_vel.x = -self.Radius * np.sin(theta_d[0]) * theta_d[1]
           self.des_vel.y =  self.Radius * np.cos(theta_d[0]) * theta_d[1]
           self.des_vel.z = z_d[1]
           self.des_acc.x = -self.Radius * np.cos(theta_d[0]) * theta_d[1]**2 - self.Radius * np.sin(theta_d[0]) * theta_d[2]
           self.des_acc.y = -self.Radius * np.sin(theta_d[0]) * theta_d[1]**2 + self.Radius * np.cos(theta_d[0]) * theta_d[2]
           self.des_acc.z = z_d[2]
           
        else:

            self.des_pose.x = 0
            self.des_pose.y = 0
            self.des_pose.z = 1
            self.des_vel.x = 0
            self.des_vel.y = 0
            self.des_vel.z = 0
            self.des_acc.x = 0
            self.des_acc.y = 0
            self.des_acc.z = 0
           
        #print 'desired position in x = ' + str(self.des_pose.z)
        #TO DO 

if __name__ == "__main__":
     TestLoop(float(5), float(1), str(1), float(0))
#    TestLoop(float(sys.argv[1]), float(sys.argv[2]), sys.argv[3], float(sys.argv[4]))

