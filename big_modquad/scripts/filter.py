#!/usr/bin/env python
'''
Author: Guanrui Li lguanrui@seas.upenn.edu
'''

import rospy
from modquad.srv import *
from modquad.msg import *
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import numpy as np
import numpy.linalg as LA

class kalmanfilter:
    image_updated = False
    mu_init = False
    def __init__(self,):
	self.ip_addr = str(rospy.get_param('ip_addr'))
        rospy.init_node('modquad' + self.ip_addr + 'filter')
        self.pub_and_sub_init()
        self.parameter_init()
      
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            Vision_Odom = VisionOdom()
            Imu = self.Imu
            current_time = Imu.header.stamp.secs + Imu.header.stamp.nsecs * 1e-9 
            self.set_up_system(self.last_Rotation_matrix,current_time)
            input = self.get_input(self.last_Rotation_matrix, self.last_acceleration)
            Prediction = self.Prediction(input)

            if self.image_updated:
                self.set_up_system(self.last_Rotation_matrix,current_time)
                input = self.get_input(self.last_Rotation_matrix, self.last_acceleration)
                mu = self.update(Prediction[0],Prediction[1],self.whycon_position,Imu)
                self.image_updated = False
            else:
                mu = Prediction[0]
                self.last_mu_bar = Prediction[0]
                self.last_Sigma = Prediction[1]

            self.Update_orientation_and_acceleration(Imu)
            Vision_Odom = self.mu_to_Vision_Odom(mu)
            self.state_pub.publish(Vision_Odom)
            rate.sleep()

    def Prediction(self,input):
        mu_bar = self.A * self.last_mu_bar + self.B * input
        Sigma_bar = self.A * self.last_Sigma * self.AT + self.Vt*self.Q1*self.VtT + self.Q2
        return [mu_bar,Sigma_bar]

    def update(self,mu_bar,Sigma_bar,whycon_position,Imu):
        '''
        :param input: gravity + acceleration
        :param measure_position: quad position in the tag frame.
        :return: The filtered position and velocity.
        '''
        Current_image_Rotmatrix = self.get_Rotation_matrix(Imu)
        tag_vector = np.matrix([whycon_position.poses[0].position.x,whycon_position.poses[0].position.y,whycon_position.poses[0].position.z]).transpose()
	measure_position = self.d_tag_quad2-self.R_w_waitmod*Current_image_Rotmatrix*(self.R_cam_quad1*tag_vector+self.d_cam_quad1)

        Kt = Sigma_bar*self.CT*LA.inv(self.C*Sigma_bar*self.CT + self.R)

        mu = mu_bar + Kt*(measure_position - self.C*mu_bar)
        Sigma = Sigma_bar - Kt * self.C * Sigma_bar
        self.last_mu_bar = mu
        self.last_Sigma = Sigma

        return mu

    def mu_to_Vision_Odom(self,mu):
        Vision_Odom = VisionOdom()
        now = rospy.get_rostime()
        Vision_Odom.header.stamp.secs = now.secs
        Vision_Odom.header.stamp.nsecs = now.nsecs
        if self.mu_init:
		Vision_Odom.position.x = mu[0, 0]
		Vision_Odom.position.y = mu[1, 0]
		Vision_Odom.position.z = mu[2, 0]
		Vision_Odom.velocity.x = mu[3, 0]
		Vision_Odom.velocity.y = mu[4, 0]
		Vision_Odom.velocity.z = mu[5, 0]
        else: 
		Vision_Odom.position.x = mu[0, 0]
		Vision_Odom.position.y = mu[1, 0]
		Vision_Odom.position.z = mu[2, 0]
		Vision_Odom.velocity.x = mu[3, 0]
		Vision_Odom.velocity.y = mu[4, 0]
		Vision_Odom.velocity.z = mu[5, 0]
                self.mu_init = True
        self.mu_prev = mu
        Vision_Odom.orientation = self.Imu.orientation
        return Vision_Odom

    def image_detection_cb(self,msg):
        self.whycon_position = msg
        try:
          self.Current_image_Rotmatrix = self.get_Rotation_matrix(self.Imu)
        except AttributeError:
          rospy.logwarn("Waiting for IMU input...")
          return
        self.image_updated = True

    def Imu_cb(self,msg):
        self.Imu = msg

    def dock_side_cb(self,msg):
        if msg.data == "left":
            self.R_w_waitmod = np.matrix([[0, -1, 0],
                                         [1, 0, 0],
                                         [0, 0, 1]]) #left docking matrix
        elif msg.data == "back":
            self.R_w_waitmod = np.identity(3)
        elif msg.data == "right":
            self.R_w_waitmod = np.matrix([[0, 1, 0],
                                         [-1, 0, 0],
                                         [0, 0, 1]]) #right docking matrix
        else:
            rospy.logerr("Improper docking side request!")

    def parameter_init(self,):
        self.x = np.matrix(np.zeros((9,1)))
        self.last_mu_bar = np.matrix(np.zeros((9,1)))

        self.DT = 0.02
        error_cam_x = 0.1132 #5.36698e-05 #0.05
        error_cam_y = 0.1984 #2.91742e-05 #0.001
        error_cam_z = 0.2245 #2.72226e-05 #0.001
        error_pos_x = 0.2176
        error_pos_y = 0.0600
        error_pos_z = 0.2356
        error_vel_x = 0.1059#0.1
        error_vel_y = 0.0822
        error_vel_z = 0.1133 #0.5
        error_accelerometer_x = 0.275#0.120203 #0.001#0.1`
        error_accelerometer_y = 0.550035 #0.463#0.01 #0.09
        error_accelerometer_z = 0.245668 #0.481#0.1 #0.02
        error_bias_accel_x = 0.005#0.05
        error_bias_accel_y = 0.005#0.05  #0.05
        error_bias_accel_z = 0.002#0.02
        bias_accel_x = 0.3
        bias_accel_y = 0.3
        bias_accel_z = 0.1

        self.I = np.identity(9)

        noise_vector = np.matrix([error_pos_x,error_pos_y,error_pos_z,error_vel_x,error_vel_y,error_vel_z,
                                  error_bias_accel_x,error_bias_accel_y,error_bias_accel_z]).transpose()


        self.A = np.matrix([[1, 0, 0, self.DT , 0          ,0      , 0, 0, 0],
                            [0, 1, 0, 0       , self.DT    ,0      , 0, 0, 0],
                            [0, 0, 1, 0       , 0          ,self.DT, 0, 0, 0],
                            [0, 0, 0, 1       , 0          ,0      , 0, 0, 0],
                            [0, 0, 0, 0       , 1          ,0      , 0, 0, 0],
                            [0, 0, 0, 0       , 0          ,1      , 0, 0, 0],
                            [0, 0, 0, 0       , 0          ,0      , 1, 0, 0],
                            [0, 0, 0, 0       , 0          ,0      , 0, 1, 0],
                            [0, 0, 0, 0       , 0          ,0      , 0, 0, 1]])


        self.B = np.matrix([[0, 0, 0],
                            [0, 0, 0],
                            [0, 0, 0],
                            [self.DT, 0          , 0          ],
                            [0          , self.DT, 0          ],
                            [0          , 0          , self.DT],
                            [0, 0, 0],
                            [0, 0, 0],
                            [0, 0, 0]])

        self.C = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0, 0, 0, 0]])

        self.CT = self.C.transpose()

        self.k = np.matrix([[0.95], [0.95], [0.95], [0.95], [0.95], [0.95], [0.0], [0.0], [0.0]])

        self.Q1 = np.matrix([[error_accelerometer_x,0,0],
                            [0,error_accelerometer_y,0],
                            [0,0,error_accelerometer_z]])

        self.Q2 = self.I * noise_vector

        self.last_Sigma = np.matrix(np.zeros((9,9)))

        self.R_w_waitmod = np.identity(3) #back docking matrix 

        self.R = np.matrix([[error_cam_x, 0, 0],
                           [0, error_cam_y, 0],
                           [0, 0, error_cam_z]])

        self.gravity = np.matrix([[0],[0],[9.81]])

        self.Rot_z_180 = np.matrix([[-1.0, 0, 0],
                            [0, -1.0, 0],
                            [0, 0, 1.0]])
	self.d_cam_quad1 = np.matrix([0.1083,-0.00426,0.04344]).transpose()
	self.d_tag_quad2 = np.matrix([-0.149,-0.011,0.04787]).transpose()
	#FOR LEFT TAG DETECTION, REPLACE self.d_tag_quad2 WITH THE FOLLOWING:
	#self.d_tag_quad2 = np.matrix([0.0000,0.149,0.04787]).transpose()
        self.R_cam_quad1 = np.matrix([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])

        self.Vt = np.matrix(np.zeros((9, 3)))
        while True:
            if self.image_updated:
                tag_vector = np.matrix([self.whycon_position.poses[0].position.x, self.whycon_position.poses[0].position.y,
                                        self.whycon_position.poses[0].position.z]).transpose()
                measure_position = self.R_cam_quad1 * tag_vector
                self.last_mu_bar[0:3,0] =  measure_position
                self.last_mu_bar[6: ,0] =  np.matrix([bias_accel_x,bias_accel_y,bias_accel_z]).transpose()
                self.Update_orientation_and_acceleration(self.Imu)
                break
            #else:
                #print "Whycon tag is not detected, initializing filter."


    def pub_and_sub_init(self,):
        self.state_pub = rospy.Publisher('/modquad' + self.ip_addr + '/filtered_Vision_Odom', VisionOdom, queue_size=10)
        rospy.Subscriber('/modquad' + self.ip_addr + '/whycon' + self.ip_addr + '/poses', PoseArray, callback=self.image_detection_cb)
        rospy.Subscriber('/modquad' + self.ip_addr + '/mavros' + self.ip_addr + '/imu/data', Imu, callback=self.Imu_cb)
        rospy.Subscriber('/modquad' + self.ip_addr + '/dock_side', String, callback=self.dock_side_cb)

    def get_Rotation_matrix(self,Imu):
        curr_R = np.matrix(
            quaternion_matrix([Imu.orientation.x, Imu.orientation.y, Imu.orientation.z, Imu.orientation.w]))
        curr_R = curr_R[0:3, 0:3]
        return curr_R

    def get_input(self,R,acceleration):
        input = self.R_w_waitmod*(-self.gravity + R * acceleration)
        return input

    def set_up_system(self,R,current_time):
        self.DT = current_time - self.last_time 
        self.A[3:6, 6:] = -self.DT * self.R_w_waitmod * R
        self.AT = self.A.transpose()
        self.Vt[3:6, :] = -self.R_w_waitmod*R
        self.VtT = self.Vt.transpose()

    def Update_orientation_and_acceleration(self,Imu):
        self.last_time = Imu.header.stamp.secs + Imu.header.stamp.nsecs * 1e-9
        self.last_Rotation_matrix = self.get_Rotation_matrix(Imu)
        self.last_acceleration = np.matrix(
            [[Imu.linear_acceleration.x], [Imu.linear_acceleration.y], [Imu.linear_acceleration.z]])

if __name__ == "__main__":
    kalmanfilter()
