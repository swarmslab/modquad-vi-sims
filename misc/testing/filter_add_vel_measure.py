#!/usr/bin/env python
'''
Author: Guanrui Li lguanrui@seas.upenn.edu
'''

import rospy
from modquad.srv import *
from modquad.msg import *
from tf.transformations import *
from geometry_msgs.msg import Vector3, PoseStamped, TwistStamped, PoseArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from mavros_msgs.msg import AttitudeTarget, Thrust, State
import numpy as np
import pdb
import tf
import numpy.linalg as LA


class kalmanfilter:

    image_updated = False
    last_measure_time = 0
    #Imu = Imu()
    def __init__(self,):
        rospy.init_node('modquad1filter')
        self.pub_and_sub_init()
        self.parameter_init()


        rate = rospy.Rate(100)  # Hz
        rate.sleep()

        while not rospy.is_shutdown():
            Vision_Odom = VisionOdom()
            self.set_up_system(self.last_Rotation_matrix)
            input = self.get_input(self.last_Rotation_matrix, self.last_acceleration)
            Prediction = self.Prediction(input)

            if self.image_updated:

                #print input
                mu = self.update(Prediction[0],Prediction[1],self.whycon_position)
                self.image_updated = False

            else:
                mu = Prediction[0]
                self.last_mu_bar = Prediction[0]
                self.last_Sigma = Prediction[1]

            self.Update_orientation_and_acceleration(self.Imu)
            Vision_Odom = self.mu_to_Vision_Odom(mu)
            #rospy.loginfo(mu)
            self.state_pub.publish(Vision_Odom)
            rate.sleep()

    def Prediction(self,input):
        mu_bar = self.A * self.last_mu_bar + self.B * input

        Sigma_bar = self.A * self.last_Sigma * self.AT + self.Vt*self.Q1*self.VtT + self.Q2
        return [mu_bar,Sigma_bar]


    def update(self,mu_bar,Sigma_bar,whycon_position):
        #mu_bar =


        '''
        :param input: gravity + acceleration
        :param measure_position: quad position in the tag frame.
        :return: The filtered position and velocity.
        '''
	measure_time = whycon_position.header.stamp.secs + whycon_position.header.stamp.nsecs* 1e-9

        if self.last_measure_time==0:

		tag_vector = np.matrix([whycon_position.poses[0].position.x,whycon_position.poses[0].position.y,whycon_position.poses[0].position.z]).transpose()
        	measure_position = self.d_tag_quad2-self.Current_image_Rotmatrix*(self.R_cam_quad1*tag_vector+self.d_cam_quad1)
	 
        	Kt = Sigma_bar*self.CT*LA.inv(self.C*Sigma_bar*self.CT + self.R)

	        mu = mu_bar + Kt*(measure_position - self.C*mu_bar)
        	Sigma = Sigma_bar - Kt * self.C * Sigma_bar
		print "I went in here first"
        else:
		tag_vector = np.matrix([whycon_position.poses[0].position.x,whycon_position.poses[0].position.y,whycon_position.poses[0].position.z]).transpose()
                measure_position = self.d_tag_quad2-self.Current_image_Rotmatrix*(self.R_cam_quad1*tag_vector+self.d_cam_quad1)
                measure_velocity = (measure_position-self.last_measure_position)/(measure_time-self.last_measure_time)
		
		measure_state = np.vstack((measure_position,measure_velocity))
		#pdb.set_trace()
                Kt = Sigma_bar*self.C_velT*LA.inv(self.C_vel*Sigma_bar*self.C_velT + self.R_vel)

                mu = mu_bar + Kt*(measure_state - self.C_vel*mu_bar)
                Sigma = Sigma_bar - Kt * self.C_vel * Sigma_bar
		print "I am measuring velocity now"
		print measure_velocity

	self.last_mu_bar = mu
        self.last_Sigma = Sigma
	self.last_measure_time = measure_time
	self.last_measure_position = measure_position
	self.last_tag_vector = tag_vector

        return mu

    def mu_to_Vision_Odom(self,mu):
        Vision_Odom = VisionOdom()
        now = rospy.get_rostime()
        Vision_Odom.header.stamp.secs = now.secs
        Vision_Odom.header.stamp.nsecs = now.nsecs
        Vision_Odom.position.x = mu[0, 0]
        Vision_Odom.position.y = mu[1, 0]
        Vision_Odom.position.z = mu[2, 0]
        Vision_Odom.velocity.x = mu[3, 0]
        Vision_Odom.velocity.y = mu[4, 0]
        Vision_Odom.velocity.z = mu[5, 0]
        Vision_Odom.orientation = self.Imu.orientation
        return Vision_Odom
        #print mu



    def image_detection_cb(self,msg):
	#self.last_whycon_position = self.whycon_position
        self.whycon_position = msg
        self.Current_image_Rotmatrix = self.get_Rotation_matrix(self.Imu)
        self.image_updated = True

    def Imu_cb(self,msg):
        self.Imu = msg

    def parameter_init(self,):
        self.x = np.matrix(np.zeros((9,1)))
        self.last_mu_bar = np.matrix(np.zeros((9,1)))

        self.DT = 0.01
        error_cam_x = 0.05
        error_cam_y = 0.001 #0.01
        error_cam_z = 0.001 #0.01
	error_cam_vel_x = 1.0 #0.001
	error_cam_vel_y = 0.001
	error_cam_vel_z = 0.001
        error_pos_x = 0.01
        error_pos_y = 0.01
        error_pos_z = 0.01
        error_vel_x = 0.1
        error_vel_y = 0.1
        error_vel_z = 0.1 #0.5
        error_accelerometer_x = 0.02 #0.01
        error_accelerometer_y = 0.09 #0.01
        error_accelerometer_z = 0.02
        error_bias_accel_x = 0.05
        error_bias_accel_y = 0.05  #0.05
        error_bias_accel_z = 0.02
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

	self.C_vel = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                           	[0, 1, 0, 0, 0, 0, 0, 0, 0],
                           	[0, 0, 1, 0, 0, 0, 0, 0, 0],
				[0, 0, 0, 1, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 1, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 1, 0, 0, 0]])

	self.C_velT = self.C_vel.transpose()

        self.Q1 = np.matrix([[error_accelerometer_x,0,0],
                            [0,error_accelerometer_y,0],
                            [0,0,error_accelerometer_z]])

        self.Q2 = self.I * noise_vector

        self.last_Sigma = np.matrix(np.zeros((9,9)))


        self.R = np.matrix([[error_cam_x, 0, 0],
                           [0, error_cam_y, 0],
                           [0, 0, error_cam_z]])

	self.R_vel = np.matrix([[error_cam_x, 0, 0, 0, 0, 0],
                           	[0, error_cam_y, 0, 0, 0, 0],
                           	[0, 0, error_cam_z, 0, 0, 0],
				[0, 0, 0, error_cam_vel_x, 0, 0],
				[0, 0, 0, 0, error_cam_vel_y, 0],
				[0, 0, 0, 0, 0, error_cam_vel_z]])

        self.gravity = np.matrix([[0],[0],[9.81]])

        self.Rot_z_180 = np.matrix([[-1.0, 0, 0],
                            [0, -1.0, 0],
                            [0, 0, 1.0]])
	self.d_cam_quad1 = np.matrix([0.0905,0.0,0.066]).transpose()
	self.d_tag_quad2 = np.matrix([-0.020,0.0,0.135]).transpose()
        #self.R_cam_quad1 = np.matrix([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]) # Actual camera
        self.R_cam_quad1 = np.matrix([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])  # Simulation

        self.Vt = np.matrix(np.zeros((9, 3)))

        while True:
            if self.image_updated:
                tag_vector = np.matrix([self.whycon_position.poses[0].position.x, self.whycon_position.poses[0].position.y,
                                        self.whycon_position.poses[0].position.z]).transpose()
                measure_position = self.R_cam_quad1 * tag_vector
                self.last_mu_bar[0:3,0] =  measure_position
                self.last_mu_bar[6: ,0] = np.matrix([bias_accel_x,bias_accel_y,bias_accel_z]).transpose()
                self.Update_orientation_and_acceleration(self.Imu)
                break
            #else:
                #print "Whycon tag is not detected, initializing filter."


    def pub_and_sub_init(self,):
        self.state_pub = rospy.Publisher('/modquad1/filtered_Vision_Odom', VisionOdom, queue_size=10)
        rospy.Subscriber('/whycon/poses', PoseArray, callback=self.image_detection_cb)
        rospy.Subscriber('/mavros1/imu/data', Imu, callback=self.Imu_cb)


    def get_Rotation_matrix(self,Imu):
        curr_R = np.matrix(
            quaternion_matrix([Imu.orientation.x, Imu.orientation.y, Imu.orientation.z, Imu.orientation.w]))
        curr_R = curr_R[0:3, 0:3]
        return curr_R

    def get_input(self,R,acceleration):
        input = -self.gravity + R * acceleration
        return input

    def set_up_system(self,R):
        self.A[3:6, 6:] = -self.DT * R
        self.AT = self.A.transpose()
        self.Vt[3:6, :] = -R
        self.VtT = self.Vt.transpose()

    def Update_orientation_and_acceleration(self,Imu):
        self.last_Rotation_matrix = self.get_Rotation_matrix(Imu)
        self.last_acceleration = np.matrix(
            [[Imu.linear_acceleration.x], [Imu.linear_acceleration.y], [Imu.linear_acceleration.z]])




if __name__ == "__main__":
    kalmanfilter()
