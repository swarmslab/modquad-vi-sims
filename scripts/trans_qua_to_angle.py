#!/usr/bin/env python
'''
Author: Guanrui Li lguanrui@seas.upenn.edu
'''

import rosbag
import rospy
import datetime
from modquad.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, PoseStamped, TwistStamped, PoseArray
from sensor_msgs.msg import Imu
import tf.transformations as ang_tf


class record:

	odom1 = Odometry()
	odom2 = Odometry()

	def __init__(self, ):

		rospy.init_node('trans_angle')
		rospy.Subscriber('/mavros2/imu/data', Imu, callback=self.Imu_cb)
		rospy.Subscriber('/vicon/Modquad2/odom',Odometry,callback = self.Odom_cb)
		self.mocap_euler_pub = rospy.Publisher('/mocap_Euler_angle',Vector3,queue_size = 10)
		self.euler_pub = rospy.Publisher('/Euler_angle',Vector3,queue_size = 10)



		while True:
			if not rospy.is_shutdown():
				try:
					a = 1
				except:
					pass

			else:
				self.bag.close()
				break

	def Odom_cb(self,msg):
		euler_angle = Vector3()
		imu_euler_angle = Vector3()
		quaternion = msg.pose.pose.orientation
		angles = ang_tf.euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])
		imu_angles = ang_tf.euler_from_quaternion([self.Imu.orientation.x,self.Imu.orientation.y,self.Imu.orientation.z,self.Imu.orientation.w])
		euler_angle.x = angles[0]
		euler_angle.y = angles[1]
		euler_angle.z = angles[2]
		imu_euler_angle.x = imu_angles[0]
		imu_euler_angle.y = imu_angles[1]
		imu_euler_angle.z = imu_angles[2]
		self.mocap_euler_pub.publish(euler_angle)
		self.euler_pub.publish(imu_euler_angle)


	def Imu_cb(self,msg):
		self.Imu = msg
		

if __name__ == "__main__":
    record()
