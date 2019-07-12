#!/usr/bin/env python
'''
Author: Guanrui Li lguanrui@seas.upenn.edu
'''

import rosbag
from matplotlib import pyplot as plt
import rospy
import numpy as np
import pdb
import sys

#bag = rosbag.Bag('data20187241447.bag')

bag_lists = sys.argv[1:]
#pdb.set_trace()
kpz = 3.2
m = 0.034
time = 200.000
start_time = 10.00
fig1 = plt.figure(1)
fig1.patch.set_facecolor('white')
fig1.suptitle('Vicon Estimation', fontsize=16)

ax11 = fig1.add_subplot(411)
ax11.set_title('Position Estimation in x')
ax11.set_ylabel('Position x')
ax11.set_ylim(-0.200, 0.400)
ax11.set_xlim(start_time, time)

ax12 = fig1.add_subplot(412)
ax12.set_title('Position Estimation in y')
ax12.set_ylabel('Position y')
ax12.set_ylim(-0.200, 0.400)
ax12.set_xlim(start_time, time)

ax13 = fig1.add_subplot(413)
ax13.set_title('Position Estimation in z')
ax13.set_ylabel('Position z')
ax13.set_ylim(0.000, 0.60)
ax13.set_xlim(start_time, time)

ax14 = fig1.add_subplot(414)
ax14.set_title('Thrust')
ax14.set_ylabel('Thrust')
ax14.set_ylim(0.300, 0.36)
ax14.set_xlim(start_time, time)

color = ['b','g']
i = 0
for bag_name in bag_lists:	
	#pdb.set_trace()
	bag = rosbag.Bag(bag_name)
	modquad1_vicon = []
	rostime_list = []
	modquad2_vicon = []
	whycon_pos = []
	vicon_velocity1 = []
	vision_position_x_list = []
	vision_position_y_list = []
	vision_position_z_list = []
	vision_velocity_x_list = []
	vision_velocity_y_list = []
	vision_velocity_z_list = []

	for topic, position, stamp in bag.read_messages(topics=['/vicon/Modquad2/odom']):
	    time = stamp.secs + stamp.nsecs * 1e-9 
	    vision_position_x_list.append(position.pose.pose.position.x)
	    vision_position_y_list.append(position.pose.pose.position.y)
	    vision_position_z_list.append(position.pose.pose.position.z)
	    #positionx.append(position.x)
	    rostime_list.append(time)

     
	rostime = np.asarray(rostime_list)
	rostime = rostime - rostime_list[0] 
	vision_position_x = np.asarray(vision_position_x_list)
	vision_position_y = np.asarray(vision_position_y_list)
	vision_position_z = np.asarray(vision_position_z_list)
	#ax11.plot(mytime, myvisionx, 'bo',label = 'Vision Estimation')
	ax11.plot(rostime, vision_position_x, color[i],label = 'Vicon Estimation')
	ax12.plot(rostime, vision_position_y, color[i],label = 'Vicon Estimation')
	ax13.plot(rostime, vision_position_z, color[i],label = 'Vicon Estimation')
    #pdb.set_trace()

	thrust_list = []
	thrust_time_list = []

	for topic, thrust, stamp in bag.read_messages(topics=['/mavros2/setpoint_attitude/thrust']):
	    #time = stamp.secs + stamp.nsecs * 1e-9 - 1525405720
	    thrust_time = stamp.secs + stamp.nsecs * 1e-9 
	    thrust_list.append(thrust.thrust)
	    thrust_time_list.append(thrust_time)
	   

	thrust_kp = kpz * (0.5-vision_position_z) * m + m * 9.8 
	thrust_rostime = np.asarray(thrust_time_list)
	thrust_rostime = thrust_rostime - thrust_time_list[0]
	_thrust = np.asarray(thrust_list)
	#pdb.set_trace()
	ax14.plot(thrust_rostime, _thrust, color[i],label = 'thrust')
	#ax14.plot(rostime, thrust_kp, 'r',label = 'thrust_kp')
	#ax13.plot(thrust_rostime, _thrust, 'r',label = 'thrust')
	bag.close()
	i = i+1

plt.show()

#size = numpy.size(positionx)
#print size



#Integral gain for the docking.
