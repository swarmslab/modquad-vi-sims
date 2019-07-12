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
import math 
from tf.transformations import euler_from_quaternion as qua2eu 
from modquad_init import modquad 




#bag = rosbag.Bag('data20187241447.bag')
bag_lists = sys.argv[1:]
#bag_name = sys.argv[1]
#pdb.set_trace()

fig1 = plt.figure(1)
fig1.patch.set_facecolor('white')
fig1.suptitle('Modquad1 velocity in inertia space', fontsize=16)

ax11 = fig1.add_subplot(311)
ax11.set_title('Velocity Estimation in x')
ax11.set_ylabel('Position x')
ax11.set_ylim(-90, 90)

ax12 = fig1.add_subplot(312)
ax12.set_title('Velocity Estimation in y')
ax12.set_ylabel('Position y')
ax12.set_ylim(-0.5, 0.5)

ax13 = fig1.add_subplot(313)
ax13.set_title('Velocity Estimation in z')
ax13.set_ylabel('Position z')
ax13.set_ylim(-0.5, 0.5)

fig2 = plt.figure(2)
fig2.patch.set_facecolor('white')
fig2.suptitle('Modquad2 velocity in inertia space', fontsize=16)

ax21 = fig2.add_subplot(311)
ax21.set_title('Velocity Estimation in x')
ax21.set_ylabel('Velocity x')
ax21.set_ylim(-0.5, 0.5)

ax22 = fig2.add_subplot(312)
ax22.set_title('Velocity Estimation in y')
ax22.set_ylabel('Velocity y')
ax22.set_ylim(-0.5, 0.5)

ax23 = fig2.add_subplot(313)
ax23.set_title('Velocity Estimation in z')
ax23.set_ylabel('Velocity z')
ax23.set_ylim(-0.5, 0.5)

fig3 = plt.figure(3)
fig3.patch.set_facecolor('white')
fig3.suptitle('Modquad1 position in inertia space', fontsize=16)

ax31 = fig3.add_subplot(311)
ax31.set_title('x')
ax31.set_ylabel('Position x')
ax31.set_ylim(-0.5, 1.0)

ax32 = fig3.add_subplot(312)
ax32.set_title('y')
ax32.set_ylabel('Position y')
ax32.set_ylim(-0.5, 1.0)

ax33 = fig3.add_subplot(313)
ax33.set_title('z')
ax33.set_ylabel('Position z')
ax33.set_ylim(0.0, 1.0)

fig4 = plt.figure(4)
fig4.patch.set_facecolor('white')
fig4.suptitle('Modquad2 position in inertia space', fontsize=16)

ax41 = fig4.add_subplot(311)
ax41.set_title('x')
ax41.set_ylabel('Position x')
ax41.set_ylim(-0.5, 1.0)

ax42 = fig4.add_subplot(312)
ax42.set_title('y')
ax42.set_ylabel('Position y')
ax42.set_ylim(-0.5, 1.0)

ax43 = fig4.add_subplot(313)
ax43.set_title('z')
ax43.set_ylabel('Position z')
ax43.set_ylim(0.0, 1.0)


line = ['-','--']
i = 0
for bag_name in bag_lists:
	modquad1_vicon = []
	rostime_list = []
	viconrostime1_list = []
	viconrostime2_list = []
	modquad2_vicon = []
	whycon_pos = []
	vicon_velocity1 = []
	vision_position_x_list = []
	vision_position_y_list = []
	vision_position_z_list = []
	vision_velocity_x_list = []
	vision_velocity_y_list = []
	vision_velocity_z_list = []
	vicon_velocityx_1list = []
	vicon_velocityy_1list = []
	vicon_velocityz_1list = []
	vicon_positionx_1list = []
	vicon_positiony_1list = []
	vicon_positionz_1list = []
	vicon_positionx_2list = []
	vicon_positiony_2list = []
	vicon_positionz_2list = []
	vicon_velocityx_2list = []
	vicon_velocityy_2list = []   
	vicon_velocityz_2list = []
	delta_pos_x_list = []
	delta_pos_y_list = []
	delta_pos_z_list = []
	delta_vel_x_list = []
	delta_vel_y_list = []
	delta_vel_z_list = []
	vicon_positionx_list = []
	vicon_positiony_list = []
	vicon_positionz_list = []
        hor_lin1 = []
        hor_lin2 = []
	acc_x_list = []
	imu_time_list = []
        yaw_list = []
	bag = rosbag.Bag(bag_name)


	for topic, odom1, stamp in bag.read_messages(topics=['/vicon/Modquad1/odom']):
	    vicontime1 = stamp.secs + stamp.nsecs * 1e-9
	    modquad1_vicon.append(odom1)
	    vicon_velocityx_1list.append(odom1.twist.twist.linear.x)
	    vicon_velocityy_1list.append(odom1.twist.twist.linear.y)
	    vicon_velocityz_1list.append(odom1.twist.twist.linear.z)
	    vicon_positionx_1list.append(odom1.pose.pose.position.x)
	    vicon_positiony_1list.append(odom1.pose.pose.position.y)
	    vicon_positionz_1list.append(odom1.pose.pose.position.z)
	    viconrostime1_list.append(vicontime1)     
            hor_lin1.append(1.0) 
            hor_lin2.append(0.9) 
        #pdb.set_trace()
	viconrostime1 = np.asarray(viconrostime1_list)
	viconrostime1 = viconrostime1 - viconrostime1_list[0]

	for topic, odom2, stamp in bag.read_messages(topics=['/vicon/Modquad2/odom']):
	    vicontime2 = stamp.secs + stamp.nsecs * 1e-9
	    viconrostime2_list.append(vicontime2)     
	    modquad2_vicon.append(odom2)
	    vicon_velocityx_2list.append(odom2.twist.twist.linear.x)
	    vicon_velocityy_2list.append(odom2.twist.twist.linear.y)
	    vicon_velocityz_2list.append(odom2.twist.twist.linear.z)
	    vicon_positionx_2list.append(odom2.pose.pose.position.x)
	    vicon_positiony_2list.append(odom2.pose.pose.position.y)
	    vicon_positionz_2list.append(odom2.pose.pose.position.z)
        #pdb.set_trace()
	viconrostime2 = np.asarray(viconrostime2_list)
	viconrostime2 = viconrostime2 - viconrostime2_list[0]

	for topic, imu, stamp in bag.read_messages(topics=['/mavros1/imu/data']):
	    #time = stamp.secs + stamp.nsecs * 1e-9 - 1525405720
	    imu_time = stamp.secs + stamp.nsecs * 1e-9 
	    acc_x_list.append(imu.linear_acceleration.x)
	    imu_time_list.append(imu_time)
            euler_angle = qua2eu([imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w],'sxyz')
            yaw = yaw_list.append(180*euler_angle[2]/math.pi)

	yaw = np.asarray(yaw_list)
        imu_time = np.asarray(imu_time_list)

	vicon_velocityx_1 = np.asarray(vicon_velocityx_1list)
	vicon_velocityy_1 = np.asarray(vicon_velocityy_1list)
	vicon_velocityz_1 = np.asarray(vicon_velocityz_1list)
	vicon_positionx_1 = np.asarray(vicon_positionx_1list)
	vicon_positiony_1 = np.asarray(vicon_positiony_1list)
	vicon_positionz_1 = np.asarray(vicon_positionz_1list)

	vicon_velocityx_2 = np.asarray(vicon_velocityx_2list)
	vicon_velocityy_2 = np.asarray(vicon_velocityy_2list)
	vicon_velocityz_2 = np.asarray(vicon_velocityz_2list)
	vicon_positionx_2 = np.asarray(vicon_positionx_2list)
	vicon_positiony_2 = np.asarray(vicon_positiony_2list)
	vicon_positionz_2 = np.asarray(vicon_positionz_2list)

	#ax11.plot(viconrostime1, vicon_velocityx_1, 'b'+line[i])
	#ax12.plot(viconrostime1, vicon_velocityy_1, 'b'+line[i])
	#ax13.plot(viconrostime1, vicon_velocityz_1, 'b'+line[i]) 
	#ax21.plot(viconrostime2, vicon_velocityx_2, 'b'+line[i])
	#ax22.plot(viconrostime2, vicon_velocityy_2, 'b'+line[i])
	#ax23.plot(viconrostime2, vicon_velocityz_2, 'b'+line[i]) 
	#ax31.plot(viconrostime1, vicon_positionx_1, 'b'+line[i])
	#ax32.plot(viconrostime1, vicon_positiony_1, 'b'+line[i])
	#ax33.plot(viconrostime1, vicon_positionz_1, 'b'+line[i]) 
	#ax41.plot(viconrostime2, vicon_positionx_2, 'b'+line[i])
	#ax42.plot(viconrostime2, vicon_positiony_2, 'b'+line[i])
	#ax43.plot(viconrostime2, vicon_positionz_2, 'b'+line[i]) 
	#ax33.plot(viconrostime1, hor_lin2, 'k') 
	ax11.plot(imu_time, yaw, 'k')

        i = i+1		
'''
    plt.suptitle('Position Estimation', fontsize=16)
    
    plt.figure(1)
    
    plt.subplot(311)
    plt.title('Position Estimation in x')
    plt.plot(time, position.x, 'bo', time, position.x, 'k')
    plt.ylabel('Position x')
    #plt.plot(time, position.x, 'k--')

    plt.subplot(312)
    plt.title('Position Estimation in y')
    plt.plot(time, position.y, 'bo', time, position.y, 'k')
    plt.ylabel('Position y')
    #plt.plot(time, position.y, 'k--')

    plt.subplot(313)
    plt.title('Position Estimation in z')
    plt.plot(time, position.z, 'bo', time, position.z, 'k')
    plt.ylabel('Position z')
    plt.xlabel('time (s)')
    #plt.plot(time, position.z, 'k--')
'''
plt.show()

#size = numpy.size(positionx)
#print size
bag.close()


#Integral gain for the docking.
