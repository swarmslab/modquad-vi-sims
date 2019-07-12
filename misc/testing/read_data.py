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
#bag_name = sys.argv[1]
#pdb.set_trace()

ax11_yl = -2.00
ax12_yl = -1.00
ax13_yl = -1.00
ax21_yl = -1.00
ax22_yl = -1.00
ax23_yl = -1.00

ax11_yh = 0.5
ax12_yh = 1.00
ax13_yh = 1.00
ax21_yh = 1.00
ax22_yh = 1.00
ax23_yh = 1.00

xl = 0.0
xh = 30.0

fig1 = plt.figure(1)
fig1.patch.set_facecolor('white')
fig1.suptitle('Relative Position', fontsize=16)

ax11 = fig1.add_subplot(311)
ax11.set_title('Relative Position in x')
ax11.set_ylabel('Position x')
ax11.set_ylim(ax11_yl, ax11_yh)

ax12 = fig1.add_subplot(312)
ax12.set_title('Relative Position in y')
ax12.set_ylabel('Position y')
ax12.set_ylim(ax12_yl, ax12_yh)

ax13 = fig1.add_subplot(313)
ax13.set_title('Relative Position in z')
ax13.set_ylabel('Position z')
ax13.set_ylim(ax13_yl, ax13_yh)

fig2 = plt.figure(2)
fig2.patch.set_facecolor('white')
fig2.suptitle('Relative Velocity', fontsize=16)

ax21 = fig2.add_subplot(311)
ax21.set_title('Relative Velocity in x')
ax21.set_ylabel('Velocity x')
ax21.set_ylim(ax21_yl, ax21_yh)

ax22 = fig2.add_subplot(312)
ax22.set_title('Relative Velocity in y')
ax22.set_ylabel('Velocity y')
ax22.set_ylim(ax22_yl, ax22_yh)

ax23 = fig2.add_subplot(313)
ax23.set_title('Relative Velocity in z')
ax23.set_ylabel('Velocity z')
ax23.set_ylim(ax23_yl, ax23_yh)

fig3 = plt.figure(3)
fig3.patch.set_facecolor('white')
fig3.suptitle('Modquad1 position in inertia space', fontsize=16)

ax31 = fig3.add_subplot(311)
ax31.set_title('x')
ax31.set_ylabel('Position x')

ax32 = fig3.add_subplot(312)
ax32.set_title('y')
ax32.set_ylabel('Position y')

ax33 = fig3.add_subplot(313)
ax33.set_title('z')
ax33.set_ylabel('Position z')

fig4 = plt.figure(4)
fig4.patch.set_facecolor('white')
fig4.suptitle('Modquad2 position in inertia space', fontsize=16)

ax41 = fig4.add_subplot(311)
ax41.set_title('x')
ax41.set_ylabel('Position x')

ax42 = fig4.add_subplot(312)
ax42.set_title('y')
ax42.set_ylabel('Position y')

ax43 = fig4.add_subplot(313)
ax43.set_title('z')
ax43.set_ylabel('Position z')

fig5 = plt.figure(5)
fig5.patch.set_facecolor('white')
fig5.suptitle('Imu and thrust', fontsize=16)

ax51 = fig5.add_subplot(211)
ax51.set_title('Imu')
ax51.set_ylabel('Imu')
ax51.set_ylim(-10, 10)

ax52 = fig5.add_subplot(212)
ax52.set_title('Thrust')
ax52.set_ylabel('Thrust')
kpz = 3.2
m = 0.039



ax11.set_xlim(0,30)
ax12.set_xlim(0,30)
ax13.set_xlim(0,30)
ax21.set_xlim(0,30)
ax22.set_xlim(0,30)
ax23.set_xlim(0,30)
ax51.set_xlim(0,30)
ax52.set_xlim(0,30)


whycon_docked = False
track_time_s =2.8 
dock_time_s = 17 




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
	thrust_list = []
	thrust_time_list = []
	acc_x_list = []
	imu_time_list = []
        whycontime = np.asarray([])
	whycon_position_x = np.asarray([])
	whycon_position_y = np.asarray([])
	whycon_position_z = np.asarray([])
        docked = False
        time_offset = 70
	bag = rosbag.Bag(bag_name)

	for topic, position, stamp in bag.read_messages(topics=['/modquad1/filtered_Vision_Odom']):
	    time = stamp.secs + stamp.nsecs * 1e-9
	    vision_position_x_list.append(position.position.x)
	    vision_position_y_list.append(position.position.y)
	    vision_position_z_list.append(position.position.z)
	    vision_velocity_x_list.append(position.velocity.x)
	    vision_velocity_y_list.append(position.velocity.y)
	    vision_velocity_z_list.append(position.velocity.z)
	    rostime_list.append(time)

	rostime = np.asarray(rostime_list)
        #pdb.set_trace()
	#rostime = rostime - rostime_list[0]
	start_time=rostime[0]
	end_time=rostime[-1]

	vision_position_x = np.asarray(vision_position_x_list)
	vision_position_y = np.asarray(vision_position_y_list)
	vision_position_z = np.asarray(vision_position_z_list)
	vision_velocity_x = np.asarray(vision_velocity_x_list)
	vision_velocity_y = np.asarray(vision_velocity_y_list)
	vision_velocity_z = np.asarray(vision_velocity_z_list)

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
            hor_lin1.append(0.5) 
            hor_lin2.append(0.4) 
        #pdb.set_trace()
	viconrostime1 = np.asarray(viconrostime1_list)
	viconrostime1 = viconrostime1 - viconrostime1_list[0] - time_offset

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
	viconrostime2 = viconrostime2 - viconrostime2_list[0]- time_offset
        rostime = rostime - viconrostime1_list[0]- time_offset
        start_time=rostime[0]
	end_time=rostime[-1]

	for topic, thrust, stamp in bag.read_messages(topics=['/mavros1/setpoint_attitude/thrust']):
	    #time = stamp.secs + stamp.nsecs * 1e-9 - 1525405720
	    thrust_time = stamp.secs + stamp.nsecs * 1e-9 
	    thrust_list.append(thrust.thrust)
	    thrust_time_list.append(thrust_time)
	   
        if not thrust_time_list:
                print 'thrust is empty'
        else: 
		#thrust_kp = kpz * (0.5-vision_position_z) * m + m * 9.8 
		thrust_rostime = np.asarray(thrust_time_list)
		#pdb.set_trace()
		thrust_rostime = thrust_rostime - thrust_time_list[0]
		_thrust = np.asarray(thrust_list)
		#ax52.plot(thrust_rostime, _thrust, 'b'+line[i],label = 'Thrust')

	for topic, imu, stamp in bag.read_messages(topics=['/mavros1/imu/data']):
	    #time = stamp.secs + stamp.nsecs * 1e-9 - 1525405720
	    imu_time = stamp.secs + stamp.nsecs * 1e-9 
	    acc_x_list.append(imu.linear_acceleration.x)
	    imu_time_list.append(imu_time)
            if  (imu.linear_acceleration.x > 3.0) & (not docked):
                imu_docktime = imu_time-imu_time_list[0]- time_offset
                ax51.plot(np.asarray([imu_docktime,imu_docktime]),np.asarray([-10,10]),'g--')
                #ax11.plot(np.asarray([imu_docktime,imu_docktime]),np.asarray([-10,10]),'g--')
                #ax12.plot(np.asarray([imu_docktime,imu_docktime]),np.asarray([-10,10]),'g--')
                #ax13.plot(np.asarray([imu_docktime,imu_docktime]),np.asarray([-10,10]),'g--')
                #ax21.plot(np.asarray([imu_docktime,imu_docktime]),np.asarray([-10,10]),'g--')
                #ax22.plot(np.asarray([imu_docktime,imu_docktime]),np.asarray([-10,10]),'g--')
                #ax23.plot(np.asarray([imu_docktime,imu_docktime]),np.asarray([-10,10]),'g--')



                docked = True

        acc_x = np.asarray(acc_x_list) + 0.5
        imu_time = np.asarray(imu_time_list) - imu_time_list[0] - time_offset
        
	for topic, position, stamp in bag.read_messages(topics=['/whycon/poses']):
            #pdb.set_trace()
	    whycontime = np.append(whycontime,stamp.secs + stamp.nsecs * 1e-9)
	    whycon_position_x = np.append(whycon_position_x,position.poses[0].position.x)
	    whycon_position_y = np.append(whycon_position_y,position.poses[0].position.y)
	    whycon_position_z = np.append(whycon_position_z,position.poses[0].position.z)
            if  (position.poses[0].position.z < 0.05) & (not whycon_docked):
                whycon_docktime = stamp.secs + stamp.nsecs * 1e-9-whycontime[0] - time_offset
	        ax52.plot([whycon_docktime,whycon_docktime],np.asarray([-10,10]),'r--')
	        ax51.plot([whycon_docktime,whycon_docktime],np.asarray([-10,10]),'r--')
	        ax52.plot([imu_docktime,imu_docktime],np.asarray([-10,10]),'g--')
		ax11.plot(np.asarray([whycon_docktime,whycon_docktime]),np.asarray([-10,10]),'r--')
		ax12.plot(np.asarray([whycon_docktime,whycon_docktime]),np.asarray([-10,10]),'r--')
		ax13.plot(np.asarray([whycon_docktime,whycon_docktime]),np.asarray([-10,10]),'r--')
		ax21.plot(np.asarray([whycon_docktime,whycon_docktime]),np.asarray([-10,10]),'r--')
		ax22.plot(np.asarray([whycon_docktime,whycon_docktime]),np.asarray([-10,10]),'r--')
		ax23.plot(np.asarray([whycon_docktime,whycon_docktime]),np.asarray([-10,10]),'r--')
		ax11.plot(np.asarray([track_time_s,track_time_s]),np.asarray([-10,10]),'g--')
		ax12.plot(np.asarray([track_time_s,track_time_s]),np.asarray([-10,10]),'g--')
		ax13.plot(np.asarray([track_time_s,track_time_s]),np.asarray([-10,10]),'g--')
		ax21.plot(np.asarray([track_time_s,track_time_s]),np.asarray([-10,10]),'g--')
		ax22.plot(np.asarray([track_time_s,track_time_s]),np.asarray([-10,10]),'g--')
		ax23.plot(np.asarray([track_time_s,track_time_s]),np.asarray([-10,10]),'g--')
		ax11.plot(np.asarray([dock_time_s,dock_time_s]),np.asarray([-10,10]),'g--')
		ax12.plot(np.asarray([dock_time_s,dock_time_s]),np.asarray([-10,10]),'g--')
		ax13.plot(np.asarray([dock_time_s,dock_time_s]),np.asarray([-10,10]),'g--')
		ax21.plot(np.asarray([dock_time_s,dock_time_s]),np.asarray([-10,10]),'g--')
		ax22.plot(np.asarray([dock_time_s,dock_time_s]),np.asarray([-10,10]),'g--')
		ax23.plot(np.asarray([dock_time_s,dock_time_s]),np.asarray([-10,10]),'g--')


                print imu_docktime - whycon_docktime
                whycon_docked = True


        whycontime = whycontime - whycontime[0] - time_offset
	ax51.plot(imu_time, acc_x, 'g',label = 'Acceleration x')
	ax52.plot(whycontime, whycon_position_z, 'b'+line[i],label = 'whycon x')

	for odom1, odom2 in zip(modquad1_vicon, modquad2_vicon):
	    delta_pos_x_list.append(odom1.pose.pose.position.x-odom2.pose.pose.position.x)
	    delta_pos_y_list.append(odom1.pose.pose.position.y-odom2.pose.pose.position.y)
	    delta_pos_z_list.append(odom1.pose.pose.position.z-odom2.pose.pose.position.z)
	    delta_vel_x_list.append(odom1.twist.twist.linear.x-odom2.twist.twist.linear.x)
	    delta_vel_y_list.append(odom1.twist.twist.linear.y-odom2.twist.twist.linear.y)
	    delta_vel_z_list.append(odom1.twist.twist.linear.z-odom2.twist.twist.linear.z)

	delta_pos_x = np.asarray(delta_pos_x_list)
	delta_pos_y = np.asarray(delta_pos_y_list)
	delta_pos_z = np.asarray(delta_pos_z_list)

	delta_vel_x = np.asarray(delta_vel_x_list)
	delta_vel_y = np.asarray(delta_vel_y_list)
	delta_vel_z = np.asarray(delta_vel_z_list)

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

	ax11.plot(rostime, vision_position_x, 'b'+line[i],label = 'Vision Estimation')
	ax12.plot(rostime, vision_position_y, 'b'+line[i],label = 'Vision Estimation')
	ax13.plot(rostime, vision_position_z, 'b'+line[i],label = 'Vision Estimation')

	ax21.plot(rostime, vision_velocity_x, 'b'+line[i],label = 'Vision Estimation')
	ax22.plot(rostime, vision_velocity_y, 'b'+line[i],label = 'Vision Estimation')
	ax23.plot(rostime, vision_velocity_z, 'b'+line[i],label = 'Vision Estimation')

        if len(viconrostime1) > len(viconrostime2):
		ax11.plot(viconrostime2, delta_pos_x, 'r'+line[i],label = 'Vicon')
		ax12.plot(viconrostime2, delta_pos_y, 'r'+line[i],label = 'Vicon')
		ax13.plot(viconrostime2, delta_pos_z, 'r'+line[i],label = 'Vicon')
		ax21.plot(viconrostime2, delta_vel_x, 'r'+line[i],label = 'Vicon')
		ax22.plot(viconrostime2, delta_vel_y, 'r'+line[i],label = 'Vicon')
		ax23.plot(viconrostime2, delta_vel_z, 'r'+line[i],label = 'Vicon')
        else: 
		ax11.plot(viconrostime1, delta_pos_x, 'r'+line[i],label = 'Vicon')
		ax12.plot(viconrostime1, delta_pos_y, 'r'+line[i],label = 'Vicon')
		ax13.plot(viconrostime1, delta_pos_z, 'r'+line[i],label = 'Vicon')
		ax21.plot(viconrostime1, delta_vel_x, 'r'+line[i],label = 'Vicon')
		ax22.plot(viconrostime1, delta_vel_y, 'r'+line[i],label = 'Vicon')
		ax23.plot(viconrostime1, delta_vel_z, 'r'+line[i],label = 'Vicon')

        ax11.text((track_time_s+dock_time_s)/2 - 1.0, (ax11_yh - ax11_yl) * 0.8 + ax11_yl,'Track',fontsize = 12)
        ax12.text((track_time_s+dock_time_s)/2 - 1.0, (ax12_yh - ax12_yl) * 0.8 + ax12_yl,'Track',fontsize = 12)
        ax13.text((track_time_s+dock_time_s)/2 - 1.0, (ax13_yh - ax13_yl) * 0.8 + ax13_yl,'Track',fontsize = 12)
        ax21.text((track_time_s+dock_time_s)/2 - 1.2, (ax21_yh - ax21_yl) * 0.8 + ax21_yl,'Track',fontsize = 12)
        ax22.text((track_time_s+dock_time_s)/2 - 1.2, (ax22_yh - ax22_yl) * 0.8 + ax22_yl,'Track',fontsize = 12)
        ax23.text((track_time_s+dock_time_s)/2 - 1.2, (ax23_yh - ax23_yl) * 0.8 + ax23_yl,'Track',fontsize = 12)

        ax11.text((track_time_s+xl)/2 -0.6, (ax11_yh - ax11_yl) * 0.8 + ax11_yl,'Hover',fontsize = 12)
        ax12.text((track_time_s+xl)/2 -0.6, (ax12_yh - ax12_yl) * 0.8 + ax12_yl,'Hover',fontsize = 12)
        ax13.text((track_time_s+xl)/2 -0.6, (ax13_yh - ax13_yl) * 0.8 + ax13_yl,'Hover',fontsize = 12)
        ax21.text((track_time_s+xl)/2 -0.6, (ax21_yh - ax21_yl) * 0.8 + ax21_yl,'Hover',fontsize = 12)
        ax22.text((track_time_s+xl)/2 -0.6, (ax22_yh - ax22_yl) * 0.8 + ax22_yl,'Hover',fontsize = 12)
        ax23.text((track_time_s+xl)/2 -0.6, (ax23_yh - ax23_yl) * 0.8 + ax23_yl,'Hover',fontsize = 12)

        ax11.text((whycon_docktime+dock_time_s)/2 - 1.0, (ax11_yh - ax11_yl) * 0.8 + ax11_yl,'Dock',fontsize = 12)
        ax12.text((whycon_docktime+dock_time_s)/2 - 1.0, (ax12_yh - ax12_yl) * 0.8 + ax12_yl,'Dock',fontsize = 12)
        ax13.text((whycon_docktime+dock_time_s)/2 - 1.0, (ax13_yh - ax13_yl) * 0.8 + ax13_yl,'Dock',fontsize = 12)
        ax21.text((whycon_docktime+dock_time_s)/2 - 1.2, (ax21_yh - ax21_yl) * 0.8 + ax21_yl,'Dock',fontsize = 12)
        ax22.text((whycon_docktime+dock_time_s)/2 - 1.2, (ax22_yh - ax22_yl) * 0.8 + ax22_yl,'Dock',fontsize = 12)
        ax23.text((whycon_docktime+dock_time_s)/2 - 1.2, (ax23_yh - ax23_yl) * 0.8 + ax23_yl,'Dock',fontsize = 12)

	ax11.legend(fontsize = 12)
	ax12.legend(fontsize = 12)
	ax13.legend(fontsize = 12)
	ax21.legend(fontsize = 12)
	ax22.legend(fontsize = 12)
	ax23.legend(fontsize = 12)

	ax31.plot(viconrostime1, vicon_positionx_1, 'b'+line[i])
	ax32.plot(viconrostime1, vicon_positiony_1, 'b'+line[i])
	ax33.plot(viconrostime1, vicon_positionz_1, 'b'+line[i]) 
	ax33.plot(viconrostime1, hor_lin1, 'k')
	ax33.plot(viconrostime1, hor_lin2, 'k') 

	#ax31.plot(viconrostime2, vicon_positionx_2, 'r'+line[i])
	#ax32.plot(viconrostime2, vicon_positiony_2, 'r'+line[i])
	#ax33.plot(viconrostime2, vicon_positionz_2, 'r'+line[i]) 
	#ax41.plot(viconrostime1, vicon_velocityx_1, 'b'+line[i])
	#ax42.plot(viconrostime1, vicon_velocityy_1, 'b'+line[i])
	#ax43.plot(viconrostime1, vicon_velocityz_1, 'b'+line[i]) 
	ax41.plot(viconrostime2, vicon_positionx_2, 'r'+line[i])
	ax42.plot(viconrostime2, vicon_positiony_2, 'r'+line[i])
	ax43.plot(viconrostime2, vicon_positionz_2, 'r'+line[i]) 
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
