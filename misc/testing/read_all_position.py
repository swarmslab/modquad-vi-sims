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
#import statistics as st

#bag = rosbag.Bag('data20187241447.bag')
bag_lists = sys.argv[1:]
#bag_name = sys.argv[1]
#pdb.set_trace()

fig1 = plt.figure(1)
fig1.patch.set_facecolor('white')
fig1.suptitle('Position Estimation', fontsize=16)
px_l = -2.00
px_h = -1.30
py_l = -0.35
py_h = 0.05
pz_l = -0.80
pz_h = 0.60

vx_l = -0.80
vx_h = 1.00
vy_l = -0.6
vy_h = 0.6
vz_l = -1.50
vz_h = 1.50
ax111 = fig1.add_subplot(321)
ax111.set_title('Position Estimation in x')
ax111.set_ylabel('Position x')
ax111.set_ylim(px_l, px_h)
   
ax112 = fig1.add_subplot(323)
ax112.set_title('Position Estimation in y')
ax112.set_ylabel('Position y')
ax112.set_ylim(py_l, py_h)
   
ax113 = fig1.add_subplot(325)
ax113.set_title('Position Estimation in z')
ax113.set_ylabel('Position z')
ax113.set_ylim(pz_l, pz_h)

ax121 = fig1.add_subplot(322)
ax121.set_title('Changed Estimation in x')
ax121.set_ylabel('Changed x')
ax121.set_ylim(px_l, px_h)
   
ax122 = fig1.add_subplot(324)
ax122.set_title('Changed Estimation in y')
ax122.set_ylabel('Changed y')
ax122.set_ylim(py_l, py_h)
    
ax123 = fig1.add_subplot(326)
ax123.set_title('Changed Estimation in z')
ax123.set_ylabel('Changed z')
ax123.set_ylim(pz_l, pz_h)

fig2 = plt.figure(2)
fig2.patch.set_facecolor('white')
fig2.suptitle('Velocity Estimation', fontsize=16)

ax211 = fig2.add_subplot(321)
ax211.set_title('Velocity Estimation in x')
ax211.set_ylabel('Velocity x')
ax211.set_ylim(vx_l, vx_h)
                            
ax212 = fig2.add_subplot(323)
ax212.set_title('Velocity Estimation in y')
ax212.set_ylabel('Velocity y')
ax212.set_ylim(vy_l, vy_h)
                            
ax213 = fig2.add_subplot(325)
ax213.set_title('Velocity Estimation in z')
ax213.set_ylabel('Velocity z')
ax213.set_ylim(vz_l, vz_h)
                            
ax221 = fig2.add_subplot(322)
ax221.set_title('Changed Estimation in x')
ax221.set_ylabel('Changed x')
ax221.set_ylim(vx_l, vx_h)
                            
ax222 = fig2.add_subplot(324)
ax222.set_title('Changed Estimation in y')
ax222.set_ylabel('Changed y')
ax222.set_ylim(vy_l, vy_h)
                            
ax223 = fig2.add_subplot(326)
ax223.set_title('Changed Estimation in z')
ax223.set_ylabel('Changed z')
ax223.set_ylim(vz_l, vz_h)

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
ax33.set_ylim(0.400, 0.500)

line = ['-','--']
i = 0
for bag_name in bag_lists:
	modquad1_vicon = []
	rostime_list = []
	viconrostime_list = []
	modquad2_vicon = []
	cam_vicon = []
	tag_vicon = []
	tagdeltax_list = []
	tagdeltay_list = []
	tagdeltaz_list = []
	camdeltax_list = []
	camdeltay_list = []
	camdeltaz_list = []
	whycon_pos = []
	vicon_velocity1 = []
	vision_position_x_list = []
	vision_position_y_list = []
	vision_position_z_list = []
	vision_velocity_x_list = []
	vision_velocity_y_list = []
	vision_velocity_z_list = []
	vicon_velocityx_list = []
	vicon_velocityy_list = []   
	vicon_velocityz_list = []
	deltax_list = []
	deltay_list = []
	deltaz_list = []
	vicon_positionx_list = []
	vicon_positiony_list = []
	vicon_positionz_list = []
        hor_lin1 = []
        hor_lin2 = []
	imu_ang_x_list = []
	imu_ang_y_list = []
	imu_ang_z_list = []
	imu_acc_x_list = []
	imu_acc_y_list = []
	imu_acc_z_list = []
	whycon_position_x_list= []
	whycon_position_y_list= []
	whycon_position_z_list= []
        
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
	rostime = rostime - rostime_list[0]

	print "vision position error in x %s "%np.std(vision_position_x_list)
	print "vision position error in y %s "%np.std(vision_position_y_list)
	print "vision position error in z %s "%np.std(vision_position_z_list)
	print "vision velocity error in x %s "%np.std(vision_velocity_x_list)
	print "vision velocity error in y %s "%np.std(vision_velocity_y_list)
	print "vision velocity error in z %s "%np.std(vision_velocity_z_list)

	for topic, imu, stamp in bag.read_messages(topics=['/mavros1/imu/data']):
	    time = stamp.secs + stamp.nsecs * 1e-9
	    imu_ang_x_list.append(imu.angular_velocity.x)
	    imu_ang_y_list.append(imu.angular_velocity.y)
	    imu_ang_z_list.append(imu.angular_velocity.z)
	    imu_acc_x_list.append(imu.linear_acceleration.x)
	    imu_acc_y_list.append(imu.linear_acceleration.y)
	    imu_acc_z_list.append(imu.linear_acceleration.z)

	for topic, position, stamp in bag.read_messages(topics=['/whycon/poses']):
	    time = stamp.secs + stamp.nsecs * 1e-9
	    whycon_position_x_list.append(position.poses[0].position.x)
	    whycon_position_y_list.append(position.poses[0].position.y)
	    whycon_position_z_list.append(position.poses[0].position.z)

	print "whycon position error in x %s " %np.std(whycon_position_x_list)
	print "whycon position error in y %s " %np.std(whycon_position_y_list)
	print "whycon position error in z %s " %np.std(whycon_position_z_list)

        print "angular velocity error in x %s "%np.std(imu_ang_x_list)
        print "angular velocity error in y %s "%np.std(imu_ang_y_list)
        print "angular velocity error in z %s "%np.std(imu_ang_z_list)
        print "acceleration error in x %s "%np.std(imu_acc_x_list)
        print "acceleration error in y %s "%np.std(imu_acc_y_list)
        print "acceleration error in z %s "%np.std(imu_acc_z_list)


	start_time=rostime[0]
	end_time=rostime[-1]

	vision_position_x = np.asarray(vision_position_x_list)
	vision_position_y = np.asarray(vision_position_y_list)
	vision_position_z = np.asarray(vision_position_z_list)
	vision_velocity_x = np.asarray(vision_velocity_x_list)
	vision_velocity_y = np.asarray(vision_velocity_y_list)
	vision_velocity_z = np.asarray(vision_velocity_z_list)

	for topic, odom1, stamp in bag.read_messages(topics=['/vicon/Modquad1/odom']):
	    vicontime = stamp.secs + stamp.nsecs * 1e-9
	    modquad1_vicon.append(odom1)
	    vicon_velocityx_list.append(odom1.twist.twist.linear.x)
	    vicon_velocityy_list.append(odom1.twist.twist.linear.y)
	    vicon_velocityz_list.append(odom1.twist.twist.linear.z)
	    vicon_positionx_list.append(odom1.pose.pose.position.x)
	    vicon_positiony_list.append(odom1.pose.pose.position.y)
	    vicon_positionz_list.append(odom1.pose.pose.position.z)
	    viconrostime_list.append(vicontime)     
            hor_lin1.append(0.5) 
            hor_lin2.append(0.4) 

	viconrostime = np.asarray(viconrostime_list)
	viconrostime = viconrostime - viconrostime_list[0]

	for topic, odom2, stamp in bag.read_messages(topics=['/vicon/Modquad2/odom']):
	    modquad2_vicon.append(odom2)

	for topic, cam, stamp in bag.read_messages(topics=['/vicon/Modquad_camera/odom']):
	    cam_vicon.append(cam)

	for topic, tag, stamp in bag.read_messages(topics=['/vicon/Modquad_tag/odom']):
	    tag_vicon.append(tag)

	for odom1, odom2 in zip(modquad1_vicon, modquad2_vicon):
	    deltax_list.append(odom1.pose.pose.position.x-odom2.pose.pose.position.x)
	    deltay_list.append(odom1.pose.pose.position.y-odom2.pose.pose.position.y)
	    deltaz_list.append(odom1.pose.pose.position.z-odom2.pose.pose.position.z)

	for tag, odom2 in zip(tag_vicon, modquad2_vicon):
	    tagdeltax_list.append(tag.pose.pose.position.x-odom2.pose.pose.position.x)
	    tagdeltay_list.append(tag.pose.pose.position.y-odom2.pose.pose.position.y)
	    tagdeltaz_list.append(tag.pose.pose.position.z-odom2.pose.pose.position.z)

	for odom1, cam in zip(modquad1_vicon, cam_vicon):
	    camdeltax_list.append(cam.pose.pose.position.x-odom1.pose.pose.position.x)
	    camdeltay_list.append(cam.pose.pose.position.y-odom1.pose.pose.position.y)
	    camdeltaz_list.append(cam.pose.pose.position.z-odom1.pose.pose.position.z)

	print "cam position error in x %s "%np.mean(camdeltax_list)       
	print "cam position error in y %s "%np.mean(camdeltay_list)
	print "cam position error in z %s "%np.mean(camdeltaz_list)
	print "tag position error in x %s "%np.mean(tagdeltax_list)
	print "tag position error in y %s "%np.mean(tagdeltay_list)
	print "tag position error in z %s "%np.mean(tagdeltaz_list)

	deltax = np.asarray(deltax_list)
	deltay = np.asarray(deltay_list)
	deltaz = np.asarray(deltaz_list)
	vicon_velocityx = np.asarray(vicon_velocityx_list)
	vicon_velocityy = np.asarray(vicon_velocityy_list)
	vicon_velocityz = np.asarray(vicon_velocityz_list)
	vicon_positionx = np.asarray(vicon_positionx_list)
	vicon_positiony = np.asarray(vicon_positiony_list)
	vicon_positionz = np.asarray(vicon_positionz_list)

	ax111.set_xlim(start_time, end_time)
	ax112.set_xlim(start_time, end_time)
	ax113.set_xlim(start_time, end_time)
          
	ax121.set_xlim(start_time, end_time)
	ax122.set_xlim(start_time, end_time)
	ax123.set_xlim(start_time, end_time)
          
	ax211.set_xlim(start_time, end_time)
	ax212.set_xlim(start_time, end_time)
	ax213.set_xlim(start_time, end_time)
          
	ax221.set_xlim(start_time, end_time)
	ax222.set_xlim(start_time, end_time)
	ax223.set_xlim(start_time, end_time)

        pdb.set_trace()

	ax111.plot(rostime, vision_position_x, 'r'+line[i],label = 'Vision Estimation')
	ax112.plot(rostime, vision_position_y, 'r'+line[i],label = 'Vision Estimation')
	ax113.plot(rostime, vision_position_z, 'r'+line[i],label = 'Vision Estimation')
                                                 
	ax211.plot(rostime, vision_velocity_x, 'r'+line[i],label = 'Vision Estimation')
	ax212.plot(rostime, vision_velocity_y, 'r'+line[i],label = 'Vision Estimation')
	ax213.plot(rostime, vision_velocity_z, 'r'+line[i],label = 'Vision Estimation')

	ax121.plot(viconrostime, deltax, 'b'+line[i],label = 'vicon')
	ax122.plot(viconrostime, deltay, 'b'+line[i],label = 'vicon')
	ax123.plot(viconrostime, deltaz, 'b'+line[i],label = 'vicon')
                        
	ax221.plot(viconrostime, vicon_velocityx, 'b'+line[i],label = 'vicon')
	ax222.plot(viconrostime, vicon_velocityy, 'b'+line[i],label = 'vicon')
	ax223.plot(viconrostime, vicon_velocityz, 'b'+line[i],label = 'vicon')
	
	ax111.plot(viconrostime, deltax, 'b'+line[i],label = 'vicon')
	ax112.plot(viconrostime, deltay, 'b'+line[i],label = 'vicon')
	ax113.plot(viconrostime, deltaz, 'b'+line[i],label = 'vicon')
                                                      
	ax211.plot(viconrostime, vicon_velocityx, 'b'+line[i],label = 'vicon')
	ax212.plot(viconrostime, vicon_velocityy, 'b'+line[i],label = 'vicon')
	ax213.plot(viconrostime, vicon_velocityz, 'b'+line[i],label = 'vicon')
        '''
        ax11.plot(viconrostime, deltax, 'r'+line[i],label = 'Vicon')
	ax12.plot(viconrostime, deltay, 'r'+line[i],label = 'Vicon')
	ax13.plot(viconrostime, deltaz, 'r'+line[i],label = 'Vicon')

	ax21.plot(viconrostime, vicon_velocityx, 'r'+line[i],label = 'Vicon')
	ax22.plot(viconrostime, vicon_velocityy, 'r'+line[i],label = 'Vicon')
	ax23.plot(viconrostime, vicon_velocityz, 'r'+line[i],label = 'Vicon')
        '''
	ax111.legend()
	ax112.legend()
	ax113.legend()
	ax121.legend()
	ax122.legend()
	ax123.legend()

	ax211.legend()
	ax212.legend()
	ax213.legend()
	ax221.legend()
	ax222.legend()
	ax223.legend()

	ax31.plot(viconrostime, vicon_positionx, 'b'+line[i])
	ax32.plot(viconrostime, vicon_positiony, 'b'+line[i])
	ax33.plot(viconrostime, vicon_positionz, 'b'+line[i]) 
	ax33.plot(viconrostime, hor_lin1, 'k')
	ax33.plot(viconrostime, hor_lin2, 'k') 

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
