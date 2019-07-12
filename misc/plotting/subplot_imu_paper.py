#!/usr/bin/env python
'''
Author: Guanrui Li lguanrui@seas.upenn.edu
'''

import rosbag
import matplotlib
matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True
from matplotlib import pyplot as plt
import rospy
import numpy as np
import pdb
import sys

#bag = rosbag.Bag('data20187241447.bag')
bag_name = sys.argv[1]
#bag_name = sys.argv[1]
#pdb.set_trace()

ax11_yl = -2.00
ax12_yl = -1.00
ax13_yl = -1.00
ax21_yl = -1.00
ax22_yl = -1.00
ax23_yl = -1.00
axacc_yl = -10.00
axaccz_yl = 5.00

ax11_yh = 0.5
ax12_yh = 1.00
ax13_yh = 1.00
ax21_yh = 1.00
ax22_yh = 1.00
ax23_yh = 1.00
axacc_yh = 10.00
axaccz_yh = 15.00


xl = 0.0
xh = 30.0

kpz = 3.2
m = 0.039



whycon_docked = False
track_time_s =2.8 
dock_time_s = 17 




line = ['-','--']
i = 0

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
acc_y_list = []
acc_z_list = []
imu_time_list = []
whycontime = np.asarray([])
whycon_position_x = np.asarray([])
whycon_position_y = np.asarray([])
whycon_position_z = np.asarray([])
docked = False
time_offset = 70
text_font = 12
legend_font = 10 
title_font = 16 
label_font = 16 
length =  9 
width = length * 0.333 
yh = axaccz_yh
yl = axacc_yl
#figname = '/home/guanrui/my_paper/Vi-ModQuad-ICRA-2019/figs/position-x.png'
#figname = '/home/guanrui/my_paper/Vi-ModQuad-ICRA-2019/figs/velocity-z.png'
figname = '/home/guanrui/my_paper/Vi-ModQuad-ICRA-2019/figs/acc-imu-all.pdf'
fig = plt.figure(1,figsize=(length,width))
fig.patch.set_facecolor('white')
#pdb.set_trace()

#fig1 = fig.add_subplot(111,figsize=(0.5*length,0.5*width))

fig1 = fig.add_subplot(111)
plt.gcf().subplots_adjust(bottom=0.2)
#fig1.patch.set_facecolor('white')

#fig1.set_title('Relative velocity in x',fontsize = title_font)
fig1.set_ylim(yl, yh)
fig1.set_xlim(0,30)
#fig1.set_ylabel(r'$\dot{z}_{\mathit{d}}^{\mathit{w}}\hspace{0.5} (m/s)$',fontsize = label_font)
#fig1.set_ylabel(r'$x_{\mathit{d}}^{\mathit{w}}\hspace{0.5} (m)$',fontsize = label_font)
#fig1.set_ylabel(r'$\mathbf{a}_{m_z}\hspace{0.5} (m/s^2)$',fontsize = label_font)
fig1.set_ylabel(r'$a_{m}\hspace{0.5} (m/s^2)$',fontsize = label_font)
fig1.set_xlabel(r'$time (s)$',fontsize = label_font)
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

for topic, imu, stamp in bag.read_messages(topics=['/mavros1/imu/data']):
    #time = stamp.secs + stamp.nsecs * 1e-9 - 1525405720
    imu_time = stamp.secs + stamp.nsecs * 1e-9 
    acc_x_list.append(imu.linear_acceleration.x)
    acc_y_list.append(imu.linear_acceleration.y)
    acc_z_list.append(imu.linear_acceleration.z)
    imu_time_list.append(imu_time)
    if  (imu.linear_acceleration.x > 3.0) & (not docked):
	imu_docktime = imu_time-imu_time_list[0]- time_offset
	#fig1.plot(np.asarray([imu_docktime,imu_docktime]),np.asarray([-10,10]),'g--')
	#ax11.plot(np.asarray([imu_docktime,imu_docktime]),np.asarray([-10,10]),'g--')
	#ax12.plot(np.asarray([imu_docktime,imu_docktime]),np.asarray([-10,10]),'g--')
	#ax13.plot(np.asarray([imu_docktime,imu_docktime]),np.asarray([-10,10]),'g--')
	#ax21.plot(np.asarray([imu_docktime,imu_docktime]),np.asarray([-10,10]),'g--')
	#ax22.plot(np.asarray([imu_docktime,imu_docktime]),np.asarray([-10,10]),'g--')
	#ax23.plot(np.asarray([imu_docktime,imu_docktime]),np.asarray([-10,10]),'g--')



	docked = True

acc_x = np.asarray(acc_x_list) 
acc_y = np.asarray(acc_y_list) 
acc_z = np.asarray(acc_z_list) 

imu_time = np.asarray(imu_time_list) - imu_time_list[0] - time_offset

for topic, position, stamp in bag.read_messages(topics=['/whycon/poses']):
    #pdb.set_trace()
    whycontime = np.append(whycontime,stamp.secs + stamp.nsecs * 1e-9)
    whycon_position_x = np.append(whycon_position_x,position.poses[0].position.x)
    whycon_position_y = np.append(whycon_position_y,position.poses[0].position.y)
    whycon_position_z = np.append(whycon_position_z,position.poses[0].position.z)
    if  (np.sqrt(position.poses[0].position.z**2 + position.poses[0].position.y**2 +position.poses[0].position.x**2) < 0.05) & (not whycon_docked):
	whycon_docktime = stamp.secs + stamp.nsecs * 1e-9-whycontime[0] - time_offset
	fig1.plot(np.asarray([whycon_docktime,whycon_docktime]),np.asarray([yl,yh]),'r--')
	fig1.plot(np.asarray([track_time_s,track_time_s]),np.asarray([yl,yh]),'g--')
	fig1.plot(np.asarray([dock_time_s,dock_time_s]),np.asarray([yl,yh]),'g--')


	print imu_docktime - whycon_docktime
	whycon_docked = True


whycontime = whycontime - whycontime[0] - time_offset

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

#fig1.plot(rostime, vision_position_x, 'b'+line[i],label = 'Vision')
#fig1.plot(rostime, vision_position_y, 'b'+line[i],label = 'Vision')
#fig1.plot(rostime, vision_position_z, 'b'+line[i],label = 'Vision')
#fig1.plot(rostime, vision_velocity_x, 'b'+line[i],label = 'Vision')
#fig1.plot(rostime, vision_velocity_y, 'b'+line[i],label = 'Vision')
#fig1.plot(rostime, vision_velocity_z, 'b'+line[i],label = 'Vision')
#fig1.plot(imu_time, acc_x, 'b'+line[i],label = 'Accelerometer')
#fig1.plot(imu_time, acc_y, 'b'+line[i],label = 'Accelerometer')
#fig1.plot(imu_time, acc_z, 'b'+line[i],label = 'Accelerometer')
fig1.plot(imu_time, acc_x, 'r'+line[i],label = 'x')
fig1.plot(imu_time, acc_y, 'g'+line[i],label = 'y')
fig1.plot(imu_time, acc_z, 'b'+line[i],label = 'z')

#if len(viconrostime1) > len(viconrostime2):
	#fig1.plot(viconrostime2, delta_pos_x, 'r'+line[i],label = 'Vicon')
	#fig1.plot(viconrostime2, delta_pos_y, 'r'+line[i],label = 'Vicon')
	#fig1.plot(viconrostime2, delta_pos_z, 'r'+line[i],label = 'Vicon')
	#fig1.plot(viconrostime2, delta_vel_x, 'r'+line[i],label = 'Vicon')
	#fig1.plot(viconrostime2, delta_vel_y, 'r'+line[i],label = 'Vicon')
	#fig1.plot(viconrostime2, delta_vel_z, 'r'+line[i],label = 'Vicon')
#else: 
	#fig1.plot(viconrostime1, delta_pos_x, 'r'+line[i],label = 'Vicon')
	#fig1.plot(viconrostime1, delta_pos_y, 'r'+line[i],label = 'Vicon')
	#fig1.plot(viconrostime1, delta_pos_z, 'r'+line[i],label = 'Vicon')
	#fig1.plot(viconrostime1, delta_vel_x, 'r'+line[i],label = 'Vicon')
	#fig1.plot(viconrostime1, delta_vel_y, 'r'+line[i],label = 'Vicon')
	#fig1.plot(viconrostime1, delta_vel_z, 'r'+line[i],label = 'Vicon')

fig1.text((track_time_s+dock_time_s)/2 - 1.0, (yh - yl) * 0.85 + yl,'Track',fontsize = text_font)

fig1.text((track_time_s+xl)/2 -1.2, (yh - yl) * 0.85 + yl,'Hover',fontsize = text_font)

fig1.text((whycon_docktime+dock_time_s)/2 - 1.0, (yh - yl) * 0.85 + yl,'Dock',fontsize = text_font)

fig1.legend(bbox_to_anchor = (0.0,1.065,1,0.102), loc = 9, ncol = 3, fontsize = legend_font)

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
plt.savefig(figname)
#plt.show()
#size = numpy.size(positionx)
#print size
bag.close()


#Integral gain for the docking.
