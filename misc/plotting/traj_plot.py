#!/usr/bin/env python
'''
Author: Guanrui Li lguanrui@seas.upenn.edu
'''

from matplotlib import pyplot as plt
import rospy
import numpy as np
import pdb
import sys
from modquad.srv import *
from modquad.msg import *

def two_pts_trajectory_init(startpoint, endpoint,t0,tf):

        x_initial = np.matrix([[startpoint.x, endpoint.x, 0, 0, 0, 0]]).transpose()
        y_initial = np.matrix([[startpoint.y, endpoint.y, 0, 0, 0, 0]]).transpose()
        z_initial = np.matrix([[startpoint.z, endpoint.z, 0, 0, 0, 0]]).transpose()
        A = np.matrix([[1, t0, t0 ** 2, t0 ** 3    , t0 ** 4     , t0 ** 5],
                            [1, tf, tf ** 2, tf ** 3    , tf ** 4     , tf ** 5],
                            [0, 1      , 2 * t0 , 3 * t0 ** 2, 4 * t0 ** 3 , 5 * t0 ** 4],
                            [0, 1      , 2 * tf , 3 * tf ** 2, 4 * tf ** 3 , 5 * tf ** 4],
                            [0, 0      , 2           , 6 * t0     , 12 * t0 ** 2, 20 * t0 ** 3],
                            [0, 0      , 2           , 6 * tf     , 12 * tf ** 2, 20 * tf ** 3]])
        A_init_flag = True
        xcoeff = np.linalg.inv(A) * x_initial
        ycoeff = np.linalg.inv(A) * y_initial
        zcoeff = np.linalg.inv(A) * z_initial
        return [A,A_init_flag,xcoeff,ycoeff,zcoeff]

def two_pts_trajectory_generator(t,waypoint,t0,tf,A,A_init_flag,xcoeff,ycoeff,zcoeff):
        des_trajectory_point = trajpoint()
        #pdb.set_trace()
        if A_init_flag:
        # print 'time now = ' + str(t)
            if t0 < t < tf:
                polynominal = np.matrix([[1, t, t ** 2, t ** 3    , t ** 4     , t ** 5],
                                     [0, 1, 2 * t , 3 * t ** 2, 4 * t ** 3 , 5 * t ** 4],
                                     [0, 0, 2     , 6 * t     , 12 * t ** 2, 20 * t ** 3]])
                x_d = polynominal * xcoeff
                y_d = polynominal * ycoeff
                z_d = polynominal * zcoeff
               # pdb.set_trace()
                des_trajectory_point.position.x = x_d[0,0]
                des_trajectory_point.position.y = y_d[0,0]
                des_trajectory_point.position.z = z_d[0,0]
                des_trajectory_point.velocity.x = x_d[1,0]
                des_trajectory_point.velocity.y = y_d[1,0]
                des_trajectory_point.velocity.z = z_d[1,0]
                des_trajectory_point.acceleration.x = x_d[2,0]
                des_trajectory_point.acceleration.y = y_d[2,0]
                des_trajectory_point.acceleration.z = z_d[2,0]

            else:

                des_trajectory_point.position.x     = waypoint.x
                des_trajectory_point.position.y     = waypoint.y
                des_trajectory_point.position.z     = waypoint.z
                des_trajectory_point.velocity.x     = 0.0
                des_trajectory_point.velocity.y     = 0.0
                des_trajectory_point.velocity.z     = 0.0
                des_trajectory_point.acceleration.x = 0.0
                des_trajectory_point.acceleration.y = 0.0
                des_trajectory_point.acceleration.z = 0.0

            des_trajectory_point.updated = True
            return des_trajectory_point

fig1 = plt.figure(1)
fig1.patch.set_facecolor('white')
fig1.suptitle('Position trajectory', fontsize=16)

ax11 = fig1.add_subplot(311)
ax11.set_title('Position trajectory in x')
ax11.set_ylabel('Position x')
ax11.set_ylim(-2.000, -1.0)

ax12 = fig1.add_subplot(312)
ax12.set_title('Position trajectory in y')
ax12.set_ylabel('Position y')
ax12.set_ylim(-0.200, 0.200)

ax13 = fig1.add_subplot(313)
ax13.set_title('Position trajectory in z')
ax13.set_ylabel('Position z')
ax13.set_ylim(-0.100, 0.200)

fig2 = plt.figure(2)
fig2.patch.set_facecolor('white')
fig2.suptitle('Velocity trajectory', fontsize=16)

ax21 = fig2.add_subplot(311)
ax21.set_title('Velocity trajectory in x')
ax21.set_ylabel('Velocity x')
ax21.set_ylim(-0.500, 0.700)

ax22 = fig2.add_subplot(312)
ax22.set_title('Velocity trajectory in y')
ax22.set_ylabel('Velocity y')
ax22.set_ylim(-0.500, 0.500)

ax23 = fig2.add_subplot(313)
ax23.set_title('Velocity trajectory in z')
ax23.set_ylabel('Velocity z')
ax23.set_ylim(-0.500, 0.500)

fig3 = plt.figure(3)
fig3.patch.set_facecolor('white')
fig3.suptitle('Acceleration trajectory', fontsize=16)

ax31 = fig3.add_subplot(311)
ax31.set_title('x')
ax31.set_ylabel('Acceleration trajectory in x')

ax32 = fig3.add_subplot(312)
ax32.set_title('y')
ax32.set_ylabel('Acceleration trajectory in y')

ax33 = fig3.add_subplot(313)
ax33.set_title('z')
ax33.set_ylabel('Acceleration trajectory in z')


pos_x_list = []
pos_y_list = []
pos_z_list = []
vel_x_list = []
vel_y_list = []
vel_z_list = []
acc_x_list = []
acc_y_list = []
acc_z_list = []

dock_waypoint = Waypoint()
dock_waypoint.x = 0.0
dock_waypoint.y = 0.0
dock_waypoint.z = 0.8

#dock_waypoint.x = -0.5
#dock_waypoint.y = 0.0
#dock_waypoint.z = 0.0

start_point = Waypoint()
start_point.x = 0.0
start_point.y = 0.0
start_point.z = 0.8


#start_point.x = -1.22
#start_point.y = -0.1
#start_point.z = 0.05
t0 = int(sys.argv[1])
tf = int(sys.argv[2])
#pdb.set_trace()
[A,A_flag,xcoeff,ycoeff,zcoeff] = two_pts_trajectory_init(start_point, dock_waypoint,t0,tf)
tstep = 0.001
time = np.arange(t0,tf,tstep)
for t in np.arange(t0,tf,tstep):
        des_trajectory_point=two_pts_trajectory_generator(t,dock_waypoint,t0,tf,A,A_flag,xcoeff,ycoeff,zcoeff)
	pos_x_list.append(des_trajectory_point.position.x)    
	pos_y_list.append(des_trajectory_point.position.y)    
	pos_z_list.append(des_trajectory_point.position.z)    
	vel_x_list.append(des_trajectory_point.velocity.x)    
	vel_y_list.append(des_trajectory_point.velocity.y)    
	vel_z_list.append(des_trajectory_point.velocity.z)    
	acc_x_list.append(des_trajectory_point.acceleration.x)   
	acc_y_list.append(des_trajectory_point.acceleration.y)   
	acc_z_list.append(des_trajectory_point.acceleration.z)  


pos_x=np.asarray(pos_x_list)
pos_y=np.asarray(pos_y_list)
pos_z=np.asarray(pos_z_list)
vel_x=np.asarray(vel_x_list)
vel_y=np.asarray(vel_y_list)
vel_z=np.asarray(vel_z_list)
acc_x=np.asarray(acc_x_list)
acc_y=np.asarray(acc_y_list)
acc_z=np.asarray(acc_z_list)


ax11.plot(time,pos_x)
ax12.plot(time,pos_y)
ax13.plot(time,pos_z)
ax21.plot(time,vel_x)
ax22.plot(time,vel_y)
ax23.plot(time,vel_z)
ax31.plot(time,acc_x)
ax32.plot(time,acc_y)
ax33.plot(time,acc_z)


plt.show()








