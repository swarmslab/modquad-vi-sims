#!/usr/bin/env python
"""
Author: Guanrui Li <lguanrui@seas.upenn.edu>
This is the position control node.
"""
import rospy
from modquad.srv import *
from modquad.msg import *
from modquad_init import modquad
from geometry_msgs.msg import Quaternion, Vector3,PoseStamped, TwistStamped,PoseArray
from mavros_msgs.msg import AttitudeTarget, Thrust, State, PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import pdb
import sys

class position_control:
    des_waypoint = Waypoint()
    gps_pose = PoseStamped()
    last_got_image_time = 0.0
    got_image_time = 0.0
    start_to_track = True
    switch_control = False
    num = 0

    def __init__(self,Environment,num,total_robots_num):
        rospy.init_node('modquad'+num)
        self.modquad_func_lib = modquad(Environment,num,total_robots_num)
        self.num = int(num)

        '''
        -------------------------------------------Subscriber----------------------------------------------------
        '''
        rospy.Subscriber('/modquad'+num+'/waypoint', Waypoint, callback=self.waypoint_cb)
        rospy.Subscriber('/modquad' + num + '/whycon' + num + '/poses', PoseArray, callback=self.image_detection_cb)
        rospy.Subscriber('/modquad'+num+'/switch_control', Bool, callback=self.switch_control_cb)
        rospy.Subscriber('/modquad' + num + '/mavros'+num+'/local_position/pose', PoseStamped, callback=self.pose_cb)

        self.setpoint_pub = rospy.Publisher('/modquad' + num + '/mavros' + num + '/setpoint_position/local', PoseStamped, queue_size = 10)

        self.pose_vel_setpoint_pub = rospy.Publisher('/modquad' + num + '/mavros' + num + '/setpoint_raw/local', PositionTarget, queue_size = 10)


	'''
        ------------------------------------------Service Proxy--------------------------------------------------
        '''
         
        rospy.wait_for_service('/modquad'+num+'/track')
        Track_Service = rospy.ServiceProxy('/modquad'+num+'/track', track)

        rospy.wait_for_service('/modquad'+num+'/dock')
        Dock_Service = rospy.ServiceProxy('/modquad'+num+'/dock', dock)
        	
        rospy.wait_for_service('/modquad'+num+'/join_groups')
        Join_Group_Service = rospy.ServiceProxy('/modquad'+num+'/join_groups', set_group)

        time_init = 0.0
        yaw_des = 0.0
        track_dock_init = False

	docked = False
        dock_flag = False
	docked_state = False
        tag_detected = False

	self.modquad_func_lib.pub_docked_state_to_px4(False)
	rate = rospy.Rate(60)  # Hz
	
        while not rospy.is_shutdown():
            tag_detected = self.modquad_func_lib.check_tag_detection(self.got_image_time)
            [dock_flag,dock_method] = self.modquad_func_lib.check_dock_flag()
            track_flag = self.modquad_func_lib.check_track_flag()
            self.modquad_func_lib.run_dock_detector()
            docked_state = self.modquad_func_lib.check_dock_state()

            #print self.modquad_func_lib.get_joined_groups()

	    #print track_flag
            if track_flag&dock_flag&tag_detected:
                if docked_state.data:
                    self.modquad_func_lib.pub_docked_state_to_px4(docked_state)
                    Track_Service(False, 
				'back', 
				0.0, 
				self.modquad_func_lib.get_target_ip())#TODO: what the hell is this? pass correct angle and get rid of that 0.0
                    Dock_Service(False, 
				'back', 
				0.0, 
				'dock_finish', 
				self.modquad_func_lib.get_target_ip())
                    rospy.loginfo("---- The quadrotors are docked! ----")
                else:
                        if dock_method == "trajectory":
                            rospy.loginfo("Tag detection OK. Quadrotor beginning to dock.")
                            if track_dock_init == True:
                                dock_waypoint = Waypoint(-0.35, 0.0, 0.0, 0.0, False)
                                #0.35 = length of one side of quad + camera-side distance in m
				#remember that modquad has a square frame, so it's the length of one side
                                start_point = self.modquad_func_lib.get_start_position_tag_frame()
				#get position of tag in relation to quad frame
                                t0 = 0
                                tf = 30 #timeout of 6 seconds?
                                self.modquad_func_lib.two_pts_trajectory_init(start_point, dock_waypoint,t0,tf) #solve Ax = b for A and calculate x,y,z, and yaw coefficients 
                                time_init = rospy.get_time()
                                Dock_Service(True,
						'back', 
						0.0, 
						dock_method, 
						self.modquad_func_lib.get_target_ip())
                                track_dock_init = False

	 		    odom = self.modquad_func_lib.get_current_vision_odom() #get current tag position

			    time_now = rospy.get_time()
			    t = time_now - time_init

			    des_trajectory = self.modquad_func_lib.two_pts_trajectory_generator(t,dock_waypoint) #calculate position, velocity, and acceleration for destination point

                            self.modquad_func_lib.track_and_trajectory_dock_control(des_trajectory,
										odom,
										yaw_des,
										time_now, 
										self.num) 
			    #do control for going from this position (odom) 
			    # to destination (des_trajectory)

			    #track_and_trajectory_dock_control publishes attitude and 
			    #thrust commands to mavros after control calculations
                            Dock_Service(True,
					'back', 
					0.0, 
					dock_method, 
					self.modquad_func_lib.get_target_ip())

	 		    #odom = self.modquad_func_lib.get_current_vision_odom() #get current tag position
                        else:
                            rospy.logerr("Dock method invalid. Dock service Cancelled.")
                            Dock_Service(False, 
					'back', 
					0.0, 
					"Invalid", 
					self.modquad_func_lib.get_target_ip())
                            track_dock_init = False

            elif track_flag&dock_flag&(not tag_detected):
                rospy.logerr("Tag lost while attempting to dock!.")
                Track_Service(False, 
				'back', 
				0.0, 
				self.modquad_func_lib.get_target_ip())
                Dock_Service(False, 
				'back', 
				0.0, 
				'Invalid', 
				self.modquad_func_lib.get_target_ip())
                track_dock_init = False

            elif track_flag & (not dock_flag) & (not tag_detected):
                rospy.logerr("Tag lost while attempting to track!.")
                Track_Service(False, 
				'back', 
				0.0, 
				self.modquad_func_lib.get_target_ip())
                self.modquad_func_lib.pub_docked_state_to_px4(False)
                track_dock_init = False

            elif track_flag&(not dock_flag)&tag_detected:
                rospy.loginfo("The tag is detected and the quad starts to track")
                if track_dock_init == False:
                    track_waypoint = Waypoint(-1.0, 0.0, 0.0, 0.0, False) #this is RELATIVE to the hovering quad
                    start_point = self.modquad_func_lib.get_start_position_tag_frame()
                    t0 = 0
                    tf = 6 
                    self.modquad_func_lib.two_pts_trajectory_init(start_point,track_waypoint,t0,tf)
			#generate trajectory for quad to go to position slightly behind the tag (getting ready for dock)
                    time_init = rospy.get_time()
                    Track_Service(True,
				'back', 
				0.0, 
				self.modquad_func_lib.get_target_ip())
                    track_dock_init = True

                odom = self.modquad_func_lib.get_current_vision_odom()

	        time_now = rospy.get_time()
                t = time_now - time_init

                des_trajectory=self.modquad_func_lib.two_pts_trajectory_generator(t,track_waypoint)
                self.modquad_func_lib.track_and_trajectory_dock_control(des_trajectory,
									odom,
									yaw_des,
									time_now,
									self.num)
                print(des_trajectory)                

                Track_Service(True,
				'back', 
				0.0, 
				self.modquad_func_lib.get_target_ip())

            else: 
		#this is the case where modquad didn't receive any tracking/docking requests
		#instead, this is for sending generic waypoints to a modquad
		#print "I am in the cooperative control loop"
                if self.switch_control: #Here the state should be gotten from the ros topic instead of internal state variable
                    #TODO: make service to get position of other quad for get_average_pose:

                    if self.des_waypoint.updated:
                            start_point = self.modquad_func_lib.get_start_position_local()
                            t0 = 0
                            tf = 3
                            self.modquad_func_lib.two_pts_trajectory_init(start_point, self.des_waypoint,t0,tf)
                            self.des_waypoint.updated = False
                            time_init = rospy.get_time()

                    t = rospy.get_time() - time_init

                    pose = self.modquad_func_lib.get_average_pose()
                    vel = self.modquad_func_lib.get_average_vel()

                    des_trajectory = self.modquad_func_lib.two_pts_trajectory_generator(t, self.des_waypoint)
                    self.modquad_func_lib.modquad_control(des_trajectory, pose, vel, yaw_des)

                else: 
		#this is the case for sending a waypoint to a single quad
                    if self.des_waypoint.updated:
                            start_point = self.modquad_func_lib.get_start_position_local()
                            t0 = 0
                            tf = 3
                            self.modquad_func_lib.two_pts_trajectory_init(start_point, self.des_waypoint,t0,tf)
                            self.des_waypoint.updated = False
                            time_init = rospy.get_time()

	            time_now = rospy.get_time()
                    t = time_now - time_init
		    
                    pose = self.modquad_func_lib.get_current_pose()
                    vel = self.modquad_func_lib.get_current_vel()

                    des_trajectory = self.modquad_func_lib.two_pts_trajectory_generator(t, self.des_waypoint)
                    self.modquad_func_lib.pos_control(des_trajectory, pose, vel, yaw_des, time_now)
    	    rate.sleep()

    def waypoint_cb(self,msg):
        self.des_waypoint.x = msg.x
        self.des_waypoint.y = msg.y
        self.des_waypoint.z = msg.z
        self.des_waypoint.yaw = msg.yaw
        self.des_waypoint.updated = True

    def image_detection_cb(self,msg):
        self.got_image_time = rospy.get_time()

    def switch_control_cb(self,msg):
	    self.switch_control = msg.data

    def pose_cb(self,msg):
	    self.gps_pose = msg

if __name__ == "__main__":
    modquad_id = str(rospy.get_param("ip_addr"))
    Environment = rospy.get_param("Environment")
    modquad_no = rospy.get_param("RobotsNumber")
    position_control(Environment,modquad_id,modquad_no)
