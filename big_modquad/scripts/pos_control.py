#!/usr/bin/env python
"""
Author: Guanrui Li <lguanrui@seas.upenn.edu>
This is the position control node.
"""
import rospy
from modquad.srv import *
from modquad.msg import *
from modquad_init import modquad
from geometry_msgs.msg import Vector3, PoseStamped, TwistStamped, PoseArray
from gazebo_magnet.srv import *
from mavros_msgs.msg import AttitudeTarget, Thrust
from std_msgs.msg import Bool
import numpy as np

class position_control:
    des_waypoint = Waypoint()
    gps_pose = PoseStamped()
    last_got_image_time = 0.0
    got_image_time = 0.0
    switch_control = False
    num = 0

    def __init__(self,Environment,num,total_robots_num):
        rospy.init_node('modquad'+num)
        self.modquad = modquad(Environment,num,total_robots_num)
        self.num = int(num)

        '''
        -------------------------------------------Subscriber----------------------------------------------------
        '''
        rospy.Subscriber('/modquad'+num+'/waypoint', Waypoint, callback=self.waypoint_cb)
        rospy.Subscriber('/modquad' + num + '/whycon' + num + '/poses', PoseArray, callback=self.image_detection_cb)
        rospy.Subscriber('/modquad'+num+'/switch_control', Bool, callback=self.switch_control_cb)
        rospy.Subscriber('/modquad' + num +'/corrected_local_pose', PoseStamped, callback=self.pose_cb)

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

        dock_flag = False
	docked_state = False
        tag_detected = False

	self.modquad.pub_docked_state_to_px4(False)
	rate = rospy.Rate(100)  # Hz
	
        while not rospy.is_shutdown():
            tag_detected = self.modquad.check_tag_detection(self.got_image_time)
            [dock_flag,dock_method] = self.modquad.check_dock_flag()
            track_flag = self.modquad.check_track_flag()
            self.modquad.run_dock_detector()
            docked_state = self.modquad.check_dock_state()

            if track_flag&dock_flag&tag_detected:
                if docked_state.data:
                    #self.modquad.pub_docked_state_to_px4(docked_state)
                    Track_Service(False, 
				self.modquad.get_dock_side(),
				self.modquad.get_tag_angle(),
				self.modquad.get_target_ip())
                    Dock_Service(False, 
				self.modquad.get_dock_side(),
				self.modquad.get_tag_angle(),
				'dock_finish', 
				self.modquad.get_target_ip())

                    rospy.loginfo("---- The quadrotors are docked! ----")
                else:
                        if dock_method == "trajectory":
                            rospy.loginfo("Tag detection OK. Quadrotor beginning to dock.")
                            if track_dock_init == True:
                                dock_waypoint = Waypoint(-0.2, 
							0.06, 
							-0.15, 
				                        -self.modquad.get_tag_angle(),
							False)
                                #0.2 = length of one side of quad + camera-side distance in m
				#remember that modquad has a square frame, so it's the length of one side
                                start_point = self.modquad.get_start_position_tag_frame()
				#get position of tag in relation to quad frame
                                t0 = 0
                                tf = 6 #timeout of 6 seconds?
                                self.modquad.two_pts_trajectory_init(start_point, dock_waypoint,t0,tf) #solve Ax = b for A and calculate x,y,z, and yaw coefficients 
                                time_init = rospy.get_time()
                                Dock_Service(True,
				                self.modquad.get_dock_side(),
				                self.modquad.get_tag_angle(),
						dock_method, 
						self.modquad.get_target_ip())
                                track_dock_init = False

	 		    odom = self.modquad.get_current_vision_odom() #get current tag position

			    time_now = rospy.get_time()
			    t = time_now - time_init

			    des_trajectory = self.modquad.two_pts_trajectory_generator(t,dock_waypoint) #calculate position, velocity, and acceleration for destination point

                            self.modquad.track_and_trajectory_dock_control(des_trajectory,
										odom,
										yaw_des,
										time_now, 
										self.num) 
			    #do control for going from this position (odom) 
			    # to destination (des_trajectory)

			    #track_and_trajectory_dock_control publishes attitude and 
			    #thrust commands to mavros after control calculations
                            Dock_Service(True,
				        self.modquad.get_dock_side(),
				        self.modquad.get_tag_angle(),
					dock_method, 
					self.modquad.get_target_ip())
                        else:
                            rospy.logerr("Dock method invalid. Dock service Cancelled.")
                            Dock_Service(False, 
				        self.modquad.get_dock_side(),
				        self.modquad.get_tag_angle(),
					"Invalid", 
					self.modquad.get_target_ip())
                            track_dock_init = False

            elif track_flag&dock_flag&(not tag_detected):
                rospy.logerr("Tag lost while attempting to dock!.")
                Track_Service(False, 
				self.modquad.get_dock_side(),
				self.modquad.get_tag_angle(),
				self.modquad.get_target_ip())
                Dock_Service(False, 
				self.modquad.get_dock_side(),
				self.modquad.get_tag_angle(),
				'Invalid', 
				self.modquad.get_target_ip())
                track_dock_init = False

            elif track_flag & (not dock_flag) & (not tag_detected):
                rospy.logerr("Tag lost while attempting to track!.")
                Track_Service(False, 
				self.modquad.get_dock_side(),
				self.modquad.get_tag_angle(),
				self.modquad.get_target_ip())
                self.modquad.pub_docked_state_to_px4(False)
                track_dock_init = False

            elif track_flag&(not dock_flag)&tag_detected:
                rospy.loginfo("The tag is detected and the quad starts to track")
                if track_dock_init == False:
                    track_waypoint = Waypoint(-0.8, 
						0.06, 
						-0.15, 
				                -self.modquad.get_tag_angle(),
						False)
		    #this is RELATIVE to the hovering quad, we also account for camera height
                    #dw about the angle, it's compensated for in the trajectory controller
                    start_point = self.modquad.get_start_position_tag_frame()
                    t0 = 0
                    tf = 6 
                    self.modquad.two_pts_trajectory_init(start_point,track_waypoint,t0,tf)
		    #generate trajectory for quad to go to position slightly behind the tag (getting ready for dock)
                    time_init = rospy.get_time()
                    Track_Service(True,
				self.modquad.get_dock_side(),
				self.modquad.get_tag_angle(),
				self.modquad.get_target_ip())
                    track_dock_init = True

                odom = self.modquad.get_current_vision_odom()

	        time_now = rospy.get_time()
                t = time_now - time_init

                des_trajectory=self.modquad.two_pts_trajectory_generator(t,track_waypoint)
                self.modquad.track_and_trajectory_dock_control(des_trajectory,
									odom,
									yaw_des,
									time_now,
									self.num)

                Track_Service(True,
				self.modquad.get_dock_side(),
				self.modquad.get_tag_angle(),
				self.modquad.get_target_ip())

            else: 
		#this is the case where modquad didn't receive any tracking/docking requests
		#instead, this is for sending generic waypoints to a modquad
                if self.switch_control: #Here the state should be gotten from the ros topic instead of internal state variable
                    if self.des_waypoint.updated:
                            start_point = self.modquad.get_start_position_local()
                            t0 = 0
                            tf = 3
                            self.modquad.two_pts_trajectory_init(start_point, self.des_waypoint,t0,tf)
                            self.des_waypoint.updated = False
                            time_init = rospy.get_time()

                    t = rospy.get_time() - time_init

                    pose = self.modquad.get_average_pose()
                    vel = self.modquad.get_average_vel()

                    des_trajectory = self.modquad.two_pts_trajectory_generator(t, self.des_waypoint)
                    self.modquad.modquad_control(des_trajectory, pose, vel, yaw_des)

                else: 
		#this is the case for sending a waypoint to a single quad
                    if self.des_waypoint.updated:
                            start_point = self.modquad.get_start_position_local()
                            t0 = 0
                            tf = 3
                            self.modquad.two_pts_trajectory_init(start_point, self.des_waypoint,t0,tf)
                            self.des_waypoint.updated = False
                            time_init = rospy.get_time()

	            time_now = rospy.get_time()
                    t = time_now - time_init
		    
                    pose = self.modquad.get_current_pose(self.num)
                    vel = self.modquad.get_current_vel(self.num)

                    des_trajectory = self.modquad.two_pts_trajectory_generator(t, self.des_waypoint)
                    self.modquad.pos_control(des_trajectory, pose, vel, yaw_des, time_now)
    	    rate.sleep()

    def waypoint_cb(self,msg):
        self.des_waypoint = msg
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
