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
        rospy.Subscriber('/modquad/whycon' + num + '/poses', PoseArray, callback=self.image_detection_cb)
        rospy.Subscriber('/modquad'+num+'/switch_control', Bool, callback=self.switch_control_cb)
        rospy.Subscriber('/mavros'+num+'/local_position/pose', PoseStamped, callback=self.pose_cb)

        self.setpoint_pub = rospy.Publisher('/mavros' + num + '/setpoint_position/local', PoseStamped, queue_size = 10)

        self.pose_vel_setpoint_pub = rospy.Publisher('/mavros' + num + '/setpoint_raw/local', PositionTarget, queue_size = 10)


	'''
        ------------------------------------------Service Proxy--------------------------------------------------
        '''
         
        rospy.wait_for_service('/modquad/modquad'+num+'/track')
        Track_Service = rospy.ServiceProxy('modquad'+num+'/track', track)

        rospy.wait_for_service('/modquad/modquad'+num+'/dock')
        Dock_Service = rospy.ServiceProxy('modquad'+num+'/dock', dock)
        	
        rospy.wait_for_service('/modquad/modquad'+num+'/join_groups')
        Join_Group_Service = rospy.ServiceProxy('modquad'+num+'/join_groups', set_group)
        	
	docked = False
        dock_flag = False
	docked_state = False
        tag_detected = False

	self.modquad_func_lib.pub_docked_state_to_px4(False)
	rate = rospy.Rate(30)  # Hz
	
        while not rospy.is_shutdown():
            tag_detected = self.modquad_func_lib.check_tag_detection(self.got_image_time)
            [dock_flag,dock_method] = self.modquad_func_lib.check_dock_flag()
            track_flag = self.modquad_func_lib.check_track_flag()
            #This line should run in the docking module. 
            self.modquad_func_lib.run_dock_detector()
            docked_state = self.modquad_func_lib.check_dock_state()

            #print self.modquad_func_lib.get_joined_groups()

	    #print track_flag
            if track_flag&dock_flag&tag_detected:
                if docked_state.data:
                    self.modquad_func_lib.pub_docked_state_to_px4(docked_state)
                    Track_Service(False, 'back', 0.0, self.modquad_func_lib.get_target_ip())#TODO: what the hell is this? pass correct angle and get rid of that 0.0
                    Dock_Service(False, 'back', 0.0, 'dock_finish', self.modquad_func_lib.get_target_ip())
                    rospy.loginfo("---- The quadrotors are docked! ----")
                else:
                        if dock_method == "trajectory":
                            rospy.loginfo("Tag detection OK. Quadrotor beginning to dock.")
                            start_point = self.modquad_func_lib.get_start_position_tag_frame()
			    #get position of tag in relation to quad frame
                            dock_waypoint = PositionTarget()
                            dock_waypoint.header.stamp = rospy.get_rostime()
                            dock_waypoint.header.frame_id = "map"
                            dock_waypoint.coordinate_frame = PositionTarget.FRAME_BODY_NED
                            dock_waypoint.type_mask = PositionTarget.IGNORE_YAW_RATE | \
                                                      PositionTarget.IGNORE_AFX | \
                                                      PositionTarget.IGNORE_AFY | \
                                                      PositionTarget.IGNORE_AFZ
                            start_point = self.modquad_func_lib.get_start_position_tag_frame()
                            dock_waypoint.position.x = self.gps_pose.pose.position.x - start_point.position.x
                            dock_waypoint.position.y = self.gps_pose.pose.position.y - start_point.position.y
                            dock_waypoint.position.z = self.gps_pose.pose.position.z - start_point.position.z
                            dock_waypoint.velocity = Vector3(0.1, 0.1, 0.1)
                            quat = self.gps_pose.pose.orientation
                            dock_waypoint.yaw = euler_from_quaternion([quat.x,
                                                                       quat.y,
                                                                       quat.z,
                                                                       quat.w])[2]
                            self.pose_vel_setpoint_pub.publish(dock_waypoint)
                            Dock_Service(True,'back', 0.0, dock_method, self.modquad_func_lib.get_target_ip())

	 		    #odom = self.modquad_func_lib.get_current_vision_odom() #get current tag position
                        else:
                            rospy.logerr("Dock method invalid. Dock service Cancelled.")
                            Dock_Service(False, 'back', 0.0, "Invalid", self.modquad_func_lib.get_target_ip())

            elif track_flag&dock_flag&(not tag_detected):
                rospy.logerr("Tag lost while attempting to dock!.")
                Track_Service(False, 'back', 0.0, self.modquad_func_lib.get_target_ip())
                Dock_Service(False, 'back', 0.0, 'Invalid', self.modquad_func_lib.get_target_ip())

            elif track_flag & (not dock_flag) & (not tag_detected):
                rospy.logerr("Tag lost while attempting to track!.")
                Track_Service(False, 'back', 0.0, self.modquad_func_lib.get_target_ip())
                self.modquad_func_lib.pub_docked_state_to_px4(False)

            elif track_flag&(not dock_flag)&tag_detected:
                rospy.loginfo("The tag is detected and the quad starts to track")
                
                track_waypoint = PoseStamped()
                track_waypoint.header.stamp = rospy.get_rostime()
                track_waypoint.header.frame_id = "map"
		#TODO: generalize here for back, right, and left tags, this means also for the np.sign!!!!!!!!!!!!!!!!!!!!
                start_point = self.modquad_func_lib.get_start_position_tag_frame()
                dist = start_point.position.x
                track_waypoint.pose.position.x = self.gps_pose.pose.position.x - dist + np.sign(dist)*1.0
                track_waypoint.pose.position.y = self.gps_pose.pose.position.y #+ start_point.position.y
                track_waypoint.pose.position.z = self.gps_pose.pose.position.z - start_point.position.z - 0.035 - 0.2
                #tag height from EKF is height of camera relative to tag
                #e.g. negative height means camera is below tag
                '''
                dock_waypoint = PositionTarget()
                dock_waypoint.header.stamp = rospy.get_rostime()
                dock_waypoint.header.frame_id = "map"
                dock_waypoint.coordinate_frame = PositionTarget.FRAME_BODY_NED
                dock_waypoint.type_mask = PositionTarget.IGNORE_YAW_RATE | \
                                          PositionTarget.IGNORE_YAW | \
                                          PositionTarget.IGNORE_AFX | \
                                          PositionTarget.IGNORE_AFY | \
                                          PositionTarget.IGNORE_AFZ
                start_point = self.modquad_func_lib.get_start_position_tag_frame()
                dock_waypoint.position.x = self.gps_pose.pose.position.x - start_point.position.x +np.sign(start_point.position.x)
                dock_waypoint.position.y = self.gps_pose.pose.position.y - start_point.position.y
                dock_waypoint.position.z = self.gps_pose.pose.position.z - start_point.position.z -0.035
                dock_waypoint.velocity = Vector3(0.08, 0.08, 0.08)
                quat = self.gps_pose.pose.orientation
                #dock_waypoint.yaw = euler_from_quaternion([quat.x,
                #                                           quat.y,
                #                                           quat.z,
                #                                           quat.w])[2]
                self.pose_vel_setpoint_pub.publish(dock_waypoint)
                '''
                print("--- VISION POSITION ---")
                print(start_point.position)
                print("--- QUAD LOCAL POSITION ---")
                print(self.gps_pose.pose.position)
                print("--- RESULTING POSITION ---")
                print(track_waypoint.pose.position)
                track_waypoint.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))
		#generate trajectory for quad to go to position slightly behind the tag (getting ready for dock)
                self.setpoint_pub.publish(track_waypoint)
                Track_Service(True,'back', 0.0, self.modquad_func_lib.get_target_ip())

            else: 
		#this is the case where modquad didn't receive any tracking/docking requests
		#instead, this is for sending generic waypoints to a modquad
		#print "I am in the cooperative control loop"
                if self.switch_control: #Here the state should be gotten from the ros topic instead of internal state variable
                    #TODO: make service to get position of other quad for get_average_pose:
                    #pose = self.modquad_func_lib.get_average_pose()
                    #vel = self.modquad_func_lib.get_average_vel()

                    actual_waypoint = PoseStamped()
                    actual_waypoint.header.stamp = rospy.get_rostime()
                    actual_waypoint.header.frame_id = "map"
                    actual_waypoint.pose.position.x = self.des_waypoint.x + 0.14 #TODO: temp fix
                    actual_waypoint.pose.position.y = self.des_waypoint.y
                    actual_waypoint.pose.position.z = self.des_waypoint.z
                    actual_waypoint.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, np.radians(self.des_waypoint.yaw)))
                    self.setpoint_pub.publish(actual_waypoint)

                else: 
		#this is the case for sending a waypoint to a single quad
                    #pose = self.modquad_func_lib.get_current_pose()
                    #vel = self.modquad_func_lib.get_current_vel() #I'm pretty sure we don't need velocity

                    actual_waypoint = PoseStamped()
                    actual_waypoint.header.stamp = rospy.get_rostime()
                    actual_waypoint.header.frame_id = "map"
                    actual_waypoint.pose.position.x = self.des_waypoint.x 
                    actual_waypoint.pose.position.y = self.des_waypoint.y
                    actual_waypoint.pose.position.z = self.des_waypoint.z
                    actual_waypoint.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, np.radians(self.des_waypoint.yaw)))
                    self.setpoint_pub.publish(actual_waypoint)
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
