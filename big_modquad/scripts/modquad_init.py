#!/usr/bin/env python
'''
Author: Guanrui Li lguanrui@seas.upenn.edu
'''

import rospy
from modquad.srv import *
from modquad.msg import *
from geometry_msgs.msg import Vector3, Pose, PoseStamped, TwistStamped, PoseArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget, Thrust, State, CooperativeControl
import numpy as np
import pdb
import tf
from tf.transformations import euler_from_quaternion as qua2eu 
from tf.transformations import euler_matrix as eumat 
import numpy.linalg as LA
from filter import kalmanfilter

class modquad:
    curr_pose = PoseStamped()
    curr_vel = TwistStamped()
    dock_curr_pose  = PoseStamped()
    dock_curr_vel  = TwistStamped()
    wait_curr_pose  = PoseStamped()
    wait_curr_vel  = TwistStamped()
    average_pose = PoseStamped()
    average_vel = TwistStamped()
    curr_Imu = Imu()
    curr_vision_odom = VisionOdom()
    Imu_queue = np.zeros(10)
    dock_flag = False
    dock_state = False
    track_flag = False
    dock_method = "Invalid"
    intx = 0.0
    inty = 0.0
    intz = 0.0
    tag_pos_in_cam = np.matrix([0,0,0,1]).transpose()
    A_init_flag = False
    isReadyToFly = False
    total_robots_num = 1
    docked = False

    def __init__(self,Environment,num,total_robots_num):
        self.pos_control_param_init()
        self.track_control_param_init()
        self.quadrotor_physical_parameter(num)
        self.num = int(num)
	self.total_robots_num = total_robots_num
        self.robot_list = rospy.get_param("robot_list") 
        self.SendWaypoint_Service = {}
        self.switch_control_hash_pub = {}
        self.joined_group_hash = {}
        self.set_control_hash = {}
        self.joined_groups = [self.num]
        '''
        ---------------------------------------Service initialization----------------------------------------------
        '''
        send_waypoint_service_init = rospy.Service('/modquad' + num + '/send_waypoint', sendwaypoint, self.handle_send_waypoint)
        send_groups_service_init = rospy.Service('/modquad' + num + '/join_groups', set_group, self.handle_groups)
        send_struct_waypoint_service_init = rospy.Service('/modquad' + num + '/send_struct_waypoint', sendwaypoint_struct, self.handle_send_struct_waypoint)
        takoff_service_init = rospy.Service('/modquad' + num + '/take_off', takeoff, self.handle_take_off)
        dock_service_init = rospy.Service('/modquad' + num + '/dock', dock, self.handle_dock)
        track_service_init = rospy.Service('/modquad' + num + '/track',track,self.handle_track)

        '''
        -------------------------------------Publisher initialization----------------------------------------------
        '''

        self.waypoint_pub = rospy.Publisher('/modquad' + num + '/waypoint', Waypoint, queue_size=10)
	self.docked_pub = rospy.Publisher('/modquad' + num + '/modquad_docked', Bool, queue_size=10)
	self.modquad_switch_control_pub = rospy.Publisher('/modquad' + num + '/switch_control', Bool, queue_size=10)
        '''
        -------------------------------------Subscriber initialization---------------------------------------------
        '''
        if Environment == "gps":
            rospy.logwarn("---- MODQUAD USING GPS ----")
            rospy.Subscriber('/modquad' + num + '/mavros' + num + '/local_position/pose', PoseStamped, callback=self.pose_cb)
            rospy.Subscriber('/modquad' + num + '/mavros' + num + '/local_position/velocity', TwistStamped, callback=self.vel_cb)

        elif (Environment == "mocap"):
            rospy.logwarn("---- MODQUAD USING MOCAP ----")
            rospy.Subscriber('/modquad' + num + '/mavros' + num + '/vision_pose/pose', PoseStamped, callback=self.pose_cb)

        rospy.Subscriber('/modquad' + num + '/whycon' + num + '/poses',PoseArray,callback = self.vision_goal_cb)
        rospy.Subscriber('/modquad' + num + '/filtered_Vision_Odom',VisionOdom,callback = self.vision_odom_cb)
        rospy.Subscriber('/modquad' + num + '/mavros' + num + '/imu/data',Imu,callback = self.Imu_cb)
	rospy.Subscriber('/modquad' + num + '/modquad_docked', Bool, callback = self.dock_state_cb)

        '''
        ---------------------------Service Proxy initialization-----------------------------------
        '''
        #self.robot_list = [ord(encoded) for encoded in self.robot_list if type(encoded) is str] #decode from string to int
        for uavID in self.robot_list:
	    self.switch_control_hash_pub[uavID] = rospy.Publisher('/modquad' + str(uavID) + '/switch_control', Bool, queue_size=10)
            rospy.wait_for_service('/modquad'+ str(uavID) +'/join_groups')
            self.joined_group_hash[uavID] = rospy.ServiceProxy('modquad'+str(uavID)+'/join_groups', set_group)
            rospy.wait_for_service('/modquad'+ str(uavID) +'/send_waypoint')
            self.SendWaypoint_Service[uavID] = rospy.ServiceProxy('/modquad/modquad'+ str(uavID) +'/send_waypoint', sendwaypoint)
            self.set_control_hash[uavID] = rospy.Publisher('/modquad' + str(uavID) + '/mavros'+ str(uavID) +'/modquad_control/control_flag', CooperativeControl, queue_size = 10)

        self.SendWaypoint_Struct_Service = rospy.ServiceProxy('/modquad'+num+'/send_struct_waypoint', sendwaypoint_struct)
        rospy.loginfo("INITIALIZATION FINISHED!")

    def pos_control_param_init(self,):
        self.Kp = Vector3()
        self.Kd = Vector3()
	self.ki = Vector3()
        self.Kp.x = 3.00
        self.Kp.y = 3.00
        self.Kp.z = 3.2
        self.Kd.x = 1.95 #1.75
        self.Kd.y = 1.75 #1.45
        self.Kd.z = 1.0
        self.ki.x = 0.01
        self.ki.y = 0.01
        self.ki.z = 0.050

    def track_control_param_init(self, ):
        self.Kp_track = Vector3()
        self.Kd_track = Vector3()
        self.Ki_track = Vector3()
        self.Kp_track.x = 3.00 #2.50
        self.Kp_track.y = 3.00 #2.50
        self.Kp_track.z = 3.2  #2.80
        self.Kd_track.x = 1.95 #1.45 
        self.Kd_track.y = 1.75 #1.45
        self.Kd_track.z = 2.5  #1.8
        self.Ki_track.x = 0.01
        self.Ki_track.y = 0.01
        self.Ki_track.z = 0.050

    def quadrotor_physical_parameter(self,num):
        '''
        mass in simulation: self.m = 0.0625
        '''
        self.g = 9.81
        self.H_cam_quad1 = np.matrix([[0,0,1,0.106],[-1,0,0,0],[0,-1,0,0.07],[0,0,0,1]])

    '''
    -------------------------------------------------Service handle------------------------------------------------
    '''
    def handle_send_struct_waypoint(self,req): #sends the same waypoint to all robots in a for loop
        req.Robot_list = [ord(encoded) for encoded in req.Robot_list if type(encoded) is str] #decode from string to int
        if not all(robot_num in self.robot_list for robot_num in req.Robot_list):
            rospy.logwarn("REQUEST DENIED: Invalid robots in waypoint request.")
            return sendwaypoint_structResponse(False)
        for ID in req.Robot_list:
            self.SendWaypoint_Service[ID](req.x,req.y,req.z,req.yaw)
            rospy.loginfo("Sent waypoint [%s, %s, %s, %s] to robot %s",req.x,req.y,req.z,req.yaw,ID)
        return sendwaypoint_structResponse(True)
    
    def handle_send_waypoint(self,req): #sends waypoint to one quad
        rospy.loginfo("Sent waypoint is [%s, %s, %s, %s]", req.x,req.y,req.z,req.yaw)
        waypoint = Waypoint() 
        waypoint.x = req.x
        waypoint.y = req.y
        waypoint.z = req.z
        waypoint.yaw = req.yaw
        self.waypoint_pub.publish(waypoint)
        return sendwaypointResponse(True)

    def handle_groups(self, req):
        req.groups = [ord(encoded) for encoded in req.groups if type(encoded) is str] #decode from string to int
        self.set_joined_groups(req.groups)
        return set_groupResponse("Group successfully joined")

    def handle_take_off(self,req): #sends command to take off to one quad
        waypoint = Vector3()
        waypoint.x = self.curr_pose.pose.position.x
        waypoint.y = self.curr_pose.pose.position.y
        waypoint.z = req.height
        waypoint.yaw = req.yaw
        self.waypoint_pub.publish(waypoint)
        return takeoffResponse(1,True)

    def handle_dock(self, req):
        if self.docked:
          return dockResponse('This robot is currently docked and cannot take orders.')
        self.target_ip = req.target_ip
        if self.target_ip not in self.robot_list:
          return dockResponse('Target robot does not exist!')
        self.dock_flag = req.dock_flag
        self.dock_method = req.method
        self.waitmod_yaw = req.waitmod_yaw
        if req.dock_side == 'left':
           self.tag_angle = 3*np.pi/2
        elif req.dock_side == 'back':
           self.tag_angle = 0
        elif req.dock_side == 'right':
           self.tag_angle =  np.pi/2 
        else: 
           return dockResponse("The angle is not legit")

        if req.dock_flag & (req.method == 'trajectory'):
            return dockResponse("The current docking method is " + req.method)
        elif ((not req.dock_flag) & (req.method == 'dock_finish')): 
            self.docked = True
            self.modquad_switch_control_pub.publish(True)
            self.switch_control_hash_pub[self.target_ip].publish(True)
            self.get_average_pose()
	    self.get_average_vel()
            orien = self.curr_pose.pose.orientation
            euler = qua2eu([orien.x,orien.y,orien.z,orien.w],'sxyz')
            yaw = euler[2] 
            rospy.logwarn("Publish switch control")
            self.SendWaypoint_Struct_Service(self.robot_list,self.average_pose.pose.position.x, self.average_pose.pose.position.y, self.average_pose.pose.position.z,yaw)
            self.joined_group_hash[self.target_ip](self.joined_groups) #TODO: initiate service here to get joined groups from target quad
            return dockResponse("The dock is finished, switch to cooperative control")
        else:
            return dockResponse("Dock cancelled.")

    def handle_track(self, req):
        if self.docked:
          return trackResponse('This robot is currently docked and cannot take orders.')
        self.target_ip = req.target_ip
        if self.target_ip not in self.robot_list:
          return trackResponse('Target robot does not exist!')
        self.track_flag = req.track_flag
        self.waitmod_yaw = req.waitmod_yaw
        if req.dock_side == 'left':
           self.tag_angle = 3*np.pi/2
        elif req.dock_side == 'back':
           self.tag_angle = 0
        elif req.dock_side == 'right':
           self.tag_angle =  np.pi/2 
        else: 
           return trackResponse("The angle is not legit")

        if req.track_flag:
            return trackResponse('Quadrotor starts to track the tag. Initializing the trajectory')
        else:
            orien = self.curr_pose.pose.orientation
            euler = qua2eu([orien.x,orien.y,orien.z,orien.w],'sxyz')
            yaw = euler[2]
            self.SendWaypoint_Service[self.num](self.curr_pose.pose.position.x - 0.5, self.curr_pose.pose.position.y, self.curr_pose.pose.position.z + 0.2,yaw)
            return trackResponse('Disabling track and rejecting tracking trajectory initialization')

    '''
    ------------------------------------------ Subscriber Callback function ----------------------------------------
    '''

    def pose_cb(self,msg):
        self.curr_pose = msg

    def vel_cb(self,msg):
        self.curr_vel = msg

    def vision_goal_cb(self,msg):
        self.tag_pos_in_cam = np.matrix([msg.poses[0].position.x,msg.poses[0].position.y,\
                                              msg.poses[0].position.z,1]).transpose()
        self.curr_tag_position = np.matrix([msg.poses[0].position.x,msg.poses[0].position.y,\
                                              msg.poses[0].position.z]).transpose()

    def mocap_odom_cb(self,msg):
        self.curr_pose.header = msg.header
        self.curr_vel.header = msg.header
        self.curr_pose.pose = msg.pose.pose
        self.curr_vel.twist = msg.twist.twist

    def mocap_odom_dock_robot_cb(self,msg):
        self.dock_curr_pose.header = msg.header
        self.dock_curr_vel.header = msg.header
        self.dock_curr_pose.pose = msg.pose.pose
        self.dock_curr_vel.twist = msg.twist.twist

    def mocap_odom_wait_robot_cb(self,msg):
        self.wait_curr_pose.header = msg.header
        self.wait_curr_vel.header = msg.header
        self.wait_curr_pose.pose = msg.pose.pose
        self.wait_curr_vel.twist = msg.twist.twist

    def Imu_cb(self, msg):
        self.curr_Imu = msg

    def vision_odom_cb(self,msg):
        self.curr_vision_odom = msg

    def dock_state_cb(self,msg):
	self.dock_state = msg

    '''
    ------------------------------------------- Package get functions -------------------------------------------
    '''
    def get_target_ip(self):
        return self.target_ip

    def get_current_pose(self,):
        return self.curr_pose

    def get_current_vel(self,):
        return self.curr_vel

    def get_average_pose(self,):
        self.average_pose.pose.position.x = (self.dock_curr_pose.pose.position.x + self.wait_curr_pose.pose.position.x)/2.0
        self.average_pose.pose.position.y = (self.dock_curr_pose.pose.position.y + self.wait_curr_pose.pose.position.y)/2.0
        self.average_pose.pose.position.z = (self.dock_curr_pose.pose.position.z + self.wait_curr_pose.pose.position.z)/2.0
        self.average_pose.header = self.dock_curr_pose.header
        self.average_pose.pose.orientation = self.dock_curr_pose.pose.orientation
        return self.average_pose

    def get_average_vel(self,):
        self.average_vel.twist.linear.x = (self.dock_curr_vel.twist.linear.x + self.wait_curr_vel.twist.linear.x)/2.0
        self.average_vel.twist.linear.y = (self.dock_curr_vel.twist.linear.y + self.wait_curr_vel.twist.linear.y)/2.0
	self.average_vel.twist.linear.z = (self.dock_curr_vel.twist.linear.z + self.wait_curr_vel.twist.linear.z)/2.0
	self.average_vel.header = self.dock_curr_vel.header
        self.average_vel.twist.angular = self.dock_curr_vel.twist.angular
        return self.average_vel

    def get_current_vision_odom(self,):
        return self.curr_vision_odom

    def get_start_position_local(self,):
        start_point = Waypoint()
        start_point.x = self.curr_pose.pose.position.x
        start_point.y = self.curr_pose.pose.position.y
        start_point.z = self.curr_pose.pose.position.z
        orien = self.curr_pose.pose.orientation
        euler = qua2eu([orien.x,orien.y,orien.z,orien.w],'sxyz')
        start_point.yaw = euler[2] 
        start_point.updated = False
        return start_point

    def get_start_position_tag_frame(self,):
        start_point = Pose()
        start_point.position = self.curr_vision_odom.position
        start_point.orientation = self.curr_vision_odom.orientation
        return start_point

    def get_Imu(self,):
        return self.curr_Imu

    def get_dock_position(self, curr_pose):
        dock_position = Waypoint()
        curr_quaternion = [curr_pose.pose.orientation.x, curr_pose.pose.orientation.y,
                           curr_pose.pose.orientation.z, curr_pose.pose.orientation.w]
        H_quad_local = np.matrix(tf.transformations.quaternion_matrix(curr_quaternion))
        r_quad_local = np.matrix(
            [curr_pose.pose.position.x, curr_pose.pose.position.y, curr_pose.pose.position.z]).transpose()

        H_quad_local[:3, 3] = r_quad_local

        r_tag_local = H_quad_local * self.H_cam_quad1 * self.tag_pos_in_cam

        dock_position.x = r_tag_local[0]
        dock_position.y = r_tag_local[1]
        dock_position.z = r_tag_local[2]
        dock_position.updated = True
        #print dock_position
        return dock_position

    def get_joined_groups(self):
        return self.joined_groups

    def set_joined_groups(self, new_group):
        self.joined_groups.extend(new_group)

    def dock_detector(self,curr_odom,curr_Imu):
        '''	
	pos_x  = curr_odom.position.x
	pos_y  = curr_odom.position.y
	pos_z  = curr_odom.position.z
        '''	

	#pos_x  = curr_odom[0]
	#pos_y  = curr_odom[1]
	#pos_z  = curr_odom[2]
        try:  
               jump = np.sqrt(curr_Imu.linear_acceleration.x**2 
                       + curr_Imu.linear_acceleration.y**2
                       + (curr_Imu.linear_acceleration.z - 9.81)**2)
        except OverflowError:
               jump = 0

	np.roll(self.Imu_queue, 1)
	self.Imu_queue[0] = jump

	Imu_norm = np.linalg.norm(self.Imu_queue, 2)

	if curr_odom[2] < 0.2 and Imu_norm > 2.1: #2.1 is arbitrary, but looking for IMU jump when z distance is less than 20cm
		dock_state = True
	else:
		dock_state = False
	self.docked_pub.publish(dock_state)
	return dock_state

    '''
    --------------------------------------------Check function-----------------------------------------------------
    '''

    def check_tag_detection(self,got_image_time):
        Delta_t = rospy.get_time() - got_image_time
        if Delta_t > 0.5:
            #Tag detection is too slow or there is no tag detected by the camera.
            image_detection = False
        else:
            image_detection = True
        return image_detection

    def check_dock_flag(self,):
        dock_flag = [self.dock_flag, self.dock_method]
        return dock_flag

    def check_track_flag(self,):
        return self.track_flag

    def run_dock_detector(self,):
	self.dock_detector(self.tag_pos_in_cam, self.curr_Imu)

    def check_dock_state(self,):
	    return self.dock_state
	
    def pub_docked_state_to_px4(self,dock_state):
        if dock_state:
            self.set_control_hash[self.num].publish(CooperativeControl(-1.0,-1.0,-1.0,-1.0,True))
            self.set_control_hash[int(self.target_ip)].publish(CooperativeControl(1.0,1.0,1.0,1.0,True))
        else:
            self.set_control_hash[self.num].publish(CooperativeControl(1.0,-1.0,1.0,-1.0,True))
            rospy.loginfo("The modules are not docked yet, fail to change control.")
