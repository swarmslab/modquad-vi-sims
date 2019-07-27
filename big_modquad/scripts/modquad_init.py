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
        self.mavros_attitude_pub = rospy.Publisher('/modquad' + num + '/mavros'+num+'/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.mavros_thrust_pub = rospy.Publisher('/modquad' + num + '/mavros'+num+'/setpoint_attitude/thrust', Thrust, queue_size=10)
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
            self.SendWaypoint_Service[uavID] = rospy.ServiceProxy('/modquad'+ str(uavID) +'/send_waypoint', sendwaypoint)
            self.set_control_hash[uavID] = rospy.Publisher('/modquad' + str(uavID) + '/mavros'+ str(uavID) +'/modquad_control/control_flag', CooperativeControl, queue_size = 10)

        self.SendWaypoint_Struct_Service = rospy.ServiceProxy('/modquad'+num+'/send_struct_waypoint', sendwaypoint_struct)
        rospy.loginfo("INITIALIZATION FINISHED!")

    def pos_control_param_init(self,):
        self.Kp = Vector3()
        self.Kd = Vector3()
	self.ki = Vector3()
        self.Kp.x = 3.5
        self.Kp.y = 3.5
        self.Kp.z = 3.2
        self.Kd.x = 1.0
        self.Kd.y = 1.0
        self.Kd.z = 1.0
        self.ki.x = 0.01
        self.ki.y = 0.01
        self.ki.z = 0.02
	#These are parameters for Gazebo models only

    def track_control_param_init(self, ):
        self.Kp_track = Vector3()
        self.Kd_track = Vector3()
        self.Ki_track = Vector3()
        self.Kp_track.x = 3.5
        self.Kp_track.y = 3.5
        self.Kp_track.z = 2.2
        self.Kd_track.x = 1.0 
        self.Kd_track.y = 1.0
        self.Kd_track.z = 2.5
        self.Ki_track.x = 0.01
        self.Ki_track.y = 0.01
        self.Ki_track.z = 0.01

    def quadrotor_physical_parameter(self,num):
	self.m = 0.06
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
        start_point = Waypoint()
        start_point.x = self.curr_vision_odom.position.x
        start_point.y = self.curr_vision_odom.position.y
        start_point.z = self.curr_vision_odom.position.z
        orien = self.curr_vision_odom.orientation
        start_point.yaw = qua2eu([orien.x,orien.y,orien.z,orien.w],'sxyz')[2] 
        start_point.updated = False
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

    '''
    ------------------------------------------- Trajectory functions -------------------------------------------
    '''

    def two_pts_trajectory_init(self,startpoint, endpoint,t0,tf):
        self.t0 = t0
        self.tf = tf
        self.x_initial = np.matrix([[startpoint.x, endpoint.x, 0, 0, 0, 0]]).transpose()
        self.y_initial = np.matrix([[startpoint.y, endpoint.y, 0, 0, 0, 0]]).transpose()
        self.z_initial = np.matrix([[startpoint.z, endpoint.z, 0, 0, 0, 0]]).transpose()
        self.yaw_initial = np.matrix([[startpoint.yaw, endpoint.yaw, 0, 0, 0, 0]]).transpose()
        self.A = np.matrix([[1, self.t0, self.t0 ** 2, self.t0 ** 3    , self.t0 ** 4     , self.t0 ** 5],
                            [1, self.tf, self.tf ** 2, self.tf ** 3    , self.tf ** 4     , self.tf ** 5],
                            [0, 1      , 2 * self.t0 , 3 * self.t0 ** 2, 4 * self.t0 ** 3 , 5 * self.t0 ** 4],
                            [0, 1      , 2 * self.tf , 3 * self.tf ** 2, 4 * self.tf ** 3 , 5 * self.tf ** 4],
                            [0, 0      , 2           , 6 * self.t0     , 12 * self.t0 ** 2, 20 * self.t0 ** 3],
                            [0, 0      , 2           , 6 * self.tf     , 12 * self.tf ** 2, 20 * self.tf ** 3]])
        self.A_init_flag = True
        self.xcoeff = np.linalg.inv(self.A) * self.x_initial
        self.ycoeff = np.linalg.inv(self.A) * self.y_initial
        self.zcoeff = np.linalg.inv(self.A) * self.z_initial
        self.yawcoeff = np.linalg.inv(self.A) * self.yaw_initial

    def two_pts_trajectory_generator(self,t,waypoint):
        des_trajectory_point = Trajectory()
        if self.A_init_flag:
            if self.t0 < t < self.tf:
                polynominal = np.matrix([[1, t, t ** 2, t ** 3    , t ** 4     , t ** 5],
                                     [0, 1, 2 * t , 3 * t ** 2, 4 * t ** 3 , 5 * t ** 4],
                                     [0, 0, 2     , 6 * t     , 12 * t ** 2, 20 * t ** 3]])
                x_d = polynominal * self.xcoeff
                y_d = polynominal * self.ycoeff
                z_d = polynominal * self.zcoeff
                yaw_d = polynominal * self.yawcoeff
                des_trajectory_point.position.x = x_d[0,0]
                des_trajectory_point.position.y = y_d[0,0]
                des_trajectory_point.position.z = z_d[0,0]
                des_trajectory_point.velocity.x = x_d[1,0]
                des_trajectory_point.velocity.y = y_d[1,0]
                des_trajectory_point.velocity.z = z_d[1,0]
                des_trajectory_point.acceleration.x = x_d[2,0]
                des_trajectory_point.acceleration.y = y_d[2,0]
                des_trajectory_point.acceleration.z = z_d[2,0]
                des_trajectory_point.yaw = yaw_d[0,0]

            else:
                des_trajectory_point.position.x = waypoint.x
                des_trajectory_point.position.y = waypoint.y
                des_trajectory_point.position.z = waypoint.z
                des_trajectory_point.velocity.x = 0.0
                des_trajectory_point.velocity.y = 0.0
                des_trajectory_point.velocity.z = 0.0
                des_trajectory_point.acceleration.x = 0.0
                des_trajectory_point.acceleration.y = 0.0
                des_trajectory_point.acceleration.z = 0.0
                des_trajectory_point.yaw = waypoint.yaw

            des_trajectory_point.updated = True
            return des_trajectory_point

    '''
    --------------------------------Control mehtods----------------------------------------------
    '''

    def pos_control(self,des_trajectory_point,curr_pose, curr_vel, yaw_des, time_now):
        Rot_des = np.matrix(np.zeros((4, 4)))
        Rot_des[3, 3] = 1
        attitude_des = AttitudeTarget()
        thrust = Thrust()
        des_acc = Vector3()

        if self.A_init_flag:
            thrust.header = curr_pose.header
            attitude_des.header = curr_pose.header

            dt = time_now - self.time_prev
	    
            ex = des_trajectory_point.position.x - curr_pose.pose.position.x
            ey = des_trajectory_point.position.y - curr_pose.pose.position.y
            ez = des_trajectory_point.position.z - curr_pose.pose.position.z
            evx = des_trajectory_point.velocity.x - curr_vel.twist.linear.x
            evy = des_trajectory_point.velocity.y - curr_vel.twist.linear.y
            evz = des_trajectory_point.velocity.z - curr_vel.twist.linear.z
            self.intx = self.intx + ex * dt
            self.inty = self.inty + ey * dt
            self.intz = self.intz + ez * dt
            des_acc.x = des_trajectory_point.acceleration.x + self.Kp.x * ex + self.Kd.x * evx + self.ki.x * self.intx
            des_acc.y = des_trajectory_point.acceleration.y + self.Kp.y * ey + self.Kd.y * evy + self.ki.y * self.inty
            des_acc.z = des_trajectory_point.acceleration.z + self.Kp.z * ez + self.Kd.z * evz + self.ki.z * self.intz
            curr_quaternion = [curr_pose.pose.orientation.x, curr_pose.pose.orientation.y,
                             curr_pose.pose.orientation.z, curr_pose.pose.orientation.w]
            H_curr = tf.transformations.quaternion_matrix(curr_quaternion)
            Rot_curr = np.matrix(H_curr[:3, :3])
            Force_des = np.matrix([[0], [0], [self.m * self.g]])+self.m * np.matrix([[des_acc.x], [des_acc.y], [des_acc.z]])
            Force_des_body = Rot_curr * Force_des
            thrust.thrust = Force_des_body[2]

            Rot_des[:3, 2] = np.matrix(Force_des / LA.norm(Force_des))
            x_body_in_world = np.matrix([[np.cos(des_trajectory_point.yaw)], [np.sin(des_trajectory_point.yaw)], [0]])
            y_body_in_world = np.cross(Rot_des[:3, 2], x_body_in_world, axis=0)
            Rot_des[:3, 1] = np.matrix(y_body_in_world / LA.norm(y_body_in_world))
            Rot_des[:3, 0] = np.matrix(np.cross(Rot_des[:3, 1], Rot_des[:3, 2], axis=0))

            quad_des = tf.transformations.quaternion_from_matrix(Rot_des)
            attitude_des.orientation.x = quad_des[0]
            attitude_des.orientation.y = quad_des[1]
            attitude_des.orientation.z = quad_des[2]
            attitude_des.orientation.w = quad_des[3]
            attitude_des.thrust = thrust.thrust

            self.mavros_attitude_pub.publish(attitude_des)
            self.mavros_thrust_pub.publish(thrust)

        self.time_prev = time_now

    def track_and_trajectory_dock_control(self,des_trajectory_point, curr_odom, yaw_des, time_now, docker):
        if docker == self.num:
	    pass
        else:
            return

        Rot_des = np.matrix(np.zeros((4, 4)))
        Rot_des[3, 3] = 1
        attitude_des = AttitudeTarget()
        thrust = Thrust()
        des_acc = Vector3()

        if self.A_init_flag:
            thrust.header = curr_odom.header
            attitude_des.header = curr_odom.header
            
            dt = time_now - self.time_prev
            ex = des_trajectory_point.position.x - curr_odom.position.x
            ey = des_trajectory_point.position.y - curr_odom.position.y
            ez = des_trajectory_point.position.z - curr_odom.position.z
            evx = des_trajectory_point.velocity.x - curr_odom.velocity.x
            evy = des_trajectory_point.velocity.y - curr_odom.velocity.y
            evz = des_trajectory_point.velocity.z - curr_odom.velocity.z

	    self.intx = self.intx + ex * dt
	    self.inty = self.inty + ey * dt
	    self.intz = self.intz + ez * dt
            
            des_acc.x = des_trajectory_point.acceleration.x + self.Kp_track.x * ex + self.Kd_track.x * evx + self.Ki_track.x * self.intx
            des_acc.y = des_trajectory_point.acceleration.y + self.Kp_track.y * ey + self.Kd_track.y * evy + self.Ki_track.y * self.inty
            des_acc.z = des_trajectory_point.acceleration.z + self.Kp_track.z * ez + self.Kd_track.z * evz + self.Ki_track.z * self.intz
            Rot_waitmod_w = np.matrix([[np.cos(self.waitmod_yaw),-np.sin(self.waitmod_yaw),0],[np.sin(self.waitmod_yaw),np.cos(self.waitmod_yaw),0],[0,0,1]]).transpose()
            curr_quaternion = [curr_odom.orientation.x, curr_odom.orientation.y,
                               curr_odom.orientation.z, curr_odom.orientation.w]
            H_curr = tf.transformations.quaternion_matrix(curr_quaternion)
            Rot_curr = np.matrix(H_curr[:3, :3])
            des_acc_relative = Rot_waitmod_w*np.matrix([[des_acc.x], [des_acc.y], [des_acc.z]])
            Force_des = np.matrix([[0], [0], [self.m * self.g]])+self.m * des_acc_relative
            Force_des_body = Rot_curr * Force_des
            thrust.thrust = Force_des_body[2]
              
            Rot_des[:3, 2] = np.matrix(Force_des / LA.norm(Force_des))
            x_body_in_world = np.matrix([[np.cos(des_trajectory_point.yaw)], [np.sin(des_trajectory_point.yaw)], [0]])
            y_body_in_world = np.cross(Rot_des[:3, 2], x_body_in_world, axis=0)
            Rot_des[:3, 1] = np.matrix(y_body_in_world / LA.norm(y_body_in_world))
            Rot_des[:3, 0] = np.matrix(np.cross(Rot_des[:3, 1], Rot_des[:3, 2], axis=0))

            quad_des = tf.transformations.quaternion_from_matrix(Rot_des)
            attitude_des.orientation.x = quad_des[0]
            attitude_des.orientation.y = quad_des[1]
            attitude_des.orientation.z = quad_des[2]
            attitude_des.orientation.w = quad_des[3]
            attitude_des.thrust = thrust.thrust

            self.mavros_attitude_pub.publish(attitude_des)
            self.mavros_thrust_pub.publish(thrust)

        self.time_prev = time_now

    def modquad_control(self,des_trajectory_point,curr_pose, curr_vel, yaw_des):
        Rot_des = np.matrix(np.zeros((4, 4)))
        Rot_des[3, 3] = 1
        attitude_des = AttitudeTarget()
        thrust = Thrust()
        des_acc = Vector3()

        if self.A_init_flag:
            thrust.header = curr_pose.header
            attitude_des.header = curr_pose.header

            ex = des_trajectory_point.position.x - curr_pose.pose.position.x
            ey = des_trajectory_point.position.y - curr_pose.pose.position.y
            ez = des_trajectory_point.position.z - curr_pose.pose.position.z
            evx = des_trajectory_point.velocity.x - curr_vel.twist.linear.x
            evy = des_trajectory_point.velocity.y - curr_vel.twist.linear.y
            evz = des_trajectory_point.velocity.z - curr_vel.twist.linear.z
            
            des_acc.x = des_trajectory_point.acceleration.x + self.Kp.x * ex + self.Kd.x * evx
            des_acc.y = des_trajectory_point.acceleration.y + self.Kp.y * ey + self.Kd.y * evy
            des_acc.z = des_trajectory_point.acceleration.z + self.Kp.z * ez + self.Kd.z * evz

            curr_quaternion = [curr_pose.pose.orientation.x, curr_pose.pose.orientation.y,
                             curr_pose.pose.orientation.z, curr_pose.pose.orientation.w]
            H_curr = tf.transformations.quaternion_matrix(curr_quaternion)
            Rot_curr = np.matrix(H_curr[:3, :3])
            
            Force_des = np.matrix([[0], [0], [self.m * self.g]])+self.m * np.matrix([[des_acc.x], [des_acc.y], [des_acc.z]])
            Force_des_body = Rot_curr * Force_des
            thrust.thrust = Force_des_body[2]

            Rot_des[:3, 2] = np.matrix(Force_des / LA.norm(Force_des))
            x_body_in_world = np.matrix([[np.cos(des_trajectory_point.yaw)], [np.sin(des_trajectory_point.yaw)], [0]])
            y_body_in_world = np.cross(Rot_des[:3, 2], x_body_in_world, axis=0)
            Rot_des[:3, 1] = np.matrix(y_body_in_world / LA.norm(y_body_in_world))
            Rot_des[:3, 0] = np.matrix(np.cross(Rot_des[:3, 1], Rot_des[:3, 2], axis=0))

            quad_des = tf.transformations.quaternion_from_matrix(Rot_des)
            attitude_des.orientation.x = quad_des[0]
            attitude_des.orientation.y = quad_des[1]
            attitude_des.orientation.z = quad_des[2]
            attitude_des.orientation.w = quad_des[3]
            attitude_des.thrust = thrust.thrust

            self.mavros_attitude_pub.publish(attitude_des)
            self.mavros_thrust_pub.publish(thrust)

    def dock_detector(self,curr_odom,curr_Imu):
        '''	
	pos_x  = curr_odom.position.x
	pos_y  = curr_odom.position.y
	pos_z  = curr_odom.position.z
        '''	

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
