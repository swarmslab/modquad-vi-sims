#!/usr/bin/env python
#!/usr/bin/env python
import rospy
import subprocess
import os
import sys
import pdb

from std_msgs.msg import Float64
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped,Pose,Vector3,Twist,TwistStamped
from std_srvs.srv import Empty
import time


def pos_cb(msg):
    global cur_pose
    cur_pose = msg

#NUM_UAV = int(sys.argv[1]

def mavrosTopicStringRoot(uavID):
    return ('mavros' + str(uavID+1))

def Communication_with_drone(NUM_UAV,mode_proxy,land_proxy):
#Comm for drones
    for uavID in range(0,NUM_UAV):
        land_proxy[uavID] = rospy.ServiceProxy(mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)

    print 'communication initialization complete'
    data = [None for i in range(NUM_UAV)]

    while None in data:
        for uavID in range(0, NUM_UAV):
            try:
                data[uavID] = rospy.wait_for_message(mavrosTopicStringRoot(uavID) + '/global_position/rel_alt', Float64, timeout=5)
            except:
                pass

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        success = [None for i in range(NUM_UAV)]
        for uavID in range(0, NUM_UAV):
            print "wait for service"
            rospy.wait_for_service(mavrosTopicStringRoot(uavID) + '/set_mode')
            print "got service"

        for uavID in range(0, NUM_UAV):
            try:
                success[uavID] = land_proxy[uavID](1,'AUTO.RTL')
            except rospy.ServiceException, e:
                print ("mavros/set_mode service call failed: %s"%e)

if __name__ == '__main__':

    cur_pose = PoseStamped()
    NUM_UAV = int(1)

    mode_proxy = [None for i in range(NUM_UAV)]
    land_proxy = [None for i in range(NUM_UAV)]

    rospy.init_node('land', anonymous=True)
    Communication_with_drone(NUM_UAV,mode_proxy,land_proxy)



