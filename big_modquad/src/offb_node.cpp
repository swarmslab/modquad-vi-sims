#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include "math.h"

mavros_msgs::State current_state;

double r = 1.0;
double theta;
int count=0;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros101/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros101/setpoint_position/local", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    
    //nh.param("pub_setpoints_traj/wn", wn, 1.0);
    //nh.param("pub_setpoints_traj/r", r, 1.0);
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
	ROS_INFO("Waiting...");
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 5.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 1.0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){ 

        theta = count*0.06;

        pose.pose.position.x = 5.0+r*sin(theta);
        pose.pose.position.y = r*cos(theta);
        pose.pose.position.z = 1.0;

        count++;

	ROS_INFO("Publishing waypoint.");
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
