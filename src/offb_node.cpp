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

/*sensor_msgs::NavSatFix global_current_state;

void state_cb(const mavros_msgs::State::ConstPtr&msg){
    current_state = *msg;
}

void global_cb(const sensor_msgs::NavSatFix::ConstPtr&msg){
    global_current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, global_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
        
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0.0;
    land_cmd.request.latitude = global_current_state.latitude;
    land_cmd.request.longitude = global_current_state.longitude;
    land_cmd.request.altitude = global_current_state.altitude;

    ros::Time start_time = ros::Time::now();
    ros::Time last_request = ros::Time::now();

    int counter;
    counter = 0;

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else if(counter == 0) {
            ROS_INFO("Arming the vehicle");
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
		    counter = 1;
                }
                last_request = ros::Time::now();
            }
        }

        if( current_state.armed &&
                (ros::Time::now() - start_time > ros::Duration(40.0))){
                if( land_client.call(land_cmd) &&
                    land_cmd.response.success){
                    ROS_INFO("Vehicle landing");
		    last_request = ros::Time::now();
                    //arm_cmd.request.value = false;
                    //	if( arming_client.call(arm_cmd) &&
                    //		arm_cmd.response.success){
                    //		ROS_INFO("Vehicle disarmed");
                	//}
                }
                //ROS_INFO("Trying to land");
        } else if (current_state.armed &&
                (ros::Time::now() - start_time < ros::Duration(40.0))){
                ROS_INFO("Publishing pose");
                local_pos_pub.publish(pose);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
*/
