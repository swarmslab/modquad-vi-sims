/**
 * @brief ModquadControl plugin
 * @file modquad_control.cpp
 * @author Guanrui Li <garylion20@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2018 Guanrui Li.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CooperativeControl.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include <sstream>

namespace mavros {
namespace extra_plugins{
/**
 * @brief ModquadControl plugin
 *
 * Sends motion capture data to FCU.
 */
class ModquadControlPlugin : public plugin::PluginBase
{
public:
	ModquadControlPlugin() : PluginBase(),
		modquad_nh("~modquad_control")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		bool use_modquad;


		/** @note For Optitrack ROS package, subscribe to PoseStamped topic */
		modquad_nh.param("use_modquad", use_modquad, true);

		if (use_modquad) {
			modquad_control_sub = modquad_nh.subscribe("control_flag", 1, &ModquadControlPlugin::modquad_control_cb, this);
		}else {
			ROS_ERROR_NAMED("modquad_control", "The modquad control is not initialized.");
		}
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle modquad_nh;

	ros::Subscriber modquad_control_sub;

	/* -*- low-level send -*- */
	/*void modquad_control_send
		(uint64_t usec,
			bool control_flag,
			float x,
                        float y)*/
	void modquad_control_send(int modquad_control_flag,float* coeffs)
	{
		mavlink::common::msg::MODQUAD_CONTROL modquad_control;
		
		modquad_control.time_usec = 0.0f;
		modquad_control.modquad_control_flag = modquad_control_flag;
		for (int i=0;i<4;i++) {
			modquad_control.modquad_control_coeffs[i] = coeffs[i];
			ROS_INFO("%f",modquad_control.modquad_control_coeffs[i]);
		}
		//ROS_INFO("%f,%f,%f,%f,%f",coeffs[0],coeffs[1],coeffs[2],coeffs[3],x);
		modquad_control.x = 0.0; //TODO: remove x and y from px4 firmware mavlink msg
		modquad_control.y = 0.0;

		UAS_FCU(m_uas)->send_message_ignore_drop(modquad_control);
		/*std::stringstream ss;
		std_msgs::String msg;

		ss << "I'm trying to send the message" << x << y;
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str());*/
	}

	/* -*- mid-level helpers -*- */
	void modquad_control_cb(const mavros_msgs::CooperativeControl::ConstPtr& modquad_control)
	{
		/*auto usec = modquad_control->time_usec;	
		auto modquad_control_flag = modquad_control->modquad_control_flag;
		auto x = modquad_control->x;
		auto y = modquad_control->y;*/
		int flag;
		float coeffs[4];
		coeffs[0] = modquad_control->x_plusd; 
		coeffs[1] = modquad_control->x_minusd;
		coeffs[2] = modquad_control->y_plusd;
		coeffs[3] = modquad_control->y_minusd;
		auto control_flag = modquad_control->control_flag;
		
		if (control_flag){
			flag = 1;
			//float x = modquad_control->x;
			//float y = 0.0;
			modquad_control_send(flag,coeffs);
			ROS_INFO("%f,%f,%f,%f",coeffs[0],coeffs[1],coeffs[2],coeffs[3]);
		}else{
			ROS_INFO("The modquad control flag is not enabled, fail to send coefficients");	
		}
		
	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ModquadControlPlugin, mavros::plugin::PluginBase)
