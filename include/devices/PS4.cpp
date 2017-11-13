#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>

#include "PS4.h"

struct PS4::Impl
{
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::Subscriber joy_sub;
    ps4_ns::Data data;
    ps4_ns::Data old_data;
};

void PS4::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    old_data          = data;
    
    data.hat_LX       = msg->axes[0];
    data.hat_LY       = msg->axes[1];
    data.hat_RX       = msg->axes[2];
    data.L2_analog    = msg->axes[3];
    data.R2_analog    = msg->axes[4];
    data.hat_RY       = msg->axes[5];
    
    data.dpad_x       = msg->axes[9];
    data.dpad_y       = msg->axes[10];
    
    data.rectangle    = msg->buttons[0];
    data.cross        = msg->buttons[1];
    data.circle       = msg->buttons[2];
    data.triangle     = msg->buttons[3];
    data.L1           = msg->buttons[4];
    data.R1           = msg->buttons[5];
    data.L2           = msg->buttons[6];
    data.R2           = msg->buttons[7];
    data.share        = msg->buttons[8];
    data.options      = msg->buttons[9];
    data.L3           = msg->buttons[10];
    data.R3           = msg->buttons[11];
    data.ps			  = msg->buttons[12];
	data.tpad_click   = msg->buttons[13];
}

PS4::PS4(ros::NodeHandle* nh)
{
    pimpl_ = new Impl;
    pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &PS4::Impl::joyCallback, pimpl_);
    
    pimpl_->data = {};
    pimpl_->old_data = {};
    
    /*
    //  initialize the topics
    ros::Publisher joy_pub = nh->advertise<sensor_msgs::Joy>("joy", 1);
    sensor_msgs::Joy joy_pub_data = {};
    joy_pub_data.axes[3] = 1;
    joy_pub_data.axes[4] = 1;    
    joy_pub.publish(joy_pub_data);
    */
}

ps4_ns::Data PS4::get_data() const  {    return pimpl_->data;   }

ps4_ns::Data PS4::get_old_data() const  {    return pimpl_->old_data;   }

