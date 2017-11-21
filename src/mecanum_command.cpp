#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
//#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <cstdlib>
#include <cmath>
#include "devices/PS4.h"

#include "m2_umd/MotorStatus.h"
#include "m2_umd/Get_MotorBoard.h"
#include "m2_umd/Set_MotorBoard.h"
//#include "m2_umd/Set_ServoBoard.h"
#include "m2_umd/PneumaticState.h"

#include "m2_umd/UmdUtils.hpp"
#include "m2_umd/UmdBoard.hpp"
#include "m2_umd/UmdManager.hpp"

#include "m2_umd/StepperBoard.hpp"
//#include "m2_umd/ServoBoard.hpp"
#include "m2_umd/PneumaticBoard.hpp"
//#include "m2_umd/EncoderBoard.hpp"
#include "m2_umd/MotorBoard.hpp"

//#include <omni_control/ResetCount.h>

ros::Publisher pub_magnetic;
ros::Publisher pub_elevator[2];
//ros::Publisher pub_twist_target;
ros::ServiceClient client_rotate;

int main(int argc, char **argv){

	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	ros::init(argc, argv, "mecanum_command");

	ros::NodeHandle n("~");
	
	ros::Rate loop_rate(100);

	// read param of PS4 topic
	std::string ps4_topic = "joy";
	if ( !n.getParam("ps4_topic", ps4_topic) ) {
		ROS_ERROR("Param ps4_topic not exist/wrong type");
	}

	PS4 ps4(&n, ps4_topic);

	pub_magnetic = n.advertise<m2_umd::PneumaticState>("/mecanum_top/pneumatic_0/state", 1);
	m2_umd::PneumaticState msg_magnetic;

	pub_elevator[0] = n.advertise<geometry_msgs::Vector3>("/mecanum_top/motor_0/setpoint", 1);
	pub_elevator[1] = n.advertise<geometry_msgs::Vector3>("/mecanum_top/motor_1/setpoint", 1);
	geometry_msgs::Vector3 msg_elevator[2];
	for (int i=0;i<2;i++) {
		msg_elevator[i].x = 0.0;
		msg_elevator[i].y = 0.0;
		msg_elevator[i].z = 0.0;
	}

	client_rotate = n.serviceClient<m2_umd::Instruct_StepperBoard>("/mecanum_top/stepper_0/instruct_StepperBoard");
	m2_umd::Instruct_StepperBoard srv;
	srv.request.flag_disable = false;
	srv.request.flag_period = false;
	srv.request.flag_reverse = false;
	srv.request.stepper_id = 0;
	srv.request.num = 600;

	//pub_twist_target = n.advertise<geometry_msgs::Pose2D>("command/twistTarget", 1);

	//geometry_msgs::Pose2D msg;
	//std_msgs::UInt8 mode;mode.data=0;

	//ros::service::waitForService("omni_odom/reset_count");  
	//client = n.serviceClient<omni_control::ResetCount>("omni_odom/reset_count");
	

	ps4_ns::Data ps4_data_old;
	ps4_ns::Data ps4_data = ps4.get_data();
	bool mode_golden = 0, mode_precise = 0;

	while (ros::ok()){

		ps4_data_old = ps4_data;
		ps4_data = ps4.get_data();

		ROS_DEBUG("NOW Op:%d /\\:%d X:%d []:%d O:%d L1:%d Sr:%d",
			ps4_data.options, ps4_data.triangle, ps4_data.cross, 
			ps4_data.rectangle, ps4_data.circle, ps4_data.L1, ps4_data.share);
		ROS_DEBUG("OLD Op:%d /\\:%d X:%d []:%d O:%d L1:%d Sr:%d",
			ps4_data_old.options, ps4_data_old.triangle, ps4_data_old.cross, 
			ps4_data_old.rectangle, ps4_data_old.circle, ps4_data_old.L1, ps4_data.share);
//		ROS_DEBUG("hat_lx:%.4f, hat_ly:%.4f, circle:%d", ps4_data.hat_LX, ps4_data.hat_LY, ps4_data.circle);
//		ROS_DEBUG("l2:%.4f, r2:%.4f, dpad_x: %.1f, dpad_y: %.1f", ps4_data.L2_analog, ps4_data.R2_analog);
//		ROS_DEBUG("dpad_x: %.1f, dpad_y: %.1f", ps4_data.dpad_x, ps4_data.dpad_y);


		// Toggle Mode
		if (ps4_data.L1 and not ps4_data_old.L1){
			mode_golden = !mode_golden;
		}

		if (ps4_data.share and not ps4_data_old.share){
			mode_precise = !mode_precise;
		}

		ROS_DEBUG("Golden:%d Precise:%d", mode_golden, mode_precise);

		// Remove Rack
		if (ps4_data.options){
			if (ps4_data.options) {
			msg_magnetic.b1 = true;
			msg_magnetic.b0 = true;
			}
		} else {
			msg_magnetic.b1 = false;
			msg_magnetic.b0 = false;
		}

		// Elevator
		if (ps4_data.triangle){
			int i;

			if (mode_golden) {
				i = 1;
				msg_elevator[0].x = 0.0;
			} else {
				i = 0;
				msg_elevator[1].x = 0.0;
			}

			if (mode_precise) {
				msg_elevator[i].x = 20;
			} else {
				msg_elevator[i].x = 150;
			}
		} else if (ps4_data.cross){
			int i;

			if (mode_golden) {
				i = 1;
				msg_elevator[0].x = 0.0;
			} else {
				i = 0;
				msg_elevator[1].x = 0.0;
			}

			if (mode_precise) {
				msg_elevator[i].x = -20;
			} else {
				msg_elevator[i].x = -150;
			}
		} else {
			msg_elevator[0].x = 0.0;
			msg_elevator[1].x = 0.0;
		}

		// Rotate Rack
		if (ps4_data.rectangle and not ps4_data_old.rectangle){
			if (mode_golden) {
				srv.request.stepper_id = 1;
				srv.request.flag_reverse = true;
			} else {
				srv.request.stepper_id = 0;
				srv.request.flag_reverse = false;
			}

			if (mode_precise) {
				srv.request.num = 60;
			} else {
				srv.request.num = 1200;
			}

			if (client_rotate.call(srv)) {
				ROS_INFO("Sum: %d", srv.response.num);
			} else {
				ROS_ERROR("Failed to call service add_two_ints");
				return 1;
			}
		}

		if (ps4_data.circle and not ps4_data_old.circle){
			if (mode_golden) {
				srv.request.stepper_id = 1;
				srv.request.flag_reverse = true;
			} else {
				srv.request.stepper_id = 0;
				srv.request.flag_reverse = false;
			}

			if (mode_precise) {
				srv.request.num = 60;
			} else {
				srv.request.num = 1200;
			}

			if (client_rotate.call(srv)) {
				ROS_INFO("Sum: %d", srv.response.num);
			} else {
				ROS_ERROR("Failed to call service add_two_ints");
				return 1;
			}
		}

		pub_magnetic.publish(msg_magnetic);
		pub_elevator[0].publish(msg_elevator[0]);
		pub_elevator[1].publish(msg_elevator[1]);

//		if(ps4_data.dpad_x||ps4_data.dpad_y){
//				if (fabs(ps4.get_data().dpad_y - ps4.get_old_data().dpad_y) > 0.5){
//					linear_velocity_const+=ps4_data.dpad_y*0.1;
//				}
//				if (fabs(ps4.get_data().dpad_x - ps4.get_old_data().dpad_x) > 0.5){
//					angular_velocity_const+=(-ps4_data.dpad_x)*0.1;
//				}
//				ros::spinOnce();
//		}

//		if(ps4_data.L1){
//			omni_control::ResetCount srv;
//			srv.request.x = 0;
//			srv.request.y = 0;
//			srv.request.th = 0;
//			if (client.call(srv)){
//				ROS_DEBUG("Service call successful: %f", srv.response.stamp.toSec());
//			}else{
//				ROS_ERROR("Failed to call service reset_odom");
//			}
//			ros::spinOnce();
//		}

//		if (ps4.get_data().tpad_click&&(!ps4.get_old_data().tpad_click)){
//			//if (ps4.get_data().circle&&(!ps4.get_old_data().circle)){
//			mode.data=mode.data==0?1:0;
//		}

//		msg.x=ps4_data.hat_LY*linear_velocity_const;
//		msg.y=ps4_data.hat_RX*linear_velocity_const;
//		msg.theta=(-1.0)*(ps4_data.L2_analog-1)*0.5*angular_velocity_const;
//		msg.theta+=(ps4_data.R2_analog-1)*0.5*angular_velocity_const;

//		limiting(msg.x,linear_velocity_const*2.0);
//		limiting(msg.y,linear_velocity_const*2.0);
//		limiting(msg.theta,angular_velocity_const*2.0);

//		ROS_DEBUG("l_vel: %.4f, a_vel: %.4f", linear_velocity_const, angular_velocity_const);
//		ROS_DEBUG("x:%.2f, y:%.2f, z:%.2f", msg.x, msg.y, msg.theta);

//		pub_mode.publish(mode);
//		pub_twist_target.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
