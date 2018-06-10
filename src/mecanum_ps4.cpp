#include <ros/ros.h>
#include <ros/console.h>
#include <cyclone_mecanum/IOCmd.h>
#include <cyclone_mecanum/ServoCmd.h>
//#include <cyclone_mecanum/ElevatorCont.h>
//#include <cyclone_mecanum/EmCont.h>
//#include <std_srvs/SetBool.h>
#include "PS4.hpp"

/*ros::Publisher pub_magnetic;
ros::Publisher pub_elevator;
ros::ServiceClient spin_client;*/

int main(int argc, char **argv){
	
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	ros::init(argc, argv, "mecanum_ps4");

	ros::NodeHandle n;
	
	ros::Rate loop_rate(100);

	PS4 ps4(&n);
	ros::Publisher pub_pneum = n.advertise<cyclone_mecanum::IOCmd>("/mecanum_command/pneum_cmd", 1);
	cyclone_mecanum::IOCmd msg_pneum;

	ros::Publisher pub_servo = n.advertise<cyclone_mecanum::ServoCmd>("/mecanum_command/servo_cmd", 1);
	cyclone_mecanum::ServoCmd msg_servo;

/*	pub_elevator = n.advertise<cyclone_mecanum::ElevatorCont>("ps4_elevator", 1);
	cyclone_mecanum::ElevatorCont msg_elevator;

	spin_client = n.serviceClient<std_srvs::SetBool>("ps4_spin");
	std_srvs::SetBool srv;
*/
	while (ros::ok()){

		ps4_ns::Data ps4_data=ps4.get_data();

		ROS_DEBUG_NAMED("ps4_data","-----");
		ROS_DEBUG_NAMED("ps4_data","cross: %d, circle: %d, triangle: %d, square: %d", ps4_data.cross, ps4_data.circle, ps4_data.triangle, ps4_data.square);

		// Em
		if (ps4_data.cross){
			msg_pneum.id = 0;
			msg_pneum.on = true;
		} else {
			msg_pneum.id = 0;
			msg_pneum.on = false;
		}
		pub_pneum.publish(msg_pneum);

		if (ps4_data.circle){
			msg_pneum.id = 1;
			msg_pneum.on = true;
		} else {
			msg_pneum.id = 1;
			msg_pneum.on = false;
		}
		pub_pneum.publish(msg_pneum);

		if (ps4_data.triangle){
			msg_servo.id = 0;
			msg_servo.open = true;
		} else {
			msg_servo.id = 0;
			msg_servo.open = false;
		}
		pub_servo.publish(msg_servo);

		if (ps4_data.square){
			msg_servo.id = 1;
			msg_servo.open = true;
		} else {
			msg_servo.id = 1;
			msg_servo.open = false;
		}
		pub_servo.publish(msg_servo);
/*
		// Elevator up
		if (ps4_data.triangle){
			msg_elevator.elevator_id = 1;
			msg_elevator.vel_setpoint = 130;
		} else {
			msg_elevator.elevator_id = 0;
			msg_elevator.vel_setpoint = 0;
		}

		// Elevator down
		if (ps4_data.square){
			msg_elevator.elevator_id = 1;
			msg_elevator.vel_setpoint = -130;
		} else {
			msg_elevator.elevator_id = 0;
			msg_elevator.vel_setpoint = 0;
		}

		// Elevator down
		if (ps4_data.circle){
			srv.request.data = true;
		} else {
			srv.request.data = false;
		}

		pub_magnetic.publish(msg_magnetic);
		//pub_elevator.publish(msg_elevator);
		ROS_INFO("Spin: %1d", (int)srv.request.data);
*/
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
