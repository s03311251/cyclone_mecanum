#include <ros/ros.h>
#include <ros/console.h>
//#include <std_msgs/Float64.h>

//#include <geometry_msgs/Pose2D.h>

//#include <nav_msgs/Odometry.h>
//#include <cstdlib>
//#include <cmath>

#include <m2_umd/IOPort.h>
#include <m2_umd/IOState.h>
//#include "m2_umd/MotorStatus.h"
//#include "m2_umd/Get_MotorBoard.h"
//#include "m2_umd/Set_MotorBoard.h"
#include <geometry_msgs/Vector3.h>
//#include <mechaduino_stepper/Move.h>
//#include <cyclone_mecanum/ElevatorCont.h>
//#include <cyclone_mecanum/EmCont.h>

//#include <std_srvs/SetBool.h>
//#include <cyclone_mecanum/ElevatorOneshot.h>
#include <cyclone_mecanum/IOCmd.h>
#include <cyclone_mecanum/ServoCmd.h>

/*const int NUM_MOTOR = 1;
const int NUM_LIGHT_GATE = 2;*/
int servo_setpoint_open;
int servo_setpoint_close;

std::vector<int> IO_port;
std::vector<int> servo_port;

ros::Publisher pub_IO;
std::vector<ros::Publisher> pub_servo;

ros::Subscriber sub_IO_cmd;
ros::Subscriber sub_servo_cmd;

/*int disk_spin_angle;
std::vector<int> light_gate_IO_port;
std::vector<int> em_IO_port;
std::vector<bool> light_gate_state;

ros::Publisher pub_mechaduino;
ros::Publisher pub_motor[NUM_MOTOR];
ros::Publisher pub_IO;
ros::Subscriber sub_IO;

ros::Subscriber sub_elevator_cont;
ros::Subscriber sub_em_cont;
ros::ServiceServer ser_disk;
ros::ServiceServer ser_elevator_one;
//ros::ServiceClient client_rotate;

void IOCallback(const m2_umd::IOState::ConstPtr& msg) {
	for (int i = 0; i < light_gate_IO_port.size(); i++) {
		light_gate_state[i] = msg->ports[light_gate_IO_port[i]];
	}
	// and maybe boardcast some events which stop elevator_oneshot?
}

void elevatorContCallback(const cyclone_mecanum::ElevatorCont::ConstPtr& msg_elevator) {
	if (msg_elevator->elevator_id >= NUM_MOTOR) {
		return;
	}

	geometry_msgs::Vector3 msg_motor;

	msg_motor.x = msg_elevator->vel_setpoint;
	msg_motor.y = 0.0;
	msg_motor.z = 0.0;
	pub_motor[msg_elevator->elevator_id].publish(msg_motor);
}

bool diskSpinCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
	mechaduino_stepper::Move msg;

	// rotate closewise if true
	msg.deg = req.data ? -disk_spin_angle : disk_spin_angle;

	if (pub_mechaduino.getNumSubscribers() < 1) {
		res.success = false;
		res.message = "No subscriber for Mechaduino";
		ROS_DEBUG("No subscriber for Mechaduino");
		return true;
	}

	pub_mechaduino.publish(msg);
	ros::spinOnce();

	res.success = true;
	res.message = "Command sent";
	return true;
}

bool elevatorOneCallback(cyclone_mecanum::ElevatorOneshot::Request& req, cyclone_mecanum::ElevatorOneshot::Response& res) {
	//req.elevator_id;
	//req.up;

	if (req.elevator_id >= NUM_MOTOR) {
		res.success = false;
		res.message = "Elevator ID doesn't exist";
		return true;
	}

	res.success = false;
	res.message = "Not yet implemented!";
	return true;
}

void emContCallback(const cyclone_mecanum::EmCont::ConstPtr& msg_em) {
	if (msg_em->em_id >= em_IO_port.size()) {
		return;
	}
	m2_umd::IOPort msg_io;
	msg_io.port = em_IO_port[msg_em->em_id];
	msg_io.on = msg_em->release;
	pub_IO.publish(msg_io);
}*/

void pneumCallback(const cyclone_mecanum::IOCmd::ConstPtr& msg_IO) {
	if (msg_IO->id >= IO_port.size()) {
		return;
	}
	m2_umd::IOPort msg;
	msg.port = IO_port[msg_IO->id];
	msg.on = msg_IO->on;
	pub_IO.publish(msg);
}

void servoCallback(const cyclone_mecanum::ServoCmd::ConstPtr& msg_servo) {
	if (msg_servo->id >= servo_port.size()) {
		return;
	}
	geometry_msgs::Vector3 msg;
	msg.x = msg_servo->open ? servo_setpoint_open : servo_setpoint_close;
	pub_servo[msg_servo->id].publish(msg);
}



int main(int argc, char **argv) {
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	ros::init(argc, argv, "mecanum_command");

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");



	/* Load parameters */
	/*nh_private.param("disk_spin_angle", disk_spin_angle, 36);
	ROS_DEBUG("Disk Spin Angle: %d", disk_spin_angle);

	nh_private.getParam("light_gate_IO_port", light_gate_IO_port);
	light_gate_state.clear();
	light_gate_state.resize(light_gate_IO_port.size(), 0);
	ROS_DEBUG("Light Gate IO Port: (%lu)", light_gate_IO_port.size());
	for (int i = 0; i < light_gate_IO_port.size(); i++) {
		// 1-12 is printed on IO Board, but IO_Board.hpp uses 0-11
		light_gate_IO_port[i]--;
		ROS_DEBUG("%d", light_gate_IO_port[i]);
	}

	nh_private.getParam("em_IO_port", em_IO_port);
	ROS_DEBUG("EM IO Port: (%lu)", em_IO_port.size());
	for (int i = 0; i < em_IO_port.size(); i++) {
		// 1-12 is printed on IO Board, but IO_Board.hpp uses 0-11
		em_IO_port[i]--;
		ROS_DEBUG("%d", em_IO_port[i]);
	}*/


	nh_private.param("servo_setpoint_open", servo_setpoint_open, 371);
	nh_private.param("servo_setpoint_close", servo_setpoint_close, 370);
	ROS_DEBUG("Servo: %d - %d", servo_setpoint_open, servo_setpoint_close);

	nh_private.getParam("IO_port", IO_port);
	ROS_DEBUG("IO Port: (%lu)", IO_port.size());
	for (int i = 0; i < IO_port.size(); i++) {
		// 1-12 is printed on IO Board, but IO_Board.hpp uses 0-11
		IO_port[i]--;
		ROS_DEBUG("%d", IO_port[i]);
	}

	nh_private.getParam("servo_port", servo_port);
	ROS_DEBUG("Servo Port: (%lu)", servo_port.size());
	for (int i = 0; i < servo_port.size(); i++) {
		ROS_DEBUG("%d", servo_port[i]);
	}



	/* Define Publisher, Subscriber, ServiceServer */
	std::string prefix = "/mecanum_command";

	/*pub_mechaduino = nh.advertise<mechaduino_stepper::Move>("/stepper_control/movedeg", 1000);
	pub_motor[0] = nh.advertise<geometry_msgs::Vector3>("/mecanum_umd/motor_4/v_setpoint", 1);
	pub_IO = nh.advertise<m2_umd::IOPort>("/mecanum_umd/IO_0/set_port", 1000);
	sub_IO = nh.subscribe("/mecanum_umd/IO_0/get_state", 1000, IOCallback);

	sub_elevator_cont = nh.subscribe(prefix + "/elevator_cont", 1000, elevatorContCallback);
	sub_em_cont = nh.subscribe(prefix + "/em_cont", 1000, emContCallback);
	ser_disk = nh.advertiseService(prefix + "/disk_spin", diskSpinCallback);
	ser_elevator_one = nh.advertiseService(prefix + "/elevator_oneshot", elevatorOneCallback);*/

	pub_IO = nh.advertise<m2_umd::IOPort>("/mecanum_umd/IO_0/set_port", 1000);
	for(int i=0;i<servo_port.size();i++){
		ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("/mecanum_umd/servo_0/command" + std::to_string(servo_port[i]), 1000);
		pub_servo.push_back(std::move(pub));
	}

	sub_IO_cmd = nh.subscribe(prefix + "/pneum_cmd", 1000, pneumCallback);
	sub_servo_cmd = nh.subscribe(prefix + "/servo_cmd", 1000, servoCallback);

	/* Run */
	ros::Rate loop_rate(100);
	ros::spin();

	return 0;
}
