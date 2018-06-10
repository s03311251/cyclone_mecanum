#include <ros/ros.h>
#include <m2_umd/UmdManager.hpp>
#include <m2_umd/IOBoard.hpp>
#include <m2_umd/ServoBoard.hpp>
//#include <m2_umd/MotorBoard.hpp>

uint32_t umd_packets_sent;
uint32_t umd_packets_received;
uint32_t umd_packets_checksum_error;
uint32_t umd_packets_frame_error;
uint32_t max_rttime;

using namespace m2;

int main(int argc, char **argv) {
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	ros::init(argc, argv, "mecanum_umd");

	//ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");



	/* Load parameters */
	bool use_umd, use_IO;
	nh_private.param<bool>("use_umd", use_umd, true);
	nh_private.param<bool>("use_IO", use_IO, true);
	ROS_DEBUG("Use UMD: %d Use IO: %d", use_umd, use_IO);

	std::string umd_port;
	nh_private.param<std::string>("umd_port", umd_port, "/dev/ttyACM1");
	int umd_baudrate;
	nh_private.param<int>("umd_baudrate", umd_baudrate, 921600);
	ROS_DEBUG("UMD: %s %d", umd_port.c_str(), umd_baudrate);

	std::string IO_port;
	nh_private.param<std::string>("IO_port", IO_port, "/dev/ttyACM0");
	int IO_baudrate;
	nh_private.param<int>("IO_baudrate", IO_baudrate, 921600);
	ROS_DEBUG("IO: %s %d", IO_port.c_str(), IO_baudrate);



	/* Initialize UMD boards */
	UmdManager um(umd_port, umd_baudrate, 10, 100);
	UmdManager um_IO(IO_port, IO_baudrate, 10, 100);
	umd_packets_sent = 0;
	umd_packets_received = 0;
	umd_packets_frame_error = 0;
	umd_packets_checksum_error = 0;
	max_rttime = 0;

	IOBoard i0(nh_private, um_IO, 0);
	ServoBoard s0(nh_private, 0);
	/*MotorBoard m4(nh_private, um, 4);
	/MotorBoard m5(nh_private, um, 5);

	MotorBoard::Setting m_setting;
	m_setting.mode = MotorBoard::VMODE;
	m_setting.accLimit = 1;
	m_setting.velLimit = 5000;
	m_setting.cmdCycle_us = 4000;
	m_setting.vLimit_mv = 12000;
	m_setting.iLimit_ma = 10000;
	m_setting.kP = 2000;
	m_setting.kI = 700;
	m_setting.kD = 50;
	m_setting.kFF = 1500;
	m4.setting = m_setting;
	/m5.setting = m_setting;*/



	/* Run */
	if (use_umd) {
		um.connect();
		um.startLoop();

		um.addTransfer(s0, 0);
		/*um.addTransferOnce(m4, m4.MSG_SETTING);
		um.addTransferOnce(m5, m5.MSG_SETTING);

		um.addTransfer(m4, m4.MSG_VMODE);
		um.addTransfer(m4, m4.MSG_STATUS);
		um.addTransfer(m5, m5.MSG_VMODE);
		um.addTransfer(m5, m5.MSG_STATUS);*/
	}
	if (use_IO) {
		um_IO.connect();
		um_IO.startLoop();

		um_IO.addTransfer(i0, i0.MSG_STATE);
	}

	ros::spin();
}


