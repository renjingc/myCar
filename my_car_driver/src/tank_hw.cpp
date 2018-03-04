/*
 * tank_hw.cpp
 *
 *  Created on: Dec 14, 2015
 *      Author: liao
 */

#include <zju_works/tank_hw.h>

TankHW::TankHW(ros::NodeHandle &nh): nh_(nh), port_()
{
	update_freq_ = 20;
	memset(cmd_, 0, sizeof cmd_);
	memset(vel_, 0, sizeof vel_);
	memset(pos_, 0, sizeof pos_);
	memset(eff_, 0, sizeof eff_);

	SerialParams params;
	params.serialPort = "/dev/ttyUSB0";
	params.baudRate = 115200;
	params.flowControl = 0;
	params.parity = 0;
	params.stopBits = 0;

	port_.setSerialParams(params);
	port_.startThread();

	debug_output_ = nh_.advertise<zju_lab_msgs::Uint8Array>("debug_serialOutput", 1);
	localize_output_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("sonar_localized", 1);

	hardware_interface::JointStateHandle state_handle_1(std::string("left_wheel"), &pos_[0], &vel_[0], &eff_[0]);
	jnt_state_interface_.registerHandle(state_handle_1);
	hardware_interface::JointStateHandle state_handle_2(std::string("right_wheel"), &pos_[1], &vel_[1], &eff_[1]);
	jnt_state_interface_.registerHandle(state_handle_2);

	hardware_interface::JointHandle vel_handle_1(state_handle_1, &cmd_[0]);
	hardware_interface::JointHandle vel_handle_2(state_handle_2, &cmd_[1]);
	jnt_vel_interface_.registerHandle(vel_handle_1);
	jnt_vel_interface_.registerHandle(vel_handle_2);

	registerInterface(&jnt_state_interface_);
	registerInterface(&jnt_vel_interface_);
}

void TankHW::write()
{
	zju_lab_msgs::Uint8Array msg;
	//protocol  header
	msg.data.push_back(0xFF);
	//vel data 341.2 pulse every cycle
	msg.data.push_back(sgn(cmd_[0]));
	msg.data.push_back((uint8_t)ceil(abs(cmd_[0]) / 2 / 3.14159 / PULSE_MEASURE_FREQ * PULSE_PER_CYCLE));
	msg.data.push_back(sgn(cmd_[1]));
	msg.data.push_back((uint8_t)ceil(abs(cmd_[1]) / 2 / 3.14159 / PULSE_MEASURE_FREQ * PULSE_PER_CYCLE));
	//protocol tail
	msg.data.push_back(0xFD);

	std::cout.setf (std::cout.hex , std::cout.basefield);
	for (int i = 0 ; i < msg.data.size(); i++)
		std::cout<< (int)msg.data[i];

	std::cout<<std::endl;

	std::cout.setf (std::cout.dec , std::cout.basefield);
	port_.writeDataGram(msg);
}
void TankHW::read()
{
	boost::unique_lock<boost::mutex> lck(port_.m_read_lock);
	while (!port_.m_ready) port_.cv.wait(lck);
	port_.m_ready = false;

	double period = (ros::Time::now() - updated_).toSec();
	updated_ = ros::Time::now();
	debug_output_.publish(port_.outputData);
	//extract the symbol
	double vel1 = (double)(((port_.outputData.data[0] * 2) - 1) * port_.outputData.data[1]) / PULSE_PER_CYCLE * 2 * 3.14159 * PULSE_MEASURE_FREQ;
	double vel2 = (double)(((port_.outputData.data[2] * 2) - 1) * port_.outputData.data[3]) / PULSE_PER_CYCLE * 2 * 3.14159 * PULSE_MEASURE_FREQ;

	vel_[0] = vel1;
	vel_[1] = vel2;

	pos_[0] += vel1 / update_freq_;
	pos_[1] += vel2 / update_freq_;

	geometry_msgs::PoseWithCovarianceStamped temp_pose;
	temp_pose.pose.pose.position.x = port_.outputData.data[4] * 100.0 + port_.outputData.data[5];
	temp_pose.pose.pose.position.y = port_.outputData.data[6] * 100.0 + port_.outputData.data[7];
	temp_pose.pose.pose.position.z = port_.outputData.data[8] * 100.0 + port_.outputData.data[9];
	temp_pose.header.frame_id = "sonar_gps";
	temp_pose.header.stamp = ros::Time::now();
	//TODO: fill in the covariance
	localize_output_.publish(temp_pose);

}
