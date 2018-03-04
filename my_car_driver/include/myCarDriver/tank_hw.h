/*
 * tank_hw.h
 *
 *  Created on: Dec 14, 2015
 *      Author: liao
 */

#ifndef TANK_HW_H_
#define TANK_HW_H_

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/thread/condition_variable.hpp>
#include <limits>

#include <stdlib.h>
#include <sstream>

#include <iostream>
#include <zju_works/SerialPort.h>

#define PULSE_PER_CYCLE 1524
#define PULSE_MEASURE_FREQ 50
class TankHW :public hardware_interface::RobotHW
{
public:
	TankHW(ros::NodeHandle &nh);

	virtual void read();
	virtual void write();

	int getUpdateFreq() {return  update_freq_;}

	~TankHW()
	{
		port_.stopThread();
	}
private:
	ros::NodeHandle nh_;
	ros::Publisher debug_output_;
	ros::Publisher localize_output_;

	hardware_interface::JointStateInterface jnt_state_interface_;
	hardware_interface::VelocityJointInterface jnt_vel_interface_;
	double cmd_[2];
	double vel_[2];
	double pos_[2];
	double eff_[2];

	SerialPort port_;
	ros::Time updated_;
	int update_freq_;
	inline uint8_t sgn(double x)
	{
		if (x > 0)
		return 1;
		return 0;
	}
};

#endif /* TANK_HW_H_ */
