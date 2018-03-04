/*
 * tank_hw.h
 *
 *  Created on: Dec 14, 2015
 *      Author: liao
 */

#ifndef CARDRIVER_H_
#define CARDRIVER_H_

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
#include <string>

#include <iostream>
#include <myCarDriver/SerialPort.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#define PULSE_PER_CYCLE 1524
#define PULSE_MEASURE_FREQ 50
#define DEG_TO_ANGLE   57.324
class CarDriver
{
public:
	CarDriver(ros::NodeHandle &nh,double a,double b,string s,double p);

	void read();
	void write();

	int getUpdateFreq() {return  update_freq_;}

	~CarDriver()
	{
		port_.stopThread();
	}
private:
	ros::NodeHandle nh_;
	ros::Publisher debug_output_;

	ros::Publisher odom_pub;
	ros::Subscriber cmd_sub;
	tf::TransformBroadcaster odom_broadcaster;

	double a,b;
	double cmd_[4];
	double data[];
	double vel_[2];
	//double pos_[2];
	//double eff_[2];
	double pose_x;
	double pose_y;
	double pose_th;

	double vx;
	double vy;
	double vth;

	SerialPort port_;
	ros::Time updated_;
	int update_freq_;
	string portString;
	double pulse_cycle;

	void carVelConvert();
	void cmdCallback(const geometry_msgs::Twist& cmd);

	inline uint8_t sgn(double x)
	{
		if (x > 0)
		return 1;
		return 0;
	}
};

#endif /* TANK_HW_H_ */
