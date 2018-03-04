/*
 * main.cpp
 *
 *  Created on: Dec 14, 2015
 *      Author: liao
 */

#include <controller_manager/controller_manager.h>
#include <myCarDriver/carDriver.h>

double a,b,p;
string s;
void initParam(ros::NodeHandle& nh)
{
    nh.param<double>("/carDriver/a",a,0.165);
    nh.param<double>("/carDriver/b",b,0.160);
    nh.param<string>("/carDriver/s",s,"/dev/ttyUSB0");
    nh.param<double>("/carDriver/p",p,1524);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "carDriver");
	ros::NodeHandle nh;
	initParam(nh);

	CarDriver car(nh,a,b,s,p);

	//controller_manager::ControllerManager cm(&tank, nh);

	ros::Rate rate(car.getUpdateFreq());
	ros::AsyncSpinner spinner(1);
	spinner.start();
	while(ros::ok())
	{
		car.write();
		//cm.update(ros::Time::now(), ros::Duration(1 / tank.getUpdateFreq()));
		car.read();
		rate.sleep();
	}
	spinner.stop();
	return 0;
}


