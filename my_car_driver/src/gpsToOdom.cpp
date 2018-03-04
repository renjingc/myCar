#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include "ros/ros.h"
#include <math.h>

#include <iostream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <my_car_msgs/rtkGPSmessage.h>
using namespace std;

void gpsCallback(const my_car_msgs::rtkGPSmessage& message);

/*
void initParam(ros::NodeHandle& nh)
{
    nh.param<double>("/carDriver/a",a,0.165);
    nh.param<double>("/carDriver/b",b,0.160);
    nh.param<string>("/carDriver/s",s,"/dev/ttyUSB0");
    nh.param<double>("/carDriver/p",p,1524);
}
*/
double x = 0.0;
double y = 0.0;
double th = 0.0;

double init_x=0.0;
double init_y=0.0;
double init_th=0.0;
 
double old_x = 0.0;
double old_y = 0.0;
double old_th = 0.0;

double delta_x=0.0;
double delta_y=0.0;
double delta_th=0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

bool first_gps=false;
ros::Time updated_;
ofstream outfile;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gpsToOdom");
	ros::NodeHandle nh;

	tf::TransformBroadcaster odom_broadcaster;
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 50);
	ros::Subscriber gps_sub=nh.subscribe("/rtkGPS",10,gpsCallback);

	//string resultsPath = rectName+".txt";
	outfile.open("/home/renjing/datalog/gps.txt");
	ros::Rate r(20.0);
	//ros::AsyncSpinner spinner(1);
	//spinner.start();

	while(ros::ok())
	{
		ros::spinOnce();               // check for incoming messages

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	    //first, we'll publish the transform over tf
	    geometry_msgs::TransformStamped odom_trans;
	    odom_trans.header.stamp = updated_;
	    odom_trans.header.frame_id = "gps_odom";
	    odom_trans.child_frame_id = "base_link";

	    odom_trans.transform.translation.x = x;
	    odom_trans.transform.translation.y = y;
	    odom_trans.transform.translation.z = 0.0;
	    odom_trans.transform.rotation = odom_quat;

	    //send the transform
	    odom_broadcaster.sendTransform(odom_trans);

	    //next, we'll publish the odometry message over ROS
	    nav_msgs::Odometry odom;
	    odom.header.stamp = updated_;
	    odom.header.frame_id = "gps_odom";

	    //set the position
	    odom.pose.pose.position.x = x;
	    odom.pose.pose.position.y = y;
	    odom.pose.pose.position.z = 0.0;
	    odom.pose.pose.orientation = odom_quat;

	    //set the velocity
	    odom.child_frame_id = "base_link";
	    odom.twist.twist.linear.x = delta_x;
	    odom.twist.twist.linear.y = delta_y;
	    odom.twist.twist.angular.z = delta_th;

	    //publish the message
	    odom_pub.publish(odom);
		r.sleep();
	}
	outfile.close();
	//spinner.stop();
	return 0;
}

void gpsCallback(const my_car_msgs::rtkGPSmessage& message)
{
	double period = (ros::Time::now() - updated_).toSec();
	updated_ = ros::Time::now();
	cout<<"init_x:"<<init_x<<" "<<"init_y:"<<init_y<<" "<<"init_th"<<init_th<<endl;
	if(!first_gps)
	{

		init_x=message.north_meter;
		init_y=message.east_meter;
		init_th=6.28-message.yaw_rad;
		first_gps=true;
	}
	else
	{
		x=message.north_meter-init_x;
		y=message.east_meter-init_y;
		th=6.28-message.yaw_rad;
	
		vx=x-old_x;
		vy=y-old_y;
		vth=th-old_th;

		delta_x =0;// ((vx+vy*tan(th))/(cos(th)+sin(th)*tan(th)))*period;
		delta_y =0;// ((vy-vx*tan(th))/(cos(th)+sin(th)*tan(th)))*period;
		delta_th =0;// vth*period;

		cout<<"period:"<<period<<endl;

		old_x=x;
		old_y=y;
		old_th=th;
	}
	string lineString;
	stringstream poseGPSXSS;
	poseGPSXSS<<message.north_meter;
	string poseGPSXS=poseGPSXSS.str();
	stringstream poseGPSYSS;
	poseGPSYSS<<message.east_meter;
	string poseGPSYS=poseGPSYSS.str();
	ostringstream poseGPSthetaSS;
	poseGPSthetaSS<<message.yaw_rad;
	string poseGPSthetaS=poseGPSthetaSS.str();
	lineString=lineString+" "+poseGPSXS+" "+poseGPSYS+" "+poseGPSthetaS;
	outfile<<lineString<<endl;
//	double delta_x = vx * cos(th) - vy * sin(th));
//	double delta_y = vx * sin(th) + vy * cos(th));

 
}


