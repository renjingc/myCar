#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tfMessage.h>
#include <vector>
#include <math.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;

ofstream outfile;
vector<float> beams;
#define PAI 3.14
#define P 57.29
struct pose
{
	double x;
	double y;
	double orientation_x;
	double orientation_y;
	double orientation_z;
	double orientation_w;
};
struct twist
{
	double vx;
	double vy;
	double vth;
};
pose p;
twist v;

pose amcl_p;
twist amcl_v;

pose gps_p;
twist gps_v;
string rectName;
int n=0;
bool first=false;
double init_x=0.00000,init_y=0.00000;
void odomCallback(const nav_msgs::Odometry& odom)
{
	if(first==false)
	{
		if(odom.pose.pose.position.x!=0&&odom.pose.pose.position.y!=0)
		{
			init_x=odom.pose.pose.position.x;
			init_y=odom.pose.pose.position.y;
			first=true;
		}
	}
	else if(first)
	{
	  //ROS_INFO("I heard: [%s]", odom);
	  p.x=odom.pose.pose.position.x-init_x;
	  p.y=odom.pose.pose.position.y-init_y;
	  p.orientation_x=odom.pose.pose.orientation.x;
	  p.orientation_y=odom.pose.pose.orientation.y;
	  p.orientation_z=odom.pose.pose.orientation.z;
	  p.orientation_w=odom.pose.pose.orientation.w;

	  v.vx=odom.twist.twist.linear.x;
	  v.vy=odom.twist.twist.linear.y;
	  v.vth=odom.twist.twist.angular.z;
	}
}
void filterOdomCallback(const nav_msgs::Odometry& odom)
{

	  //ROS_INFO("I heard: [%s]", odom);
	gps_p.x=odom.pose.pose.position.x;
	gps_p.y=odom.pose.pose.position.y;
	gps_p.orientation_x=odom.pose.pose.orientation.x;
	gps_p.orientation_y=odom.pose.pose.orientation.y;
	gps_p.orientation_z=odom.pose.pose.orientation.z;
	gps_p.orientation_w=odom.pose.pose.orientation.w;

	gps_v.vx=odom.twist.twist.linear.x;
	gps_v.vy=odom.twist.twist.linear.y;
	gps_v.vth=odom.twist.twist.angular.z;

}
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped amcl_pose)
{
	if(first)
	{
		amcl_p.x=amcl_pose.pose.pose.position.x;
		amcl_p.y=amcl_pose.pose.pose.position.y;
		amcl_p.orientation_x=amcl_pose.pose.pose.orientation.x;
		amcl_p.orientation_y=amcl_pose.pose.pose.orientation.y;
		amcl_p.orientation_z=amcl_pose.pose.pose.orientation.z;
		amcl_p.orientation_w=amcl_pose.pose.pose.orientation.w;

		stringstream x,y,orientation_x,orientation_y,orientation_z,orientation_w,vx,vy,vth;
		x<<p.x;
		y<<p.y;
		orientation_x<<p.orientation_x;
		orientation_y<<p.orientation_y;
		orientation_z<<p.orientation_z;
		orientation_w<<p.orientation_w;
		string odomStr=x.str()+" "+y.str()+" "+orientation_x.str()+" "+orientation_y.str()+" "+orientation_z.str()+" "+orientation_w.str();
		string lineString=odomStr;

		stringstream amcl_x,amcl_y,amcl_orientation_x,amcl_orientation_y,amcl_orientation_z,amcl_orientation_w,amcl_vx,amcl_vy,amcl_vth;
		amcl_x<<amcl_p.x;
		amcl_y<<amcl_p.y;
		amcl_orientation_x<<amcl_p.orientation_x;
		amcl_orientation_y<<amcl_p.orientation_y;
		amcl_orientation_z<<amcl_p.orientation_z;
		amcl_orientation_w<<amcl_p.orientation_w;
		string amclStr=amcl_x.str()+" "+amcl_y.str()+" "+amcl_orientation_x.str()+" "+amcl_orientation_y.str()+" "+amcl_orientation_z.str()+" "+amcl_orientation_w.str();


		stringstream gps_x,gps_y,gps_orientation_x,gps_orientation_y,gps_orientation_z,gps_orientation_w,gps_vx,gps_vy,gps_vth;
		gps_x<<gps_p.x;
		gps_y<<gps_p.y;
		gps_orientation_x<<gps_p.orientation_x;
		gps_orientation_y<<gps_p.orientation_y;
		gps_orientation_z<<gps_p.orientation_z;
		gps_orientation_w<<gps_p.orientation_w;
		string gpsStr=gps_x.str()+" "+gps_y.str()+" "+gps_orientation_x.str()+" "+gps_orientation_y.str()+" "+gps_orientation_z.str()+" "+gps_orientation_w.str();

		lineString=lineString+" "+amclStr+" "+gpsStr;
		outfile<<lineString<<endl;
	}
}
void laserCallback(const sensor_msgs::LaserScan& scan)
{
	if(first)
	{
		uint32_t time1=scan.header.stamp.sec;
		uint32_t time2=scan.header.stamp.nsec;
		stringstream timeSS1,timeSS2;
		timeSS1<<time1;
		timeSS2<<time2;
		string timeS=timeSS1.str()+'.'+timeSS2.str();
		int count=scan.ranges.size();
		beams.clear();
		for(int i=0;i<count;i++)
		{
			beams.push_back(scan.ranges[i]);
		}
		stringstream x,y,orientation_x,orientation_y,orientation_z,orientation_w,vx,vy,vth;
		x<<p.x;
		y<<p.y;
		orientation_x<<p.orientation_x;
		orientation_y<<p.orientation_y;
		orientation_z<<p.orientation_z;
		orientation_w<<p.orientation_w;
		vx<<v.vx;
		vy<<v.vy;
		vth<<v.vth;

		string odomStr=x.str()+" "+y.str()+" "+orientation_x.str()+" "+orientation_y.str()+" "+orientation_z.str()+" "+orientation_w.str()+" "+vx.str()+" "+vy.str()+" "+vth.str();
		string lineString=timeS+" "+odomStr;
		stringstream countS;
		countS<<count;
		lineString=lineString+" "+countS.str();
		for(vector<float>::iterator it=beams.begin();it!=beams.end();++it)
		{
			stringstream beamsSS;
			beamsSS<<(*it);
			string beamsS=beamsSS.str();
			if(beamsS=="inf")
			{
				beamsS="0";
			}
			lineString=lineString+" "+beamsS;
		}
		stringstream init_xS,init_yS;
		init_xS<<init_x;
		init_yS<<init_y;
		lineString=lineString+" "+init_xS.str()+" "+init_yS.str();
		outfile<<lineString<<endl;
	}
}
void initParam(ros::NodeHandle& nh)
{
    nh.param<string>("/datalog/rectName",rectName,"/home/ren/odom_scan");
}

int main(int argc,char **argv)
{
	ros::init(argc, argv, "datalog2");
	ros::NodeHandle nh;
	initParam(nh);
	p.x=0.0;
	p.y=0.0;
	p.orientation_x=0.0;
	p.orientation_y=0.0;
	p.orientation_z=0.0;
	p.orientation_w=1.0;
	amcl_p.x=0.0;
	amcl_p.y=0.0;
	amcl_p.orientation_x=0.0;
	amcl_p.orientation_y=0.0;
	amcl_p.orientation_z=0.0;
	amcl_p.orientation_w=1.0;
	ros::Subscriber sub1 = nh.subscribe("/odom", 3, odomCallback);
	ros::Subscriber sub4 = nh.subscribe("/odom_filter", 3, filterOdomCallback);
	//ros::Subscriber sub2 = nh.subscribe("/scan", 10, laserCallback);
	ros::Subscriber sub3 = nh.subscribe("/amcl_pose",3,amclCallback);

	string resultsPath = rectName+".txt";
	outfile.open("/home/ren/7-20/odom_amcl_gps.txt");
	cout<<"start"<<endl;
	ros::spin();

	while(ros::ok())
	{

	}
	outfile.close();
	return 0;
}
