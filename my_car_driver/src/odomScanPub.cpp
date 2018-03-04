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
#include <tf/transform_broadcaster.h>

using namespace std;

ifstream ifile1,ifile2;
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
string rectName1,rectName2;
std::vector<std::string> vecFilelines;

struct data
{
	double time;
	pose p;
	twist v;
	double count;
	vector<double> beams;
};
vector<data> d,filterd;
void readFile()
{
	string line;
	int x=0;
    while (!ifile1.eof())
    {
    	if(getline(ifile1,line))
    	{
			istringstream iss(line);
			string col;
			data temp;
			int i=0;
			while(getline(iss,col,' '))
			{
				stringstream sstr(col);
				if(i==0)
				{
					sstr>>temp.time;
				}
				else if(i==1)
				{
					sstr>>temp.p.x;

				}
				else if(i==2)
				{
					sstr>>temp.p.y;
				}
				else if(i==3)
				{
					sstr>>temp.p.orientation_x;
				}
				else if(i==4)
				{
					sstr>>temp.p.orientation_y;
				}
				else if(i==5)
				{
					sstr>>temp.p.orientation_z;
				}
				else if(i==6)
				{
					sstr>>temp.p.orientation_w;
				}
				else if(i==7)
				{
					sstr>>temp.v.vx;
				}
				else if(i==8)
				{
					sstr>>temp.v.vy;
				}
				else if(i==9)
				{
					sstr>>temp.v.vth;
				}
				else if(i==10)
				{
					sstr>>temp.count;
				}
				else if(i<731)
				{
					double b;
					sstr>>b;
					temp.beams.push_back(b);
				}
				i++;
			}
			d.push_back(temp);
			x++;
			//cout<<x<<endl;
		}
    }
}
void readFile2()
{
	string line;
	int x=0;
    while (!ifile2.eof())
    {
    	if(getline(ifile2,line))
    	{
			istringstream iss(line);
			string col;
			data temp;
			int i=0;
			while(getline(iss,col,' '))
			{
				stringstream sstr(col);
				if(i==0)
				{
					sstr>>temp.time;
				}
				else if(i==1)
				{
					sstr>>temp.p.x;

				}
				else if(i==2)
				{
					sstr>>temp.p.y;
				}
				else if(i==3)
				{
					sstr>>temp.p.orientation_x;
				}
				else if(i==4)
				{
					sstr>>temp.p.orientation_y;
				}
				else if(i==5)
				{
					sstr>>temp.p.orientation_z;
				}
				else if(i==6)
				{
					sstr>>temp.p.orientation_w;
				}
				else if(i==7)
				{
					sstr>>temp.v.vx;
				}
				else if(i==8)
				{
					sstr>>temp.v.vy;
				}
				else if(i==9)
				{
					sstr>>temp.v.vth;
				}
				else if(i==10)
				{
					sstr>>temp.count;
				}
				else if(i<731)
				{
					double b;
					sstr>>b;
					temp.beams.push_back(b);
				}
				i++;
			}
			filterd.push_back(temp);
			x++;
			//cout<<x<<endl;
		}
    }
}
void initParam(ros::NodeHandle& nh)
{
    nh.param<string>("/odomScanPub/rectName1",rectName1,"/home/ren/7-20/odom_scan_07-20-16-23-57");
    nh.param<string>("/odomScanPub/rectName2",rectName2,"/home/ren/7-20/odom_filter_scan_07-20-16-23-57.txt");
}

int main(int argc,char **argv)
{
	ros::init(argc, argv, "odomScanPub");
	ros::NodeHandle nh;
	initParam(nh);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
	ros::Publisher odom_filter_pub = nh.advertise<nav_msgs::Odometry>("/odom_filter", 10);
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 10);

	tf::TransformBroadcaster odom_broadcaster;
	string resultsPath1 = rectName1+".txt";
	string resultsPath2 = rectName2+".txt";
	cout<<resultsPath1<<endl;
	cout<<resultsPath2<<endl;
	ifile1.open((const char*)resultsPath1.c_str());
	ifile2.open((const char*)resultsPath2.c_str());
	ros::Rate r(40);
	cout<<"start read"<<endl;
	readFile();
	readFile2();
	cout<<"start pub"<<endl;
			//first, we'll publish the transform over tf
		for(int i=0;i<d.size()&&i<filterd.size();i++)
		{
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = ros::Time::now();
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_link";

		    odom_trans.transform.translation.x = d[i].p.x;
		    odom_trans.transform.translation.y = d[i].p.y;
		    odom_trans.transform.translation.z = 0.0;
		    odom_trans.transform.rotation.x = d[i].p.orientation_x;
		    odom_trans.transform.rotation.y = d[i].p.orientation_y;
		    odom_trans.transform.rotation.z = d[i].p.orientation_z;
		    odom_trans.transform.rotation.w = d[i].p.orientation_w;
		    //send the transform
		    odom_broadcaster.sendTransform(odom_trans);

		    //next, we'll publish the odometry message over ROS
		    nav_msgs::Odometry odom;
		    odom.header.stamp = ros::Time::now();
		    odom.header.frame_id = "odom";

		    //set the position
		    odom.pose.pose.position.x = d[i].p.x;
		    odom.pose.pose.position.y = d[i].p.y;
		    odom.pose.pose.position.z = 0.0;
		    odom.pose.pose.orientation.x = d[i].p.orientation_x;
		    odom.pose.pose.orientation.y = d[i].p.orientation_y;
		    odom.pose.pose.orientation.z = d[i].p.orientation_z;
		    odom.pose.pose.orientation.w = d[i].p.orientation_w;

		    geometry_msgs::Quaternion q;
		    q.x=d[i].p.orientation_x;
		    q.y=d[i].p.orientation_y;
		    q.z=d[i].p.orientation_z;
		    q.w=d[i].p.orientation_w;
		    double th=tf::getYaw(q);
		    cout<<d[i].p.x<<" "<<d[i].p.y<<" "<<th<<endl;

		    //set the velocity
		    odom.child_frame_id = "base_link";
		    odom.twist.twist.linear.x = d[i].v.vx;
		    odom.twist.twist.linear.y = d[i].v.vy;
		    odom.twist.twist.angular.z = d[i].v.vth;

		    //next, we'll publish the odometry message over ROS
		    nav_msgs::Odometry filterOdom;
		    filterOdom.header.stamp = ros::Time::now();
		    filterOdom.header.frame_id = "odom_filtetr";

		    //set the position
		    filterOdom.pose.pose.position.x = filterd[i].p.x;
		    filterOdom.pose.pose.position.y = filterd[i].p.y;
		    filterOdom.pose.pose.position.z = 0.0;
		    filterOdom.pose.pose.orientation.x = filterd[i].p.orientation_x;
		    filterOdom.pose.pose.orientation.y = filterd[i].p.orientation_y;
		    filterOdom.pose.pose.orientation.z = filterd[i].p.orientation_z;
		    filterOdom.pose.pose.orientation.w = filterd[i].p.orientation_w;

		    geometry_msgs::Quaternion filterq;
		    filterq.x=d[i].p.orientation_x;
		    filterq.y=d[i].p.orientation_y;
		    filterq.z=d[i].p.orientation_z;
		    filterq.w=d[i].p.orientation_w;
		    double filterth=tf::getYaw(filterq);

		    //set the velocity
		    filterOdom.child_frame_id = "base_link";
		    filterOdom.twist.twist.linear.x = filterd[i].v.vx;
		    filterOdom.twist.twist.linear.y = filterd[i].v.vy;
		    filterOdom.twist.twist.angular.z = filterd[i].v.vth;


		    sensor_msgs::LaserScan scan;
		    scan.header.stamp= ros::Time::now();
		    scan.header.frame_id="laser";
		    scan.angle_min=-1.57079637051;
		    scan.angle_max=1.56643295288;
		    scan.angle_increment=0.00436332309619;
		    scan.time_increment=1.73611115315e-05;
		    scan.scan_time=0.0250000003725;
		    scan.range_min=0.0230000000447;
		    scan.range_max=30.0;
		    for(int j=0;j<d[i].beams.size();j++)
		    {
		    		scan.ranges.push_back(d[i].beams[j]);
		    }
		    //publish the message
		    odom_pub.publish(odom);
		    odom_filter_pub.publish(filterOdom);
		    scan_pub.publish(scan);
		    r.sleep();
		}
	ifile1.close();
	ifile2.close();
	return 0;
}
