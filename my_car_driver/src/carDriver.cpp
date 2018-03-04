/*
 * carDriver.cpp
 *
 *  Created on: Dec 14, 2015
 *      Author: liao
 */

#include <myCarDriver/carDriver.h>

void CarDriver::cmdCallback(const geometry_msgs::Twist& cmd)
{
	double delta_x,delta_y,delta_th;

	//ROS_INFO("Linear Components:[%f,%f,%f]",cmd.linear.x,cmd.linear.y,cmd.linear.z);
	//ROS_INFO("Angular Components:[%f,%f,%f]",cmd.angular.x,cmd.angular.y,cmd.angular.z);

	delta_x=cmd.linear.x;
	delta_y=cmd.linear.y;

	delta_th=cmd.angular.z;

	cmd_[0]=delta_x+delta_y+delta_th*(a+b);
	cmd_[1]=delta_x-delta_y-delta_th*(a+b);
	cmd_[2]=delta_x+delta_y-delta_th*(a+b);
	cmd_[3]=delta_x-delta_y+delta_th*(a+b);

}

CarDriver::CarDriver(ros::NodeHandle &nh,double a,double b,string s,double p): nh_(nh), port_(),a(a),b(b),portString(s),pulse_cycle(p)
{
	update_freq_ = 20;
	memset(cmd_, 0, sizeof cmd_);

	SerialParams params;
	params.serialPort = portString;//"/dev/ttyUSB1";
	params.baudRate = 115200;
	params.flowControl = 0;
	params.parity = 0;
	params.stopBits = 0;

	port_.setSerialParams(params);
	port_.startThread();

	pose_x = 0.0;
	pose_y = 0.0;
	pose_th = 0.0;

	vx = 0.0;
	vy = 0.0;
	vth = 0.0;

	cout<<"start"<<endl;
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
	cmd_sub=nh.subscribe("/cmd",10,&CarDriver::cmdCallback,this);
	debug_output_=nh.advertise<my_car_msgs::Uint8Array>("debug_output", 10);
}

void CarDriver::write()
{
	my_car_msgs::Uint8Array msg;
	uint8_t send[4]={0,0,0,0};
	for(int i=0;i<4;i++)
	{
		send[i]=(uint8_t)ceil(abs(cmd_[i]) / 2 / 3.14159 / PULSE_MEASURE_FREQ * pulse_cycle);
	}

	//protocol  header
	msg.data.push_back(0xFF);
	//vel data 341.2 pulse every cycle
	msg.data.push_back(sgn(cmd_[0]));
	msg.data.push_back(send[0]);//PULSE_PER_CYCLE));
	msg.data.push_back(sgn(cmd_[1]));
	msg.data.push_back(send[1]);//PULSE_PER_CYCLE));
	msg.data.push_back(sgn(cmd_[2]));
	msg.data.push_back(send[2]);//PULSE_PER_CYCLE));
	msg.data.push_back(sgn(cmd_[3]));
	msg.data.push_back(send[3]);//PULSE_PER_CYCLE));
	//protocol tail
	msg.data.push_back(0xFD);

    /*	std::cout.setf (std::cout.hex , std::cout.basefield);
	for (int i = 0 ; i < msg.data.size(); i++)
		std::cout<< (int)msg.data[i];

	std::cout<<std::endl;*/

	std::cout.setf (std::cout.dec , std::cout.basefield);
	port_.writeDataGram(msg);
}
double x=0.0,y=0.0,th=0.0;
void CarDriver::read()
{
	boost::unique_lock<boost::mutex> lck(port_.m_read_lock);
	while (!port_.m_ready) port_.cv.wait(lck);
	port_.m_ready = false;

	double period = (ros::Time::now() - updated_).toSec();
	updated_ = ros::Time::now();
	debug_output_.publish(port_.outputData);

	//extract the symbol
	double vel1 = (double)(((port_.outputData.data[0] * 2) - 1) * port_.outputData.data[1]) / pulse_cycle * 2 * 3.14159 * PULSE_MEASURE_FREQ;
	double vel2 = (double)(((port_.outputData.data[2] * 2) - 1) * port_.outputData.data[3]) / pulse_cycle * 2 * 3.14159 * PULSE_MEASURE_FREQ;
	double vel3 = (double)(((port_.outputData.data[4] * 2) - 1) * port_.outputData.data[5]) / pulse_cycle * 2 * 3.14159 * PULSE_MEASURE_FREQ;
	double vel4 = (double)(((port_.outputData.data[6] * 2) - 1) * port_.outputData.data[7]) / pulse_cycle * 2 * 3.14159 * PULSE_MEASURE_FREQ;

	vel_[0] = vel1;
	vel_[1] = vel2;
	vel_[2] = vel3;
	vel_[3] = vel4;

	carVelConvert();

	double delta_x = (vx * cos(th) - vy * sin(th))/update_freq_;
	double delta_y = (vx * sin(th) + vy * cos(th))/update_freq_;
	double delta_th = vth/update_freq_;
	cout<<"delta_x:"<<delta_x<<" "<<"delta_y:"<<delta_y<<" "<<"delta_th:"<<delta_th<<endl;
	/*
	pose_x += delta_x;
	pose_y += delta_y;
	pose_th += delta_th;*/
	x+=delta_x;
	y+=delta_y;
	th+=delta_th;

	cout<<"x:"<<x<<" "<<"y:"<<y<<" "<<"th:"<<th<<endl;

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = updated_;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    //odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = updated_;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
}
void CarDriver::carVelConvert()
{
	vx=(vel_[0]+vel_[1]+vel_[2]+vel_[3])/4.0;
	vy=-(vel_[1]+vel_[3]-vel_[0]-vel_[2])/4.0;
	vth=((vel_[0]-vel_[2])/2.0/(a+b)+(vel_[3]-vel_[1])/2.0/(a+b))/2.0;
}
