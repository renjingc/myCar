#-*- coding:utf-8 -*- 

import os
import numpy as np
from numpy import *
import random
import math
import string


import roslib
import rospy
import tf
from std_msgs.msg import String  
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf import TransformBroadcaster 
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from _dbus_bindings import Message
from sensor_msgs.msg import LaserScan

file_object=open("/home/ren/odom_scan.txt","w")

x=0.0
y=0.0
orientation_x=0.0
orientation_y=0.0
orientation_z=0.0
orientation_w=0.0
vx=0.0
vy=0.0
vth=0.0

#beams=[]       
def odomCallback(odom):
    global x
    global y
    global orientation_x
    global orientation_y
    global orientation_z
    global orientation_w
    global vx
    global vy
    global vth
    
    x=odom.pose.pose.position.x
    y=odom.pose.pose.position.y
    orientation_x=odom.pose.pose.orientation.x
    orientation_y=odom.pose.pose.orientation.y
    orientation_z=odom.pose.pose.orientation.z
    orientation_w=odom.pose.pose.orientation.w
    
    vx=odom.twist.twist.linear.x
    vy=odom.twist.twist.linear.y
    vth=odom.twist.twist.linear.th

def scanCallback(scan):
    #time1=scan.header.stamp.sec
    #time2=scan.header.stamp.nsec
    #time=str(scan.header.stamp.sec)+'.'+str(scan.header.stamp.nsec);
    beams=[]
    time=0.0
    count=int(scan.ranges.size)
    for i in len(range(count)):
        beams.append(scan.ranges[i])
    
    str_odom=str(x)+','+str(y)+','+str(orientation_x)+','+str(orientation_y)+','+str(orientation_z)+','+str(orientation_w)+','+str(vx)+','+str(vy)+','+str(vth)
    strline=time+str_odom+str(count)
    for i in len(range(count)):
        strline=strline+str(beams)
    
    file_object.write(strline)
    
if __name__ =="__main__":
    rospy.init_node('odom_filter')
    sub1 = rospy.Subscriber('/odom', Odometry, odomCallback)
    sub2 = rospy.Subscriber('/scan', LaserScan, scanCallback)
     
    #rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rospy.spin()
        