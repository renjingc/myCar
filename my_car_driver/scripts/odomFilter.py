#-*- coding:utf-8 -*- 

import os
import numpy as np
from numpy import *
import random
import math
import string

from code.book_plots import interactive_plot
from code.kf_design_internal import sensor_fusion_kf, set_radar_pos
from filterpy.stats import plot_covariance_ellipse
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
import code.book_plots as bp

import roslib
import rospy
import tf
from std_msgs.msg import String  
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from my_car_msgs.msg import rtkGPSmessage
from tf import TransformBroadcaster 
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from _dbus_bindings import Message
'''
def FirstOrderKF(R, Q, gpsW,odomW,dt):
    """ Create first order Kalman filter.
    Specify R and Q as floats."""
    kf = KalmanFilter(dim_x=6, dim_z=6)
    kf.x = np.array([[0, 0, 0, 0, 0, 0]]).T
    kf.P  = np.eye(6) * 0.0001
    kf.R  = [[odomW,   0.,              0.,                  0.,              0.,              0.],
             [0.,               odomW,     0.,                  0.,              0.,              0.],
             [0.,               0.,              odomW,         0.,              0.,              0.],
             [0.,               0.,              0.,                  gpsW,         0.,              0.],
             [0.,               0.,              0.,                  0.,              gpsW,         0.],
             [0.,               0.,              0.,                  0.,              0.,              gpsW]]
    #np.eye(3) * R
    kf.Q  = np.eye(6) * Q
    kf.F = np.array([[1, dt, 0,  0, 0, 0],
                     [0,  1, 0,  0, 0, 0],
                     [0,  0, 1, dt, 0, 0],
                     [0,  0, 0,  1, 0, 0],
                     [0,  0, 0,  0, 1, dt],
                     [0,  0, 0,  0, 0, 1]])
    kf.H = np.array([[0,  1., 0,  0,  0,  0],
                     [0,  0,  0,  1., 0,  0],
                     [0,  0,  0,  0,  0, 1.],
                     [1., 0,  0,  0,  0,  0],
                     [0,  0,  1., 0,  0,  0],
                     [0,  0,  0,  0,  1., 0]])
    return kf
'''


def FirstOrderKF(R, Q, dt):
    """ Create first order Kalman filter.
    Specify R and Q as floats."""
    kf = KalmanFilter(dim_x=4, dim_z=4)
    kf.x = np.array([[0, 0, 0, 0]]).T
    kf.P  = np.eye(4) * 0.0001
    kf.R  = [[0.04017680,   0.,                     0.,                          0.],
             [0.,                      0.04082702,     0.,                          0.],
             [0.,                      0.,                     0.0002017680,    0.],
             [0.,                      0.,                     0.,                          0.0002017680]]
    #np.eye(3) * R
    kf.Q  = np.eye(4) * Q
    kf.F = np.array([[1, dt, 0,  0],
                            [0,  1, 0,  0],
                            [0,  0, 1, dt],
                            [0,  0, 0,  1]])
    kf.H = np.array([[0,  1., 0,  0],
                              [0,  0,  0,  1.],
                              [1., 0,  0,  0],
                              [0,  0,  1., 0]])
    return kf

def xyz_kalman_fliter(zs):
    robot_tracker = FirstOrderKF(0.0002,0.0000001,0.05)
    mu, cov, _, _= robot_tracker.batch_filter(zs)
    return mu

def xyz_kalman_filter_time(robot_tracker,zs):
    prior = robot_tracker.predict()
    robot_tracker.update(zs)
    return robot_tracker.x

gps_x = 0.0
gps_y = 0.0
gps_th = 0.0

init_x=0.0
init_y=0.0
init_th=0.0


odom_x=0.0
odom_y=0.0
odom_th=0.0
odom_vx=0.0
odom_vy=0.0
odom_vth=0.0


first_gps=False
first_mix=False
ifGPS=False
#result=[]
result=[0.0,0.0,0.0,0.0]
old_result=[0.0,0.0,0.0,0.0]

P=57.2956
pai=3.1415926
period=0.0
#updated_=rospy.Time.now()
#robot_tracker = FirstOrderKF(0.0002,0.0000001,0.00002017680,0.02017680,0.05)
robot_tracker = FirstOrderKF(0.0002,0.0000001,0.05)
pub = rospy.Publisher('/odom_filter',Odometry,queue_size=3)  
odom_broadcaster = tf.TransformBroadcaster()
file_object1=open("/home/ren/gps_filter.txt",'w')
file_object2=open("/home/ren/odom_gps.txt","w")
def pub_odom_filter(pub):
    
        result_th=gps_th
        
        if result_th>2*pai:
            result_th=result_th-2*pai
        elif result_th<0:
            result_th=result_th+2*pai
            
        #send the transform
        odom_broadcaster.sendTransform((result[0]+init_x, result[2]+init_y, 0),
                                      #tf.transformations.quaternion_from_euler(0, 0, result[4]/P+3.14),
                                                                    (0,0,sin(result_th/2),cos(result_th/2)),
                                                                    rospy.Time.now(),
                                                                    "base_link",
                                                                    "odom_filter"
                                                                    )
         

        #next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom_filter"
        odom.child_frame_id = "base_link"

        #set the position
        odom.pose.pose.position.x = result[0]+init_x
        odom.pose.pose.position.y = result[2]+init_y
        odom.pose.pose.position.z = 0.0
        #odom.pose.pose.orientation=odom_quat
        odom.pose.pose.orientation.x = 0
        odom.pose.pose.orientation.y = 0
        odom.pose.pose.orientation.z = sin((result_th)/2)
        odom.pose.pose.orientation.w = cos((result_th) / 2)

        
        #set the velocity
        odom.header.frame_id = "odom_filter"
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = result[1]
        odom.twist.twist.linear.y = result[3]
        odom.twist.twist.angular.z = odom_vth
        
        print "init_x:%.2f" %init_x+" init_y:%.2f" %init_y+" init_th:%.3f" %init_th+" x:%.3f" %result[0]+" y:%.3f" %result[2]+" th:%.3f" %result_th
        pub.publish(odom)
        
def odomCallback(odom):
    global result
    global old_result
    global gps_th
    global updated_
    global first_mix
    global odom_x
    global odom_y
    global odom_th
    global odom_vx
    global odom_vy
    global odom_vth
    temp_odom_x=odom.pose.pose.position.x
    temp_odom_y=odom.pose.pose.position.y
    x=odom.pose.pose.orientation.x
    y=odom.pose.pose.orientation.y
    z=odom.pose.pose.orientation.z
    w=odom.pose.pose.orientation.w
    odom_th=math.atan2(2.0*(w*z+x*y),(1.0-2.0*(y*y+z*z)))
    
    temp_odom_vx=odom.twist.twist.linear.x
    temp_odom_vy=odom.twist.twist.linear.y
    temp_odom_vth=odom.twist.twist.angular.z
    
    if init_th!=0 and init_th!=2*pai:
        #period = (rospy.Time.now()-updated_).toSec()
        updated_ = rospy.Time.now()
        odom_th = odom_th+init_th
        
        if odom_th>2*pai:
            odom_th=odom_th-2*pai
        elif odom_th<0:
            odom_th=odom_th+2*pai
            
        if gps_th>2*pai:
            gps_th=gps_th-2*pai
        elif gps_th<0:
            gps_th=gps_th+2*pai
            
        
        odom_x = temp_odom_x*cos(init_th)-temp_odom_y*sin(init_th)
        odom_y = temp_odom_x*sin(init_th)+temp_odom_y*cos(init_th)
        
        odom_vx=temp_odom_vx* cos(float(gps_th)) - temp_odom_vy * sin(float(gps_th))
        odom_vy=temp_odom_vx* sin(float(gps_th)) + temp_odom_vy * cos(float(gps_th))
        odom_vth=temp_odom_vth

        if first_mix==False:
            robot_tracker.x=np.array([[0, 0, 0, 0]]).T
            first_mix=True
        if ifGPS==True:
            '''
            robot_tracker.R=[[0.02017680,   0.,              0.,                  0.,             0.,              0.],
                                        [0.,             0.02082702,    0.,                  0.,             0.,              0.],
                                        [0.,             0.,              0.0207528,        0.,             0.,              0.],
                                        [0.,             0.,              0.,                  0.00002017680,   0.,              0.],
                                        [0.,             0.,              0.,                  0.,             0.00002017680,    0.],
                                        [0.,             0.,              0.,                  0.,             0.,              0.00002017680]]
            '''
            robot_tracker.R=[[0.04017680,   0.,                     0.,                              0.],
                                        [0.,                   0.04082702,     0.,                              0.],
                                        [0.,                   0.,                     0.0002017680,        0.],
                                        [0.,                   0.,                     0.,                              0.0002017680],]
            temp = []
            temp.append(odom_vx)
            temp.append(odom_vy)
            #temp.append(odom_vth)
            temp.append(gps_x)
            temp.append(gps_y)
            #temp.append(gps_th)
        
            zs = np.array([temp[:]]).T
            result=xyz_kalman_filter_time(robot_tracker,zs)
            if gps_th>2*pai:
                gps_th=gps_th-2*pai
            elif gps_th<0:
                gps_th=gps_th+2*pai
            
            old_result=result
            
        else:
            gps_th=gps_th+float(temp_odom_vth)*0.05
            if gps_th>2*pai:
                gps_th=gps_th-2*pai
            elif gps_th<0:
                gps_th=gps_th+2*pai
            odom_vx=temp_odom_vx* cos(float(gps_th)) - temp_odom_vy * sin(float(gps_th))
            odom_vy=temp_odom_vx* sin(float(gps_th)) + temp_odom_vy * cos(float(gps_th))

            result[0]=result[0]+odom_vx*0.05
            result[1]=odom_vx
            result[2]=result[2]+odom_vy*0.05
            result[3]=odom_vy
        
        write_result_x=float(result[0])
        write_result_y=float(result[2])
        write_result_th=float(gps_th)
        '''
        strline=str(gps_x)+','+str(gps_y)+','+str(gps_th)+','+str(write_result_x)+','+str(write_result_y)+','+str(write_result_th)+'\n'
        file_object1.write(strline)
        '''
        if ifGPS==True:
            isGPS=1
        else:
            isGPS=0
        #strline2=str((gps_x+init_x))+','+str((-gps_y+init_y))+','+str(gps_th)+','+str(temp_odom_vx)+','+str(temp_odom_vy)+','+str(temp_odom_vth)+','+str((write_result_x+init_x))+','+str((-write_result_y+init_y))+','+str(write_result_th)+','+str(isGPS)+'\n'
        
        #file_object2.write(strline2)
        #if ifGPS==True:
        pub_odom_filter(pub)
        
def gpsOdomCallback(message):
    global first_gps
    global init_x
    global init_y
    global init_th
    global gps_x
    global gps_y
    global gps_th
    global ifGPS
    if message.vaild_flag==True:
        ifGPS=True
        if first_gps==False:
            if message.north_meter!=0 and message.east_meter!=0 and message.yaw_rad!=0:            
                init_x=message.north_meter
                init_y=message.east_meter
                init_th=2*pai-message.yaw_rad
                first_gps=True
        else:
            gps_x=message.north_meter-init_x
            gps_y=-(message.east_meter-init_y)
            gps_th=2*pai-message.yaw_rad
    else:
        ifGPS=False

if __name__ =="__main__":
    rospy.init_node('odom_filter')
    sub1 = rospy.Subscriber('/odom', Odometry, odomCallback)
    sub2 = rospy.Subscriber('/rtkGPS', rtkGPSmessage, gpsOdomCallback)
     
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rospy.spin()
        