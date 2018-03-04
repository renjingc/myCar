#!/usr/bin/env python
# license removed for brevity
import rospy
import Motion_Planning
import Map_Point
import TF
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from beginner_tutorials.msg import rtkGPSmessage






class functionAll:
     def __init__(self):
         rospy.init_node('FunctionAll', anonymous=False)
         rospy.Subscriber("joy", Joy, self.callback1)
         rospy.Subscriber("rtkGPS", rtkGPSmessage, self.callback2)
         self.cmd_vel_pub = rospy.Publisher('cmd', Twist, queue_size=5)
         self.rate = rospy.Rate(30)
         self.cmd = Twist()
         self.cmd.linear.x = 0.0
         self.cmd.linear.y = 0.0
         self.cmd.linear.z = 0
         self.cmd.angular.z = 0
         self.cmd.angular.z = 0
         self.cmd.angular.z = 0.0

         self.MotionPlaner_speedXY = float(rospy.get_param("MotionPlaner_speedXY",'0.2'))
         self.MotionPlaner_speedYaw = float(rospy.get_param("MotionPlaner_speedYaw",'0.2'))
         self.MotionPlaner_errorMeter = float(rospy.get_param("MotionPlaner_errorMeter",'0.2'))#m
         self.MotionPlaner_errorAngle = float(rospy.get_param("MotionPlaner_errorAngle",'0.2'))#rad

         self.MotionPlaner = Motion_Planning.Motion_Planning(self.MotionPlaner_speedXY, self.MotionPlaner_speedYaw, self.MotionPlaner_errorMeter, self.MotionPlaner_errorAngle)
         self.MapPoint = Map_Point.Map_Point()
         self.TF_body = TF.TF()
         #MapPoint is in rtkGPS coordinate system

         self.North_base = float(rospy.get_param("North_base",'0.2'))
         self.East_base = float(rospy.get_param("East_base",'0.2'))
         self.Yaw_base = float(rospy.get_param("Yaw_base",'0.0'))
         print self.North_base ,self.East_base
         '''
         readfile = open("round4.txt",'r')
         line = readfile.readline()
         num=1
         
         while line:
              print line
              num = num + 1
              temp = line.split(',')
              self.MapPoint.add_point(float(temp[0]), float(temp[1]), float(temp[2]))
              line = readfile.readline()
              
         readfile.close()
         print "data load. point num = " + str(num)
'''
         #self.MapPoint.add_point(self.North_base+0, self.East_base+0,self.Yaw_base+0)
         #self.MapPoint.add_point(self.North_base+5, self.East_base+0,self.Yaw_base+0)
         #self.MapPoint.add_point(self.North_base+5, self.East_base+0,self.Yaw_base+math.pi/2)
         #self.MapPoint.add_point(self.North_base+5, self.East_base+5,self.Yaw_base+math.pi/2)
         self.MapPoint.add_point(3349259.83,  511222.59,3.47215795517)
         self.MapPoint.add_point(3349257.58, 511221.74,3.47215795517)
         self.MapPoint.add_point(3349257.58, 511221.74, 2.73)
         self.MapPoint.add_point(3349256.70, 511221.95,2.73)
         self.MapPoint.add_point(3349259.83,  511222.59,3.47215795517)
      
         
         self.MapPointNum_Now = 0
         self.MapPointNum_All = len(self.MapPoint.pointlist_x)

         self.Position_now_rtkGPS = [self.North_base+0, self.East_base+0,self.Yaw_base]
         self.EnableFlag = False
         self.EnableCount = [0,0]
         self.Position_BaseLink_Target=[0,0,0]

         while not rospy.is_shutdown():
             self.rate.sleep()

     def callback1(self, data):
         print data

     def callback2(self, data):
         if self.MapPointNum_Now < self.MapPointNum_All:
             if data.flash_state == 'YAW':
                 self.Position_now_rtkGPS[2] = data.yaw_rad
                 self.EnableCount[1] = self.EnableCount[1]+1
             if data.flash_state == 'POSITION':
                 self.Position_now_rtkGPS[0] = data.north_meter+0.5*math.sin(self.Position_now_rtkGPS[2])
                 self.Position_now_rtkGPS[1] = data.east_meter+0.5*math.cos(self.Position_now_rtkGPS[2])
                 print self.Position_now_rtkGPS
                 self.EnableCount[0] = self.EnableCount[0]+1
             if data.flash_state == 'Data_Valid_Fault':
                 self.EnableFlag = False
                 self.EnableCount[0] = 0
                 self.EnableCount[1] = 0
             if self.EnableCount[0] > 10 and self.EnableCount[1] > 10 :
                 self.EnableFlag = True
                 self.EnableCount[0] = 11
                 self.EnableCount[1] = 11

             if self.EnableFlag:
                 self.Position_BaseLink_Target=self.TF_body.rtkGPStoBaseLink(self.MapPoint.get_point(self.MapPointNum_Now), self.Position_now_rtkGPS)
		 print self.Position_BaseLink_Target
                 self.MotionPlaner.setNowPosition([0,0,0])
                 self.MotionPlaner.setNextTarget(self.Position_BaseLink_Target)
                 self.Motion_temp = self.MotionPlaner.MotionPlan()
                 self.cmd.linear.x = self.Motion_temp[0]
                 self.cmd.linear.y = self.Motion_temp[1]
                 self.cmd.linear.z = 0
                 self.cmd.angular.z = 0
                 self.cmd.angular.z = 0
                 self.cmd.angular.z = self.Motion_temp[2]*2
                 self.cmd_vel_pub.publish(self.cmd)
             else:
                 self.cmd.linear.x = 0
                 self.cmd.linear.y = 0
                 self.cmd.linear.z = 0
                 self.cmd.angular.z = 0
                 self.cmd.angular.z = 0
                 self.cmd.angular.z = 0
                 self.cmd_vel_pub.publish(self.cmd)

             if self.MotionPlaner.reach():
                 print "MapPointNum_Now" + str(self.MapPointNum_Now)
                 self.MapPointNum_Now = self.MapPointNum_Now+1
                 if self.MapPointNum_Now == self.MapPointNum_All:
                      self.MapPointNum_Now = self.MapPointNum_All-1



if __name__ == "__main__":
    try:
        body = functionAll()
    except:
        rospy.logwarn("functionAll closed!")

