#!/usr/bin/env python
# license removed for brevity
import rospy
from beginner_tutorials.msg import rtkGPSmessage
import pylab
import math
import matplotlib
north = []
east = []
yaw =[]
count_yaw=[]

north_sparse = []
east_sparse = []


def callback1(data):
    if data.vaild_flag:
        if data.flash_state == 'POSITION':
            if len(yaw) > 0:
                north.append(data.north_meter+0.5*math.sin(yaw[-1]))
                east.append(data.east_meter+0.5*math.cos(yaw[-1]))
                #north.append(data.north_meter)
                #east.append(data.east_meter)
                if len(north_sparse) == 0:
                    north_sparse.append(north[-1])
                    east_sparse.append(east[-1])
                else:
                    if (math.sqrt((north[-1] - north_sparse[-1])*(north[-1] - north_sparse[-1])\
                             +(east[-1] - east_sparse[-1])*(east[-1] - east_sparse[-1])) > 2.0):
                        north_sparse.append(north[-1])
                        east_sparse.append(east[-1])
                        savefile = file("round4.txt", 'a+')
                        savefile.write(str(north_sparse[-1])+','+str(east_sparse[-1])+','+str(yaw[-1])+'\r\n')
                        savefile.close()
                        

        if data.flash_state == 'YAW':
            yaw.append(data.yaw_rad)
            count_yaw.append(len(yaw))
            #print len(yaw)







if __name__ == "__main__":
    rospy.init_node('rtkGPS_anlyse', anonymous=False)
    rospy.Subscriber("rtkGPS", rtkGPSmessage, callback1)
    rate = rospy.Rate(1000)
    plt=matplotlib.pyplot
    while not rospy.is_shutdown():
        if len(north)>100:

            print len(north_sparse)
            #plt.plot(east,north,'r*')
            #plt.plot(east_sparse,north_sparse,'b-')
            pylab.hold(True)
            pylab.plot(count_yaw,yaw,'r*')
            plt.grid(True)
            #plt.axis('equal')
            plt.show()

        rate.sleep()
'''
    while not rospy.is_shutdown():
        if len(north)>100:
            print len(north_sparse)
            pylab.plot(east,north,'r*')
            pylab.plot(east_sparse,north_sparse,'b-')
            #pylab.hold(True)
            #pylab.plot(count_yaw,yaw,'r*')
            pylab.grid(True)
            pylab.axis('equal')
            pylab.show()
            print "asadfsdgfsdgfs"
            #pylab.drawnow()
'''

    

            #pylab.drawnow()

        #rate.sleep()


