#!/usr/bin/env python
import math

class TF:
    def __init__(self):
        self.a=0

    #TargetPoint[0]==North(x,head) TargetPoint[1]==East(y,right) TargetPoint[3]=yaw(shun shi zhen,x dir is 0 and 2pi)
    #get the target point in baselink coordinate system
    #TargetPoint and BaseLinkPoint is in rtkGPS coordinate system
    def rtkGPStoBaseLink(self,TargetPoint, BaseLinkPoint):
        X_rtkGPS_Target = TargetPoint[0]
        Y_rtkGPS_Target = TargetPoint[1]
        YAW_rtkGPS_Target = TargetPoint[2]
        X_rtkGPS_BaseLink = BaseLinkPoint[0]
        Y_rtkGPS_BaseLink = BaseLinkPoint[1]
        YAW_rtkGPS_BaseLink = BaseLinkPoint[2]
        #cal the Target point in baselink coordinate system
        X_BaseLink_Target=(X_rtkGPS_Target - X_rtkGPS_BaseLink)*math.cos(YAW_rtkGPS_BaseLink)\
            + (Y_rtkGPS_Target - Y_rtkGPS_BaseLink)*math.sin(YAW_rtkGPS_BaseLink)

        Y_BaseLink_Target=-(X_rtkGPS_Target - X_rtkGPS_BaseLink)*math.sin(YAW_rtkGPS_BaseLink)\
            + (Y_rtkGPS_Target - Y_rtkGPS_BaseLink)*math.cos(YAW_rtkGPS_BaseLink)

        YAW_BaseLink_Target = YAW_rtkGPS_Target - YAW_rtkGPS_BaseLink

        Y_BaseLink_Target = -Y_BaseLink_Target
        YAW_BaseLink_Target = -YAW_BaseLink_Target
        
        if YAW_BaseLink_Target > 2*math.pi:
            YAW_BaseLink_Target = YAW_BaseLink_Target - 2*math.pi
        if YAW_BaseLink_Target < -2*math.pi:
            YAW_BaseLink_Target = YAW_BaseLink_Target + 2*math.pi
        if YAW_BaseLink_Target > math.pi:
            YAW_BaseLink_Target = YAW_BaseLink_Target - 2*math.pi
        if YAW_BaseLink_Target < -math.pi:
            YAW_BaseLink_Target = YAW_BaseLink_Target + 2*math.pi

        return [X_BaseLink_Target, Y_BaseLink_Target, YAW_BaseLink_Target]



if __name__ == "__main__":
    body = TF()
    TargetPoint = [10, 10, (-1/4.0)*math.pi]
    BaseLinkPoint = [5, 5, math.pi]
    print body.rtkGPStoBaseLink(TargetPoint, BaseLinkPoint)
