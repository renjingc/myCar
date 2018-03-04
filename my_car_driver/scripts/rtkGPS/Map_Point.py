#!/usr/bin/env python
import math

class Map_Point:
    def __init__(self):
        self.pointlist_x = []
        self.pointlist_y = []
        self.pointlist_yaw = []



    def add_point(self, point_x, point_y,point_yaw):
        self.pointlist_x.append(point_x)
        self.pointlist_y.append(point_y)
        self.pointlist_yaw.append(point_yaw)

    def get_point(self,num):
        return [self.pointlist_x[num], self.pointlist_y[num],self.pointlist_yaw[num]]




if __name__ == "__main__":
    body = Map_Point()
    body.add_point(0,0,0)
    body.add_point(0,1,0)
    body.add_point(1,1,0)
    body.add_point(1,0,0)
    print body.get_point(0)
    print body.get_point(1)
    print body.get_point(2)
    print body.get_point(3)

    print body.get_point_number()
