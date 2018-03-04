#!/usr/bin/env python

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from path_follower.msg import *
from geometry_msgs.msg import *
import actionlib
from actionlib_msgs.msg import *
from nav_msgs.msg import Path

from random import random
from math import sin

server = None
menu_handler = MenuHandler()
current_path = Path()
current_point = PoseStamped()
world_frame = "odom"
client = actionlib.SimpleActionClient('FollowPath', FollowPathAction)
current_path_pub = rospy.Publisher('current_path', Path, queue_size=10)


def processFeedback(feedback):
    global current_path
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        current_point.pose = feedback.pose
        current_point.header.frame_id = world_frame
        current_point.header.stamp = rospy.Time.now()

    if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        if feedback.menu_entry_id == 1:
            current_point.header.frame_id = world_frame
            current_point.header.stamp = rospy.Time.now()
            current_path.poses.append(copy.deepcopy(current_point))
        elif feedback.menu_entry_id == 2:
            current_path.header.frame_id = world_frame
            current_path.header.stamp = rospy.Time.now()
            current_path_pub.publish(current_path)

            goal = FollowPathGoal()
            goal.path = current_path
            client.send_goal(goal)

        elif feedback.menu_entry_id == 4:
            print "haha"
            client.cancel_goal()

        elif feedback.menu_entry_id == 3:
            current_path = Path()

    server.applyChanges()

def makeBox( msg ):
    marker = Marker()
    marker.type = Marker.CYLINDER
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control


def makePathMarker():
    position = Point()
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = world_frame
    int_marker.pose.position = position
    int_marker.pose.orientation.w = 1.0
    int_marker.scale = 1

    int_marker.name = "Path"
    int_marker.description = "Path"

    makeBoxControl(int_marker)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker.controls.append(copy.deepcopy(control))

    control.interaction_mode = InteractiveMarkerControl.MENU
    int_marker.controls.append(copy.deepcopy(control))

    server.insert(int_marker, processFeedback)
    menu_handler.apply(server, int_marker.name)

if __name__ == "__main__":
    rospy.init_node("path_helper")
    server = InteractiveMarkerServer("path_helper")

    menu_handler.insert("insert a path point", callback=processFeedback)
    menu_handler.insert("execute current path", callback=processFeedback)
    menu_handler.insert("cancel current path", callback=processFeedback)
    menu_handler.insert("cancel current executing", callback=processFeedback)
    makePathMarker()
    server.applyChanges()
    rospy.spin()