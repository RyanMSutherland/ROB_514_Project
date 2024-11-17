#!/usr/bin/env python3

# This code is intended to roam between defined points in our map

import rospy
from geometry_msgs.msg import *

class Location:
    def __init__(self):
        pose_publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped)
        p = PoseWithCovarianceStamped()
        msg = PoseWithCovariance()
        msg.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.000, 0.000, 0.000, 1.0))
        msg.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                          0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        
        p.pose = msg
        pose_publisher.publish(p)

    def set_movement_goal():
        

        

if __name__ == '__main__':
    rospy.init_node('initial_pose')
    rospy.loginfo("Setting Pose")