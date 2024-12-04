#!/usr/bin/env python3

# This code is intended to roam between defined points in our map

import rospy
from geometry_msgs.msg import *

class Location:
    def __init__(self):
        pose_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 10)
        p = PoseWithCovarianceStamped()
        p.header.frame_id = "map"
        p.pose.pose.position.x = 0.0
        p.pose.pose.position.y = 0.0
        p.pose.pose.orientation.w = 1.0
        
        # msg = PoseWithCovariance()
        # msg.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.000, 0.000, 0.000, 1.0))
        # msg.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        #                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        #                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        #                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        #                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        #                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # p.pose = msg
        pose_publisher.publish(p)

    def set_movement_goal():
        pass
        

        

if __name__ == '__main__':
    rospy.init_node('initial_pose')
    rospy.loginfo("Setting Pose")