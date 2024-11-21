#!/usr/bin/env python3

import rospy
import sys

from geometry_msgs.msg import Twist

class Pickup:
    def __init__(self):
        rospy.Service("pickup_complete", bool, self.movement)
    
    # When this server is called, this method should not release until the remote is picked up
    def movement(self):
        
        return True

if __name__ == '__main__':
    rospy.init_node('arm_pickup', argv = sys.argv)
    picker_upper = Pickup()
    rospy.spin()