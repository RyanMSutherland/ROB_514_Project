#!/usr/bin/env python3

import rospy
import sys

from geometry_msgs.msg import Twist

class Dropoff:
    def __init__(self):
        rospy.Service("dropoff_complete", bool, self.movement)

    # This method should not release until the remote has been successfully dropped off - this can be pre-programmed
    def movement(self):
        return True

if __name__ == '__main__':
    rospy.init_node('arm_dropoff', argv = sys.argv)
    picker_upper = Dropoff()
    rospy.spin()