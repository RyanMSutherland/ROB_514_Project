#!/usr/bin/env python3

import rospy
import sys

from geometry_msgs.msg import Twist

class Camera:
    def __init__(self):
        # Need to change twist to a point
        self.publisher_drive = rospy.Publisher('remote_location', Twist, queue_size = 10)
        self.publisher_remote_camera = rospy.Publisher('remote_camera', bool, self.found_command, queue_size = 10)
        
        self.found_status = False
        self.remote_location = Point()

    def found_command(self):
        self.found_status = False

        self.publisher_remote_camera.publish(self.found_status)
        self.publisher_drive.publish(self.remote_location)

if __name__ == '__main__':
    rospy.init_node('camera', argv = sys.argv)
    picker_upper = Camera()
    rospy.spin()