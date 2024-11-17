#!/usr/bin/env python3

import rospy
import sys

from geometry_msgs.msg import Twist

class Drive:
    def __init__(self):
        self.drive_command = "roam"

        self.publisher_drive = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

        self.subscriber_roam = rospy.Subscriber('roam_vel', Twist, self.roam, queue_size = 10)
        self.subscriber_to_remote = rospy.Subscriber('to_remote_vel', Twist, self.to_remote, queue_size = 10)
        self.subscriber_orient_remote = rospy.Subscriber('orient_remote_vel', Twist, self.orient_remote, queue_size = 10)
        self.subscriber_to_user = rospy.Subscriber('to_user_vel', Twist, self.to_user, queue_size = 10)

        self.subscriber_remote_camera = rospy.Subscriber('remote_camera', bool, self.drive_command_to_remote, queue_size = 10)
        self.subscriber_orient_camera = rospy.Subscriber('remote_camera', bool, self.drive_command_orient_remote, queue_size = 10)
        self.subscriber_orient_camera = rospy.Subscriber('remote_camera', bool, self.drive_command_to_user, queue_size = 10)
        self.subscriber_orient_camera = rospy.Subscriber('remote_camera', bool, self.drive_command_roam, queue_size = 10)

        self.t = Twist()
        self.t.linear.x = 0.0
        self.t.linear.y = 0.0
        self.t.linear.z = 0.0
        self.t.angular.x = 0.0
        self.t.angular.y = 0.0
        self.t.angular.z = 0.0

    def roam(self, desired_vel):
        if self.drive_command == "roam":
            self.t = desired_vel.data

            self.publisher_drive.publish(self.t)
            rospy.loginfo(f'Roaming with linear speed {self.t.linear.x} and angular speed {self.t.angular.z}')

    def to_remote(self, desired_vel):
        if self.drive_command == "to_remote":
            self.t = desired_vel.data

            self.publisher_drive.publish(self.t)
            rospy.loginfo(f'Driving to remote with linear speed {self.t.linear.x} and angular speed {self.t.angular.z}')

    def orient_remote(self, desired_vel):
        if self.drive_command == "orient_remote":
            self.t = desired_vel.data

            self.publisher_drive.publish(self.t)
            rospy.loginfo(f'Orienting around remote with linear speed {self.t.linear.x} and angular speed {self.t.angular.z}')

    def to_user(self, desired_vel):
        if self.drive_command == "to_user":
            self.t = desired_vel.data

            self.publisher_drive.publish(self.t)
            rospy.loginfo(f'Driving to user with linear speed {self.t.linear.x} and angular speed {self.t.angular.z}')
    
    def drive_command_to_remote(self, camera):
        if camera.data:
            self.drive_command = "to_remote"
            rospy.loginfo(f'Remote found')

    def drive_command_orient_remote(self, camera):
        if camera.data:
            self.drive_command = "orient_remote"
            rospy.loginfo("At remote, now orienting")
    
    def drive_command_to_user(self, camera):
        if camera.data:
            self.drive_command = "to_user"
            rospy.loginfo("Picked up remote, driving to user")
    
    def drive_command_roam(self, camera):
        if camera.data:
            self.drive_command = "roam"
            rospy.loginfo("Remote deliverd, now roaming")

if __name__ == '__main__':
    rospy.init_node('drive_command', argv = sys.argv)
    driver = Drive()
    rospy.spin()