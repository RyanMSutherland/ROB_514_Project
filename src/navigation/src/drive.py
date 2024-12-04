#!/usr/bin/env python3

import rospy
import sys

from geometry_msgs.msg import Twist

class Drive:
    def __init__(self):
        self.drive_command = "roam"

        # This is the only function that will publish to cmd_vel for safety
        self.publisher_drive = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

        # These functions will subscribe to the appropriate function to determine desired velocity
        # Each will return a Twist indicating the desired movement of the robot
        self.subscriber_roam = rospy.Subscriber('roam_vel', Twist, self.roam, queue_size = 10)
        self.subscriber_to_remote = rospy.Subscriber('to_remote_vel', Twist, self.to_remote, queue_size = 10)
        self.subscriber_orient_remote = rospy.Subscriber('orient_remote_vel', Twist, self.orient_remote, queue_size = 10)
        self.subscriber_to_user = rospy.Subscriber('to_user_vel', Twist, self.to_user, queue_size = 10)

        # These subscribers will subscribe to trigger systems to switch between states of the robot
        self.subscriber_remote_camera = rospy.Subscriber('remote_camera', bool, self.drive_command_to_remote, queue_size = 10)
        self.subscriber_orient_remote = rospy.Subscriber('orient_remote', bool, self.drive_command_orient_remote, queue_size = 10)
        self.subscriber_orient_user = rospy.Subscriber('orient_user', bool, self.drive_command_to_user, queue_size = 10)
        self.subscriber_command_roam = rospy.Subscriber('command_roam', bool, self.drive_command_roam, queue_size = 10)

        # These are the arm commands which must be completed by the server before releasing the system (signaled by bool)
        rospy.wait_for_service('arm_pickup')
        self.client_pickup_remote = rospy.ServiceProxy('arm_pickup', bool)
        rospy.wait_for_service('arm_dropoff')
        self.client_dropoff_remote = rospy.ServiceProxy('arm_dropoff', bool)

        # Initialize without moving
        self.t = Twist()
        self.t.linear.x = 0.0
        self.t.linear.y = 0.0
        self.t.linear.z = 0.0
        self.t.angular.x = 0.0
        self.t.angular.y = 0.0
        self.t.angular.z = 0.0

    # When roam velocity returned, ensure we are trying to roam and update speed accordingly
    def roam(self, desired_vel):
        if self.drive_command == "roam":
            self.t = desired_vel.data

            self.publisher_drive.publish(self.t)
            rospy.loginfo(f'Roaming with linear speed {self.t.linear.x} and angular speed {self.t.angular.z}')

    # When wanting to drive to remote, set velocity
    def to_remote(self, desired_vel):
        if self.drive_command == "to_remote":
            self.t = desired_vel.data

            self.publisher_drive.publish(self.t)
            rospy.loginfo(f'Driving to remote with linear speed {self.t.linear.x} and angular speed {self.t.angular.z}')

    # Once near remote, orient apropriately by applying this data
    def orient_remote(self, desired_vel):
        if self.drive_command == "orient_remote":
            self.t = desired_vel.data

            self.publisher_drive.publish(self.t)
            rospy.loginfo(f'Orienting around remote with linear speed {self.t.linear.x} and angular speed {self.t.angular.z}')

    # Once remote picked up, drive to user (couch)
    def to_user(self, desired_vel):
        if self.drive_command == "to_user":
            self.t = desired_vel.data

            self.publisher_drive.publish(self.t)
            rospy.loginfo(f'Driving to user with linear speed {self.t.linear.x} and angular speed {self.t.angular.z}')
    
    # Switch command from roaming to drive to remote
    def drive_command_to_remote(self, camera):
        if camera.data:
            self.drive_command = "to_remote"
            rospy.loginfo(f'Remote found')
    
    # Switch command from remote location to orient for easy pickup
    def drive_command_orient_remote(self, camera):
        if camera.data:
            self.drive_command = "orient_remote"
            rospy.loginfo("At remote, now orienting")
    
    # Switch command from remote location, wait for remote to pickup and drive to user (couch)
    def drive_command_to_user(self, camera):
        if camera.data:
            self.client_pickup_remote()
            self.drive_command = "to_user"
            rospy.loginfo("Picked up remote, driving to user")
    
    # Switch command from user location, wait for remote to be delivered and return to roaming
    def drive_command_roam(self, camera):
        if camera.data:
            self.client_dropoff_remote()
            self.drive_command = "roam"
            rospy.loginfo("Remote deliverd, now roaming")

if __name__ == '__main__':
    rospy.init_node('drive_command', argv = sys.argv)
    driver = Drive()
    rospy.spin()