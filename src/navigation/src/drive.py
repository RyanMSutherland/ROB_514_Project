#!/usr/bin/env python3

import rospy
import sys
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray, Bool
from nav_msgs.msg import Odometry

class Drive:
    def __init__(self):
        #Initialize Roam command - this controls which "state" the system is in
        self.drive_command = "roam"

        # Get current position and initial pose to confirm correct initilization
        self.pose_subscriber = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial, queue_size=10)
        self.current_pos = PoseWithCovarianceStamped()

        # Wait until position established before continuing
        while True:
            if self.current_pos.pose.pose.position.x != 0.0:
                break
            rospy.sleep(0.5)

        # This publisher will publish the desired goal location to another node
        self.publisher_goal = rospy.Publisher("goal_subscribe", Float64MultiArray, queue_size = 10)
    
        # Allows us to get the odomotery of the system and check current state accordingly
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.deligate, queue_size = 10)

        # These subscribers will change the state of the system when called
        self.close_remote_sub = rospy.Subscriber("close_remote", Bool, self.drive_command_orient_remote, queue_size = 10)
        self.subscriber_remote_camera = rospy.Subscriber('remote_camera', Bool, self.drive_command_to_remote, queue_size = 10)

        # # These are the arm commands which must be completed by the server before releasing the system (signaled by bool)
        # rospy.wait_for_service('arm_pickup')
        # self.client_pickup_remote = rospy.ServiceProxy('arm_pickup', bool)
        # rospy.wait_for_service('arm_dropoff')
        # self.client_dropoff_remote = rospy.ServiceProxy('arm_dropoff', bool)

        # Pre-defined locations for roaming (x, y, z, w) - z, w are orientation quaternions
        self.roam_locations = [[2.727, 0.593, -0.0333, 0.999], 
                               [6.539, 0.568, .699, 0.714], 
                               [5.456, 4.096, .999, 0.005], 
                               [1.130, 4.138, -0.757, 0.653]]
        
        # End location of user (x, y, z, w) as above
        self.user_location = [-4.763, 4.099, -0.941, 0.336]

        # Tolerance for stopping threshold
        self.tol = 0.001
        self.wait_time = 1.5
        
        # Remember current roaming location (allows us to start at location 0)
        self.current_location = len(self.roam_locations) - 1
        
        # Don't want to update robot every cycle, wait for things to happen with this
        self.last_action = rospy.Time.now().to_sec()
        rospy.sleep(3)
    
    # Get initial location of robot - once updated will allow us to run other code
    def initial(self, current_pos):
        self.current_pos = current_pos

    # Tells system which command is active and assigns the appropriate action
    def deligate(self, odom):
        rospy.loginfo(self.drive_command)
        if self.drive_command == "roam":
            self.roam(odom)
        elif self.drive_command == "to_remote":
            self.to_remote(odom)
        elif self.drive_command == "orient_remote":
            self.orient_remote(odom)
        elif self.drive_command == "to_user":
            self.to_user(odom)
    
    # When roaming, wait until fully stopped before sending next desired location
    def roam(self, odom):
        if np.isclose(odom.twist.twist.linear.x, 0, atol = self.tol) and np.isclose(odom.twist.twist.linear.y, 0, atol = self.tol) and np.isclose(odom.twist.twist.angular.z, 0.0, atol= self.tol):
            if rospy.Time.now().to_sec() - self.last_action > self.wait_time:
                rospy.loginfo("Update location")
                self.current_location += 1
                self.current_location %= len(self.roam_locations)
                msg = Float64MultiArray()
                msg.data = self.roam_locations[self.current_location]
                self.publisher_goal.publish(msg)
                self.last_action = rospy.Time.now().to_sec()

    # When wanting to drive to remote, do nothing (this changed in our final implementation)
    def to_remote(self, odom):
        rospy.loginfo("FOUND REMOTE - DRIVING TO SUSPECT")

    # Once near remote, orient apropriately by applying this data
    def orient_remote(self, odom):
        if np.isclose(odom.twist.twist.linear.x, 0, atol = self.tol) and np.isclose(odom.twist.twist.linear.y, 0, atol = self.tol) and np.isclose(odom.twist.twist.angular.z, 0.0, atol=self.tol):
            if rospy.Time.now().to_sec() - self.last_action > self.wait_time:
                rospy.loginfo("SUSPECT ACQUIRED - RETURNING TO BASE COMMANDER")
                self.drive_command = "to_user"
                self.last_action = rospy.Time.now().to_sec()

    # Once remote picked up, drive to user (couch)
    def to_user(self, odom):
        if np.isclose(odom.twist.twist.linear.x, 0, atol = self.tol) and np.isclose(odom.twist.twist.linear.y, 0, atol = self.tol) and np.isclose(odom.twist.twist.angular.z, 0.0, atol= self.tol):
            if rospy.Time.now().to_sec() - self.last_action > self.wait_time:
                rospy.loginfo("Update location to user")
                msg = Float64MultiArray()
                msg.data = self.user_location
                self.publisher_goal.publish(msg)
                self.last_action = rospy.Time.now().to_sec()
    
    # Switch command from roaming to drive to remote
    def drive_command_to_remote(self, status):
        if self.drive_command == "roam":
            self.drive_command = "to_remote"
            rospy.loginfo(f'REMOTE FOUND')
    
    # Switch command from remote location to orient for easy pickup
    def drive_command_orient_remote(self, status):
        if self.drive_command == "to_remote":
            self.drive_command = "orient_remote"
            self.last_action = rospy.Time.now().to_sec()
    
    # Switch command from user location, wait for remote to be delivered and return to roaming
    def drive_command_roam(self, odom):
        if self.drive_command == "to_user" and np.isclose(odom.twist.twist.linear.x, 0, atol = self.tol) and np.isclose(odom.twist.twist.linear.y, 0, atol = self.tol) and np.isclose(odom.twist.twist.angular.z, 0.0, atol=self.tol):
            if rospy.Time.now().to_sec() - self.last_action > self.wait_time:
                rospy.loginfo("RETURNED TO COMMANDER - BACK TO ROOM")
                self.drive_command = "roam"
                self.last_action = rospy.Time.now().to_sec()

if __name__ == '__main__':
    rospy.init_node('drive_command', argv = sys.argv)
    driver = Drive()
    rospy.spin()