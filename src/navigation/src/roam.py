#!/usr/bin/env python3

# This code is intended to roam between defined points in our map

import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Float64MultiArray


class Location:
    def __init__(self):
        # Get and set initial pose (this will run multiple times to ensure message has been acquired)
        self.pose_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 10)
        self.pose_subscriber = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial, queue_size=10)

        # Allows us to tell robot where to go
        self.goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal , queue_size = 10)

        # Tells us which location to send to next
        self.goal_subscriber = rospy.Subscriber('goal_subscribe', Float64MultiArray, self.set_movement_goal, queue_size= 10)

        # This is the initial position of the robot in the world
        p = PoseWithCovarianceStamped()
        p.header.frame_id = "map"
        p.pose.pose.position.x = -3.0
        p.pose.pose.position.y = 1.0
        p.pose.pose.orientation.w = 1.0

        self.current_pos = PoseWithCovarianceStamped()

        # Wait and actively publish the initial location until it has been recieved
        while True:
            if self.current_pos.pose.pose.position == p.pose.pose.position and self.current_pos.pose.pose.orientation == p.pose.pose.orientation:
                rospy.loginfo("Published initial location")
                break
            self.pose_publisher.publish(p)
            rospy.sleep(0.5)
        rospy.sleep(2)
    
    # Get initial location of robot - once updated will allow us to run other code
    def initial(self, current_pos):
        self.current_pos = current_pos

    # Create movement goal and publish
    def set_movement_goal(self, current_goal):
        goal_message = MoveBaseActionGoal()
        rospy.loginfo(current_goal)
        goal_message.goal.target_pose.header.frame_id = "map"
        goal_message.goal.target_pose.pose.position.x = current_goal.data[0]
        goal_message.goal.target_pose.pose.position.y = current_goal.data[1]
        goal_message.goal.target_pose.pose.orientation.z = current_goal.data[2]
        goal_message.goal.target_pose.pose.orientation.w = current_goal.data[3]

        self.goal_publisher.publish(goal_message)

if __name__ == '__main__':
    rospy.init_node('initial_pose', argv=sys.argv)
    loc = Location()
    rospy.spin()