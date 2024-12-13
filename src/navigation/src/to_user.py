#!/usr/bin/env python3

# This code is intended to roam between defined points in our map

import rospy
from geometry_msgs.msg import sys
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Float64MultiArray

class User:
    def __init__(self):
        # Allows us to tell robot where to go
        self.goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal , queue_size = 10)
        # Tells us which location to send to next
        self.goal_subscriber = rospy.Subscriber('goal_subscribe', Float64MultiArray, self.set_movement_goal, queue_size= 10)
    
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
    rospy.init_node('user_location', argv=sys.argv)
    user = User()
    rospy.spin()