#!/usr/bin/env python3

# Majority of the code from: http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html 

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# NOTE This code does not work - if it had it would be in a Server but connection issues made this difficult
# moveit_commander.roscpp_initialize(sys.argv)
# rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()

# group_name = "arm"  #has to be something specific 
# move_group = moveit_commander.MoveGroupCommander(group_name)

# # We can get the name of the reference frame for this robot:
# planning_frame = move_group.get_planning_frame()
# print ("============ Planning frame: %s" % planning_frame)

# # We can also print the name of the end-effector link for this group:
# eef_link = move_group.get_end_effector_link()
# print ("============ End effector link: %s" % eef_link)

# # We can get a list of all the groups in the robot:
# group_names = robot.get_group_names()
# print ("============ Available Planning Groups:", robot.get_group_names())

# # Sometimes for debugging it is useful to print the entire state of the
# # robot:
# print ("============ Printing robot state")
# print (robot.get_current_state())
# print ("")

# pose_goal = geometry_msgs.msg.Pose()
# pose_goal.orientation.w = 1.0
# pose_goal.position.x = 0.04
# pose_goal.position.y = 0.01
# pose_goal.position.z = 0.04

# #move_group.set_pose_target(pose_goal)

# plan = move_group.go([1,-1,1,-1],wait=True)
# # Calling `stop()` ensures that there is no residual movement
# move_group.stop()
# # It is always good to clear your targets after planning with poses.
# # Note: there is no equivalent function for clear_joint_value_targets()
# move_group.clear_pose_targets()

# move_group.execute(plan, wait=True)
class Pickup:
   def __init__(self):
       rospy.Service("pickup_complete", bool, self.movement)
    
   #  When this server is called, this method should not release until the remote is picked up
   def movement(self):
        
       return True

if __name__ == '__main__':
   rospy.init_node('arm_pickup', argv = sys.argv)
   picker_upper = Pickup()
   rospy.spin()