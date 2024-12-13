#!/usr/bin/env python3

import rospy
import sys
import cv2
import numpy as np

from geometry_msgs.msg import *
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
import tf.transformations as tf 

class Camera:
    def __init__(self):
        # Get odomotery from robot
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.current_odom, queue_size = 10)

        # Get initial pose (want to wait for this to be set before running)
        self.pose_subscriber = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial, queue_size=10)
        self.current_pos = PoseWithCovarianceStamped()

        # Wait and actively publish the initial location until it has been recieved
        while True:
            rospy.loginfo(self.current_pos)
            if self.current_pos.pose.pose.position.x != 0.0:
                break
            rospy.sleep(0.5)

        # These publishers allow us to control the state of the robot, and where it plans to drive to
        self.publisher_remote_camera = rospy.Publisher('remote_camera', Bool, queue_size = 10)
        self.goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal , queue_size = 10)
        self.drive_to_suspect = rospy.Publisher('close_remote', Bool, queue_size=10)
        
        # Gives raw image data and LIDAR reading
        self.subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.process_image, queue_size = 10)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.approach_suspect, queue_size = 10)
        
        # Checks that ensure only active when wanted
        self.found_status = False
        self.close_status = False

        # Let's OpenCV convert to and from ROS
        self.bridge = CvBridge()

        # Threshold for finding remote
        self.lower_threshold = np.array([36,25, 25])
        self.upper_threshold = np.array([60,255,255])
        self.y_max = 100

        # Set initial odometry
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0
        self.odom_w = 0.0
        
        # Save last goal coordinates used
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_z = 0.0
        self.last_w = 0.0
    
    # Get initial location of robot - once updated will allow us to run other code
    def initial(self, current_pos):
        self.current_pos = current_pos

    # Process image and determine whether or not remote found
    def process_image(self, image):
        self.found_status = False
        self.close_status = False

        # Convert image to ROS compatible
        cv2_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        hsv = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)

        mask_in_range = cv2.inRange(hsv, self.lower_threshold, self.upper_threshold)
        mask_bw = cv2.erode(mask_in_range, kernel=None, iterations=2)
        
        contours, _ = cv2.findContours(mask_bw,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

        if len(contours) == 1:
            # This will let us find the center of the found contour
            M = cv2.moments(contours[0])
            try:
                row = int(M['m10']/M['m00'])
                col = int(M['m01']/M['m00'])
                width = cv2_image.shape[1]
                height = cv2_image.shape[0]

                # Only look at pixels below a certain height (remote will not be floating)
                if row > self.y_max:
                    y = height - row
                    x = col - width/2
                    self.theta = np.arctan2(x, y)
                
                    self.found_command()
            except:
                rospy.loginfo("YOU DUMB DUMB")
                pass
        
        # Code to see images/bounding boxes
        # cv2.drawContours(cv2_image,contours,-1,(0,255,0),1)
        # cv2.imshow("image", cv2_image)
        # cv2.waitKey(0)
    
    # Interupt sent to move from roaming to remote acquisition
    def found_command(self):
        rospy.loginfo("FOUND POSSIBLE DRUGS")
        self.found_status = True
        self.publisher_remote_camera.publish(self.found_status)
    
    # Update current movement stats
    def current_odom(self, odom):
        self.odom_x = odom.pose.pose.position.x
        self.odom_y = odom.pose.pose.position.y
        self.odom_z = odom.pose.pose.orientation.z
        self.odom_w = odom.pose.pose.orientation.w 

    # Allows us to approach remote based off of laser scan (this code does not fully work - ran out of time)
    def approach_suspect (self, laser):
        if self.found_status: 
            laser_values = np.arange(laser.angle_min, laser.angle_max, laser.angle_increment)
            for idx, value in enumerate(laser_values):
                if value > np.pi:
                    laser_values[idx] = (value - 2*np.pi)
            rospy.loginfo(f'All laser: {laser.ranges}, Angles: {laser_values}')
            laser_index=((laser_values-self.theta)).argmin()
            suspect_distance = laser.ranges[laser_index]
            x_dist=suspect_distance*np.cos(self.theta)
            y_dist=suspect_distance*np.sin(self.theta)
            if x_dist > 10.0 or y_dist > 10.0:
                return
            rospy.loginfo(f'Laser: {suspect_distance}, Theta: {self.theta}')
            rospy.loginfo(f'x_dist: {x_dist} y_dist: {y_dist}')

            goal_message = MoveBaseActionGoal()
            quaternion = tf.quaternion_from_euler(0.0, 0.0, self.theta)

            goal_message.goal.target_pose.header.frame_id = "map"
            goal_message.goal.target_pose.pose.orientation.z = self.odom_z + quaternion[2]
            goal_message.goal.target_pose.pose.orientation.w = self.odom_w + quaternion[3]
            goal_message.goal.target_pose.pose.position.x = self.odom_x + x_dist
            goal_message.goal.target_pose.pose.position.y = self.odom_y + y_dist

            rospy.loginfo(f'Goal Location: {goal_message}')

            self.drive_to_suspect.publish(True)
            
if __name__ == '__main__':
    rospy.init_node('camera', argv = sys.argv)
    camera = Camera()
    rospy.spin()