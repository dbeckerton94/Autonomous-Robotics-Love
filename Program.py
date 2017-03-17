# -*- coding: utf-8 -*-
"""
Created on Fri Feb 26 15:08:24 2016

@author: BEC123753722
"""

import rospy
import cv2
import math
import numpy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


# Variables
pub = rospy.Publisher("/wheel_vel_left", Float32, queue_size=1)
p = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
visible = 0
current_Range = 0
wheel_radius = .35
robot_radius = .35      

class RoboticsAssignment:
    def __init__(self):
        
        #Create Windows    
        cv2.namedWindow("Image window", 1)
        cv2.namedWindow("thresh", 1)
        cv2.namedWindow("detected circles", 1)
        cv2.startWindowThread()
        
        #Robot Components
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.image_process)
        self.range = rospy.Subscriber("/scan", LaserScan,self.scanit)
        self.current_range = None

 #----------------------Image Proccessing Functionality---------------------------------------------------------- 
             
    def image_process(self, image_d):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_d, "bgr8")
        except CvBridgeError, e:
            print e
        cimg = cv_image
        img = cv2.cvtColor(cimg,cv2.COLOR_BGR2GRAY)
        img = cv2.medianBlur(img,5)
        
        bgr_thresh = cv2.inRange(cv_image,
        numpy.array((0, 0, 0)),
        numpy.array((0, 255, 0)))

        # Find Colour Green using Contour
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img, numpy.array((40, 100, 50)), numpy.array((90, 255, 255))) # HSV Green
        hsv_thresh = cv2.medianBlur(hsv_thresh,5) # Filter Noise       
        bgr_contours, hierachy = cv2.findContours(bgr_thresh.copy(),
        cv2.RETR_TREE,
        cv2.CHAIN_APPROX_SIMPLE)
        hsv_contours, hierachy = cv2.findContours(hsv_thresh.copy(),
        cv2.RETR_TREE,
        cv2.CHAIN_APPROX_SIMPLE)
        visible = 0 # Colour visibility Initialise      
        
        
        # if Green found visibile value is 1
        for c in hsv_contours:
            a = cv2.contourArea(c)
            visible = 1
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0))
                
        # Show Windows 
        cv2.imshow("thresh", hsv_thresh)
        cv2.imshow("Image window", cv_image)
        cv2.imshow('detected circles',cimg)
        self.move(visible)
 #---------------------------Robots Movement Functionality----------------------------------------------------     
            
    # computing the forward kinematics for a differential drive
    def scanit(self, image_d):          
            self.current_range = numpy.nanmin(image_d.ranges)
            print self.current_range
              
    def forward_kinematics(self, w_r, w_l):
            c_l = wheel_radius * w_l
            c_r = wheel_radius * w_r
            v = (c_l + c_r) / 2
            a = (c_l - c_r) / robot_radius
            return (v, a)
        
    def move(self, visible):
        twist_msg = Twist() # Creating a new message to send to the robot
        twist_msg.linear.x = 0.2
        
        spin_msg = Twist() # Creating a new message to send to the robot
        spin_msg.angular.z = 4
        
        spin_msg1 = Twist() # Creating a new message to send to the robot
        spin_msg1.angular.z = 1.0
        
        found_rob = Twist() # Creating a new message to send to the robot
        found_rob.angular.z = (math.pi/0.3)
       
     #Distance and Green visibility
        if self.current_range > 0.7 and visible == 0: # Search for Green
            p.publish(twist_msg)
            p.publish(spin_msg) 
            print "finding bot"
        elif math.isnan(self.current_range): # Filter Nan values
            p.publish(spin_msg)
            print "nan value avoid"
        elif visible == 1 and self.current_range > 0.7: # Heading towards Green
            p.publish(twist_msg)
            print "found bot heading towards"
        elif visible == 0 and self.current_range < 0.7: # Object Avoidence
            p.publish(spin_msg)
        elif visible == 1 and self.current_range < 0.7: # Found Green
            print "I Love You Never Leave me Again <3"
          #  p.publish(found_rob)      
          #  d = rospy.Duration(10, 0) 
          #  rospy.sleep(d)
        
    wheel_left = rospy.Subscriber("/wheel_vel_left", Float32, move)
        
        
    # computing the inverse kinematics for a differential drive
    def inverse_kinematics(self, v, a):
        c_l = v + (robot_radius * a) / 2
        c_r = v - (robot_radius * a) / 2
        w_l = c_l / wheel_radius
        w_r = c_r / wheel_radius
        return (w_l, w_r)
    
    
    # inverse kinematics from a Twist message (This is what a ROS robot has to do)
    def inverse_kinematics_from_twist(self,t):
        return self.inverse_kinematics(t.linear.x, t.angular.z)
        
rospy.init_node("move_robot")
rospy.loginfo("Starting node")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
RoboticsAssignment()
rospy.spin()


 #---------------------------END----------------------------------------------------     










