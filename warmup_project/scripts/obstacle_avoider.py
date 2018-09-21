#!/usr/bin/env python
"""
obstacle_avoider.py
Isaac Vandor
CompRobo 2018

"""
from __future__ import print_function, division
import rospy
import math
import numpy as np 

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker

class obstacle_avoider():
    def __init__(self):
        #Init ROS things
        rospy.init_node('obstacle_avoider')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/stable_scan', LaserScan, \
                                    self.laserCallback, queue_size=10)
        # set some obstacle variables
        self.obstacles = []
        self.leftObstacle = 0
        self.rightObstacle = 0
        self.max_dist = 1.0 # meters
        self.last_turn = None
        self.last_time = rospy.get_time()
        self.state = "forward"
        self.sideways = False
        self.angle_offset = 0
        self.count = 0

    def laserCallback(self, msg):
        '''Get laser data and identify left or right obstacles'''
        #Check for obstacles in front of neato and put into self.obstacles
        for i in range(len(msg.ranges)):
            angle = i
            distance = msg.ranges[i]
            if (distance > 0.0 and distance < self.max_dist):
                upper_offset = self.angle_offset+20
                lower_offset = self.angle_offset-20
                if (lower_offset < 0):
                    lower_offset += 360
                if (lower_offset == 340):
                    if (angle > lower_offset or angle < upper_offset):
                        self.obstacles.append((angle, distance))
                else:
                    if (angle > lower_offset and angle < upper_offset):
                        self.obstacles.append((angle, distance))
        #Classify obstacles into either left of right
        for obstacle in self.obstacles:
            if (obstacle[0] < upper_offset):
                self.leftObstacle += 1
            elif (obstacle[0] > lower_offset):
                self.rightObstacle += 1

    def runRobot(self):
        ''' Publish messages based on position of obstacles surrounding neato'''
        
        twist_msg = Twist()
        
        # Move Forward
        if (self.state == "forward"):
            if (not self.sideways):
                if (len(self.obstacles) > 0):
                    self.count += 1
                    if (self.count > 2):
                        self.count = 0
                        if (self.leftObstacle >= self.rightObstacle):
                            twist_msg.angular.z = -.5
                            self.state = "right turn"
                            self.last_turn = "right"
                            self.last_time = rospy.get_time()
                            self.angle_offset += 90
                            if (self.angle_offset >= 360):
                                self.angle_offset -= 360
                        else:
                            twist_msg.angular.z = .5
                            self.state = "left turn"
                            self.last_turn = "left"
                            self.last_time = rospy.get_time()
                            self.angle_offset -= 90
                            if (self.angle_offset < 0):
                                self.angle_offset += 360
                else:
                    twist_msg.linear.x = .1
            else:
                if (len(self.obstacles) == 0):
                    if (self.last_turn == "left"):
                        twist_msg.angular.z = -.5
                        self.state = "right turn"
                        self.last_turn = "right"
                        self.last_time = rospy.get_time()
                        self.angle_offset += 90
                        if (self.angle_offset >= 360):
                            self.angle_offset -= 360
                    else:
                        twist_msg.angular.z = .5
                        self.state = "left turn"
                        self.last_turn = "left"
                        self.last_time = rospy.get_time()
                        self.angle_offset -= 90
                        if (self.angle_offset < 0):
                            self.angle_offset += 360
                else:
                    twist_msg.linear.x = .1
                    
        # Turn Left b/c obstacle is on right
        elif (self.state == "left turn"):
            if (rospy.get_time() - self.last_time >= math.pi):
                twist_msg.linear.x = .1
                self.state = "forward"
                self.sideways = not self.sideways
            else:
                twist_msg.angular.z = .5
                
        # Turn right b/c obstacle is on left
        elif (self.state == "right turn"):
            if (rospy.get_time() - self.last_time >= math.pi):
                twist_msg.linear.x = .1
                self.state = "forward"
                self.sideways = not self.sideways
            else:
                twist_msg.angular.z = -.5

        self.pub.publish(twist_msg)

if __name__=="__main__":
    while not rospy.is_shutdown():
        obstacle_avoider = obstacle_avoider()
        r = rospy.Rate(10)
        r.sleep()
        obstacle_avoider.runRobot()