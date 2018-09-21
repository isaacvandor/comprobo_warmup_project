#!/usr/bin/env python
'''
wall_follower.py
Isaac Vandor
CompRobo 2018
Implements a wall following behavior using the difference between laser data
detected in different quadrants and a basic (really basic) P control setup.

Also see basic_wall_follow_PAULINCLASS for generalized approach that I took
'''
import future_builtins
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import rospy
import math

class wall_follower():

    def __init__(self):
        # initialize ros variables
        rospy.init_node('wall_follower')
        rospy.Subscriber('/stable_scan', LaserScan, self.laserCallback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.rate = rospy.Rate(2)

        #set wall following variables
        self.wall_err = 0
        self.P = 50.0
        self.wall_offset = 2.5
        self.linearVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)



    def laserCallback(self, msg):
        # Set ranges for front, back, left, and right
        front_left = msg.ranges[0:90]
        back_left  = msg.ranges[90:180]
        back_right   = msg.ranges[180:270]
        front_right  = msg.ranges[270:360]

        laser_diff = 0 #set difference between quadrants = 0

        # Calculate difference between left front and back to determine where wall is on left side
        for front_left, back_left in zip(front_left, reversed(back_left)):
            if front_left == 0.0 or back_left == 0.0:
                continue
            laser_diff += front_left - back_left

        # Calculate diff between right front and back to determine where wall is on right side
        for front_right, back_right in zip(front_right, reversed(back_right)):
            if front_right == 0.0 or back_right == 0.0:
                continue
            laser_diff += back_right - front_right

        #determine distance to wall by subtracting laser differences from wall offset & divide by our P variable
        self.wall_err = (math.tanh((laser_diff-self.wall_offset)/self.P))

    def run_wf(self):
        '''Use P control to follow wall with neato'''
        self.linearVector = Vector3(x=0.2, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=self.wall_err)
        self.pub.publish(Twist(linear=self.linearVector, angular=self.angularVector))


    def runRobot(self):
        '''run these jewels, run these jewels fast'''
        while not rospy.is_shutdown():
            self.run_wf()
            self.rate.sleep()

if __name__ == '__main__':
    wall_follower = wall_follower()
    wall_follower.runRobot()