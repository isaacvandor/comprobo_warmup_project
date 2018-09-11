#!/usr/bin/env python
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import rospy
import math

class wall_follower():

    def __init__(self):
        rospy.init_node('wall_follower')
        rospy.Subscriber('/scan', LaserScan, self.laserCallback)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(2)

        self.lin_vector = Vector3(x=0.0, y=0.0, z=0.0)
        self.ang_vector = Vector3(x=0.0, y=0.0, z=0.0)

        self.ang_err = 0
        self.P_control = 40.0
        self.offset = 1.0

        self.sendMessage()



    def laserCallback(self, msg):
        'A function for laser sensor msgs'
        print("laserCallback")

        leftFrontQuadrant = msg.ranges[0:90]
        leftBackQuadrant  = msg.ranges[90:180]
        rightBackQuadrant   = msg.ranges[180:270]
        rightFrontQuadrant  = msg.ranges[270:360]

        diff = 0

        for rf, rb in zip(rightFrontQuadrant, reversed(rightBackQuadrant)):
            # print ("right side")
            if rf == 0.0 or rb == 0.0:
                continue
            diff += rb - rf

        for lf, lb in zip(leftFrontQuadrant, reversed(leftBackQuadrant)):
            # print("left side")
            if lf == 0.0 or lb == 0.0:
                continue
            diff += lf - lb

        self.ang_err = (math.tanh((diff-self.offset)/self.P_control))
        print(self.ang_err)

    def correctAngle(self):
        """ Uses P control to correct the angle of the neato """

        self.lin_vector = Vector3(x=0.2, y=0.0, z=0.0)
        self.ang_vector = Vector3(x=0.0, y=0.0, z=self.ang_err)


    def sendMessage(self):
        """ Publishes the Twist containing the linear and angular vector """
        print("sendMessage")
        self.pub.publish(Twist(linear=self.lin_vector, angular=self.ang_vector))


    def run(self):
        while not rospy.is_shutdown():
            self.correctAngle()
            self.sendMessage()
            self.rate.sleep()



if __name__=="__main__":
    wall_follower = wall_follower()
