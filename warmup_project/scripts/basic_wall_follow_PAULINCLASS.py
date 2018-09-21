#!/usr/bin/env python
from __future__ import print_function, division
''' Example code from Paul on basic approaches to wall_following'''
'''@author: Paul Ruvolo, not @me'''
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

class wallApproach(object):
    def __init__(self):
        rospy.init_node('wall_approach')
        rospy.Subscriber('/scan',LaserScan,self.process_scan)
        rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.target_distance = rospy.get_param('~target_distance',1.0)
        self.k = rospy.get_param('~k value',.5)
        self.distance_to_wall = None

    def process_scan(self, msg):
        if msg.ranges[0] != 0.0:
            self.distance_to_wall =msg.ranges[0]

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.distance_to_wall is not None:
                err = self.distance_to_wall - self.target_distance
                vel_msg = Twist(linear=Vector3(x=self.k*err))
            r.sleep()

if __name__ == "__main__":
    node = wallApproach()
    node.run()