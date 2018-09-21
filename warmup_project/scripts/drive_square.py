#!/usr/bin/env python
"""
drive_square.py
Isaac Vandor
CompRobo 2018
Uses a timing-based approach to drive the neatos in a square.
"""
from __future__ import print_function, division
import rospy
from geometry_msgs.msg import Twist

class drive_square():
    """
    Holds attributes that define the shape and size of the driven square. Also
    contains an 'act' method that gives the robot commands to drive in the
    desired square.
    """

    def __init__(self):
        #Init ROS things
        rospy.init_node('drive_square')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        #Square variables
        self.square_time = rospy.get_time()
        self.state = "forward"
        self.forward_time = 5
        self.turn_time = 7

    def runthesejewelsfast(self):
        """
        Publishes 'move_cmd' motion commands to drive in the desired square. 
        """

        move_cmd = Twist()
        
        #forward
        if (self.state == "forward"):
            if (rospy.get_time() - self.square_time >= self.forward_time):
                move_cmd.angular.z = .25
                self.state = "turn"
                self.square_time = rospy.get_time()
            else:
                move_cmd.linear.x = .2
                
        #turn
        elif (self.state == "turn"):
            if (rospy.get_time() - self.square_time >= self.turn_time):
                move_cmd.linear.x = .2
                self.state = "forward"
                self.square_time = rospy.get_time()
            else:
                move_cmd.angular.z = .25
        self.pub.publish(move_cmd)



if (__name__=="__main__"):
    """
    Main method, which initializes Square and runs it
    """
    
    square = drive_square()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()
        square.runthesejewelsfast()