#!/usr/bin/env python
''' This is a script to drive the neato in an APPROXIMATELY 1mx1m square
	Isaac Vandor, Comprobo Fall 18
'''	
from __future__ import print_function, division
import rospy
from math import radians
from geometry_msgs.msg import Twist

class drive_square():
    def __init__(self):
        print('Starting Square Dance')
        rospy.init_node('drive_square')

        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist,queue_size=10)

        # Let's set a rate
        rate = rospy.Rate(10)

        move_cmd = Twist()
        move_cmd.linear.x = 0.25

        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = radians(30)
 
        count = 0
        while not rospy.is_shutdown():
        # go forward
            rospy.loginfo("Full Speed Ahead")
            for x in range(0,30):
                self.cmd_vel.publish(move_cmd)
                rate.sleep()
        # turn 90 degrees
            rospy.loginfo("Hard Turn")
            for x in range(0,30):
                self.cmd_vel.publish(turn_cmd)
                rate.sleep()            
            count = count + 1
            if(count == 4): 
                    count = 0
            if(count == 0): 
                    rospy.loginfo("Honey I'm home")
                    rospy.is_shutdown(True)

    # she's dead jim
    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        driver = drive_square()
    except:
        rospy.loginfo("Game Over")

