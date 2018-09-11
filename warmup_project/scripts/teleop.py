#!/usr/bin/env python
''' This is a script to teleop the neato robots
	Isaac Vandor, Comprobo Fall 18
'''	
from __future__ import print_function, division

import rospy
import tty
import select
import sys
import termios

from geometry_msgs.msg import Twist, Vector3

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        w    
   a         d
        s    
CTRL-C to quit
Spacebar to stop
"""
#Creating key bindings for moving and turning
moveBindings = {
		'w':(1),
		's':(-1)
	       }

turnBindings={
		'a':(1),
		'd':(-1),
	      }
stopKey = ''		  

#some fanciness from Paul for setting button press and screen focus
def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
		key = None #Assign key to something (it doesnt really matter what)
		settings = termios.tcgetattr(sys.stdin)
		pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1) #Initialize ROS publisher
		rospy.init_node('telop_twist') #Initialize ros node
		move = rospy.get_param("~moving",.5)
		turn = rospy.get_param("~turn",1.0)
		
		while key != '\x03': #some fanciness from Paul to use Ctrl-C
			key = getKey()
			if key in moveBindings.keys():
				move = moveBindings[key]
			elif key in turnBindings.keys():
				turn = turnBindings[key]
			else:
				move = 0
				turn = 0

		#Using twist and Vector3 from geometry_msgs to break down velocity into linear and angular parts and assign them to moving/turning
			twist = Twist(linear=Vector3(y=0,z=0), angular=Vector3(x=0,y=0))
			twist.linear.x = move 
			twist.angular.z = turn
			if key == stopKey:
				twist.linear.x = 0
				twist.angular.z = 0
			pub.publish(twist) #run these jewels fast


