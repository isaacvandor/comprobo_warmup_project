#!/usr/bin/env python
"""
person_follower.py
Isaac Vandor
CompRobo 2018
Implements a person following behavior for the neato which enables the neato to follow 
people in front of it or spin around looking for people to follow when it can't find anyone
"""
from __future__ import print_function, division
import rospy
import math
from geometry_msgs.msg import Twist, Vector3, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class person_follower():

    def __init__(self):
        # Set some variables
        self.follow_distance = 1.0 # meters
        self.sense_distance = (0.2,1.0) # meters
        self.follow_range = 115 # degrees
        self.found_object = False
        self.laser_data = None
        self.turn = 1.5
        self.move = 0.8

        # Setup ROS stuffs
        rospy.init_node('person_follower')
        self.laser_sub = rospy.Subscriber('/stable_scan', LaserScan, self.laserCallback)
        self.vis_pub = rospy.Publisher('/person_pt', Marker, queue_size=2)
        self.vis2_pub = rospy.Publisher('/laser_pt', Marker, queue_size=2)
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.update_rate = rospy.Rate(10)

    def laserCallback(self,msg):
        # Get laser data
        self.laser_data = msg.ranges


    def get_COM(self, visualize=False):
        '''Finds persons COM utilizing center of mass of all laser pts within predefined range in front of robot'''
        all_pts = (0,0)      # avg all lidar pts
        counter = 0        # increment used lidar pts

        # Visualize this dude
        lidar_pts = Marker()
        lidar_pts.header.frame_id = "base_link"
        lidar_pts.header.stamp = rospy.Time.now()
        lidar_pts.type = 8
        lidar_pts.scale = Vector3(x=0.2,y=0.2,z=0.2)
        lidar_pts.color = ColorRGBA(1,1,1,0)
        lidar_pts.points = []

        # Sums all lidar points within specified range around neato
        for laser_measurements in (i for j in (range(0, int(self.follow_range/2+1)), range(int(360 - self.follow_range/2), 360)) for i in j):
            # Check that point is within sensing distance
            if self.sense_distance[0] < self.laser_data[laser_measurements] < self.sense_distance[1]:
                new_pt = (self.laser_data[laser_measurements]*math.cos(math.radians(laser_measurements)), self.laser_data[laser_measurements]*math.sin(math.radians(laser_measurements)))
                all_pts = (all_pts[0] + new_pt[0], all_pts[1] + new_pt[1]) #add new points within range
                counter += 1 # increment the counter to make sure we are getting new lidar pts
                if visualize:
                    lidar_pts.points.append(Point(new_pt[0],new_pt[1],0))

        if visualize:
            self.vis_pub.publish(lidar_pts)

        # Return avg of lidar points
        if counter > 1:
            return(all_pts[0]/counter, all_pts[1]/counter,0)
        else:
            return(0,0,0)


    def get_person(self, point):
        # Lets laser mark this guy
        person_pt = Marker()
        person_pt.pose.position = Point(point[0],point[1],point[2])
        person_pt.header.frame_id = "base_link"
        person_pt.header.stamp = rospy.Time.now()
        person_pt.scale = Vector3(x=0.3,y=0.3,z=0.3)
        person_pt.type = 3 # cylinder is basically human shape, right?
        person_pt.color = ColorRGBA(0,1,0,1)
        self.vis2_pub.publish(person_pt)


    def run_pf(self, point):
        # Updates robot angular/linear velocity based on x,y pos of person

        turn = self.turn * point[1]                     # New angular velocity based on person position
        move = self.move * (point[0] - self.follow_distance) # New linear velocity based on person position - follow_distance
        return Twist(Vector3(x=move,y=0,z=0),Vector3(x=0,y=0,z=turn)) # Twist those vectors


    def runRobot(self):
        while not rospy.is_shutdown():
            if self.laser_data != None: # Check for some laser data
                goal = self.get_COM(True)    # Find some friends
                self.get_person(goal) # Make spending time with friends a goal
                if goal[0] != 0 or goal[1] != 0:    # Achieve your goals (if you see your friends)
                    self.move_pub.publish(self.run_pf(goal)) # PUBLISH
                    print("we've got a live one")
                else:
                    self.move_pub.publish(Twist(Vector3(x=0,y=0,z=0),Vector3(x=0,y=0,z=0.25))) #don't stop, but you know look around for some friends
                    print("I'm so lonely (so lonely)")
            else:
                self.move_pub.publish(Twist(Vector3(x=0,y=0,z=0),Vector3(x=0,y=0,z=0))) #really stop, there's no one there
                print("She's dead jim")

            self.update_rate.sleep()


if __name__ == '__main__':
    person_follower = person_follower()
    person_follower.runRobot()