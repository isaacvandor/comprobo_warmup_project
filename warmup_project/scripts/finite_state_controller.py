#!/usr/bin/env python
"""
finite_state_controller.py
Isaac Vandor
CompRobo 2018
Implements a finite state machine to switch between people following and driving in a square.
It will follow people until it doesn't see anyone and then drive in a square.
"""
from __future__ import print_function
import rospy
import math
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class state_controller(object):
    def __init__(self):

        # What's up ROS
        rospy.init_node("state_controller")
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vis_pub = rospy.Publisher('/person_pt', Marker, queue_size=2)
        self.vis2_pub = rospy.Publisher('/laser_pt', Marker, queue_size=2)
        rospy.Subscriber('/stable_scan', LaserScan, self.laserCallback)
        self.state = "forward"
        
        # Initialize some variables

        # person follower
        self.follow_distance = 1.0 # meters
        self.sense_distance = (0.2,1.0) # meters
        self.follow_range = 115 # degrees
        self.turn = 1.5
        self.move = 0.8
        self.found_object = False
        self.laser_data = None

        # drive square
        self.square_time = rospy.get_time()
        self.state = "forward"
        self.forward_time = 5
        self.turn_time = 7

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
        lidar_pts.scale = Vector3(0.2,0.2,0.2)
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
        person_pt.scale = Vector3(0.3,0.3,0.3)
        person_pt.type = 3 # cylinder is basically human shape, right?
        person_pt.color = ColorRGBA(0,1,0,1)
        self.vis2_pub.publish(person_pt)


    def run_pf(self, point):
        # Updates robot angular/linear velocity based on x,y pos of person

        spin = self.turn * point[1]                     # New angular velocity based on person position
        move = self.move * (point[0] - self.follow_distance) # New linear velocity based on person position - follow_distance
        return Twist(Vector3(move,0,0),Vector3(0,0,spin)) # Twist those vectors


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

    def run(self):
        # Takes no args, runs main loop of node

        while not rospy.is_shutdown():
            if self.laser_data != None: # Check for some laser data
                goal = self.get_COM(True)    # Find some friends
                self.get_person(goal) # Make spending time with friends a goal
                if goal[0] != 0 or goal[1] != 0:    # Achieve your goals (if you see your friends)
                    self.pub.publish(self.run_pf(goal)) # PUBLISH
                    print("we've got a live one")
                else:
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
            else:
                self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0.25)))
                print("I'm so lonely (so lonely)")

if __name__ == '__main__':
    fsm = state_controller()
    r=rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()
        fsm.run()