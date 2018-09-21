# Computational Robotics Warmup Project
## Isaac Vandor
### Comprobo 18

The goal of this project was to program a Neato Robotics Vacuum Cleaner to execute a number of autonomous behaviors. This project was an excellent opportunity to work with ROS's Pub-Sub architecture, use sensor data, and develop robot debugging skills.

### Driving in a Square
Driving in a 1 meter x 1 meter square:
In order to successfully drive the robot in a square, I used a timing-based approach as I knew this would make it easy to switch between states (forward and turn) thus enabling more complex state-based control in the future. A possible improvement on this approach would be an odometry-based implementation where I use the encoders and robot's known position to drive it in a square as this approach is more reliable and accurate.

### Wall Follower
Driving parallel to a wall:
In order to successfully implement wall-following behavior, I pre-defined ranges for lidar data in the frontleft, backleft, frontright, and backright of the robot. With these ranges, I was able to calculate the difference between them in order to determine where the wall was relative to the robot (minus a user-defined offset from the wall). I then used P control to ensure the robot stayed parallel to the wall as it drove. While this behavior worked, it is not super robust as the laser data isn't incredibly accurate and the range definitions can sometimes cause the robot to not see the wall or fully understand where it is in relation to the wall. To compensate for this in the future, I might add in other behaviors that find the wall better so the robot can more accurately track it. I also would implement better control of the robot as P control worked okay, but definitely isn't the most robust error control mechanism.


### Person Follower
Following a person around as they move:
In order to successfully implement a person follower, I found the center of mass of the laser measurements in a pre-defined range in front of the robot. Using this method, I was able to determine an x,y position for the person and followed them by staying within user-defined threshold values and a user-defined following distance.

While this approach worked pretty well in the classroom, it struggled in the hallway due to the large number of items in the environment. Often, the robot became confused whether trash cans, signposts, or the stools were people and would begin following them. Changing some of the variables related to range around the front of the robot that detection is looking for, following distance, and others might help avoid this problem. I could also use my lidar data to identify a centroid made up of a person's legs and utilize that for following.

### Obstacle Avoidance
Drive around and avoid obstacles:
In order to successfully implement obstacle avoidance, I took a similar approach to the wall follower where I split lidar data into right and left sides to identify on which side of the robot obstacles in front of it were. I then adjusted the angular velocity of the robot based on which side the obstacles are on. While this approach worked reasonably well, it did require a lot of if/else statements that I probably could have avoided had I chosen a more robust method for obstacle avoidance. Utilizing potential fields or odometry data and a specified goal are both potential areas for future work. However, having the robot move forward and avoid obstacles as it encounters them by turning either left or right worked reasonably well and was by far the coolest demonstration as Oliners walked by and tried to kick the Neato.

### Finite State Controller
Change between two behaviors:
In order to successfully implement a finite state controller, I combined my person following behaviors and square driving behaviors in order to switch between following people when you see them and driving in a square when no people are found. While this worked alright, the hallway was an incredibly noisy environment for people following and that ended up being the dominant state. The robot really only enters the driving square state when there is nothing and no one (literally nothing and no one) around it. In the future, I can make my person following more robust so that it only follows actual humans and not inanimate objects. This should allow the finite state controller to enter the drive square behavior more often.


