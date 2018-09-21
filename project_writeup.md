# Computational Robotics Warmup Project
## Isaac Vandor
### Comprobo 18

The goal of this project was to program a Neato Robotics Vacuum Cleaner to execute a number of autonomous behaviors. This project was an excellent opportunity to work with ROS's Pub-Sub architecture, use sensor data, and develop robot debugging skills.

### Driving in a Square
Driving in a 1 meter x 1 meter square:
In order to successfully drive the robot in a square, I used a timing-based approach as I knew this would make it easy to switch between states (forward and turn) thus enabling more complex state-based control in the future. A possible improvement on this approach would be an odometry-based implementation where I use the encoders and robot's known position to drive it in a square as this approach is more reliable and accurate.

My code itself is structured in an object-oriented programming (OOP) approach with a main drive_square class with an init method for variable instantiation and a runthesejewelsfast method for determining current state of the robot and time in order to effectively complete the square. This is all run on a while loop that runs the runthesejewelsfast method repeatedly so long as ros is running.

For a video of the robot driving in a square, see ![robot driving in a square](https://github.com/isaacvandor/comprobo_warmup_project/blob/master/warmup_project/media/drive_square_video.mp4)

### Wall Follower
Driving parallel to a wall:
In order to successfully implement wall-following behavior, I pre-defined ranges for lidar data in the frontleft, backleft, frontright, and backright of the robot (see image below courtesy of Paul Ruvolo). With these ranges, I was able to calculate the difference between them in order to determine where the wall was relative to the robot (minus a user-defined offset from the wall). I then used P control to ensure the robot stayed roughly parallel to the wall as it drove (see reference diagram below courtesy of Tufts NXT Robotics Team). While this behavior worked, it is not super robust as the laser data isn't incredibly accurate and the range definitions can sometimes cause the robot to not see the wall or fully understand where it is in relation to the wall. To compensate for this in the future, I might add in other behaviors that find the wall better so the robot can more accurately track it. I also would implement better control of the robot as P control worked okay, but definitely isn't the most robust error control mechanism.

My code is structured using an OOP approach with a wall_follower class that contains an init method for variable instantiation, a laserCallback method for processing laser scans and sorting them into quadrants and a run_wf method for using P control to adjust the linear and angular velocity of the robot. There is also a runRobot method that runs the wall_follower in a loop so long as ROS is running.

![four quadrants picture](https://raw.githubusercontent.com/isaacvandor/comprobo_warmup_project/master/warmup_project/media/four_quadrants_laser.png)

![Wall following diagram](https://raw.githubusercontent.com/isaacvandor/comprobo_warmup_project/master/warmup_project/media/wall_following_diagram.png)

For a video of the robot following a wall, see:
![Wall following video](https://github.com/isaacvandor/comprobo_warmup_project/blob/master/warmup_project/media/wall_follower_video.mp4)

### Person Follower
Following a person around as they move:
In order to successfully implement a person follower, I found the center of mass of the laser measurements in a pre-defined range in front of the robot. Using this method, I was able to determine an x,y position for the person and followed them by staying within user-defined threshold values and a user-defined following distance. The diagram below illustrates this approach (Diagram credit: Paul Ruvolo).

![person following diagram](https://raw.githubusercontent.com/isaacvandor/comprobo_warmup_project/master/warmup_project/media/person_following.png)

My code is structured using an OOP approach with a main person_follower class that contains an init method for setting up all the variables, a laserCallback method for processing laser data, a get_COM method for determining the center of mass of all of the laser points within the pre-specified range in front of the robot, and a get_person method for visualizing the human being (or more often trash can). There is also a run_pf method for adjusting linear and angular velocity based on the person's position and a runRobot method for publishing all that good stuff and running everything in a while loop.

While this approach worked pretty well in the classroom, it struggled in the hallway due to the large number of items in the environment. Often, the robot became confused whether trash cans, signposts, or the stools were people and would begin following them. Changing some of the variables related to range around the front of the robot that detection is looking for, following distance, and others might help avoid this problem. I could also use my lidar data to identify a centroid made up of a person's legs and utilize that for following.

### Obstacle Avoidance
Drive around and avoid obstacles:
In order to successfully implement obstacle avoidance, I took a similar approach to the wall follower where I split lidar data into right and left sides to identify on which side of the robot obstacles in front of it were. I then adjusted the angular velocity of the robot based on which side the obstacles are on. 

My code is structured via an OOP approach with an obstacle_avoider class that contains an init method for variable setup, a laserCallback method for identifying obstacles and sorting them into left or right of the neato, and a massive runRobot method for setting the robot to go forward and adjusting direction to the left or right to avoid an obstacle and then forward again once it has passed an obstacle. 

While this approach worked reasonably well, it did require a lot of if/else statements that I probably could have avoided had I chosen a more robust method for obstacle avoidance. Utilizing potential fields or odometry data and a specified goal are both potential areas for future work. However, having the robot move forward and avoid obstacles as it encounters them by turning either left or right worked reasonably well and was by far the coolest demonstration as Oliners walked by and tried to kick the Neato.

### Finite State Controller
Change between two behaviors:
In order to successfully implement a finite state controller, I combined my person following behaviors and square driving behaviors in order to switch between following people when you see them and driving in a square when no people are found. 

My code is structured using an OOP approach with a state_controller class that has an init method for setting up all of my variables, a laserCallback method for handling laser data, a get_COM method for the center of mass of all the laser points in front of the robot, a get_person method for visualizing the trash fire identified as the COM, a run_pf method for updating angular and linear velocity, a runthesejewelsfast method for driving in a square, and a run method for switching between the two based on the presence of laser data.

While this worked alright, the hallway was an incredibly noisy environment for people following and that ended up being the dominant state. The robot really only enters the driving square state when there is nothing and no one (literally nothing and no one) around it. In the future, I can make my person following more robust so that it only follows actual humans and not inanimate objects. This should allow the finite state controller to enter the drive square behavior more often.


