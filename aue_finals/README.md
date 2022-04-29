# Final Project of Group 1 

***aue_finals*** contains the final code for simulation and real world demosnstration of the final project.

1.  ## Environment

           - The final project will be an obstacle course which will contain elements of work carried out during this semester.The Turtlebot Burger will have to autonomously navigate through the Gazebo and also through the track developed in a real environment at **CU-ICAR**.

2.  ##Task 1: **Wall following/Obstacle avoidance** -  The Turtlebot starts here. It must successfully follow the wall and avoid the obstacles until it reaches the yellow line.
3.  ##Task 2: **Line following & Stop Sign Detection** - The Turtlebot must successfully follow the yellow line.
            **Stop Sign detection** - While navigating the yellow line, the the Turtlebot should stop at the stop sign for 3 seconds before continuing. The stop-sign will be detected by **TinyYOLO**
4.  ##Task 3: ***April Tag Tracking*** - For this task you will need to spawn another TB3 in the environment in the empty space past the yellow line and attach an AprilTag to the robot. The TB3 with the AprilTag  will be teleoperated by the user and the preceding TB3 needs to track its motion.


## Folders of ***aue_finals*** package:
  - ## Launch : 




There are 3 main launch files:
1) turtlebot3_auonomybackup.launch to run the scripts coded for Gazebo simulation
2) turtlebot3_real.launch that runs the scripts for the real world turtlebot demo

Codes were first developed for Gazebo and then tweaked for the real world scenario hence two different scripts for gazebo and real world.
