# Final Project of Group 1 

***aue_finals*** contains the final code for simulation and real world demosnstration of the final project.

1.  ## Environment

    The final project will be an obstacle course which will contain elements of work carried out during this semester.The Turtlebot Burger will have to autonomously navigate through the Gazebo and also through the track developed in a real environment at **CU-ICAR**.

2.  ## Task 1: **Wall following/Obstacle avoidance** 
    The Turtlebot starts here. It must successfully follow the wall and avoid the obstacles until it reaches the yellow line.
4.  ## Task 2: **Line following & Stop Sign Detection** 
    **Line following** - The Turtlebot must successfully follow the yellow line.
     **Stop Sign detection** - While navigating the yellow line, the the Turtlebot should stop at the stop sign for 3 seconds before continuing. The stop-sign will be detected by **TinyYOLO**
4.  ## Task 3: ***April Tag Tracking*** 
    For this task you will need to spawn another TB3 in the environment in the empty space past the yellow line and attach an AprilTag to the robot. The TB3 with the AprilTag  will be teleoperated by the user and the preceding TB3 needs to track its motion.


## Folders of ***aue_finals*** package:
  - ## **Launch**
    - The folder contains the launch files required to perform the simulation and real world testing of required tasks.
   - ## **models**
    - The folder contains the models for Apriltag, course and traffic material used in Gazebo simulation.
    -  These can be added to models folder of **turtlebot_simulation** package.
    -  or we can use the following command: 
        export GAZEBO_MODEL_PATH=~/path/to/workspace/src/aue_finals/models:~/path/to/workspace/src/turtlebot3_simulations/turtlebot3_gazebo/models
 
  - ## **scripts**
    - The folder contains the launch files required to perform the simulation and real world testing of required tasks.
    - It contains a script called **ObstacleAvoid.py** for obstacle avoidance and wall following tasks of project
    - This code is tested for simulation aspect of final project.
    - This code is tested and modified later for Real world testing aspect of final project
    - It contains a script called **line_follower2.py** for line following and stop sign detection tasks of project
    - This code is tested for simulation aspect of final project.
    - This code is tested and modified later for Real world testing aspect of final project
    - It contains a script called **apriltag_follow2.py** for line following and stop sign detection tasks of project
    - This code is tested for simulation aspect of final project.
    - This code is tested and modified later for Real world testing aspect of final project 
  - ## **urdf**
    - The folder contains the **urdf** file which describes turtlebot parameters, joints, links, sensors necessary for final project. 
    - It also has **gazebo.xacro** to describe the bot in gazebo
    - The folder contains  a secondary **urdf** file which describes turtlebot parameters, joints, links, sensors and apriltag necessary for final project. 
    - It also has **gazebo.xacro** to describe the bot in gazebo
  - ## **videos**
    - The folder contains the videos taken during simultion of specified tasks
  - ## **worlds**
    - Environment of gazebo world
  
  - ## **Deprecated**
  - This folder contains script/launch files that are no longer in use. These are only for the use of contributors' reference.
  - This folder must be deleted after cloning the repository

- ## **Rosbags*
- Rosbags collected during parameter tuning for line following.



There are 3 main launch files:
1) turtlebot3_auonomybackup.launch to run the scripts coded for Gazebo simulation
2) turtlebot3_real.launch that runs the scripts for the real world turtlebot demo

Codes were first developed for Gazebo and then tweaked for the real world scenario hence two different scripts for gazebo and real world.
