Part 1
SLAM on the real turtlebot
Here we used gmapping and karto slam algorithms
Comparison between two different lidars for building the map for SLAM in gazebo
Here we did simulation in gazebo with Original Lidar and Hokuyo 3D lidar
URDF file for burger and gazebo xacro was changed to accommodate Hokuyo 3D lidar
The videos folder contains `gazebomap2D for 2D slam` and `gazebonav2DLDS` for generating slam map and navigation with original Lidar
The videos folder contains `gazeboslam3DHokuyo for slam` and `gazebonav3DHokuyo` for generating slam map and navigation with Hokuyo Lidar
`Comparison`:
with 3D Lidar we need to select a ray for scan matching and faced issues with setting up hokuyo onto turtlebot in gazebo simulation. There was a considerable lag during slam map generation with 3D Lidar. 
Map did not update as expected and it is not similar to slam map generated with 2D lidar
In Navigation part: 2D Pose estimate did not scan match as expected but performed navigation anyway
