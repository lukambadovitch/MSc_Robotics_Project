# MSc_Robotics_Project

## Synopsis
  1. [Introduction](#introduction)
  2. [Project Description](#project-description)
  3. [Navigation](#navigation)
  4. [Visual Servoing](#visual-servoing)
  5. [Outcomes](#outcomes)

## Introduction
The goal of this project is to navigate the robot from an initial location to an approximate target location and fine-position the robot at a certain distance from the QR tag using Visual Servoing. The implementation is organized as follows.

- <b>Step 1:</b> Setup the Hardware and Configure - TurtleBot, LiDAR, Kinect, JoyStick.
- <b>Step 2:</b> Create a Map using RPLIDAR laser scan.
- <b>Step 3:</b> Navigate the Robot in the Environment from its current position to the Goal Position(near to QR-tag).
- <b>Step 4:</b> Begin Visual Servoing and determine the pose of the robot from Visual Information obtained from Kinect.
- <b>Step 5:</b> With the current pose information as feedback align the robot in the desired pose.

Thanks to ROS-Middleware which is an open-source platform for robotic researches and project development, we can achieve various tasks of our project implementation by exploring the functionalities of specific ROS-packages.

## Project Description
This project is developed under ROS-Kinetic environment on Ubuntu 16.04 LTS implemented in the TurtleBot-2. TurtleBot-2 has two Degrees-of-Freedom (DoF), translation along x-axis and rotation along z-axis. The TurtleBot we used in the implementation of our project is equiped with Kinect Sensor which is a RGB-D camera with view span of 60 degree and LiDAR with 360 degree view span.

Creating and developing necessary approaches through a combination and possible modifications of launch files, nodes, topics in order to achieve our tasks. The main ROS-packages necessary for this project are discussed below.

- <b> GMapping: </b> This package responsible for map generation. Thanks to the package turtlebot_vibot_nav provided by the vibot lab github platform, We were able to setup the navigation as defined in launch files.

- <b> AMCL (Adaptive Monte Carlo Localization): </b> AMCL is a ROS package that deals with Robot localization (particle filter localization). The idea is to store position and orientation data robot as reference data. Initially, Particles are all sampled randomly. When the robot moves, particles are resampled based on their current state as well as the robot’s action using Recursive Bayesian Estimation [1](#1-kaiyu-zheng-ros-navigation-tuning-guide-september-2-2016).

- <b> ViSP: </b> IBVS is introduced to solve image local minimal based on the principle that the kinematical error in Cartesian space approaches zero when the image feature error in 2-D image space is approaching zero.


### Required ROS Packages

- <b> Building ROS Environment for the Project </b>:

  + http://wiki.ros.org/kinetic/Installation/Ubuntu

  + http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

  + https://wiki.ros.org/ROS/EnvironmentVariables

- <b> Mobile Robot Configuration (TurtleBot-2)</b>:

  + http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation

- <b>TurtleBot management from the workstation </b>:

  + http://wiki.ros.org/ROS/Tutorials/MultipleMachines

- <b> Configuration of the Laser sensor with Turtlebot </b>:

  + clone the project from https://github.com/roboticslab-fr/turtlebot_vibot into ```/src``` directory of your catkin workspace in the turtlebot pc
      
```
    $ git clone https://github.com/roboticslab-fr/turtlebot_vibot
    $ cd </catkin_ws>
    $ catkin make
    $ rospack profile
```
- <b> Installing VISP package for visual servoing </b>:

  + http://wiki.ros.org/visp

- <b> setting Up project on Turtlebot </b>:

  + on the turtlebot pc:
```
    $ cd </catkin_ws>/src
    $ git clone https://github.com/Macaulay-Sadiq/MSc_Robotics_Project.git
    $ cd  </catkin_ws>
    $ catkin make
    $ rospack profile
```

## Mapping & Navigation
### Map Creation
In our application, scan data from a laser sensor (RPLIDAR) is used to estimate the location of landmarks in the environment through the process of dead reckoning.  With the required ROS packages, we can compute the scan data collected from the sensor to build a corresponding 2D map of the environment. An alternative to the RPLIDAR scan sensor, scandata from an RGB-D (Kinect) camera can as well be used to build map. The Kinect which seems ready to use with our robot ROS packages is less efficient for the map building. We have considered the use of the RPLIDAR over the Kinect camera for the map building since it gives scan data from its 360-degree angle of view from the robot’s current position, while the Kinect can only give about 57-60 degree angle of view scan data.  The angle of view of the laser sensor is of great importance to us to manage the problem of loop-closure and discontinuities during map building, therefore, a more accurate map can be obtained for a reliable and realistic environment measurements[2](#2-a-d-mkorkmaz-and-y-e-tusun-sensor-comparison-for-a-real-time-slam-ap-plication-in-international-journal-of-information-and-electronics-engineering-march-2018).

We use ```slam_gmapping``` to create a map of our environment. After that, we can load the created map into a map-server and use AMCL for navigation. Here is the command interminal:

- On the Turtlebot through ```ssh```:
```
    $ roslaunch turtlebot_bringup minimal.launch
    $ roslaunch turtlebot_vibot_nav gmapping_demo_rplidar.launch
```
```
    $ rosrun map_server map_saver -f /tmp/my_map
```
- On the WorkStation:
```
    $ roslaunch turtlebot_rviz_launchers view_navigation.launch
```
- Save the map to a file:
```
    $ roscd turtlebot_vibot_nav/maps/
```
```
    $ rosrun map_server map_saver -f my_map
```

By default,map_saver retrieves map data and writes it to map.pgm and map.yaml. Use the -f option to provide a different base name for the output files (-f ymap=>my_map.pgm+ my_map.yaml).

<b>Note: Gmapping should not be closed until the map is saved</b> 

The map built for this project with an RPLIDAR scan sensor is shown below:

<p align="center">
   <img src="/Images/Map.png" width="400" height="400" alt="Map Creation" />
</p>
<p align="center">
   Figure 1: Map
</p>

### Navigation
The next task after building a map is to make the robot to move on the created map. For this task the robot is given commands to move from its current position to a goal position inside the map without colliding any obstacles. Given a goal position, the algorithm will estimate the robot trajectory from its current position in the presence of static obstacles (requires global path planning) and dynamic obstacles (requires local path planning). The problem of path-planning focuses on the minimization of travel distance between the robot current position and its given goal position on the map in the presence of obstacles[3](#3--a-a-anis-koubaa-jsahar-trigui-and-i-chaari-introduction-to-mobile-robotpath-planning-robot-path-planning-and-cooperation-p-chapter-1-january2018). By exploring the functionalities of the Dynamic-Window Approach (DWA) algorithm [4](#4-w-b-d-fox-and-s-thrun-the-dynamic-window-approach-to-collision-avoid-ance-in-asian-conference-on-computer-vision-springer-2009) (included in base local planner ROS package) for the reactive obstacle avoidance for local path-planning together with the A* and the Dijkstra algorithms for global path planning (included in global planner ROS package) the results will then be compared with other methods of path-planning. In the end, we were able to conveniently navigate our robot on the built map from an initial position of the robot to a target position without collision of the robot on any form of obstacles. Thanks to the ```turtlebot_vibot_nav``` package, we performed the navigation and obstacle avoidance using the combination of Kinect and RPLidar. 

By executing the following line of code we are able to perform the navigation with the turtlebot (Run this command on the turtlebot through ```ssh```).

``` 
    $ roslaunch turtlebot_vibot_nav amcl_demo_rplidar.launch
```

## Visual Servoing
Visual Servoing (VS), also known as Vision-based Robot Control is a technique which helps in fine-positioning of a robot by using the Visual Features (obtained from a Vision Sensor) as Feedback Information to control the Movement of the Robot. Visual Servoing can be performed in two configurations based on the placement of Vision Sensors, Eye-in-Hand (the Camera is placed in the Robot and the observing target is static or dynamic placed in the environment) or Eye-to-Hand (the Camera is fixed in the world and observes the moving target attached to the Robot)[5](#5-peter-corke-robotics-vision-and-control---fundamental-algorithms-in-matlab-springer-tracts-in-advanced-robotics-vol73). In this project Eye-in-Hand configuration has been opted.  

Based on the type of Visual Features used as a feedback control, Visual Servoing can be classified as,

- <b> IBVS (Image Based Visual Servoing): </b> The idea is to move the robot from its current location to a desired location
by using only the Image Features of a Pattern in 3D space. The Image features can be Points, Lines or Planes or any other corresponding 2D shapes of the Object in 3D, QR Codes, etc.

- <b> PBVS (Position Based Visual Servoing): </b> The full 3D object pose is estimated using a model of the object, and the control error is then defined in terms of the current and desired 3D poses. This method makes it possible to attain straight line motion resulting in a minimum trajectory length in 3D. 

- <b> Hybrid Visual Servoing: </b> This method combines PBVS and IBVS to avoid the drawbacks of those techniques. It is based on the estimation of the partial camera movement from the current position to the desired position.

### Image Based Visual Servoing
IBVS with QR codes as Image Feature has been implemented in this project. As shown in Figure 2 the Visual Servoing Algorithms perform a closed loop process for minimizing the Error Function <b>e(t)</b>. The Error function, <b>e(t)</b> is the disparity between the Desired Features <b>S*</b> and Measured Features <b>S(t)</b> at time <b>t</b> [6](#6-francois-chaumette-k-ikeuchi-visual-servoing--a-reference-guide-ed-pp-869-874-springer-2014).

<p align="center">
  <img src="/Images/Error.png" width="150" height="30" alt="Error" /> 
</p>

In IBVS, the Features are a set of 2D parameters directly expressed in the image like coordinates of feature points, line segments, planes, etc. After computing the Error function, the Velocity for the Robot's Motion can be calculated by,

<p align="center">
   <img src="/Images/Control-Law.png" width="190" height="40" alt="Control Law" />
</p>

where,
 - <img src="/Images/Vel.png" width="20" height="20" alt="Velocity" /> is the Control applied to the Robot for Motion.
 - <img src="/Images/Lambda.png" width="20" height="20" alt="Lambda" /> is a positivive gain tuning the rate of convergence of the system.
 - <img src="/Images/Pseudo-Inverse.png" width="20" height="20" alt="Jacobaian Matrix" /> is the Moore-Penrose pseudo inverse of an approximation or an estimation of the features Jacobian (also called the Interaction matrix). 
 
The following Figure clearly explains the sequential procedure for IBVS.

<p align="center">
   <img src="/Images/IBVS.png" alt="IBVS" />
</p>
<p align="center">
   Figure 2: Block Diagram of Image Based Visual Servoing
</p>

### Implementation
The control law is computed in the ```turtlebot_follower``` node which can be found in the ```visual_servoing_prj``` directory. In this file we subscribe to the ```/object_position``` topic published by the ```visp_auto_tracker``` package to extract the coordinates of the detected points on the QR-tag and the output velocity is published to ```/cmd_vel_mux/input/navi``` on the turtlebot.

By executing the following line of code we are able to perform the visual servoing with the turtlebot (Run this command on the turtlebot through ```ssh```).

``` 
    $ roslaunch visual_servoing_prj visual_servo.launch
```
## Outcomes
Both Navigation and Fine Positioning can be performed sequentially by executing the following line of code (Run this command on the turtlebot through ```ssh```). 

```
    $ roslaunch visual_servoing_prj visual_servoing.launch
```

After executing the launch file press <img src="/Images/key_space_bar.png" width="80" height="20" alt="Space Bar" /> key to start the process. We have achieved the goal of the project and here are some Recorded Results of our implementation.

### Visual Servoing with A3 Size QR-tag
[![Servoing](http://img.youtube.com/vi/jL4apg_gBc8/0.jpg)](http://www.youtube.com/watch?v=jL4apg_gBc8 "Fine Positioning")


### Visual Servoing with A4 Size QR-tag
[![Servo](http://img.youtube.com/vi/4ZlNWGChNNU/0.jpg)](http://www.youtube.com/watch?v=4ZlNWGChNNU "Visual Servoing")

## References

#### 1. Kaiyu Zheng, "ROS Navigation Tuning Guide", September 2, 2016.

#### 2. A. D. M.Korkmaz and Y. E. Tusun, "Sensor comparison for a real-time slam ap-plication", in International Journal of Information and Electronics Engineering, March 2018.

#### 3.  A. A. Anis Koubaa, JSahar Trigui and I. Chaari, "Introduction to mobile robotpath planning", Robot Path Planning and Cooperation, p. Chapter 1, January2018.

#### 4. W. B. D. Fox and S. Thrun, "The dynamic window approach to collision avoid-ance.", in Asian conference on computer vision, Springer, 2009.

#### 5. Peter Corke, "Robotics, Vision and Control - FUNDAMENTAL ALGORITHMS IN MATLAB", Springer tracts in Advanced Robotics, Vol.73.

#### 6. Francois Chaumette, K. Ikeuchi, "Visual servoing : a Reference Guide", (ed.) pp. 869-874, Springer, 2014.
