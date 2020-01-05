# MSc_Robotics_Project

## Synopsis
  1. [Introduction](#introduction)
  2. [Project Description](#project-description)
  3. [Navigation](#navigation)
  4. [Visual Servoing](#visual-servoing)
  5. [Outcomes](#outcomes)

## Introduction
In this project, we implemented the Localization, Mapping & Navigation using Visual Servoing in the ROS platform under the Kinetic environment. The goal of this project will be to assist the Robot, which is the Turtlebot 2 in this case, to move autonomously in a given environment. Researchers have proposed various techniques; one among them is the Visual Servoing.

The Visual Servoing will consist of fine-positioning the robot based on the information acquired from the Vision sensor, the RGB-D Kinect for our case. The step-by-step process are explained in brief in the next lines.

In this project , we will organize the implementation as follows:
1.  Setup the Hardware and Configure - Turtlebot, RPLIDAR, Kinect, JoyStick;
2.  Create a Map using RPLIDAR laser scan;3.  Navigate the Robot in the Environment from its current position to the Goal Position;
4.  Perform the Navigation close to the target QR Code;
5.  Begin Visual Servoing and and determine the pose of the robot;
6.  With the current pose information as feedback align the robot in the desired pose.

Thanks to ROS-Middleware which is an open-source platform for robotic researches andproject development, we can achieve the various tasks involved in our project developmentby exploring the functionalities of the required ROS-packages for each process and as well.


Creating and developing necessary approaches through a combination and possible modifi-cation of launch files, nodes, topics in order to achieve our tasks. The main ROS-packages necessary for this project are discussed in the below:

1.GMapping: This package responsible for map generation.  Thanks to the package turtlebot_vibot_nav provided by the vibot lab github plateform :https://github.com/roboticslab-fr/turtlebot_vibot/tree/master/turtlebot_vibot_nav. we were able to set the navigation defined in launch files.

2.AMCL (Adaptive Monte Carlo Localization):  amcl is a ROS package that deals withrobot localization. It is also known as particle filter localization. The principle is thatposition and orientation data representing the robot’s pose are stored as a sample.At initial, Particles are all sampled randomly.  When the robot moves, particles areresampled based on their current state as well as the robot’s action using recursiveBayesian estimation [ ROS Navigation Tuning Guide, 2016].

3.ViSP:IBVS introduced to solve image local minimal based on the principle that whenthe image feature error in 2-D image space is approaching zero, the kinematical errorin Cartesian space approaches zero too.

## Project Description
The goal of this project is to navigate the robot from an initial location to an approximate target location and fine-position the robot at a certain distance from the QR tag using Visual Servoing.

This project is developed under ROS-Kinetic environment on Ubuntu 16.04 LTS implemented in the TurtleBot-2. TurtleBot-2 has two Degrees-0f-Freedom (DoF), translation along x-axis and rotation along z-axis. The TurtleBot we used in the implementation of our project is equiped with Kinect Sensor which is a RGB-D camera with view span of 60 degree and LiDAR with 360 degree view span.

### Required ROS Packages
- <b> Building ROS Environment for the Project </b>:

http://wiki.ros.org/kinetic/Installation/Ubuntu

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

https://wiki.ros.org/ROS/EnvironmentVariables

- <b> Mobile Robot Configuration (TurtleBot-2)</b>:

http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation

- <b>TurtleBot management from the workstation </b>:

http://wiki.ros.org/ROS/Tutorials/MultipleMachines

- <b> Configuration of the Laser sensor with Turtlebot </b>:

clone the project from https://github.com/roboticslab-fr/turtlebot_vibot into ```/src``` directory of your catkin workspace in the turtlebot pc
```git clone https://github.com/roboticslab-fr/turtlebot_vibot
cd </catkin_ws>
catkin make
rospack profile
```
- <b> Installing VISP package for visual servoing </b>:

http://wiki.ros.org/visp

- <b> setting Up project on Turtlebot </b>:

on the turtlebot pc:
```cd </catkin_ws>/src
git clone https://github.com/Macaulay-Sadiq/MSc_Robotics_Project.git
cd  </catkin_ws>
catkin make
rospack profile
```


## Navigation
### Map Creation
In our application, scan data from a laser scan sensor RPLIDAR is used to estimate the lo-cation of landmarks in the environment through the process of dead reckoning.  With therequired ROS packages, we can compute the scan data collected from the sensor to build acorresponding 2D map of the environment. An alternative to the RPLIDAR scan sensor, scandata from an RGB-D (Kinect) camera can as well be used to build map.  The Kinect whichseems ready to use with our robot ROS packages is less efficient for the map building. Wehave considered the use of the RPLIDAR over the Kinect camera for the map building sinceit gives scan data from its 360-degree angle of view from the robot’s current position, whilethe Kinect can only give about 57-60 degree angle of view scan data.  The angle of view ofthe laser sensor is of great importance to us to manage the problem of loop-closure anddiscontinuities during map building, therefore, a more accurate map can be obtained for areliable and realistic environment measurements[3].

We useSlam_gmappingto create a map of our environment. After that, we can loadthe created map into a map-server and use AMCL for navigation. Here is the command interminal:

On the Turtlebot:
```
$ roslaunch turtlebot_bringup minimal.launch$ roslaunch turtlebot_navigation gmapping_demo.launch
```
```
$ rosrun map_server map_saver -f /tmp/my_map
```
On the WorkStation:
```
$ roslaunch turtlebot_rviz_launchers view_navigation.launch
```
Save the map to a file
```
$ roscd turtlebot_vibot_nav/maps/
```
```
$ rosrun map_server map_saver -f my_map
```
By default,map_saver retrieves map data and writes it out to map.pgm and map.yaml. Use the -f option to provide a different base name for the output files. (-f ymap=>my_map.pgm+ my_map.yaml)

<b>Note: Never close the Gmapping until the map is saved</b> 

A comparison of the result obtained from a map building with an RPLIDAR scan sensorand Kinect is shown below:

<p align="center">
   <img src="/Images/image_github2.JPG" width="500" height="300" alt="Map Creation" />
</p>

The answers to these questions refer to localisation to determine where the robot is on themap, and path-planning to know how to reach a target position for the robot on the map.To accoplish the navigation , we need global costmap and local costmap. Global costmap is obtained from the map creation and local costmap is the map that the robot creates simultaneously while it navigates through the global costmap. futhermore, whilst navigating through global costmap, it uses the features being matched from local costmap and global costmap to do its path-planning.

The next task after building a map is to make the robot to move on the created map. For this task the robot is given commands to move from its current position to a goal positioninside the map without colliding any obstacles. given a goal position to our robot, we will needto estimate the robot trajectory from its current position in the presence of static obstacles(requires global path planning) and dynamic obstacles (requires local path planning). Theproblem of path-planning focuses on the minimization of travel distance between the robotcurrent position and its given goal position on the map in the presence of obstacles[5]. By exploring the functionalities of the Dynamic-Window Approach (DWA) algorithm [6](included in base local planner ROS package) for the reactive obstacle avoidance for local path-planning together with the A* and the Dijkstra algorithms for global path planning (included in global planner ROS package) the results will then be compared with other methods of path-planning. In the end, we were able to conveniently navigate our robot on the built map from an initial position of the robot to a target position without collision of the robot on any form of obstacles. Thanks to the package provided by the robotics lab, we performed the navigation and obstacle avoidance using the combination of Kinect and RPLidar. This is defined in the launch file called amcl_demo_rplidar.launch

## Visual Servoing
Visual Servoing (VS), also known as Vision-based Robot Control is a technique which helps in fine-positioning of a robot by using the Visual Features (obtained from a Vision Sensor) as Feedback Information to control the Movement of the Robot. Visual Servoing can be performed in two configurations based on the placement of Vision Sensors, Eye-in-Hand (the Camera is placed in the Robot and the observing target is static or dynamic placed in the environment) or Eye-to-Hand (the Camera is fixed in the world and observes the moving target attached to the Robot). In this project Eye-in-Hand configuration has been opted.  

Based on the type of Visual Features used as a feedback control, Visual Servoing can be classified as,

- <b> IBVS (Image Based Visual Servoing): </b> The idea is to move the robot from its current location to a desired location
by using only the Image Features of a Pattern in 3D space. The Image features can be Points, Lines or Planes or any other corresponding 2D shapes of the Object in 3D, QR Codes, etc.

- <b> PBVS (Position Based Visual Servoing): </b> The full 3D object pose is estimated using a model of the object, and the control error is then defined in terms of the current and desired 3D poses. This method makes it possible to attain straight line motion resulting in a minimum trajectory length in 3D. 

- <b> Hybrid Visual Servoing: </b> This method combines PBVS and IBVS to avoid the drawbacks of those techniques. It is based on the estimation of the partial camera movement from the current position to the desired position.

### Image Based Visual Servoing
IBVS with QR codes as Image Feature has been implemented in this project. As shown in Figure 2 the Visual Servoing Algorithms perform a closed loop process for minimizing the Error Function <b>e(t)</b>. The Error function, <b>e(t)</b> is the disparity between the Desired Features <b>S*</b> and Measured Features <b>S(t)</b> at time <b>t</b>.

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
The control law is computed in the ```turtlebot_follower``` node which can be found in the ```visual_servoing_prj``` directory. In this file we subscribe to the ```/object_position``` topic published by the ```visp_auto_tracker``` package to extract the coordinates of the detected points on the QR-tag and the output velocity is published to ```/cmd_vel``` on the turtlebot.

By executing the following line of code we are able to perform the visual servoing with the turtlebot (Run this command on the turtlebot).

``` 
roslaunch visual_servoing_prj kinect_visp.launch
```
## Outcomes
Both Navigation and Fine Positioning can be performed sequentially by executing the following line of code (Run this command on the turtlebot through ssh). 

```
roslaunch visual_servoing_prj turtlebot_follower.launch
```

After executing the launch file press <img src="/Images/key_space_bar.png" width="80" height="20" alt="Space Bar" /> key to start the process. We have achieved the goal of the project and here are some Recorded Results of our implementation.

### Visual Servoing with A4 Size QR-tag
[![SCENARIO](http://i3.ytimg.com/vi/Egyl9DstUiU/maxresdefault.jpg)](https://youtu.be/4ZlNWGChNNU)
