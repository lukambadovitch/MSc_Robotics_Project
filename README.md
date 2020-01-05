# MSc_Robotics_Project

## Synopsis
  1. [Introduction](#introduction)
  2. [Project Description](#project-description)
  3. [Navigation](#navigation)
  4. [Visual Servoing](#visual-servoing)
  5. [Outcomes](#outcomes)

## Introduction

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
Both Navigation and Fine Positioning can be performed sequentially by executing the following line of code (Run this command on the turtlebot). 

```
roslaunch visual_servoing_prj turtlebot_follower.launch
```

After executing the launch file press <img src="/Images/key_space_bar.png" width="80" height="20" alt="Space Bar" /> key to start the process. We have achieved the goal of the project and here are some Recorded Results of our implementation.

### Visual Servoing with A4 Size QR-tag
[![SCENARIO](http://i3.ytimg.com/vi/Egyl9DstUiU/maxresdefault.jpg)](https://youtu.be/4ZlNWGChNNU)
