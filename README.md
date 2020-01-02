# MSc_Robotics_Project

## Synopsis
  1. [Project Description](#Pro)
  2. [Navigation](#Nav)
  3. [Visual Servoing](#VS)
  

## Project Description
This project is developed under ROS-Kinetic environment on Ubuntu 16.04 LTS with a TurtleBot. 

## Navigation

## Visual Servoing
Visual Servoing (VS), also known as Vision-based Robot Control is a technique which helps in fine-positioning of a robot by using the Visual Features (obtained from a Vision Sensor) as Feedback Information to control the Movement of the Robot as represented in [Figure 1]. Visual Servoing can be performed in two configurations based on the placement of Vision Sensors, Eye-in-Hand (the Camera is placed in the Robot and the observing target is static or dynamic placed in the environment) or Eye-to-Hand (the Camera is fixed in the world and observes the moving target attached to the Robot). In our implementation we opt for Eye-in-Hand  

![VSLoop]
  <img align="center" src='/Images/VS.png' />
 

Based on the type of Visual Features used as a feedback control, Visual Servoing can be classified as,

- IBVS (Image Based Visual Servoing): The idea is to move the robot from its current location to a desired location
by using only the Image Features of a Pattern in 3D space. The Image features can be Points, Lines or Planes or any other corresponding 2D shapes of the Object in 3D, QR Codes, etc.

- PBVS (Position Based Visual Servoing): The full 3D object pose is estimated using a model of the object, and the control error is then defined in terms of the current and desired 3D poses. This method makes it possible to attain straight line motion resulting in a minimum trajectory length in 3D. 

- Hybrid Visual Servoing: This method combines PBVS and IBVS to avoid the drawbacks of those techniques. It is based on the estimation of the partial camera movement from the current position to the desired position.

