# Autonomous Contact Avoiding Robot in Mujoco Simulator

## Introduction

This project demonstrates autonomous navigation of a mobile robot in a randomly generated dungeon environment. A combination of A* path planning and Dynamic Window Approach (DWA) is used for motion planning and collision avoidance. The simulation is built using MuJoCo physics engine. 

 ## A* Path Planning

A* path planning algorithm is used in this project since narrow corridors and closed environment in dungeons create a perfect case for A* algorithm. 
To get an optimal path for robot:
 - The grid resolution was intentionally set low. 
 - The robot's radius is intentionally set higher than its real size.

These parameters ensured that points of the A* algorithm are more centered on corridors and away from walls. 

The points of A* can be observed in the MuJoCo viewer as green tiles. 

The algorithm code is from PythonRobotics repository by Atsushi Sakai on GitHub without modifications.

## Dynamic Window Approach (DWA)

The DWA algorithm is used for local motion planning and obstacle avoidance. The goal points of DWA algorithm were set as A* points. As the robot approaches to each point, goal point advances to next until the final point is reached.

The DWA code is taken from PythonRobotics repository by Atsushi Sakai on GitHub with some improvements. The original code had three parameters that could be optimized: goal cost, speed cost, and obstacle cost. Original code calculated the goal cost by only calculating the angle difference. This is improved into calculating the error by both the angle difference and the distance between goal point and current point. 

Optimizing the parameters of DWA algorithm was the most challenging part of this project. 

## Robot Control

The output of DWA controller (velocity and yaw rate) is not applied directly to the robot. Instead:
 - The steering command is filtered using a low-pass filter to ensure smoother movement on robot.
 - Velocity and steering are clipped to prevent sudden big changes.

## Special cases

Two key improvements were implemented:

 1. If the yaw angle difference between robot's heading and its goal is bigger than 80 degrees, robot enters "Orientation Mode" .It moves forward and backward while applying locked steering to realign itself. This stabilizes the robot and improves DWA performance.

 2. If a goal point is occupied by an obstacle, the robot skips it and selects a further point along the path. This prevents the DWA controller targeting locations that are too close to dynamic obstacles. By this improvement the robot is headed into a further point, making the robot's advance on the dungeon smoother.

## Difficulties and Potential Improvements

The DWA algorithm is a computationally expensive algorithm and since the algorithm is implemented in Python, the code cannot run very fast. Several optimizations are implemented to overcome this challange.
 - Only using obstacles that are near the robot for DWA algorithm.
 - Tuning the parameters of DWA to reduce calculation complexity. 
 - An extra "Higher dt mode" is implemented to increase the simulation timestep in MuJoCo. This reduces the computational cost but makes the algorithm less stable and more fragile. If real-time loop is running too slowly, this mode can be used with adjustable dt multiplier. 
 
Rewriting the code in a compiled language such as C or C++ would allow for higher resolution control.

This project was developed as part of the course CMPE434: Introduction to Robotics at Boğaziçi University, taught by Doğan Ulus. Assignments for this course can be found on Introduction to Mobile Robotics repository I have created. If you are new to mobile robotics or want to build a similar project from scratch I suggest you to check out that repository. Assignments are structured from basic to advanced. Specifications of this project are given in the link below.

https://hackmd.io/@doganulus/Skmr9C20yx
