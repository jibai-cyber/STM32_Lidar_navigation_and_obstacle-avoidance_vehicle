# STM32_Lidar_navigation&obstacle-avoidance_vehicle

## What is it?
A small vehicle that can perform navigation, obstacle avoidance, visual recognition,   
wireless communication (with drones & ground station) and human-machine interaction.  
Below are some demonstration videos that I recorded myself. The extraction code is: 02hc

> https://www.alipan.com/s/T4jDgnfJMRZ

Including the results of each project.

## Requirements?
1. Parse the point cloud data and convert it into coordinates
2. Continuous point-to-point navigation.
3. Given the map information, it is prohibited to cross the lines when driving normally or making turns.
4. Achieves point-to-point shortest path navigation through path planning algorithms
5. Avoid multiple obstacles during the journey
6. The user can set the destination of the car and visualize its trajectory through an LCD ground station. 

## Peripheral devices
Including but not limited to the following peripheral devices:  
- Raspberry Pi
- TB6612FNG
- zigbee
- Timer
- Lidar
- Gimbal

## Methods included
Including but not limited to:  
- Cascade PID design and parameter adjustment
- Achieve the transformation of LiDAR point cloud data to the world coordinate system and complete the positioning of the unmanned vehicle (such as coordinate and heading angle information). 
- Obstacle recognition, acquisition of obstacle coordinate information and conversion between the world coordinate system and the vehicle body coordinate system. 
- Apply cascade PID and other control methods to the vehicle for point-to-point motion control and steering control. 
- Design of Wireless Serial Port Communication Protocol between Ground Station and Unmanned Vehicle 
- Coordinate the radar data parsing frequency and control frequency to minimize response latency as much as possible.
- Path planning algorithm such as dijkstra and obstacle avoidance path point generation.

## Project start time
September 2023

## To-do task
1. The code needs to be cleaned.
2. Provide a instruction Doc.
