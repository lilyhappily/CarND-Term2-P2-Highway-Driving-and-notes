# Self-driving-car Term 2

In this repository I will share the **source code** of Project 1 Kidnapped-Vehicle coming from Term 2 of [**Udacity Self-Driving Car Engineer Nanodegree**](<https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013>). 

In this  repository, you will find detailed note about this project. I will share all projects and related notes the next repositories. I hope this will be useful to someone. 

# Term 2 Course Outline

## Term 2

#### Localization, Path Planning, Control, and System Integration

In this term, you'll expand on your sensor knowledge to localize and control the vehicle. You'll evaluate sensor data from camera, radar, lidar, and GPS, and use these in closed-loop controllers that actuate the vehicle.

After that, you'll learn how to plan where the vehicle should go, and how the vehicle systems work together to get it there.

- Project 1  [Kidnapped Vehicle](https://github.com/lilyhappily/Udacity-Project1-CarND-Kidnapped-Vehicle-and-notes)
- Project2   [Highway Driving](https://github.com/lilyhappily/Udacity-Project2-CarND-Highway-Driving-and-notes)
- Project3   PID Controller
- Project4   Improve Your LinkedIn Profile
- Project5   Optimize Your GitHub Profile
- Project6   Programming a Real Self-Driving Car

# Visualization of Result

   ![visualization](assets/visualization.gif)

# Overview

The goal of this project is to build a path planner that creates smooth, safe trajectory for cars on highway driving. My code creates a successful path planner which is able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by localization, sensor fusion, and map data.

# Completion Process

According to the requests and materials coming from the lessons, I know clearly how to generate a path planner.

![1](assets/1.png)

Above the picture,  the inputs to behavior planning come from prediction model and localization model, both of which gets their inputs from the sensor fusion. And the output from the behavior model goes directly into the trajectory planner, which also takes input from prediction and localization. so that it can sent trajectory to the motion controller.

## Prediction 

About this project, predictions come from the sensor fusion data [id, x, y, vx, vy, s, d]. The id is a unique identifier. The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference the global map.Finally s and d are the Frenet coordinates for the car. According to these information, I can directly judge whether there is car blocking the traffic ahead of ego car, whether there is a car to the right of the ego car , and whether there is a car to the left of ego car. It is considered "dangerous" if the distance between ego car and other cars is less than 30 meters in front or behind of ego car. The Boolean variables  of  ***too_close***, ***car_left*** and ***car_right***  are used to make correct behaviors.(line 118 to line158 in main.cpp)

## Behavior

According to the prediction results,  the ego car can make correct decisions. if ***too_close*** is true which indicates there is a car blocking in front of ego car, the ego car will plan to change lane. If  ***car_left*** and ***car_right*** are both false, it is not safety to change lane,  and the ego car will decelerate 0.224mph every point at current velocity in its own lane until the sensor fusion tells the environment around the ego car is safety. (line 160 to line189 in main.cpp).

## Trajectory

The code builds a 50 points path, every time the ego car starts a new path with whatever previous path points were left over from last time, then append new waypoints until the new path has 50 total waypoints. The method can make a smoother trajectory, and handle with the different frequencies of  prediction model, behavior model and trajectory model.

Spline fitting is used to generate trajectory from Project Q&A of Aaron Brown. For making calculation easy, it is necessary to shift the car or the last point of previous path is at zero, and its angel is at zero degree.In the local car coordinate, the code calculates how to break up spline points so that the ego car can travel at a desired velocity. After finishing spline fitting, it must be rotated back to map coordinate.(line 191 to line 296 in main.cpp)

# Running the Program

1. Download the simulator and open it. In the main menu screen select : Path Planning.
2. The [CarND-Path-Planning README](https://github.com/lilyhappily/Udacity-Project2-CarND-Highway-Driving-and-notes/blob/master/CarND-Path-Planning-Project/CMakeLists.txt) has more detailed instructions for installing and using c++ uWebScoketIO.

