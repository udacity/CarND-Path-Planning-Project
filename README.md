# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Objective
Implementing a Path Planner to drive a car in Udacity simulator

## Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.The car's localization and sensor fusion data are provided, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Requirements
1-The car drives according to the speed limit (50 Mph)

2-The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

3-The car does not collide with any other car

4-The car stays inside one of the 3 lanes on the right hand side of the road and change lanes in at most 3 seconds.

5-The car is able to drive at least 4.32 miles without incident.

## Results
The car is able to drive around the track satisfying all the requirements mentioned above 

## Methodology
The output of the path planner is to provide a list of waypoints to be tracked by the controller. Below is a description of how this path is generated.

**The path consists of 50 waypoints**
1-A list of widely sparsed waypoints were created using the waypoints of the previous path.

2-The rest of the 50 waypoints are sampled from a spline.

**Lane Changes**
1-The car changes it's lane when there is a car infront which is slower, and it's safe to change lanes.

2-Left lanes are preferable than right lanes, so if a car infront is slower and the left lane is safe, the car will move to the left lane. If the left lane is not safe the car will move to the right lane if it is safe.

## TODO
1-Refactor code and encapsulate the main code in a Path Planner Class

2-if it's ok to changed to both adjacent lanes, calculate a cost function for each one of them and choose the lane change with least cost.