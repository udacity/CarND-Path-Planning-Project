# Write-Up

Author: Abhishek Mantha
Date: 5/14/18

This document is an official write-up for SDCND Term 3 Project 1: Path Planing. I will further elaborate on implementation details and rubric requirements below.

---

## Rubric Criteria

### The car drives according to the speed limit.
The ego vehicle is able to drive under the speed limit of 50 miles per hour comfortably in the Unity simulator. If a vehicle is detected in front of it in the same lane, it will accelerate by -0.112 meters/second^2 until it can safely move to another lane. Otherwise, it will accelerate at 0.224 meters/second^2 until traffic conditions are ideal for a lane change maneuver or a lane keep maneuver.

### Max acceleartion and jerk are not exceeded.
Neither max acceleration nor jerk are exceeded. Trajectory waypoints are interpolated with a nonlinear spline, therefore, output trajectories are far more smoother and do not risk exceeding acceleration or jerk.

### Car does not have collisions.
The ego vehicle makes sure to check in front and to the left and right prior to any lane change maneuver by a safe distance of 30 m. In addition, the ego vehicle slows down if it cannot execute a lane change maneuver.

### The car stays in its lane, except for time between changing lanes.
The ego vehicle obeys this requirement. The ego vehicle stays in its lane at all times, apart from a lane change for no longer than 3 seconds.

### The car is able to change lanes.
The car is able to smoothly change langes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

---

## Implementation 

### main.cpp
In this file, appropriate websockets are established to connect to the Unity simulator. The majority of this file was already implemented for us. The primary modification includes the instantiation of a Vehicle object of the Vehicle class that is responsible for generating a smooth spline trajectory after each vehicle state update. The vehicle's state consists of: x-coordinate, y-coordinate, s-value, d-value, yaw, car_speed, previous_path_x, previous_path_y, end_path_s, end_path_d, and sensor_fusion values. The Vehicle class uses this information on each simulation step to determine the next appropriate action to take. 

### helpers.h
This file contains all given helper functions that were originally located in main.cpp. These functions include:
	* pi()
	* deg2rad()
	* rad2deg()
	* distance()
	* ClosestWaypoint()
	* NextWaypoint()
	* getFrenet()
	* getXY()

### vehicle.h/vehicle.cpp
This is the meat of the project's implementation. The Vehicle class represents a single vehicle in the simulator. We initialize only one vehicle object for the Ego Vehicle. The Vehicle class keeps track of the following information: 
	* lane
	* reference velocity
	* x, y
	* s, d
	* yaw
	* speed
	* previousPathX, previousPathY
	* endPathS, endPathD
	* sensorFusion
	* trajectory X, trajectoryY
	* mapWaypointsX, mapWaypointsY
	* mapWaypointsS, mapWaypointsDX, mayWaypointsDY

The Vehicle class contains the following methods: 
	* updateGlobalMap()
	* updateVehicleState()
	* getSmoothSplineTrajectory()

#### updateGlobalMap()
This is a simple method that collects information about the simulator's waypoints to generation the ego vehicle's trajectory. In total, there are 181 waypoints, the last waypoint mapping back around to the first. The waypoints are in the middle of the double-yellow diving line in the center of the highway. Each waypoint has an (x,y) global map position, and a Frenet s value and Frenet d unit normal vector (split up into the x component, and the y component). This method is called prior to connecting to the simulator in the main() method of src/main.cpp.

#### updateVehicleState()
This is a simple method that collects the vehicle's current state for each step of the simulation. The vehicle's state consists of:
	* x-coordinate
	* y-coordinate
	* s-value
	* d-value
	* yaw
	* car_speed
	* previous_path_x, previous_path_y, 
	* end_path_s, end_path_d, 
	* sensor_fusion values

#### getSmoothSplineTrajectory()
This is the most important function of the Vehicle class. It calculates a safe trajectory for the ego vehicle on each time step. The function immediately iterates through the sensorFusion vector that contains all information of vehicles on the right side of the road and determines if another vehicle is ahead of the ego vehicle, to the left of the ego vehicle and to the right of the ego vehicile. 

The function then checks if the ego vehicle will collide with another vehicle based on where the vehicle will be in the future and if it encroaches a safe distance. This occurs by checking the d and s values respectively of a given vehicle object in the sensorFusion vector. Corresponding boolean flags indicate if a vehicle is to the front, left, or right of the ego vehile. In addition, if a vehicle is to the left or right, the distance of the closest vehicle to the ego vehicle is recorded. This distance is later used to determine which lane has more free space. At the end of the sensorFusion iteration, if the vehicleFront boolean flag is true, then the ego vehicle will attempt to move to another lane. 

The vehicle will only move to th left or to the right if there is no vehicle in that lane within a safe distance of the vehicle and if the nearest vehicle in that lane provides the most amount of free space for the ego vehicle to use. If neither of these cases is true, then the vehicle accelerates by -0.112 miles/hour. If a vehicle is not in front of the ego vehicle, the ego vehicle will attempt to move to the middle lane if it is safe to do so. This is to enable the number of possible decision states the ego vehicle can visit to overtake slower traffic. If the ego vehicle travels at slower than 49.5 miles per hour, the ego vehicle accelerates by 0.224 meters/hour.

After checking for a lane change maneuver, the function calculates a new trajectory for the ego vehicle. If the previous size of the ego vehicle's trajectory is less than 2, this represents that the simulator has just started or that the simulator has just reset. If that is the case, a (x,y)-coordinate pair that makes a tangent path to the vehicle's motion based on the vehicle's yaw is used as a starting point. Otherwise, the last two points of the previous trajectory are used as a starting point. 

Then, 3 new points are generated using Frenet coordinates that are evenly spaced every 30 meters ahead of the starting reference. This is accomplished using the getXY() method. These coordinates are pushed onto vectors that hold the x- and y-coordinate components of the new ego vehicle trajectory points separately. 

The new ego vehicle trajectory point vectors are transformed to local coordinates using a standard trigonometric rotation formula. Once those values are transformed, a spline object is instantiated. The spline object is responsible for generating smooth curves for drivable trajectories. Due to the nature of splines and the parameters specified, it is possible to design a trajectory that does not explicitly focus on Jerk Minimization. Instead, the focus with this approach has been to appropriately detect nearby vehicles. 

Now, we define new points for the planner. We start with all the previous path points and add new points to the existing path points, smoothened from one path to another. This is accomplished by defining a target horizon (x -- 30 meters away; y -- corresponding spline value to 30 meters). Then, the planner generates a new trajectory that is evenly spaced from the current position to the horizon, with incremental additions for each new point, comprising of a path of NUM_POINTS points. NUM_POINTS is 50 by default. Finally, the new points are shifted and rotated back to global coordinates using a standard trigonometric rotation formula.

Each new point is pushed onto x- and y- coordinate component vectors. At the end of this function, these 2 vectors are returned back to src/main.cpp and are passed onto the simulator to be displayed as green trajectory waypoints. 

## Reflection
I chose to approach this project with the simplest possible implemenation due to time constraints. With additional time, I would like to implement a more sophisticated finite state machine similar to the quiz solutions in the classroom. In addition, I would also like to use more rigorous shortest path algorithms like A* to improve the generation of more realistic, drivable trajectories. 

All in all, I am quite happy with my submission. This was definitely a really fun and really tricky assignment. 