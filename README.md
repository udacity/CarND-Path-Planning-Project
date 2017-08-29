# CarND-Path-Planning-Project
This is my submission for the Path Planning project in term 3 of the Udacity Self-Driving Car Engineer Nanodegree Program. For details of the project, build/run instructions, simulator and dependencies, refer to the source [repo](https://github.com/udacity/CarND-Path-Planning-Project).
   
## Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Code compopents
The code is split into 3 parts:
* `main.cpp` handles the communication with the udacity simulator and calls each individual functions during each processing frame.
* `path.cpp/path.h` contains all main functionalities of the path planner.
* `helpers.h` contains all the helper functions, tools and conversions. It also contains some of the constants defintions.

## Project Rubric
The following rubric points are met. 

### Compilation
Code compiles without error with `cmake` and `make`.

### Valid Trajectories
#### Required Distance
The car is able to drive more than 4.32 miles without incident.

#### Speed Limit
The car doesn't drive faster than the speed limit of 50mph. This limit is set in `path.cpp: line 43`:
```speed_limit = 49.8;```

#### Max Accerelation and Jerk
The car does not exceed total acceleration of 10m/s^2 and a jerk of 10m/s^3. This is taken care by carefully making sure each path point sent to the simulator is properly spaced from the previous point such that the maximum distance (max_accel x dt^2) is respected. When accelerating/decelerating, the points spacing are gradually increased/decreased until the desired spacing corresponding to the target velocity is met. This is done is `path.cpp: lines 636-650`.

#### Collisions
The car does not come into contact with other cars on the road, and the car is able to drive around the track without collisions. Rules for adapting the car's speed is put place to prevent the car from tailgating into front vehicles when driving in its own lane (`path.cpp: lines 190-222`). (However, sometimes other cars in adjacent lanes swerve into our lane and may cause an unavoidable collision event. See example video at [https://youtu.be/EnUeVVJZnc8](https://youtu.be/EnUeVVJZnc8)).

#### Stay in Lane
The car stays inside one of the 3 lanes unless changing lane, for which it does not spend more than 3 seconds outside the lane lines. This is achieved via a state machine implementation as illustrated below.
![alt text](Path_Planning_Behavior_State_Machine.png)
The car only change lanes in the `CHANGE LANE` state, where the `path.trajectory()` function generates a single lane change trajectory. The car can only change lanes after the lane change is complete.

#### Changing Lanes
The car is able to smoothly change lanes when it makes sense. This is achieved via a set of heuristic rules in the `path.behavior()` function, which implements the above-mentioned behavior state machine. 

In the `Keep Lane` state, as long as the car can proceed near the speed limit and not come within 5 car-lengths of the car in front (`path.cpp: line 192`), it will stay in the current lane. Otherwise, it progresses to the `Prepare Lane Change` state.

In the `Prepare Lane Change` state, the car decides the target lane, target speed and when to excecute the lane change, depending on the traffic situation (`path.cpp: lines 228-464`). It can also decide to give up changing lanes and go back to driving in its current lane if the traffic becomes favorable.

In the `Change Lanes` state, the car simply execute the lane change trajectory (`path.cpp: lines 472-485`).

### Code Model
The code model is structured based on the three blocks in path planning - 1) Prediction, 2) Behavior Planning and 3) Trajectory Planning. The Prediction block (`path.prediction()`) uses the information from sensor fusion to update the current and predicted future positions and speed of surrounding cars. It also calculates the score of each lane, which is a simple function based on the available space in front of the car 1.0 sec in the future (`path.cpp: lines 124-130`). The Behavior Planning block (`path.behavior()`) is where the main decision making takes place. It relies on a set of pre-determined scenarios and rules to guide the car in its decision to stay / change lanes, set speed and react to traffic. Finally the Trajectory planner generates a set of path points that are passed to the control unit for execution.

### Reflection
This project took a longer time to complete that expected. Part of the reason is because there are endless traffic conditions that could happen on the track. As such, many of the rules are put in to react to or prevent certain accidents. For example, while changing lanes from the outer to the center lane, another car can also cut in from the other side (see video example at [https://youtu.be/6zqVfNLIDiU](https://youtu.be/6zqVfNLIDiU)). Hence, a new rule is added to avoid changing to the center lane when there is a fast car in the other outer lane (`path.cpp: lines 280-301`). 

This reflects one of the major difficulty of a rule-based / robotics way of planning paths. It is very challenging to construct enough rules to take care of all types of traffic situation. Perhaps an initial set of rules with reinforcement learning will be a better solution.

### Demo
Here's a video of a demo of the result after several rounds around the simulation track (~20 mies):
[https://youtu.be/Ugx2VEx-JSM](https://youtu.be/Ugx2VEx-JSM)
