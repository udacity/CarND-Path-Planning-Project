Project: Highway Driving(Path Planning)
---

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

|<img src="data/images/final-output.gif" width="500" height="300" />|
|----------------------------------|
|[Running instructions](https://youtu.be/SLMwKynQ9MU) |

### Introduction
*The goal of this project is to build a path planner that creates smooth, safe trajectories for the car to follow. The highway track has other vehicles, all going different speeds, but approximately obeying the 50 MPH speed limit.*

*The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.*

### The code model for generating paths

```cpp
std::pair<std::vector<double>, std::vector<double >> PathPlanner::planPath(
        const path_planning::SimulatorRequest &simReqData)
{
    // Update trajectory history
    updateTrajectoryHistory(simReqData);
    // Get the lane speeds
    std::array<double, 3> laneSpeeds = getLaneSpeeds(simReqData.mainCar, simReqData.otherCars);
    // Scheduling the lane's changes
    scheduleLaneChange(simReqData.mainCar, laneSpeeds, simReqData.otherCars);
    // generate Spiline x and y trajectories
    std::pair<std::vector<double>, std::vector<double>> xy_trajectories =
            generateTrajectorySplines(simReqData.mainCar, laneSpeeds[m_targetLaneIndex], simReqData.previous_path_x,
                                      simReqData.previous_path_y);
    m_lastX = xy_trajectories.first;
    m_lastY = xy_trajectories.second;
    return xy_trajectories;
}

```
