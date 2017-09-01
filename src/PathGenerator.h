//
// Created by Jose Rojas on 7/23/17.
// Copyright © 2017, Jose Luis Rojas
// All Rights Reserved.
//

#ifndef PATH_PLANNING_PATHGENERATOR_H
#define PATH_PLANNING_PATHGENERATOR_H

#include <vector>
#include "SplineLibrary/spline_library/vector.h"
#include "SplineLibrary/spline_library/splines/uniform_cr_spline.h"

/* Public Data structures used by main.cpp to access the PathGenerator class */

// Waypoints - contains the map waypoints
typedef struct {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> s;
    std::vector<double> dx;
    std::vector<double> dy;
} Waypoints;

// PathPoints - contains the points for the simulated trajectory path
typedef struct {
    std::vector<double> x;
    std::vector<double> y;
} PathPoints;

// SensorVehicleState - contains the sensor information regarding other vehicles on the road
typedef struct {
    int id;
    double x;
    double y;
    double v_x;
    double v_y;
    double s;
    double d;
    double speed;
} SensorVehicleState;

// VehicleState - contains all the sensor and localization data of the ego vehicle
typedef struct {
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    double end_d;
    double end_s;
    std::vector<double> remaining_path_x;
    std::vector<double> remaining_path_y;
    std::vector<SensorVehicleState> sensor_state;
    int forceLane;
} VehicleState;

// The PathGenerator class public interface.
class PathGenerator {

public:
    PathGenerator();
    ~PathGenerator();
    PathGenerator(Waypoints waypoints, double s_max);

    //The main function to call - given the vehicle state, returns a set of PathPoints for the trajectory
    PathPoints generate_path(VehicleState state);

private:
    class impl;
    std::unique_ptr<impl> pimpl;
};


#endif //PATH_PLANNING_PATHGENERATOR_H
