//
// Created by Jose Rojas on 7/23/17.
//

#ifndef PATH_PLANNING_PATHGENERATOR_H
#define PATH_PLANNING_PATHGENERATOR_H

#include <vector>
#include "SplineLibrary/spline_library/vector.h"
#include "SplineLibrary/spline_library/splines/uniform_cr_spline.h"

typedef struct {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> s;
    std::vector<double> dx;
    std::vector<double> dy;
} Waypoints;

typedef struct {
    std::vector<double> x;
    std::vector<double> y;
} PathPoints;

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
} VehicleState;

class PathGenerator {

public:
    PathGenerator();
    ~PathGenerator();
    PathGenerator(Waypoints waypoints, double s_max);

    PathPoints generate_path(VehicleState state);

private:
    class impl;
    std::unique_ptr<impl> pimpl;
};


#endif //PATH_PLANNING_PATHGENERATOR_H
