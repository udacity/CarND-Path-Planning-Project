//
// Created by Joey Liu on 2017/08/16.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <random>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "Vehicle.h"
#include "WayPoints.h"
#include "CarState.h"

using namespace std;

class Planner {
public:
  Planner() = default;

  vector<vector<double>> plan(CarState car_state, vector<vector<double>> &previous_path, vector<vector<double>> &sensor_fusion);

private:
  // Have a reference velocity to target
  const double ref_vel = 49.5;
  // Have a reference acceleration to target
  const double ref_acc = 10.0 / 50.0;
  // Have a refernece jerk to target
  const double ref_jerk = 10.0/ 50.0;
  WayPoints way_points;
};

#endif //PATH_PLANNING_PLANNER_H
