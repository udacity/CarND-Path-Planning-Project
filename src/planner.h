//
// Created by Joey Liu on 2017/08/16.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <cmath>
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <random>
#include "vehicle.h"
#include "waypoints.h"

using namespace std;

class Planner {
public:
  Planner();
  virtual ~Planner();

  vector<vector<double>> plan(vector<double> car_state, vector<vector<double>> &previous_path, vector<vector<double>> &sensor_fusion);

private:
  const double dist_inc = 0.5;
  WayPoints way_points;
};

#endif //PATH_PLANNING_PLANNER_H
