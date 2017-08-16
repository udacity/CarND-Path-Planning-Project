//
// Created by Joey Liu on 2017/08/16.
//
#include <vector>
#include "planner.h"


using namespace std;

Planner::Planner() {
}
Planner::~Planner() {}

vector<vector<double>> Planner::plan(vector<double> car_state, vector<vector<double>> &previous_path, vector<vector<double>> &sensor_fusion) {

  vector<double> next_x;
  vector<double> next_y;

  for (int i = 0; i < 50; i++) {
    double next_s = car_state[2] + (i + 1)*dist_inc;
    double next_d = 6;

    vector<double> new_xy = way_points.getXY(next_s, next_d);
    next_x.push_back(new_xy[0]);
    next_y.push_back(new_xy[1]);
  }

  return {next_x, next_y};
}