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
#include "way_points.h"
#include "utils.h"
#include "helpers.h"
#include "cost_functions.h"
#include "car.h"
#include "polynomial.h"

using namespace std;

class Planner {
public:
  Planner() = default;
  virtual ~Planner() = default;

  void preprocess(double car_s, double car_d, const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                  vector<vector<double>> &sensor_fusion);

  void plan();
  void postprocess();

  vector<vector<double>> get_planned_result();

  bool do_update;
private:


  /* Planner constants */
  // mp/h
  const double kDefaultSpeedLimit = 47;
  const int kDefaultTimeStep = 180;
  const int kDefaultInterval = 40;

  // convert mp/h to timestep
  const double kConversion = .02/2.24;

  /* Planner config for cost calculations */
  // 50 mp/h and a little buffer
  const double kMaxVel = kConversion * 49.5;
  // 10 m/s
  const double kMaxAcc = 10.0 / 50.0;
  // 10 m/s
  const double kMaxJerk = 10.0 / 50.0;

  // Define car's size and critical and safe ranges
  const double kCarWidth = 2.5;
  const double kCarLength = 5.0;
  const double kCarCriticalWidth = 0.5 * kCarWidth;
  const double kCarCriticalLength = 0.5 * kCarLength;
  const double kCarSafeWidth = kCarWidth;
  const double kCarSafeLength = 5 * kCarLength;
  const int kNPerturbSample = 10;
  const double kInf = numeric_limits<double>::infinity();


  /* Private variable with the initial values */
  int current_lane_ = 1;
  string current_action_ = "straight";
  double speed_limit_ = kDefaultSpeedLimit;
  int time_step_ = kDefaultTimeStep;
  int interval_ = kDefaultInterval;
  double max_vel_ = 0.0;
  double max_delta_s_ = 0.0;

  /* Planner's containers */
  WayPoints way_points_;
  Car my_car_;
  vector<Car> other_cars_;
  vector<double> prev_path_x_;
  vector<double> prev_path_y_;
  int prev_path_size_;
  vector<double> traj_s_;
  vector<double> traj_d_;
  vector<double> next_x_;
  vector<double> next_y_;

  /* Methods */
  double calculate_cost(const pair<Polynomial, Polynomial> &traj, const vector<double> &ends,
                      vector<vector<double>> &costs);
  void generate_perturbed_end_points(vector<double> &end_vals, vector<vector<double>> &end_points, bool change_left=false);
  Polynomial jmt(vector<double> const &start, vector<double> const &end, int t);

  /* Misc */
  std::default_random_engine rand_generator_;

};

#endif //PATH_PLANNING_PLANNER_H
