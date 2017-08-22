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
#include "WayPoints.h"
#include "utils.h"
#include "helpers.h"
#include "cost_functions.h"
#include "Car.h"
#include "Polynomial.h"

using namespace std;

class Planner {
public:
  Planner() = default;
  virtual ~Planner() = default;

  vector<vector<double>> plan(vector<double> &car_state, vector<vector<double>> &previous_path,
                              vector<vector<double>> &sensor_fusion);

private:


  /* Planner config */
  // mp/h
  const double default_speed_limit_ = 48.0;
  const int default_global_interval_ = 180;
  const int default_local_interval_ = 40;

  // convert mp/h to timestep
  const double conversion_ = .02/2.24;

  /* Planner config for cost calculations */
  // 50 mp/h and a little buffer
  const double hard_max_vel_per_timestep_ = conversion_ * 49.0;
  // 10 m/s
  const double hard_max_acc_per_timestep_ = 10.0 / 50.0;
  // 10 m/s
  const double hard_max_jerk_per_timestep_ = 10.0 / 50.0;

  const double car_width_ = 2.5;
  const double car_length_ = 5.0;
  const double car_critical_width_ = 0.5 * car_width_;
  const double car_critical_length_ = 0.5 * car_length_;
  const double car_safe_width_ = car_width_;
  const double car_safe_length_ = 5 * car_length_;
  const int number_perturb_sample_ = 10;
  const double inf_value = numeric_limits<double>::infinity();


  /* Planner's initial value */
  int current_lane = 1;
  string current_action = "straight";
  double speed_limit_ = default_speed_limit_;
  int global_interval_ = default_global_interval_;
  int local_interval_ = default_local_interval_;

  double ref_vel_ = 0.0;
  double ref_delta_s_ = 0.0;

  /* Planner's container */
  WayPoints way_points;
  Car my_car;
  vector<Car> other_cars;

  /* Methods */
  void preprocess(vector<double> &car_state, vector<vector<double>> &previous_path,
                  vector<vector<double>> &sensor_fusion);

  double calculate_cost(const pair<Polynomial, Polynomial> &traj, const vector<double> &ends,
                      vector<vector<double>> &costs);
  void perturb_end(vector<double> &end_vals, vector<vector<double>> &end_points, bool change_left=false);
  Polynomial jmt(vector<double> const &start, vector<double> const &end, int t);

  /* Misc */
  std::default_random_engine rand_generator_;

};

#endif //PATH_PLANNING_PLANNER_H
