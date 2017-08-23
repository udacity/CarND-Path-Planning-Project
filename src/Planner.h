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

  void preprocess(double car_s, double car_d, const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                  vector<vector<double>> &sensor_fusion);

  void plan();
  void postprocess();

  vector<vector<double>> get_planned_result();

  bool do_update;
private:


  /* Planner constant */
  // mp/h
  const double DEFAULT_SPEED_LIMIT = 48.0;
  const int DEFAULT_TIMESTEPS = 180;
  const int DEFAULT_INTERVAL = 40;

  // convert mp/h to timestep
  const double CONVERSION = .02/2.24;

  /* Planner config for cost calculations */
  // 50 mp/h and a little buffer
  const double MAX_VAL = CONVERSION * 49.5;
  // 10 m/s
  const double MAX_ACC = 10.0 / 50.0;
  // 10 m/s
  const double MAX_JERK = 10.0 / 50.0;

  const double CAR_WID = 2.5;
  const double CAR_LEN = 5.0;
  const double CAR_CRI_WID = 0.5 * CAR_WID;
  const double CAR_CRI_LEN = 0.5 * CAR_LEN;
  const double CAR_SAFE_WID = CAR_WID;
  const double CAR_SAFE_LEN = 5 * CAR_LEN;
  const int N_PERTURB_SAMPLE = 10;
  const double INF = numeric_limits<double>::infinity();


  /* Planner's initial value */
  int current_lane = 1;
  string current_action = "straight";
  double speed_limit_ = DEFAULT_SPEED_LIMIT;
  int timesteps_ = DEFAULT_TIMESTEPS;
  int interval_ = DEFAULT_INTERVAL;

  double max_vel_ = 0.0;
  double max_delta_s_ = 0.0;

  /* Planner's container */
  WayPoints way_points;
  Car my_car;
  vector<Car> other_cars;
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
  void perturb_end(vector<double> &end_vals, vector<vector<double>> &end_points, bool change_left=false);
  Polynomial jmt(vector<double> const &start, vector<double> const &end, int t);

  /* Misc */
  std::default_random_engine rand_generator_;

};

#endif //PATH_PLANNING_PLANNER_H
