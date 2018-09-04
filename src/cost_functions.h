#ifndef COST_FUNCTIONS_H_
#define COST_FUNCTIONS_H_

#include "helpers.h"

using CostFun = double (*)(const vector<double> &traj, const Vehicle &target_vehicle, const vector<double> &delta, const double T, const vector<Vehicle> &predictions);

double calculate_cost(const vector<double> &traj, const Vehicle &target_vehicle, const vector<double> &delta, const double T, const vector<Vehicle> &predictions, bool verbose = false);

double time_diff_cost(const vector<double> &traj, const Vehicle &target_vehicle, const vector<double> &delta, const double T, const vector<Vehicle> &predictions);

double s_diff_cost(const vector<double> &traj, const Vehicle &target_vehicle, const vector<double> &delta, const double T, const vector<Vehicle> &predictions);

double d_diff_cost(const vector<double> &traj, const Vehicle &target_vehicle, const vector<double> &delta, const double T, const vector<Vehicle> &predictions);

double collision_cost(const vector<double> &traj, const Vehicle &target_vehicle, const vector<double> &delta, const double T, const vector<Vehicle> &predictions);

double max_accel_cost(const vector<double> &traj, const Vehicle &target_vehicle, const vector<double> &delta, const double T, const vector<Vehicle> &predictions);

double total_accel_cost(const vector<double> &traj, const Vehicle &target_vehicle, const vector<double> &delta, const double T, const vector<Vehicle> &predictions);

double max_jerk_cost(const vector<double> &traj, const Vehicle &target_vehicle, const vector<double> &delta, const double T, const vector<Vehicle> &predictions);

double total_jerk_cost(const vector<double> &traj, const Vehicle &target_vehicle, const vector<double> &delta, const double T, const vector<Vehicle> &predictions);

#endif //COST_FUNCTIONS_H_