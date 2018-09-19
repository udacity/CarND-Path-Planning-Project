#ifndef COST_FUNCTIONS_H_
#define COST_FUNCTIONS_H_

#include <functional>
#include <iterator>
#include <vector>
#include <algorithm>
#include "helpers.h"

using namespace std;

struct Vehicle;
struct Traj2D;

using CostFun = function<double(const vector<double> &traj,
                                const int &target_vehicle,
                                const vector<double> &delta, const double T,
                                const vector<Vehicle> &predictions)>;

double calculate_cost(const vector<double> &traj, const int &target_vehicle,
                      const vector<double> &delta, const double T,
                      const vector<Vehicle> &predictions, bool verbose = false);

double calculate_cost_traj(const vector<double> &traj, const int &target_vehicle,
                           const vector<double> &delta, const double T,
                           const vector<Vehicle> &predictions, int lane, bool verbose = false);

double calculate_cost_veh_traj(const Traj2D &traj,
                               const double T,
                               const vector<Vehicle> &predictions, bool verbose = false);

double time_diff_cost(const vector<double> &traj, const int &target_vehicle,
                      const vector<double> &delta, const double T,
                      const vector<Vehicle> &predictions);

double s_diff_cost(const vector<double> &traj, const int &target_vehicle,
                   const vector<double> &delta, const double T,
                   const vector<Vehicle> &predictions);

double d_diff_cost(const vector<double> &traj, const int &target_vehicle,
                   const vector<double> &delta, const double T,
                   const vector<Vehicle> &predictions);

double collision_cost(const vector<double> &traj, const int &target_vehicle,
                      const vector<double> &delta, const double T,
                      const vector<Vehicle> &predictions);

double collision_cost(const Traj2D &traj, const double T,
                      const vector<Vehicle> &predictions);

double max_accel_cost(const vector<double> &traj, const int &target_vehicle,
                      const vector<double> &delta, const double T,
                      const vector<Vehicle> &predictions);

double total_accel_cost(const vector<double> &traj,
                        const int &target_vehicle,
                        const vector<double> &delta, const double T,
                        const vector<Vehicle> &predictions);

double max_jerk_cost(const vector<double> &traj, const int &target_vehicle,
                     const vector<double> &delta, const double T,
                     const vector<Vehicle> &predictions);

double total_jerk_cost(const vector<double> &traj,
                       const int &target_vehicle,
                       const vector<double> &delta, const double T,
                       const vector<Vehicle> &predictions);

double buffer_cost(const vector<double> &traj,
                   const int &target_vehicle,
                   const vector<double> &delta, const double T,
                   const vector<Vehicle> &predictions);

double buffer_cost(const Traj2D &traj,
                   const double T,
                   const vector<Vehicle> &predictions);

double efficiency_cost(const vector<double> &traj,
                       const int &target_vehicle,
                       const vector<double> &delta, const double T,
                       const vector<Vehicle> &predictions);

double max_accel_cost(const vector<double> &traj,
                      const int &target_vehicle,
                      const vector<double> &delta, const double T,
                      const vector<Vehicle> &predictions);

#endif // COST_FUNCTIONS_H_