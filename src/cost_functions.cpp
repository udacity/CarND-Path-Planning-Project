#include "cost_functions.h"

double time_diff_cost(const vector<double> &traj, const Vehicle &target_vehicle, const vector<double> &delta, const double T, const vector<Vehicle> &predictions)
{
    double t = traj[12];
    return logistic((t-T)/T);
}
