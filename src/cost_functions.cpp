#include "cost_functions.h"

double calculate_cost(const vector<double> &traj, const int &target_vehicle,
                      const vector<double> &delta, const double T,
                      const vector<Vehicle> &predictions, bool verbose)
{
    double cost = 0.;
    //cout << predictions.size();
    vector<CostFun> cf_list = {time_diff_cost, s_diff_cost, d_diff_cost, collision_cost, buffer_cost, efficiency_cost};
    vector<double> weights = {1, 1, 1, 20, 1, 20};

    for (size_t i = 0; i < cf_list.size(); ++i)
    {
        cost += weights[i] * cf_list[i](traj, target_vehicle, delta, T, predictions);
    }

    return cost;
}

double time_diff_cost(const vector<double> &traj, const int &target_vehicle,
                      const vector<double> &delta, const double T,
                      const vector<Vehicle> &predictions)
{
    double t = traj[12];
    return logistic((t - T) / T);
}

double s_diff_cost(const vector<double> &traj, const int &target_vehicle,
                   const vector<double> &delta, const double T,
                   const vector<Vehicle> &predictions)
{
    vector<double> s(traj.begin(), traj.begin() + 3);
    double t = traj[12];
    Vehicle target = predictions[target_vehicle];
    vector<double> target_state = VecAdd(target.state_in(t), delta);

    vector<double> s_targ(target_state.begin(), target_state.begin() + 3);
    vector<double> S(3);
    S[0] = polyval(s, T);
    vector<double> s1 = differntiate(s);
    vector<double> s2 = differntiate(s1);
    S[1] = polyval(s1, T);
    S[2] = polyval(s2, T);

    double cost = 0.;
    double diff = 0.;
    for (int i = 0; i < 3; ++i)
    {
        diff = float(abs(S[i] - s_targ[i]));
        cost += logistic(diff / SIGMA_SD[i]);
    }
    return cost;
}

double d_diff_cost(const vector<double> &traj, const int &target_vehicle,
                   const vector<double> &delta, const double T,
                   const vector<Vehicle> &predictions)
{
    vector<double> d(traj.begin() + 3, traj.begin() + 6);
    double t = traj[12];
    Vehicle target = predictions[target_vehicle];
    vector<double> target_state = VecAdd(target.state_in(t), delta);

    vector<double> d_targ(target_state.begin() + 3, target_state.begin() + 6);
    vector<double> D(3);
    D[0] = polyval(d, T);
    vector<double> d1 = differntiate(d);
    vector<double> d2 = differntiate(d1);
    D[1] = polyval(d1, T);
    D[2] = polyval(d2, T);

    double cost = 0.;
    double diff = 0.;
    for (int i = 0; i < 3; ++i)
    {
        diff = float(abs(D[i] - d_targ[i]));
        cost += logistic(diff / SIGMA_SD[3 + i]);
    }
    return cost;
}

double collision_cost(const vector<double> &traj, const int &target_vehicle,
                      const vector<double> &delta, const double T,
                      const vector<Vehicle> &predictions)
{
    double d = nearest_approach_to_any_vehicle(traj, predictions);
    if (d < (2 * VEHICLE_RADIUS))
    {
        return 1.;
    }
    else
    {
        return 0.;
    }
}

double buffer_cost(const vector<double> &traj,
                   const int &target_vehicle,
                   const vector<double> &delta, const double T,
                   const vector<Vehicle> &predictions)
{
    double d = nearest_approach_to_any_vehicle(traj, predictions);
    return logistic(2 * VEHICLE_RADIUS / d);
}

/**
 * Reward high average speed
*/
double efficiency_cost(const vector<double> &traj,
                       const int &target_vehicle,
                       const vector<double> &delta, const double T,
                       const vector<Vehicle> &predictions)
{
    vector<double> s(traj.begin(), traj.begin() + 3);
    double avg_v = polyval(s, T) / T;
    auto tmp = predictions[target_vehicle].state_in(T);
    double targ_v = tmp[0] / T;

    return logistic(2 * abs((targ_v - avg_v) / avg_v));
}

double max_accel_cost(const vector<double> &traj,
                      const int &target_vehicle,
                      const vector<double> &delta, const double T,
                      const vector<Vehicle> &predictions)
{
    vector<double> s(traj.begin(), traj.begin() + 3);
    vector<double> d(traj.begin() + 3, traj.begin() + 6);
    double t = traj[12];

    auto sd = differntiate(s);
    auto sdd = differntiate(sd);
    vector<double> t_vec;
    for (double _ = 0; _ < t; _ += 0.02)
    {
        t_vec.push_back(_);
    }
    auto a = polyval(sdd, t_vec);

    auto ma = max_element(a.begin(), a.end());
    if (*ma > MAX_ACC)
        return 1.0;
    else
        return 0.;
}