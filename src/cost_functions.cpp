#include "cost_functions.h"

double calculate_cost(const vector<double> &traj, const int &target_vehicle,
                      const vector<double> &delta, const double T,
                      const vector<Vehicle> &predictions, bool verbose)
{
    double cost = 0.;
    //cout << predictions.size();
    vector<CostFun> cf_list = {time_diff_cost, s_diff_cost, d_diff_cost, collision_cost, buffer_cost, efficiency_cost};
    vector<double> weights = {1, 1, 1, 30, 1, 30};
    if (verbose)
        cout << endl;
    for (size_t i = 0; i < cf_list.size(); ++i)
    {
        double c = weights[i] * cf_list[i](traj, target_vehicle, delta, T, predictions);
        cost += c;
        if (verbose)
        {
            cout << "cost of " << i << "-th function is: " << c << "\n";
        }
    }
    if (verbose)
        cout << endl;
    return cost;
}

double calculate_cost_traj(const vector<double> &traj, const int &target_vehicle,
                           const vector<double> &delta, const double T,
                           const vector<Vehicle> &predictions, int lane, bool verbose)
{
    double cost = 0;
    vector<double> s(traj.begin(), traj.begin() + 6);
    auto s_dot = differntiate(s);
    auto s_ddot = differntiate(s_dot);
    auto s_tdot = differntiate(s_ddot);

    vector<double> d(traj.begin() + 6, traj.begin() + 12);
    auto d_dot = differntiate(d);
    auto d_ddot = differntiate(d_dot);
    auto d_tdot = differntiate(d_ddot);

    vector<double> t;
    for (int i = 0; i < 101; ++i)
    {
        t.push_back(float(i) / 100. * T);
    }

    auto s_s = polyval(s, t);
    auto s_v = polyval(s_dot, t);
    auto s_a = polyval(s_ddot, t);
    auto s_j = polyval(s_tdot, t);

    auto d_s = polyval(d, t);
    auto d_v = polyval(d_dot, t);
    auto d_a = polyval(d_ddot, t);
    auto d_j = polyval(d_tdot, t);

    // cost of going straight at the center line of the current lane
    double md = meanVecAbs(d_s) - 4.0 * lane - 2.0;
    double cost_straight = logistic(md);
    cost += cost_straight * 1.0;

    // cost of going fast, or distance to the goal
    double dist_to_goal = MAX_SPEED * T - (s_s[s_s.size() - 1] - s_s[0]);
    double cost_to_goal = logistic(2 * dist_to_goal / (MAX_SPEED * T));
    cost += cost_to_goal * 20.0;

    // cost of speed limit
    double mv = maxVelocity(s_v, d_v);
    double cost_speed_limit = 0;
    if (mv > MAX_SPEED)
        cost_speed_limit = 1.0 * 50;
    cost += cost_speed_limit;

    // cost of acc limit
    double ma = maxVelocity(s_a, d_a);
    double cost_acc_limit = 0;
    if (ma > MAX_ACC)
        cost_acc_limit = 1.0 * 50;
    cost += cost_acc_limit;

    // cost of jerk limit
    double mj = maxVelocity(s_j, d_j);
    double cost_jerk_limit = 0;
    if (mj > MAX_JERK)
        cost_jerk_limit = 1.0 * 50;
    cost += cost_jerk_limit;

    // cost of collision
    double cost_collision = collision_cost(traj, target_vehicle, delta, T, predictions);
    cost += cost_collision * 50;

    return cost;
}

double time_diff_cost(const vector<double> &traj, const int &target_vehicle,
                      const vector<double> &delta, const double T,
                      const vector<Vehicle> &predictions)
{
    double t = traj[12];
    return logistic(abs(t - T) / T);
}

double s_diff_cost(const vector<double> &traj, const int &target_vehicle,
                   const vector<double> &delta, const double T,
                   const vector<Vehicle> &predictions)
{
    vector<double> s(traj.begin(), traj.begin() + 6);
    double t = traj[12];
    Vehicle target = predictions[target_vehicle];
    vector<double> target_state = VecAdd(target.state_in(t), delta);

    vector<double> s_targ(target_state.begin(), target_state.begin() + 3);
    vector<double> S(3);
    S[0] = polyval(s, t);
    vector<double> s1 = differntiate(s);
    vector<double> s2 = differntiate(s1);
    S[1] = polyval(s1, t);
    S[2] = polyval(s2, t);

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
    vector<double> d(traj.begin() + 6, traj.begin() + 12);
    double t = traj[12];
    Vehicle target = predictions[target_vehicle];
    vector<double> target_state = VecAdd(target.state_in(t), delta);

    vector<double> d_targ(target_state.begin() + 3, target_state.begin() + 6);
    vector<double> D(3);
    D[0] = polyval(d, t);
    vector<double> d1 = differntiate(d);
    vector<double> d2 = differntiate(d1);
    D[1] = polyval(d1, t);
    D[2] = polyval(d2, t);

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
    vector<double> s(traj.begin(), traj.begin() + 6);
    double t = traj[12];
    double sv = polyval(s, t);
    auto tmp = predictions[target_vehicle].state_in(t);
    double targ_s = tmp[0];

    //targ_v = MAX_SPEED * 0.98;
    return logistic(2 * abs((targ_s - sv) / targ_s));
}

double max_accel_cost(const vector<double> &traj,
                      const int &target_vehicle,
                      const vector<double> &delta, const double T,
                      const vector<Vehicle> &predictions)
{
    vector<double> s(traj.begin(), traj.begin() + 6);
    vector<double> d(traj.begin() + 6, traj.begin() + 12);
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