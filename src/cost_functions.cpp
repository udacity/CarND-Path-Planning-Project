#include "cost_functions.h"

double time_diff_cost(const vector<double> &traj, const Vehicle &target_vehicle,
                      const vector<double> &delta, const double T,
                      const vector<Vehicle> &predictions) {
  double t = traj[12];
  return logistic((t - T) / T);
}

double s_diff_cost(const vector<double> &traj, const int target_vehicle,
                   const vector<double> &delta, const double T,
                   const vector<Vehicle> &predictions) {
  vector<double> s(traj.begin(), traj.begin() + 3);
  Vehicle target = predictions[target_vehicle];
  vector<double> target_state = VecAdd(target.state, delta);

  vector<double> s_targ(target_state.begin(), target_state.begin() + 3);
  vector<double> S(3);
  S[0] = polyval(s, T);
  vector<double> s1 = differntiate(s);
  vector<double> s2 = differntiate(s1);
  S[1] = polyval(s1, T);
  S[2] = polyval(s2, T);

  double cost = 0.;
  double diff = 0.;
  for (int i = 0; i < 3; ++i) {
    diff = float(abs(S[i] - s_targ[i]));
    cost += logistic(diff / SIGMA_SD[i]);
  }
  return cost;
}

double d_diff_cost(const vector<double> &traj, const int &target_vehicle,
                   const vector<double> &delta, const double T,
                   const vector<Vehicle> &predictions) {
  vector<double> d(traj.begin()+3, traj.begin() + 6);
  Vehicle target = predictions[target_vehicle];
  vector<double> target_state = VecAdd(target.state, delta);

  vector<double> d_targ(target_state.begin()+3, target_state.begin() + 6);
  vector<double> D(3);
  D[0] = polyval(d, T);
  vector<double> d1 = differntiate(d);
  vector<double> d2 = differntiate(d1);
  D[1] = polyval(d1, T);
  D[2] = polyval(d2, T);

  double cost = 0.;
  double diff = 0.;
  for (int i = 0; i < 3; ++i) {
    diff = float(abs(D[i] - d_targ[i]));
    cost += logistic(diff / SIGMA_SD[3+i]);
  }
  return cost;
}
