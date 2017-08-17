//
// Created by Joey Liu on 2017/08/02.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

using namespace std;

class Vehicle {
public:
  Vehicle();
  virtual ~Vehicle();
  void set_frenet_pos(double pos_s, double pos_d);
  void set_frenet_motion(double vel_s, double acc_s, double vel_d, double acc_d);
  vector<double> get_s();
  vector<double> get_d();
  vector<double> state_at(double t);

private:
  double p_s_;
  double p_d_;
  double v_s_;
  double v_d_;
  double a_s_;
  double a_d_;
};
#endif //PATH_PLANNING_VEHICLE_H
