#include "vehicle.hpp"
#include "helper.hpp"

void Vehicle::Next() {
  this->next_x_vals.clear();
  this->next_y_vals.clear();
  int max_num = 20;
  int num = 20;
  double s_diff = 0.03;
  double d_diff = 0;
  vector<double> next_s_vals;
  vector<double> next_d_vals;
  double ref_s = this->car_s;
  double ref_d = this->car_d;

  int remain = previous_path_x.size();
  num = max_num - remain;
  for (auto px : previous_path_x)
  {
    this->next_x_vals.push_back(px);
  }
  for (auto py : previous_path_y)
  {
    this->next_y_vals.push_back(py);
  }

  if (remain > 0) {
    ref_s = this->end_path_s;
    ref_d = this->end_path_d;
  }

  for (int i=0; i<num; ++i) {
    double new_s = ref_s + s_diff * (i+1);
    next_s_vals.push_back(new_s);
    next_d_vals.push_back(ref_d);
  }

  for (int i=0; i<num; ++i) {
    auto xy = helper::getXY(next_s_vals[i], next_d_vals[i], this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
    this->next_x_vals.push_back(xy[0]);
    this->next_y_vals.push_back(xy[1]);
  }

}