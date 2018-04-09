#include "vehicle.hpp"
#include "helper.hpp"

void Vehicle::Update(json msg) {
  this->car_x = msg[1]["x"];
  this->car_y = msg[1]["y"];
  this->car_s = msg[1]["s"];
  this->car_d = msg[1]["d"];
  this->car_yaw = msg[1]["yaw"];
  this->car_speed = msg[1]["speed"];
  // Previous path's end s and d values 
  this->end_path_s = msg[1]["end_path_s"];
  this->end_path_d = msg[1]["end_path_d"];
  // Previous path data given to the Planner
  this->previous_path_x = msg[1]["previous_path_x"].get<vector<double>>();
  this->previous_path_y = msg[1]["previous_path_y"].get<vector<double>>();
  // Sensor fusion data.
  this->sensor_fusion = msg[1]["sensor_fusion"].get<vector<vector<double>>>();
}

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