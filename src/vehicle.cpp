#include "vehicle.hpp"
#include "helper.hpp"

void Vehicle::Init()
{
  this->update_count = 0;
}

void Vehicle::Update(json msg, double timestamp)
{
  this->prev_timestamp = this->timestamp;
  this->timestamp = timestamp;
  double duration = timestamp - this->prev_timestamp;

  double car_x = msg[1]["x"];
  double car_y = msg[1]["y"];
  double car_s = msg[1]["s"];
  double car_d = msg[1]["d"];
  double yaw = msg[1]["yaw"];
  yaw = helper::deg2rad(yaw);
  double mph = msg[1]["speed"];
  double speed = convert_mph_ms(mph);
  // Previous path's end s and d values
  double end_path_s = msg[1]["end_path_s"];
  double end_path_d = msg[1]["end_path_d"];
  // Previous path data given to the Planner
  auto previous_path_x = msg[1]["previous_path_x"].get<vector<double>>();
  auto previous_path_y = msg[1]["previous_path_y"].get<vector<double>>();
  // Sensor fusion data.
  auto sensor_fusion = msg[1]["sensor_fusion"].get<vector<vector<double>>>();
  if (update_count == 1)
  {
    this->prev_car_s = this->car_s;
    this->prev_car_d = this->car_d;
    this->car_s_d = (car_s - prev_car_s) / duration;
    this->car_d_d = (car_d - prev_car_d) / duration;
  }
  if (update_count > 1) {
    this->prev_car_s = this->car_s;
    this->prev_car_d = this->car_d;
    this->prev_car_s_d = this->car_s_d;
    this->prev_car_d_d = this->car_d_d;
    this->car_s_d = (car_s - prev_car_s) / duration;
    this->car_d_d = (car_d - prev_car_d) / duration;

    this->car_s_dd = (car_s_d - prev_car_s_d) / duration;
    this->car_d_dd = (car_d_d - prev_car_d_d) / duration;
  }

  this->car_x = car_x;
  this->car_y = car_y;
  this->car_s = car_s;
  this->car_d = car_d;
  this->car_yaw = yaw;
  this->car_speed = speed;
  this->end_path_d = end_path_d;
  this->end_path_s = end_path_s;
  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
  this->sensor_fusion = sensor_fusion;
  ++(this->update_count);
}

void Vehicle::Next()
{
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

  if (remain > 0)
  {
    ref_s = this->end_path_s;
    ref_d = this->end_path_d;
  }

  for (int i = 0; i < num; ++i)
  {
    double new_s = ref_s + s_diff * (i + 1);
    next_s_vals.push_back(new_s);
    next_d_vals.push_back(ref_d);
  }

  for (int i = 0; i < num; ++i)
  {
    auto xy = helper::getXY(next_s_vals[i], next_d_vals[i], this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
    this->next_x_vals.push_back(xy[0]);
    this->next_y_vals.push_back(xy[1]);
  }
}