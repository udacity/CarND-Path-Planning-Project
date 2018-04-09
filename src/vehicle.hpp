#ifndef PPP_VEHICLE
#define PPP_VEHICLE

#include <vector>
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

static constexpr double mile_in_meter = 1609.344;

constexpr inline double convert_mph_ms(double mph) {
  return (mph * mile_in_meter) / 3600.;  
}

constexpr inline double convert_ms_mph(double ms) {
  return (ms / mile_in_meter) * 3600.;
}

class Vehicle {
  public:
  double car_x;
  double car_y;
  double car_yaw;
  double car_s;
  double car_d;
  // meter/s
  double car_speed;
  // meter/s
  static constexpr double target_speed = convert_mph_ms(50.);

  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;

  vector<vector<double>> sensor_fusion;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  void Update(json msg);
  void Next();
};

#endif