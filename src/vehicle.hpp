#ifndef PPP_VEHICLE
#define PPP_VEHICLE

#include <vector>
#include "json.hpp"
#include "roadmap.hpp"
#include "other_vehicle.hpp"
#include "pid.hpp"

using namespace std;
using json = nlohmann::json;

static constexpr double mile_in_meter = 1609.344;

constexpr inline double convert_mph_ms(double mph) {
  return (mph * mile_in_meter) / 3600.;  
}

constexpr inline double convert_ms_mph(double ms) {
  return (ms / mile_in_meter) * 3600.;
}

enum State {
  KL,
  LCL,
  LCR
};

class Vehicle {
  public:
  double car_x;
  double car_y;
  double car_yaw;
  double car_s;
  double car_d;

  double car_x_d;
  double car_x_dd;

  double car_y_d;
  double car_y_dd;

  double prev_car_x;
  double prev_car_y;
  double prev_car_yaw;

  double prev_car_x_d;
  double prev_car_y_d;

  double timestamp;
  double prev_timestamp;

  int update_count;
  int lane;

  // meter/s
  double car_speed;
  double prev_car_speed;

  // meter/s
  static constexpr double target_speed = convert_mph_ms(45.);
  // meter/s^2
  double acc;
  double prev_acc;

  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;

  vector<vector<double>> sensor_fusion;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;

  static constexpr int vals_num = 20;
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  RoadMap roadmap;
  map<int, vector<OtherVehicle>> nearest_vehicles_front;
  map<int, vector<OtherVehicle>> nearest_vehicles_rear;

  PID front_dist_PID = PID(0.4, 0.05, 0.01);
  PID center_PID = PID(0.5, 0.1, 0.05);

  bool too_close = false;
  State state = State::KL;
  State prev_state = State::KL;

  void Init();
  void Update(json msg, double timestamp);
  void UpdateNearestVehicles();
  void Next();
  void NextKL();
  void NextLC();
  void SetState(State newState);

  bool LaneChangeAvailable(int lane);

  void PrintPath();
  void PrintState();
  void PrintNearest();
  double TargetD() { return 2. + 4. * this->lane; };

};

#endif