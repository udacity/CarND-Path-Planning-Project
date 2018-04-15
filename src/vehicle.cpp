#include "vehicle.hpp"
#include "helper.hpp"

void Vehicle::Init()
{
  this->update_count = 0;
  this->car_x_d = 0;
  this->car_x_dd = 0;
  this->car_y_d = 0;
  this->car_y_dd = 0;
  this->lane = 1;
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

  double prev_car_x = 0;
  double prev_car_y = 0;
  double prev_car_yaw = 0;
  double prev_car_speed = 0;

  double prev_car_x_d = 0;
  double prev_car_y_d = 0;

  double car_x_d = speed * cos(car_yaw);
  double car_y_d = speed * sin(car_yaw);
  double car_x_dd = 0;
  double car_y_dd = 0;

  if (update_count > 0) {
    prev_car_x = this->car_x;
    prev_car_y = this->car_y;
    prev_car_yaw = this->car_yaw;
    prev_car_speed = this->car_speed;
    prev_car_x_d = this->car_x_d;
    prev_car_y_d = this->car_y_d;
    car_x_dd = (car_x_d - prev_car_x_d) / (duration / 1000);
    car_y_dd = (car_y_d - prev_car_y_d) / (duration / 1000);
  }

  this->car_x = car_x;
  this->car_y = car_y;

  this->car_x_d = car_x_d;
  this->car_y_d = car_y_d;

  this->prev_car_x_d = prev_car_x_d;
  this->prev_car_y_d = prev_car_y_d;

  this->car_x_dd = car_x_dd;
  this->car_y_dd = car_y_dd;

  this->car_s = car_s;
  this->car_d = car_d;
  
  this->car_yaw = yaw;
  this->prev_car_yaw = prev_car_yaw;
  
  this->prev_car_speed = this->car_speed;
  this->car_speed = speed;
  
  this->end_path_d = end_path_d;
  this->end_path_s = end_path_s;

  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
  this->sensor_fusion = sensor_fusion;
  ++(this->update_count);

  this->prev_acc = this->acc;
  this->UpdateNearestVehicles();

}

void Vehicle::UpdateNearestVehicles()
{
  for (auto &f : this->nearest_vehicles_front) {
    f.second.clear();
  }
  for (auto &r : this->nearest_vehicles_rear) {
    r.second.clear();
  }

  for (auto sf : this->sensor_fusion) {
    OtherVehicle ov;
    ov.Init(sf);
    double ov_lane = 0;
    for (int lane=0; lane<3; ++lane) {
      double lane_d = 2. + 4 * lane;
      if (lane_d - 2 <= ov.d && ov.d < lane_d + 2) {
        ov_lane = lane;
        break;
      }
    }
    double distance = ov.s - this->car_s;
    if (distance > 0)
    {
      this->nearest_vehicles_front[ov_lane].push_back(ov);
    }
    else
    {
      this->nearest_vehicles_rear[ov_lane].push_back(ov);
    }
  }

  // Sort by S.
  for (auto &fvs : nearest_vehicles_front) {
    sort(fvs.second.begin(), fvs.second.end(), sortByS);
  }

  for (auto &rvs : nearest_vehicles_rear) {
    sort(rvs.second.begin(), rvs.second.end(), sortBySDSC);
  }
}

void Vehicle::Next()
{
  this->next_x_vals.clear();
  this->next_y_vals.clear();
  int max_num = 10;
  double s_diff = 0.05;
  double d_diff = 0;
  vector<double> next_s_vals;
  vector<double> next_d_vals;
  double ref_s = this->car_s;
  double ref_d = this->car_d;

  int remain = previous_path_x.size();
  double num = max_num - remain;
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
    auto xy = this->roadmap.getXY(next_s_vals[i], next_d_vals[i]);
    this->next_x_vals.push_back(xy[0]);
    this->next_y_vals.push_back(xy[1]);
  }
}

void Vehicle::NextHybrid2()
{
  cout << "NextHybrid2()" << endl;
  this->next_x_vals.clear();
  this->next_y_vals.clear();

  int max_num = 20;
  int remain = this->previous_path_x.size();

  double ref_x = this->car_x;
  double ref_y = this->car_y;
  double ref_x_prev = 0;
  double ref_y_prev = 0;
  double ref_yaw = this->car_yaw;
  double ref_s = this->car_s;
  double ref_d = 2 + this->lane * 4;
  double t = 0.02;

  if (remain > 0) {
    for (int i=0; i<remain; ++i) {
      this->next_x_vals.push_back(this->previous_path_x[i]);
      this->next_y_vals.push_back(this->previous_path_y[i]);
    }
    ref_s = this->end_path_s;
    ref_d = this->end_path_d;
    ref_x = this->previous_path_x.back();
    ref_y = this->previous_path_y.back();
    if (remain > 1) {
      ref_x_prev = previous_path_x[previous_path_x.size()-2];
      ref_y_prev = previous_path_y[previous_path_y.size()-2];
    } else {
      ref_x_prev = this->car_x;
      ref_y_prev = this->car_y;
    }
    double delta_x = helper::nonzero(ref_x - ref_x_prev, 0.001);
    double delta_y = helper::nonzero(ref_y - ref_y_prev, 0.001);
    ref_yaw = atan2(delta_y, delta_x);
  }


  vector<pair<double, double>> pts;
  if (remain > 0) {
    pts.push_back({ref_x_prev, ref_y_prev});
  }
  pts.push_back({ref_x, ref_y});
  
  vector<double> next_s_points = {
    ref_s + 15,
    ref_s + 30,
    ref_s + 60,
    ref_s + 90
  };

  double center_error = 2. + lane * 4. - this->car_d;
  double rev_center = this->center_PID.Update(center_error) * 0.1;
  cout << "rev_center: " << rev_center << endl;
  ref_d += rev_center;

  for (auto ns : next_s_points) {
    auto xy = this->roadmap.getXY(ns, ref_d);
    pts.push_back({xy[0], xy[1]});
  }

  vector<pair<double, double>> pts_veh;
  for (auto pt : pts) {
    auto pt_veh = helper::convertToVehicleCoordinate(pt, ref_x, ref_y, ref_yaw);
    pts_veh.push_back(pt_veh);
  }

  sort(pts_veh.begin(), pts_veh.end());

  vector<double> ptsx;
  vector<double> ptsy;
  double prevx = -10000;
  for (auto pt_veh : pts_veh) {
    double x = pt_veh.first;
    // Spline rejects if dx = 0;
    if (pt_veh.first == prevx) {
      x += 0.001;
    }
    prevx = x;
    ptsx.push_back(x);
    ptsy.push_back(pt_veh.second);
  }
  tk::spline spl;
  spl.set_points(ptsx, ptsy);

  bool too_close = false;
  auto front_veh = this->nearest_vehicles_front[lane];
  double dist = 10000;
  if (front_veh.size() > 0) {
    dist = front_veh[0].s - this->car_s;
    if (dist < 30) {
      too_close = true;
    }
  }

  double _ref_speed = this->car_speed;

  if (too_close) {
    double pid_ret = this->front_dist_PID.Update(30 - dist);
    cout << "pid_ret: " << pid_ret << endl;
    double ds = pid_ret * 0.5 * -1.;
    if (ds < -0.5) {
      ds = -0.5;
    }
    if (ds > 0.5) {
      ds = 0.5;
    }
    if (_ref_speed + ds > 0) {
      _ref_speed += ds;
    }
  } else {
    if (_ref_speed > Vehicle::target_speed) {
      _ref_speed -= 0.5;
    } else {
      _ref_speed += 0.5;
    }
  }

  cout << "too close: " << too_close << endl;
  cout << "ref_speed: " << _ref_speed << endl;

  double target_x = 30;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double N = target_dist / (0.02 * _ref_speed);
  double x_diff = target_x / N;
  double x_step = 0;
  for (int i=0; i < max_num - remain; ++i) {
    x_step += x_diff;
    double y_step = spl(x_step);
    
    auto xy = helper::convertToMapCoordinate({x_step, y_step}, ref_x, ref_y, ref_yaw);
    this->next_x_vals.push_back(xy.first);
    this->next_y_vals.push_back(xy.second);
  }
}

void Vehicle::PrintPath()
{
  for (auto x : this->next_x_vals)
  {
    cout << "x: " << x << std::endl;
  }
  for (auto y : this->next_y_vals)
  {
    cout << "y: " << y << std::endl;
  }
}

void Vehicle::PrintState()
{
  cout << "============" << endl;
  cout << "car_x: " << car_x << endl;
  cout << "car_y: " << car_y << endl;
  cout << "car_yaw: " << car_yaw << endl;
  cout << "car_s: " << car_s << endl;
  cout << "car_d: " << car_d << endl;
  cout << "============" << endl;
}

void Vehicle::PrintNearest()
{
  cout << "============" << endl;
  for (auto kv : nearest_vehicles_front) {
    cout << "[Front] lane: " << kv.first;
    if (kv.second.size() > 0) {
      cout << " s: " << kv.second[0].s;
      cout << " d: " << kv.second[0].d;
      cout << " x: " << kv.second[0].x;
      cout << " y: " << kv.second[0].y;
      cout << " dist: " << kv.second[0].s - this->car_s;
    }
    cout << endl;
  }
  for (auto kv : nearest_vehicles_rear) {
    cout << "[Rear] lane: " << kv.first;
    if (kv.second.size() > 0) {
      cout << " s: " << kv.second[0].s;
      cout << " d: " << kv.second[0].d;
      cout << " x: " << kv.second[0].x;
      cout << " y: " << kv.second[0].y;
      cout << " dist: " << this->car_s - kv.second[0].s;
    }
    cout << endl;
  }
  cout << "============" << endl;
}