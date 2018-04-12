#include "vehicle.hpp"
#include "helper.hpp"

void Vehicle::Init()
{
  this->update_count = 0;
  this->car_s_d = 0;
  this->car_s_dd = 0;
  this->car_d_d = 0;
  this->car_d_dd = 0;
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
  if (update_count == 1)
  {
    this->prev_car_s = this->car_s;
    this->prev_car_d = this->car_d;
    this->car_s_d = (car_s - prev_car_s) / (duration / 1000);
    this->car_d_d = (car_d - prev_car_d) / (duration / 1000);
  }
  if (update_count > 1) {
    this->prev_car_s = this->car_s;
    this->prev_car_d = this->car_d;
    this->prev_car_s_d = this->car_s_d;
    this->prev_car_d_d = this->car_d_d;
    this->car_s_d = (car_s - prev_car_s) / (duration / 1000);
    this->car_d_d = (car_d - prev_car_d) / (duration / 1000);

    this->car_s_dd = (car_s_d - prev_car_s_d) / (duration / 1000);
    this->car_d_dd = (car_d_d - prev_car_d_d) / (duration / 1000);
  }

  this->car_x = car_x;
  this->car_y = car_y;
  this->car_s = car_s;
  this->car_d = car_d;
  this->car_yaw = yaw;
  
  this->prev_car_speed = this->car_speed;
  this->car_speed = speed;
  
  this->end_path_d = end_path_d;
  this->end_path_s = end_path_s;
  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
  this->sensor_fusion = sensor_fusion;
  ++(this->update_count);

  this->prev_acc = this->acc;
  if (this->car_speed < Vehicle::target_speed) {
    this->acc = 1;
  } else {
    this->acc = 0;
  }

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
    cout << "next s: " << next_s_vals[i] << endl;
    cout << "next d: " << next_d_vals[i] << endl;

    auto xy = this->roadmap.getXY(next_s_vals[i], next_d_vals[i]);
    this->next_x_vals.push_back(xy[0]);
    this->next_y_vals.push_back(xy[1]);
  }
}

void Vehicle::NextJMT()
{
  this->next_x_vals.clear();
  this->next_y_vals.clear();
  int max_num = 20;
  int remain = previous_path_x.size();

  if (remain > 0) {
    for (auto px : previous_path_x)
    {
      this->next_x_vals.push_back(px);
    }
    for (auto py : previous_path_y)
    {
      this->next_y_vals.push_back(py);
    }
    return;
  }

  vector<double> next_s_vals;
  vector<double> next_d_vals;
  double T = 10;
  double N = 20;
  double tstep = T/N;
  double s_diff = 10;
  vector<double> start;
  vector<double> end;

  start = {this->car_s, this->car_s_d, this->car_s_dd};
  end = {this->car_s + s_diff, 0, 0};
  auto coeffs = helper::JMT(start, end, T);
  
  for (int i=0; i<max_num; ++i) {
    double new_s = 0;
    double t = tstep * (i+1);
    for (int j=0; j<coeffs.size(); ++j) {
      new_s += coeffs[j] * pow(t, j);
    }
    cout << "new_s: " << new_s << endl;
    next_s_vals.push_back(new_s);
    next_d_vals.push_back(this->car_d);
  }

  for (int i = 0; i < max_num; ++i)
  {
    auto xy = this->roadmap.getXY(next_s_vals[i], next_d_vals[i]);
    this->next_x_vals.push_back(xy[0]);
    this->next_y_vals.push_back(xy[1]);
  }
}

void Vehicle::NextHybrid()
{
  this->next_x_vals.clear();
  this->next_y_vals.clear();

  int max_num = Vehicle::vals_num;
  int remain = previous_path_x.size();
  int num = max_num - remain;
  double ref_s = this->car_s;
  double ref_d = this->car_d;
  double ref_speed = this->car_speed;

  double tstep = 0.02;
  if (remain > 0) {
    ref_s = this->end_path_s;
    ref_d = this->end_path_d;
    ref_speed = this->prev_car_speed + max_num * tstep * this->prev_acc;
    for (auto px : previous_path_x)
    {
      this->next_x_vals.push_back(px);
    }
    for (auto py : previous_path_y)
    {
      this->next_y_vals.push_back(py);
    }
  }
  
  if (ref_speed > Vehicle::target_speed) {
    ref_speed = Vehicle::target_speed;
    this->acc = 0;
  }

  vector<double> next_s_vals;
  vector<double> next_d_vals;

  // Assume constant acceleration.
  // Note: acc is calcurated in current frame, but car point is previously calcurated.
  double speed = ref_speed;
  for (int i=0; i<max_num; ++i) {
    speed += tstep * i * this->acc;
    double new_s = ref_s + speed * tstep + (this->acc * tstep * tstep) / 2;
    next_s_vals.push_back(new_s);
    next_d_vals.push_back(ref_d);
    ref_s = new_s;
  }

  for (int i=0; i<num; i++) {
    double s = next_s_vals[i];
    double d = next_d_vals[i];
    auto xy = this->roadmap.getXY(s, d);
    this->next_x_vals.push_back(xy[0]);
    this->next_y_vals.push_back(xy[1]);
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