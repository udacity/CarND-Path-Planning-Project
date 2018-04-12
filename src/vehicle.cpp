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

  int max_num = 50;
  int remain = previous_path_x.size();
  int num = max_num - remain;
  double ref_s = this->car_s;
  double ref_d = 6;
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

  double new_s = ref_s + 30;
  double new_d = 6;

  // Assume constant acceleration.
  // Note: acc is calcurated in current frame, but car point is previously calcurated.
  // double speed = ref_speed;

  // for (int i=0; i<max_num; ++i) {
  //   speed += tstep * i * this->acc;
  //   double new_s = new_s + speed * tstep + (this->acc * tstep * tstep) / 2;
  // }
  cout << "new_s: " << new_s << endl;
  cout << "num: " << num << endl;

  vector<double> start_xy = this->roadmap.getXY(ref_s, ref_d);
  vector<double> target_xy = this->roadmap.getXY(new_s, new_d);

  auto start_x = {start_xy[0], this->car_x_d, this->car_x_dd };
  auto target_x = {target_xy[0], 0., 0.};
  auto start_y = {start_xy[1], this->car_y_d, this->car_y_dd };
  auto target_y = {target_xy[1], 0., 0.};

  auto x_coeffs = helper::JMT( start_x, target_x, tstep * max_num);
  auto y_coeffs = helper::JMT( start_y, target_y, tstep * max_num);

  double t = 0;
  for (int i=0; i<num; ++i) {
    double x = 0;
    double y = 0;
    for (int j=0; j<6; ++j) {
      double cx = x_coeffs[j];
      x = x + pow(t, j) * cx;
      double cy = y_coeffs[j];
      y = y + pow(t, j) * cy;
    }
    cout << "x: " << x << endl;
    cout << "y: " << y << endl;
    this->next_x_vals.push_back(x);
    this->next_y_vals.push_back(y);
    t = t + tstep;
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