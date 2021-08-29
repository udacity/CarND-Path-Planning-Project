struct egoVehicle {
  // Main car's localization Data
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

  // Previous path data given to the Planner
  // auto previous_path_x ;
  // auto previous_path_y ;
  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;
};

struct mapWaypoints {
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> dx;
  vector<double> dy;
};

void straight(vector<double> &next_x_vals, vector<double> &next_y_vals,
              const egoVehicle &ego) {
  double dist_inc = 0.5;
  for (int i = 0; i < 50; i++) {
    next_x_vals.push_back(ego.car_x +
                          (dist_inc * i) * cos(deg2rad(ego.car_yaw)));
    next_y_vals.push_back(ego.car_y +
                          (dist_inc * i) * sin(deg2rad(ego.car_yaw)));
  }
}

void stayInLane(vector<double> &next_x_vals, vector<double> &next_y_vals,
                const egoVehicle &ego, const mapWaypoints &map) {
  // car is to fast
  double dist_inc = 0.3;
  for (int i = 0; i < 50; i++) {
    double next_s = ego.car_s + (i + 1) * dist_inc;
    // middle of middle lane. (lane_width=4m)
    double next_d = 6;

    auto xy = getXY(next_s, next_d, map.s, map.x, map.y);

    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
  }
}