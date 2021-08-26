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

void plan(vector<double> &next_x_vals, vector<double> &next_y_vals,
          egoVehicle &ego) {
  double dist_inc = 0.5;
  for (int i = 0; i < 50; i++) {
    next_x_vals.push_back(ego.car_x +
                          (dist_inc * i) * cos(deg2rad(ego.car_yaw)));
    next_y_vals.push_back(ego.car_y +
                          (dist_inc * i) * sin(deg2rad(ego.car_yaw)));
  }
}