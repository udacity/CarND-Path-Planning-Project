#include "spline.h"

struct egoVehicle {
  // Main car's localization Data
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;

  // Previous path data given to the Planner
  vector<double> previous_path_x;
  vector<double> previous_path_y;
};

struct mapWaypoints {
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> dx;
  vector<double> dy;
};

struct points {
  vector<double> x;
  vector<double> y;
};

// start in lane 1
int lane = 1;

// reference veocity to target
double ref_val = 49.5;

void straight(points &nextPoints, const egoVehicle &ego) {
  double dist_inc = 0.5;
  for (int i = 0; i < 50; i++) {
    nextPoints.x.push_back(ego.car_x +
                           (dist_inc * i) * cos(deg2rad(ego.car_yaw)));
    nextPoints.y.push_back(ego.car_y +
                           (dist_inc * i) * sin(deg2rad(ego.car_yaw)));
  }
}

void stayInLane(points &nextPoints, const egoVehicle &ego,
                const mapWaypoints &map) {
  // car is to fast
  double dist_inc = 0.3;
  for (int i = 0; i < 50; i++) {
    double next_s = ego.car_s + (i + 1) * dist_inc;
    // middle of middle lane. (lane_width=4m)
    double next_d = 6;

    auto xy = getXY(next_s, next_d, map.s, map.x, map.y);

    nextPoints.x.push_back(xy[0]);
    nextPoints.y.push_back(xy[1]);
  }
}

void stayInLaneWithSpline(points &nextPoints, const egoVehicle &ego,
                          const mapWaypoints &map) {
  // anchor points
  points anchorPoints;
  int prev_size = ego.previous_path_x.size();

  // reference x,y,yaw state
  // either we will reference the starting point as where the car is or at the
  // previous paths end point
  double ref_x = ego.car_x;
  double ref_y = ego.car_y;
  double ref_yaw = deg2rad(ego.car_yaw);

  // if previous size is almost empty, use the car as starting reference
  if (prev_size < 2) {
    // use two points that make the path tnangent to the car
    double prev_car_x = ego.car_x - cos(ego.car_yaw);
    double prev_car_y = ego.car_y - sin(ego.car_yaw);

    anchorPoints.x.push_back(prev_car_x);
    anchorPoints.x.push_back(ego.car_x);

    anchorPoints.y.push_back(prev_car_y);
    anchorPoints.y.push_back(ego.car_y);
  }
  // use the previous path"s end point end point as starting reference
  else {
    // Redefine reference state as previous path end point
    ref_x = ego.previous_path_x[prev_size - 1];
    ref_y = ego.previous_path_y[prev_size - 1];

    double ref_x_prev = ego.previous_path_x[prev_size - 2];
    double ref_y_prev = ego.previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // Use two points that make the path tangent to the previous path's end
    // point
    anchorPoints.x.push_back(ref_x_prev);
    anchorPoints.x.push_back(ref_x);

    anchorPoints.y.push_back(ref_y_prev);
    anchorPoints.y.push_back(ref_y);
  }

  // In frenet add evently 30m spaced  points ahead of the starting reference
  vector<double> next_wp0 =
      getXY(ego.car_s + 30, (2 + 4 * lane), map.s, map.x, map.y);
  vector<double> next_wp1 =
      getXY(ego.car_s + 60, (2 + 4 * lane), map.s, map.x, map.y);
  vector<double> next_wp2 =
      getXY(ego.car_s + 90, (2 + 4 * lane), map.s, map.x, map.y);

  anchorPoints.x.push_back(next_wp0[0]);
  anchorPoints.x.push_back(next_wp1[0]);
  anchorPoints.x.push_back(next_wp2[0]);

  anchorPoints.y.push_back(next_wp0[1]);
  anchorPoints.y.push_back(next_wp1[1]);
  anchorPoints.y.push_back(next_wp2[1]);

  for (int i = 0; i < anchorPoints.x.size(); i++) {
    // shift car refernce angle to 0 degrees
    double shift_x = anchorPoints.x[i] - ref_x;
    double shift_y = anchorPoints.y[i] - ref_y;

    anchorPoints.x[i] =
        (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    anchorPoints.y[i] =
        (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // create a spline
  tk::spline s;
  // set (x,y) points to the spline
  s.set_points(anchorPoints.x, anchorPoints.y);

  // define the actial (x,y) points wewill use for the planer
  // start with all of the previous path points from last time
  for (int i = 0; i < ego.previous_path_x.size(); i++) {
    nextPoints.x.push_back(ego.previous_path_x[i]);
    nextPoints.y.push_back(ego.previous_path_y[i]);
  }

  // calculate how to break up spline points so that we travel at our desired
  // reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

  double x_add_on = 0;

  // Fill up the rest of our path planner after filling it with previous points,
  // here we will always output 50 points
  for (int i = 1; i <= 50 - ego.previous_path_x.size(); i++) {
    double N = (target_dist / (0.02 * ref_val / 2.24));
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to norma lafter rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    nextPoints.x.push_back(x_point);
    nextPoints.y.push_back(y_point);
  }
}