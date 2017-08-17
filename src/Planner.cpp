//
// Created by Joey Liu on 2017/08/16.
//
#include <vector>
#include "Planner.h"
#include "utils.h"

using namespace std;

vector<vector<double>> Planner::plan(CarState car_state, vector<vector<double>> &previous_path, vector<vector<double>> &sensor_fusion) {

  auto prev_size = (int) previous_path[0].size();

  int lane = 1;

  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_state.x;
  double ref_y = car_state.y;
  double ref_yaw = ut::deg2rad(car_state.yaw);

  if (prev_size < 2) {
    double prev_car_x = car_state.x - cos(car_state.yaw);
    double prev_car_y = car_state.y - sin(car_state.yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_state.x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_state.y);
  } else {
    ref_x = previous_path[0][prev_size - 1];
    ref_y = previous_path[1][prev_size - 1];

    double prev_ref_x = previous_path[0][prev_size - 2];
    double prev_ref_y = previous_path[1][prev_size - 2];
    ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

    ptsx.push_back(prev_ref_x);
    ptsx.push_back(ref_x);

    ptsy.push_back(prev_ref_y);
    ptsy.push_back(ref_y);
  }

  vector<double> pred_pos_1 = way_points.getXY(car_state.s + 30, (2 + 4*lane));
  vector<double> pred_pos_2 = way_points.getXY(car_state.s + 60, (2 + 4*lane));
  vector<double> pred_pos_3 = way_points.getXY(car_state.s + 90, (2 + 4*lane));

  ptsx.push_back(pred_pos_1[0]);
  ptsx.push_back(pred_pos_2[0]);
  ptsx.push_back(pred_pos_3[0]);

  ptsy.push_back(pred_pos_1[1]);
  ptsy.push_back(pred_pos_2[1]);
  ptsy.push_back(pred_pos_3[1]);

  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
    ptsy[i] = (shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
  }

  tk::spline s;

  s.set_points(ptsx, ptsy);

  vector<double> next_x;
  vector<double> next_y;

  for (int i = 0; i < prev_size; i++) {
    next_x.push_back(previous_path[0][i]);
    next_y.push_back(previous_path[1][i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for (int i = 1; i <= 50 - prev_size; i++) {
    double N = (target_dist/(.02*ref_vel/2.24));
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x.push_back(x_point);
    next_y.push_back(y_point);
  }

  return {next_x, next_y};
}