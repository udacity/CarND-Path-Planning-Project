//
// Created by Joey Liu on 2017/08/04.
//

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "way_points.h"

using namespace std;

WayPoints::WayPoints() {
  ifstream in_map_(map_file_.c_str(), ifstream::in);
  string line;

  while (getline(in_map_, line)) {
    istringstream iss(line);
    WayPoint wp = {0.0, 0.0, 0.0, 0.0, 0.0};
    iss >> wp.x >> wp.y >> wp.s >> wp.dx >> wp.dy;
    way_points.push_back(wp);
  }
}

vector<double> WayPoints::get_spline_xy(double s, double d) {
  double x_mid_road = spline_x_s(s);
  double y_mid_road = spline_y_s(s);
  double dx = spline_dx_s(s);
  double dy = spline_dy_s(s);

  double x = x_mid_road + dx * d;
  double y = y_mid_road + dy * d;

  return {x, y};
}

void WayPoints::fit_spline_segment(double s) {

  if (!way_point_global_segment_s.empty()) way_point_global_segment_s.clear();
  if (!way_point_local_segment_s.empty()) way_point_local_segment_s.clear();

  vector<double> way_point_seg_x, way_point_seg_y, way_point_seg_dx, way_point_seg_dy;
  vector<int> way_point_ids;
  const int lower_way_point_id = 9;
  const int upper_way_point_id = 20;

  int prev_way_point_id = -1;

  while (s > way_points[prev_way_point_id + 1].s && (prev_way_point_id < (int)(way_points.size() - 1))) {
    prev_way_point_id++;
  }

  for (int i = lower_way_point_id; i > 0; i--) {
    if (prev_way_point_id - i < 0) {
      way_point_ids.push_back((int)way_points.size() + (prev_way_point_id - i));
    } else {
      way_point_ids.push_back((prev_way_point_id - i)%(int)way_points.size());
    }
  }

  way_point_ids.push_back(prev_way_point_id);

  for (int i = 1; i < upper_way_point_id; i++) {
    way_point_ids.push_back((prev_way_point_id + i)%(int)way_points.size());
  }

  bool crossed_through_zero = false;
  double seg_start_s = way_points[way_point_ids[0]].s;

  for (int i = 0; i < way_point_ids.size(); i++) {
    int cur_id = way_point_ids[i];
    way_point_seg_x.push_back(way_points[cur_id].x);
    way_point_seg_y.push_back(way_points[cur_id].y);
    way_point_seg_dx.push_back(way_points[cur_id].dx);
    way_point_seg_dy.push_back(way_points[cur_id].dy);

    if (i > 0 && cur_id < way_point_ids[i - 1]) {
      crossed_through_zero = true;
    }

    way_point_global_segment_s.push_back(way_points[cur_id].s);
    if (crossed_through_zero) {
      way_point_local_segment_s.push_back(abs(seg_start_s - max_s_) + way_points[cur_id].s);
    } else {
      way_point_local_segment_s.push_back(way_points[cur_id].s - seg_start_s);
    }
  }

  spline_x_s.set_points(way_point_local_segment_s, way_point_seg_x);
  spline_y_s.set_points(way_point_local_segment_s, way_point_seg_y);
  spline_dx_s.set_points(way_point_local_segment_s, way_point_seg_dx);
  spline_dy_s.set_points(way_point_local_segment_s, way_point_seg_dy);
}

double WayPoints::get_local_s(double global_s) {
  int prev_way_point_id = 0;
  if (way_point_global_segment_s[0] > global_s) {
    while (way_point_global_segment_s[prev_way_point_id] != 0.0) {
      prev_way_point_id++;
    }
  }

  while ((way_point_global_segment_s[prev_way_point_id + 1] < global_s) && (way_point_global_segment_s[prev_way_point_id + 1] != 0)) {
    prev_way_point_id++;
  }

  double diff_s_global = global_s - way_point_global_segment_s[prev_way_point_id];
  return way_point_local_segment_s[prev_way_point_id] + diff_s_global;
}