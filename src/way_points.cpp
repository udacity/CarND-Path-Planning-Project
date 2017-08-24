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
    wps_.push_back(wp);
  }
}

vector<double> WayPoints::getXY_splines(double s, double d) {
  double x_mid_road = spline_x_s_(s);
  double y_mid_road = spline_y_s_(s);
  double dx = spline_dx_s_(s);
  double dy = spline_dy_s_(s);

  double x = x_mid_road + dx * d;
  double y = y_mid_road + dy * d;

  return {x, y};
}

void WayPoints::fit_spline_segment(double s) {

  if (!wp_global_segment_s_.empty()) wp_global_segment_s_.clear();
  if (!wp_local_segment_s_.empty()) wp_local_segment_s_.clear();

  vector<double> wp_seg_x, wp_seg_y, wp_seg_dx, wp_seg_dy;
  vector<int> wp_ids;
  const int lower_wp_i = 9;
  const int upper_wp_i = 20;

  int prev_wp_id = -1;

  while (s > wps_[prev_wp_id + 1].s && (prev_wp_id < (int)(wps_.size() - 1))) {
    prev_wp_id++;
  }

  for (int i = lower_wp_i; i > 0; i--) {
    if (prev_wp_id - i < 0) {
      wp_ids.push_back((int)wps_.size() + (prev_wp_id - i));
    } else {
      wp_ids.push_back((prev_wp_id - i)%(int)wps_.size());
    }
  }

  wp_ids.push_back(prev_wp_id);

  for (int i = 1; i < upper_wp_i; i++) {
    wp_ids.push_back((prev_wp_id + i)%(int)wps_.size());
  }

  bool crossed_through_zero = false;
  double seg_start_s = wps_[wp_ids[0]].s;

  for (int i = 0; i < wp_ids.size(); i++) {
    int cur_id = wp_ids[i];
    wp_seg_x.push_back(wps_[cur_id].x);
    wp_seg_y.push_back(wps_[cur_id].y);
    wp_seg_dx.push_back(wps_[cur_id].dx);
    wp_seg_dy.push_back(wps_[cur_id].dy);

    if (i > 0 && cur_id < wp_ids[i - 1]) {
      crossed_through_zero = true;
    }

    wp_global_segment_s_.push_back(wps_[cur_id].s);
    if (crossed_through_zero) {
      wp_local_segment_s_.push_back(abs(seg_start_s - max_s_) + wps_[cur_id].s);
    } else {
      wp_local_segment_s_.push_back(wps_[cur_id].s - seg_start_s);
    }
  }

  spline_x_s_.set_points(wp_local_segment_s_, wp_seg_x);
  spline_y_s_.set_points(wp_local_segment_s_, wp_seg_y);
  spline_dx_s_.set_points(wp_local_segment_s_, wp_seg_dx);
  spline_dy_s_.set_points(wp_local_segment_s_, wp_seg_dy);
}

double WayPoints::get_local_s(double global_s) {
  int prev_wp_id = 0;
  if (wp_global_segment_s_[0] > global_s) {
    while (wp_global_segment_s_[prev_wp_id] != 0.0) {
      prev_wp_id++;
    }
  }

  while ((wp_global_segment_s_[prev_wp_id + 1] < global_s) && (wp_global_segment_s_[prev_wp_id + 1] != 0)) {
    prev_wp_id++;
  }

  double diff_s_global = global_s - wp_global_segment_s_[prev_wp_id];
  return wp_local_segment_s_[prev_wp_id] + diff_s_global;
}