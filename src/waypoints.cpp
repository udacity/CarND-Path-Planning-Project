//
// Created by Joey Liu on 2017/08/04.
//

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "waypoints.h"
#include "utils.h"

using namespace std;

WayPoints::WayPoints() {
  max_s_ = 6945.554;

  string map_file_ = "../data/highway_map.csv";

  ifstream in_map_(map_file_.c_str(), ifstream::in);
  string line;

  while (getline(in_map_, line)) {
    istringstream iss(line);
    WayPoint wp = {0.0, 0.0, 0.0, 0.0, 0.0};
    iss >> wp.x >> wp.y >> wp.s >> wp.dx >> wp.dy;
    wps_.push_back(wp);
  }

  cout << "completed initializing wayPoints" << endl;

}

WayPoints::~WayPoints() {}

int WayPoints::ClosestWayPoint(double x, double y) {
  double closestLen = 100000;
  int closestWaypoint = 0;

  for(int i = 0; i < wps_.size(); i++)
  {
    double map_x = wps_[i].x;
    double map_y = wps_[i].y;
    double dist = distance(x, y, map_x ,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int WayPoints::NextWayPoint(double x, double y, double theta) {
  int closestWayPoint = ClosestWayPoint(x,y);

  double map_x = wps_[closestWayPoint].x;
  double map_y = wps_[closestWayPoint].y;

  double heading = atan2((map_y - y),(map_x - x));

  double angle = abs(theta - heading);

  if(angle > M_PI/4)
  {
    closestWayPoint++;
  }

  return closestWayPoint;
}

int WayPoints::PrevWayPoint(int i) {
  return i == 0? wps_.size() - 1:i - 1;
}

vector<double> WayPoints::getFrenet(double x, double y, double theta) {
  int next_wp_id = NextWayPoint(x, y, theta);
  int prev_wp_id = PrevWayPoint(next_wp_id);

  WayPoint& next_wp = wps_[next_wp_id];
  WayPoint& prev_wp = wps_[prev_wp_id];

  double n_x = next_wp.x - prev_wp.x;
  double n_y = next_wp.y - prev_wp.y;

  double x_x = x - prev_wp.x;
  double x_y = y - prev_wp.y;

  // find the projection of x onto n
  double proj_norm = (x_x*n_x + x_y*n_y)/(n_x * n_x + n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - prev_wp.x;
  double center_y = 2000 - prev_wp.y;
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if(centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp_id; i++) {
    frenet_s += distance(wps_[i].x, wps_[i].y, wps_[i + 1].x, wps_[i + 1].y);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

vector<double> WayPoints::getXY(double s, double d){
  int prev_wp_id = -1;

  while(s > wps_[prev_wp_id + 1].s && (prev_wp_id < (int)(wps_.size() - 1) )) {
    prev_wp_id++;
  }

  int next_wp_id = (prev_wp_id + 1)%wps_.size();

  WayPoint& next_wp = wps_[next_wp_id];
  WayPoint& prev_wp = wps_[prev_wp_id];

  double heading = atan2((next_wp.y - prev_wp.y),(next_wp.x - prev_wp.x));
  // the x,y,s along the segment
  double seg_s = (s - prev_wp.s);

  double seg_x = prev_wp.x + seg_s*cos(heading);
  double seg_y = prev_wp.y + seg_s*sin(heading);

  double perp_heading = heading - M_PI/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x, y};
}

void WayPoints::fit_spline_segment(double s) {
  vector<double> wp_segment_x, wp_segment_y, wp_segment_dx, wp_segment_dy;
  vector<int> wp_ids;
  const int lower_wp_i = 9;
  const int upper_wp_i = 20;

  int prev_wp_id = -1;

  while (s > wps_[prev_wp_id + 1].s && (prev_wp_id < (int)(wps_.size() - 1) )) {
    prev_wp_id++;
  }

  for (int i = lower_wp_i; i > 0; i--) {
    if (prev_wp_id < 0) {
      wp_ids.push_back(wps_.size() + (prev_wp_id - i));
    } else {
      wp_ids.push_back((prev_wp_id - i)%wps_.size());
    }
  }

  wp_ids.push_back(prev_wp_id);

  for (int i = 1; i < upper_wp_i; i++) {
    wp_ids.push_back((prev_wp_id + i) % wps_.size());
  }

  bool crossed_through_zero = false;
  double seg_start_s = wps_[wp_ids[0]].s;

  for (int i = 0; i < wp_ids.size(); i++) {
    int cur_id = wp_ids[i];
    wp_segment_x.push_back(wps_[cur_id].x);
    wp_segment_y.push_back(wps_[cur_id].y);
    wp_segment_dx.push_back(wps_[cur_id].dx);
    wp_segment_dy.push_back(wps_[cur_id].dy);

    if (i > 0 && cur_id < wp_ids[i - 1]) {
      crossed_through_zero = true;
    }

    wp_segment_s_global_.push_back(wps_[cur_id].s);
    if (crossed_through_zero) {
      wp_segment_s_.push_back(abs(seg_start_s - max_s_) + wps_[cur_id].s);
    } else {
      wp_segment_s_.push_back(wps_[cur_id].s - seg_start_s);
    }
  }

  x_spline_.set_points(wp_segment_s_, wp_segment_x);
  y_spline_.set_points(wp_segment_s_, wp_segment_y);
  dx_spline_.set_points(wp_segment_s_, wp_segment_dx);
  dy_spline_.set_points(wp_segment_s_, wp_segment_dy);

}

double WayPoints::world_s2local_s(double world_s) {
  int prev_wp_id = 0;
  if (wp_segment_s_global_[0] > world_s) {
    prev_wp_id++;
  }
  while ((wp_segment_s_global_[prev_wp_id + 1] < world_s) && (wp_segment_s_global_[prev_wp_id + 1] != 0)) {
    prev_wp_id++;
  }

  double diff = world_s - wp_segment_s_global_[prev_wp_id];
  double local_s = wp_segment_s_[prev_wp_id] + diff;
  return local_s;

}