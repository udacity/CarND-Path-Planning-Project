//
// Created by Joey Liu on 2017/08/04.
//

#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <vector>
#include "WayPoint.h"
#include "spline.h"

using namespace std;

class WayPoints {
public:
  WayPoints();

  double get_local_s(double world_s);
  vector<double> getXY_splines(double s, double d);
  void fit_spline_segment(double s);

  vector<WayPoint> wps_;
  vector<double> wp_local_segment_s_;
  vector<double> wp_global_segment_s_;

  tk::spline spline_x_s_;
  tk::spline spline_y_s_;
  tk::spline spline_dx_s_;
  tk::spline spline_dy_s_;

private:
  double max_s_ = 6945.554;
  string map_file_ = "../data/highway_map.csv";

};
#endif //PATH_PLANNING_WAYPOINTS_H
