//
// Created by Joey Liu on 2017/08/04.
//

#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <vector>
#include "spline.h"

using namespace std;

class WayPoints {
  struct WayPoint {
  public:
    double x, y, s, dx, dy;
  };
public:
  WayPoints();

  double get_local_s(double world_s);
  vector<double> get_spline_xy(double s, double d);
  void fit_spline_segment(double s);

  vector<WayPoint> way_points;
  vector<double> way_point_local_segment_s;
  vector<double> way_point_global_segment_s;

  tk::spline spline_x_s;
  tk::spline spline_y_s;
  tk::spline spline_dx_s;
  tk::spline spline_dy_s;

private:
  double max_s_ = 6945.554;
  string map_file_ = "../data/highway_map.csv";

};
#endif //PATH_PLANNING_WAYPOINTS_H
