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

  int ClosestWayPoint(double x, double y);
  int NextWayPoint(double x, double y, double theta);
  int PrevWayPoint(int i);

  double world_s2local_s(double world_s);

  vector<double> getFrenet(double x, double y, double theta);
  vector<double> getXY(double s, double d);

  void fit_spline_segment(double s);

  vector<WayPoint> wps_;
  vector<double> wp_segment_s_;
  vector<double> wp_segment_s_global_;

private:
  tk::spline x_spline_;
  tk::spline y_spline_;
  tk::spline dx_spline_;
  tk::spline dy_spline_;
  double max_s_;

};
#endif //PATH_PLANNING_WAYPOINTS_H
