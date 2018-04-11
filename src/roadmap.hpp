#ifndef PPP_ROADMAP
#define PPP_ROADMAP

#include <vector>
#include "spline.h"

using namespace std;

class RoadMap
{
public:
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  vector<pair<double, double>> sx;
  vector<pair<double, double>> sy;
  vector<pair<double, double>> sdx;
  vector<pair<double, double>> sdy;

  tk::spline sp_sx;
  tk::spline sp_sy;
  tk::spline sp_sdx;
  tk::spline sp_sdy;

  void Init(vector<double> map_waypoints_x,
            vector<double> map_waypoints_y,
            vector<double> map_waypoints_s,
            vector<double> map_waypoints_dx,
            vector<double> map_waypoints_dy);

  vector<double> getXY(double s, double d);
};

#endif