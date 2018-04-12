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

  void Init(vector<double> map_waypoints_x,
            vector<double> map_waypoints_y,
            vector<double> map_waypoints_s);

  vector<double> getXY(double s, double d);

};

#endif