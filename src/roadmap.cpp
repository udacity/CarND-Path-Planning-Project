#include "roadmap.hpp"
#include "helper.hpp"
#include <iostream>

using namespace std;

void RoadMap::Init(vector<double> map_waypoints_x,
                   vector<double> map_waypoints_y,
                   vector<double> map_waypoints_s,
                   vector<double> map_waypoints_dx,
                   vector<double> map_waypoints_dy)
{
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
  this->map_waypoints_s = map_waypoints_s;
  this->map_waypoints_dx = map_waypoints_dx;
  this->map_waypoints_dy = map_waypoints_dy;

  for (int i = 0; i < map_waypoints_x.size(); ++i)
  {
    double x = map_waypoints_x[i];
    double y = map_waypoints_y[i];
    double s = map_waypoints_s[i];
    double dx = map_waypoints_dx[i];
    double dy = map_waypoints_dy[i];
    sx.push_back({s, x});
    sy.push_back({s, y});
    sdx.push_back({s, dx});
    sdy.push_back({s, dy});
  }

  // spline requires x value sorted.
  sort(sx.begin(), sx.end());
  sort(sy.begin(), sy.end());
  sort(sdx.begin(), sdx.end());
  sort(sdy.begin(), sdy.end());

  // Use spline to smoothing the map.
  vector<vector<pair<double, double>>> all_pairs = {sx, sy, sdx, sdy};
  vector<tk::spline*> all_sps = {&sp_sx, &sp_sy, &sp_sdx, &sp_sdy};

  vector<double> pts1;
  vector<double> pts2;
  for (int i = 0; i < all_pairs.size(); ++i)
  {
    pts1.clear();
    pts2.clear();
    auto pairs = all_pairs[i];
    auto spl = all_sps[i];

    for (int i=0; i<pairs.size(); ++i) {
      auto v = pairs[i];
      auto v1 = v.first;
      if (i > 0) {
        auto prev_v1 = pts1[i-1];
        if (v1 <= prev_v1) {
          // spline will reject if dx = 0;
          v1 = prev_v1 + 0.0001;
        }
      }
      pts1.push_back(v1);
      pts2.push_back(v.second);
    }
    spl->set_points(pts1, pts2);
  }
}

vector<double> RoadMap::getXY(double s, double d)
{
  cout << "RoadMap::getXY" << endl;
  double dx = sp_sdx(s);
  double dy = sp_sdy(s);

  double x = sp_sx(s) + d * dx;
  double y = sp_sy(s) + d * dy;
  return {x, y};
}