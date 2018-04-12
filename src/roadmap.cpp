#include "roadmap.hpp"
#include "helper.hpp"
#include <iostream>
#include <cassert>
#include <fstream>

using namespace std;


// Returns interporated waypoints_x, waypoints_y, waypoints_s
void RoadMap::Interporate(int start, int end) {
  vector<pair<double, double>> xy_pairs;

  for (int i = start; i < end; ++i)
  {
    double x = this->map_waypoints_x[i];
    double y = this->map_waypoints_y[i];
    xy_pairs.push_back({x, y});
  }

  // cout << "xy_pairs.size()" << xy_pairs.size() << endl;
  // spline requires x value sorted.
  sort(xy_pairs.begin(), xy_pairs.end());

  vector<double> ptsx;
  vector<double> ptsy;
  vector<double> ptss;

  tk::spline spl;
  for (int i = 0; i < xy_pairs.size(); ++i)
  {
    auto pair = xy_pairs[i];
    auto v1 = pair.first;
    if (i > 0)
    {
      auto prev_v1 = ptsx[i - 1];
      if (v1 <= prev_v1)
      {
        // spline will reject if dx = 0;
        v1 = prev_v1 + 0.0001;
      }
    }
    ptsx.push_back(v1);
    ptsy.push_back(pair.second);
  }
  spl.set_points(ptsx, ptsy);

  // Interpolate waypoints.
  ptsx.clear();
  ptsy.clear();
  ptsx.push_back(map_waypoints_x[start]);
  ptsy.push_back(map_waypoints_y[start]);
  int interp_num = 4;
  for (int i = start+1; i < end; ++i) {
    auto x = map_waypoints_x[i];
    auto y = map_waypoints_y[i];
    auto prev_x = map_waypoints_x[i-1];
    double diff = (x - prev_x) / (double)interp_num;

    for (int j=1; j<interp_num; j++) {
      double x_interp = prev_x + diff * (double)j;
      double y_interp = spl(x_interp);
      ptsx.push_back(x_interp);
      ptsy.push_back(y_interp);
    }
    ptsx.push_back(x);
    ptsy.push_back(y);
  }
  // Calculate interpolated s.
  double s = 0;
  if (start > 0) {
    double prev_x = this->interp_waypoints_x.back();
    double prev_y = this->interp_waypoints_y.back();
    double x = ptsx[0];
    double y = ptsy[0];
    double xd = x - prev_x;
    double yd = y - prev_y;

    double prev_s = this->interp_waypoints_s.back();
    // cout << "prev_x: " << prev_x << endl;
    // cout << "prev_y: " << prev_y << endl;
    // cout << "prev_s: " << prev_s << endl;
    double sd = sqrt(xd*xd + yd*yd);
    s = prev_s + sd;
  }
  ptss.push_back(s);
  for (int i = 1; i < ptsx.size(); ++i) {
    auto x = ptsx[i];
    auto x_prev = ptsx[i-1];
    auto y = ptsy[i];
    auto y_prev = ptsy[i-1];
    
    auto xd = x - x_prev;
    auto yd = y - y_prev;
    auto sd = sqrt(xd * xd + yd * yd);
    s = s + sd;
    ptss.push_back(s);
  }
  // cout << "ptsx.size(): " << ptsx.size() << endl;
  // cout << "ptsy.size(): " << ptsy.size() << endl;
  // cout << "ptss.size(): " << ptss.size() << endl;
  for (int i=0; i<ptsx.size(); ++i) {
    double x = ptsx[i];
    double y = ptsy[i];
    double s = ptss[i];
    this->interp_waypoints_x.push_back(x);
    this->interp_waypoints_y.push_back(y);
    this->interp_waypoints_s.push_back(s);
  }
}

void RoadMap::Init(vector<double> map_waypoints_x,
                   vector<double> map_waypoints_y)
{
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;

  int start = 0;
  int batch_size = 16;
  while (start + batch_size < this->map_waypoints_x.size()) {
    this->Interporate(start, start + batch_size);
    start += batch_size;
  }
  this->Interporate(start, this->map_waypoints_x.size());

}

vector<double> RoadMap::getXY(double s, double d)
{
  return helper::getXY(s, d, this->interp_waypoints_s, this->interp_waypoints_x, this->interp_waypoints_y);
}

void RoadMap::DumpMap()
{
  ofstream myfile;
  myfile.open ("map-dump.txt");

  for (int i=0; i<this->interp_waypoints_x.size(); ++i) {
    auto x = this->interp_waypoints_x[i];
    auto y = this->interp_waypoints_y[i];
    auto s = this->interp_waypoints_s[i];
    
    myfile << x << " " << y << " " << s << endl;
  }

  myfile.close();
}