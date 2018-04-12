#include "roadmap.hpp"
#include "helper.hpp"
#include <iostream>
#include <cassert>
#include <fstream>

using namespace std;

void RoadMap::Init(vector<double> map_waypoints_x,
                   vector<double> map_waypoints_y,
                   vector<double> map_waypoints_s)
{
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
  this->map_waypoints_s = map_waypoints_s;
}

vector<double> RoadMap::getXY(double s, double d)
{
  return helper::getXY(s, d, this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
}