#ifndef UTIL_H
#define UTIL_H
#include <iostream>
#include <math.h>
#include <vector>
#include "spline.h"
using namespace std;

namespace util {

  // For converting back and forth between radians and degrees.
  constexpr double pi() { return M_PI; }
  double deg2rad(double x) { return x * pi() / 180; }
  double rad2deg(double x) { return x * 180 / pi(); }

  // Checks if the SocketIO event has JSON data.
  // If there is data the JSON object in string format will be returned,
  // else the empty string "" will be returned.
  string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
      return "";
    } else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 2);
    }
    return "";
  }

  double distance(double x1, double y1, double x2, double y2)
  {
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
  }
  int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
  {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
      {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
          {
            closestLen = dist;
            closestWaypoint = i;
          }

      }

    return closestWaypoint;

  }

  int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
  {

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);

    if(angle > pi()/4)
      {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
          {
            closestWaypoint = 0;
          }
      }

    return closestWaypoint;
  }

  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
    {
      int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

      int prev_wp;
      prev_wp = next_wp-1;
      if(next_wp == 0)
        {
          prev_wp  = maps_x.size()-1;
        }

      double n_x = maps_x[next_wp]-maps_x[prev_wp];
      double n_y = maps_y[next_wp]-maps_y[prev_wp];
      double x_x = x - maps_x[prev_wp];
      double x_y = y - maps_y[prev_wp];

      // find the projection of x onto n
      double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
      double proj_x = proj_norm*n_x;
      double proj_y = proj_norm*n_y;

      double frenet_d = distance(x_x,x_y,proj_x,proj_y);

      //see if d value is positive or negative by comparing it to a center point

      double center_x = 1000-maps_x[prev_wp];
      double center_y = 2000-maps_y[prev_wp];
      double centerToPos = distance(center_x,center_y,x_x,x_y);
      double centerToRef = distance(center_x,center_y,proj_x,proj_y);

      if(centerToPos <= centerToRef)
        {
          frenet_d *= -1;
        }

      // calculate s value
      double frenet_s = 0;
      for(int i = 0; i < prev_wp; i++)
        {
          frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
        }

      frenet_s += distance(0,0,proj_x,proj_y);

      return {frenet_s,frenet_d};

    }

  // Transform from Frenet s,d coordinates to Cartesian x,y
  vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
    {
      int prev_wp = -1;

      while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
        {
          prev_wp++;
        }

      int wp2 = (prev_wp+1)%maps_x.size();

      double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
      // the x,y,s along the segment
      double seg_s = (s-maps_s[prev_wp]);

      double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
      double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

      double perp_heading = heading-pi()/2;

      double x = seg_x + d*cos(perp_heading);
      double y = seg_y + d*sin(perp_heading);

      return {x,y};

    }
  /**
     d is the Frenet coordinate, lane width is 4m
     0 => left lane, 1 => mid lane, 2 => right lane
     8 < d < 12 ~~ d<(2+4*lane+2) && d>(2+4*lane-2) <= right  (lane=2)
     4 < d < 8  ~~ d<(2+4*lane+2) && d>(2+4*lane-2) <= middle (lane=1)
     0 < d < 4  ~~ d<(2+4*lane+2) && d>(2+4*lane-2) <= left   (lane=0)
   */
  int getLane(double d){
    return int(floor(d / 4));
  }

  void globalToLocalCoord(double ref_x, double ref_y, double ref_yaw,
                          vector<double> & ptsx, vector<double>& ptsy){
    for(int i =0; i<ptsx.size(); i++){
      // shift car reference angel to 0 degrees (coordinates Transformation)
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;
      ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
      ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
    }
  }

  void constructPath(double car_x, double car_y, double car_yaw, double ref_vel,
                     const vector<double> & prev_path_x,
                     const vector<double> & prev_path_y,
                     const vector<vector<double>> & interpolated_waypoints,
                     vector<double> & next_x_vals,
                     vector<double> & next_y_vals){
    vector<double> ptsx, ptsy;
    auto prev_size = prev_path_x.size();
    double ref_x = car_x, ref_y = car_y, ref_yaw = deg2rad(car_yaw);
    if(prev_size < 2){
      // use two points that make the path tangent to the cars
      double prev_car_x = car_x - cos(car_yaw);
      double prev_car_y = car_y - sin(car_yaw);
      ptsx.push_back(prev_car_x);
      ptsx.push_back(car_x);
      ptsy.push_back(prev_car_y);
      ptsy.push_back(car_y);
    }else{
      // use previous path's end point as the starting reference
      ref_x = prev_path_x[prev_size-1];
      ref_y = prev_path_y[prev_size-1];
      double ref_x_prev = prev_path_x[prev_size-2];
      double ref_y_prev = prev_path_y[prev_size-2];
      ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
      // use two points that make the path tangent to the previous paht's end point
      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);
      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);
    }
    // append interpolated waypoints to the path (in global coord)
    for(vector<double> it_wp: interpolated_waypoints){
      ptsx.push_back(it_wp[0]);
      ptsy.push_back(it_wp[1]);
    }
    // so far the waypoints in the new path are in global coords, convert to car's coord
    globalToLocalCoord(ref_x, ref_y, ref_yaw, ptsx, ptsy);
    // create a spline
    tk::spline s;
    s.set_points(ptsx, ptsy);
    // start with all of the previous points from last time
    for(int i=0; i<prev_size; i++){
      next_x_vals.push_back(prev_path_x[i]);
      next_y_vals.push_back(prev_path_y[i]);
    }
    // calculate how to break up spline points so that we travel at our desired reference velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
    double x_add_on = 0.0;
    // fill up the rest of path planner after filling it with previous points
    for(int i=0; i<50-prev_path_x.size(); i++){
      double N = (target_dist/(0.02*ref_vel/2.24));
      double x_point = x_add_on+(target_x)/N;
      double y_point = s(x_point);
      x_add_on = x_point;
      double x_ref = x_point;
      double y_ref = y_point;
      // rotate back to global coordinates
      x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
      y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
      x_point += ref_x;
      y_point += ref_y;
      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
    }
  }
};
#endif
