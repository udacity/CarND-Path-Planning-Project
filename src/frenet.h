#ifndef FRENET_H
#define FRENET_H
#include <vector>
#include "trigs.h"
#include "map.h"

using namespace std;

class Frenet{
  public:
    // Transform from Frenet s,d coordinates to Cartesian x,y
    static vector<double> getXY(double s, double d, Map map){
      int prev_wp = -1;

      while(s > map.waypoints_s[prev_wp+1] && (prev_wp < (int)(map.waypoints_s.size() -1) )){
        prev_wp++;
      }

      int wp2 = (prev_wp+1)%map.waypoints_x.size();

      double heading = atan2((map.waypoints_y[wp2]-map.waypoints_y[prev_wp]),(map.waypoints_x[wp2]-map.waypoints_x[prev_wp]));
      // the x,y,s along the segment
      double seg_s = (s-map.waypoints_s[prev_wp]);

      double seg_x = map.waypoints_x[prev_wp]+seg_s*cos(heading);
      double seg_y = map.waypoints_y[prev_wp]+seg_s*sin(heading);

      double perp_heading = heading-Trigs::pi()/2;

      double x = seg_x + d*cos(perp_heading);
      double y = seg_y + d*sin(perp_heading);
      return {x,y};
    }

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    static vector<double> getFrenet(double x, double y, double theta, Map map){
      int next_wp = NextWaypoint(x,y, theta, map.waypoints_x,map.waypoints_y);

      int prev_wp;
      prev_wp = next_wp-1;
      if(next_wp == 0)
      {
        prev_wp  = map.waypoints_x.size()-1;
      }

      double n_x = map.waypoints_x[next_wp]-map.waypoints_x[prev_wp];
      double n_y = map.waypoints_y[next_wp]-map.waypoints_y[prev_wp];
      double x_x = x - map.waypoints_x[prev_wp];
      double x_y = y - map.waypoints_y[prev_wp];

      // find the projection of x onto n
      double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
      double proj_x = proj_norm*n_x;
      double proj_y = proj_norm*n_y;

      double frenet_d = Trigs::distance(x_x,x_y,proj_x,proj_y);

      //see if d value is positive or negative by comparing it to a center point

      double center_x = 1000-map.waypoints_x[prev_wp];
      double center_y = 2000-map.waypoints_y[prev_wp];
      double centerToPos = Trigs::distance(center_x,center_y,x_x,x_y);
      double centerToRef = Trigs::distance(center_x,center_y,proj_x,proj_y);

      if(centerToPos <= centerToRef){
        frenet_d *= -1;
      }

      // calculate s value
      double frenet_s = 0;
      for(int i = 0; i < prev_wp; i++){
        frenet_s += Trigs::distance(map.waypoints_x[i],map.waypoints_y[i],map.waypoints_x[i+1],map.waypoints_y[i+1]);
      }

      frenet_s += Trigs::distance(0,0,proj_x,proj_y);

      return {frenet_s,frenet_d};

    }

  private:
    // For converting back and forth between radians and degrees.

    static int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y){

      double closestLen = 100000; //large number
      int closestWaypoint = 0;

      for(int i = 0; i < maps_x.size(); i++){
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = Trigs::distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
          closestLen = dist;
          closestWaypoint = i;
        }

      }

      return closestWaypoint;

    }

    static int NextWaypoint(double x, double y, double theta, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y)
    {

      int closestWaypoint = ClosestWaypoint(x,y,map_waypoints_x,map_waypoints_y);

      double map_x = map_waypoints_x[closestWaypoint];
      double map_y = map_waypoints_y[closestWaypoint];

      double heading = atan2((map_y-y),(map_x-x));

      double angle = fabs(theta-heading);
      angle = min(2*Trigs::pi() - angle, angle);

      if(angle > Trigs::pi()/4)
      {
        closestWaypoint++;
        if (closestWaypoint == map_waypoints_x.size())
        {
          closestWaypoint = 0;
        }
      }

      return closestWaypoint;
    }
};

#endif
