#ifndef MAP_H_
#define MAP_H_
#define MAP_MAX_S 6945.554

#include <vector>

struct Map {
  std::vector<double> waypoints_x;
  std::vector<double> waypoints_y;
  std::vector<double> waypoints_s;
  std::vector<double> waypoints_dx;
  std::vector<double> waypoints_dy;
};


class MapUtils{
  public:
    static double diff(double ego_s, double car_s){
      double ahead = car_s - ego_s;
      double behind = ahead - MAP_MAX_S;
      if(std::fabs(ahead) < std::fabs(behind)){
        return ahead;
      }
      return behind;
    }
};

#endif
