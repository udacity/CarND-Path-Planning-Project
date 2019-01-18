#ifndef TELEMETRY_HPP_
#define TELEMETRY_HPP_

#include <vector>
#include "json.hpp"

struct SensorFusion {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
};

struct Telemetry {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  // Previous path data given to the Planner
  nlohmann::json previous_path_x;
  nlohmann::json previous_path_y;
  // Previous path's end s and d values 
  double end_path_s;
  double end_path_d;
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  std::vector<SensorFusion> sensor_fusion;
};


class TelemetryUtils{
  public:
    static Telemetry parse(nlohmann::json j){
      Telemetry tl;
      tl.x = j[1]["x"];
      tl.y = j[1]["y"];
      tl.s = j[1]["s"];
      tl.d = j[1]["d"];
      tl.yaw = j[1]["yaw"];
      tl.speed = (double)j[1]["speed"]/2.237;

      // Previous path data given to the Planner
      tl.previous_path_x = j[1]["previous_path_x"];
      tl.previous_path_y = j[1]["previous_path_y"];

      // Previous path's end s and d values 
      tl.end_path_s = j[1]["end_path_s"];
      tl.end_path_d = j[1]["end_path_d"];

      // Sensor Fusion Data, a list of all other cars on the same side of the road.
      auto sensor_fusion = j[1]["sensor_fusion"];
      for(int i = 0; i<sensor_fusion.size(); i++) {
        SensorFusion sf;
        sf.id = sensor_fusion[i][0];
        sf.x = sensor_fusion[i][1];
        sf.y = sensor_fusion[i][2];
        sf.vx = sensor_fusion[i][3];
        sf.vy = sensor_fusion[i][4];
        sf.s = sensor_fusion[i][5];
        sf.d = sensor_fusion[i][6];
        tl.sensor_fusion.push_back(sf);
      }

      return tl;

    }
};
#endif
