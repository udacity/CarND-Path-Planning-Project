#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "pathPlanner.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  string line;
  mapWaypoints map;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map.x.push_back(x);
    map.y.push_back(y);
    map.s.push_back(s);
    map.dx.push_back(d_x);
    map.dy.push_back(d_y);
  }

  h.onMessage([&map](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          egoVehicle car;
          // Main car's localization Data
          car.xy.x = j[1]["x"];
          car.xy.y = j[1]["y"];
          car.sd.s = j[1]["s"];
          car.sd.d = j[1]["d"];
          car.yaw = j[1]["yaw"];
          car.speed = j[1]["speed"];
          // Previous path's end s and d values
          car.end_path_sd.s = j[1]["end_path_s"];
          car.end_path_sd.d = j[1]["end_path_d"];
          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          car.previous_path_x = previous_path_x;
          car.previous_path_y = previous_path_y;

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          points nextPoints;
          // straight(nextPoints, car);
          // stayInLane(nextPoints, car, map);
          stayInLaneWithSpline(nextPoints, car, map, sensor_fusion);

          json msgJson;
          msgJson["next_x"] = nextPoints.x;
          msgJson["next_y"] = nextPoints.y;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    freopen("output.txt", "w", stdout);
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}