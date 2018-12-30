#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "trajectory.h"
#include "telemetry.h"
#include "map.h"


using namespace std;

// for convenience
using json = nlohmann::json;


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


int main() {

  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  Map map;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
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
    map.waypoints_x.push_back(x);
    map.waypoints_y.push_back(y);
    map.waypoints_s.push_back(s);
    map.waypoints_dx.push_back(d_x);
    map.waypoints_dy.push_back(d_y);
  }


  TrajectoryUtil trajectory_util;

  h.onMessage([&map, &trajectory_util](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
        uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      //auto sdata = string(data).substr(0, length);
      //cout << sdata << endl;
      //
       
      if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
      nlohmann::json j = json::parse(s);

      string event = j[0].get<string>();

      if (event == "telemetry") {
      // j[1] is the data JSON object

      // Main car's localization Data
        Telemetry tl = TelemetryUtils::parse(j);


        json msgJson;

        unsigned int lane = 1;
        double velocity = 49;

        vector<SensorFusion> sf = tl.sensor_fusion;
        for(unsigned int i=0; i<sf.size(); i++) {
          double diff = sf[i].s - tl.s;
          if(diff > 0 && diff < 30){
            double check_speed = sqrt(sf[i].vx*sf[i].vx + sf[i].vy*sf[i].vy);

            if(sf[i].d < (4*lane+4) && sf[i].d> (4*lane)){
              double vs = sqrt(sf[i].vx*sf[i].vx + sf[i].vy*sf[i].vy);
              velocity = vs*2.237 - 2;
              cout<<"Ahead velocity "<<velocity<<endl;
            }

          }
        }

        //for(int i = 0; i < tl.sensor_fusion.size(); i++) {
        //   //car is in our lane
        //   float d = tl.sensor_fusion[i][6];

        //   if(d< (2*4*lane+2) && d > (2+4*lane-2) ) {
        //     double vx = tl.sensor_fusion[i][3];
        //     double vy = tl.sensor_fusion[i][4];
        //   }
        //}


        // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds


        /*
        // Look at the car ahead 
        if(prev_size > 0) {
        tl.car_s - tl.end_path_s;
        }

        bool too_close = false;

        //find ref_v to use
        // loop through the sensor fusion list
        for(int i = 0; i < tl.sensor_fusion.size(); i++) {
        //car is in our lane
        float d = tl.sensor_fusion[i][6];

        if(d< (2*4*lane+2) && d > (2+4*lane-2) ) {
        double vx = tl.sensor_fusion[i][3];
        double vy = tl.sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = tl.sensor_fusion[i][5];

        //if using previous points can project s value out
        check_car_s += (double) prev_size * .02 * check_speed;

        //check s values greater than mine and S gap
        if( check_car_s > tl.car_s && check_car_s-tl.car_s < 60) {
        //ref_vel = check_speed;
        //std::cout<<"Target car velocity " << check_speed<<std::endl;
        if ( lane > 0) {
        }
        }

        }
        }
        */





        Trajectory tr = trajectory_util.generate(tl, map, lane, velocity);

        msgJson["next_x"] = tr.x;
        msgJson["next_y"] = tr.y;

        auto msg = "42[\"control\","+ msgJson.dump()+"]";

        //this_thread::sleep_for(chrono::milliseconds(1000));
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

      }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
      }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
        size_t, size_t) {
      const std::string s = "<h1>Hello world!</h1>";
      if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
      } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
      }
      });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      std::cout << "Connected!!!" << std::endl;
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
