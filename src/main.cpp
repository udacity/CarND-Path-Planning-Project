#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "helpers.h"
#include "json.hpp"
//
#include "trajectory_planner.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Parameters for units transformation
//------------------------------------------//
double mps2mph = 2.23694; // m/s --> mph
double mph2mps = 0.44704; // mph --> m/s
//------------------------------------------//

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Parameters
  //---------------------//
  double T_sample = 0.02; // 20 ms, sampling period
  //---------------------//

  // Variables
  //---------------------//
  int lane = 1;
  // The reference speed
  double ref_vel_mph = 49.5; // mph
  //---------------------//

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // Get the size of previous_path (remained unexecuted way points)
          size_t prev_size = previous_path_x.size();

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //---------------------------------------------------//

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

           // test
           //----------------------------//
          //  double dist_inc = 0.5;
          //  for (int i = 0; i < 50; ++i) {
          //     // double next_s = car_s + (i+1) * dist_inc;
          //     double next_s = car_s + i * dist_inc;
          //     double next_d = 6.0;
          //     vector<double> xy = getXY(next_s,next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //     next_x_vals.push_back(xy[0]);
          //     next_y_vals.push_back(xy[1]);
          // }
           //----------------------------//


           // // test, a single JMT
           // //----------------------------------------//
           // double speed_set = 25.0; // m/s
           // double T_end = 1.0; //
           // // Starts
           // std::vector<double> start_cond_s(3);
           // std::vector<double> start_cond_d(3);
           // start_cond_s[0] = car_s;
           // start_cond_s[1] = car_speed*0.44704;
           // start_cond_s[2] = 0.0;
           // start_cond_d[0] = car_d;
           // start_cond_d[1] = 0.0;
           // start_cond_d[2] = 0.0;
           //
           // // Ends
           // std::vector<double> end_cond_s(3);
           // std::vector<double> end_cond_d(3);
           // end_cond_s[0] = car_s + speed_set*T_end;
           // end_cond_s[1] = speed_set;
           // end_cond_s[2] = 0.0;
           // end_cond_d[0] = car_d;
           // end_cond_d[1] = 0.0;
           // end_cond_d[2] = 0.0;
           //
           // // JMTs
           // std::vector<double> param_s = JMT(start_cond_s, end_cond_s, T_end);
           // std::vector<double> param_d = JMT(start_cond_d, end_cond_d, T_end);
           //
           // for (size_t i=0; i < 50; ++i){
           //     double t = i*0.02;
           //     double next_s = get_JMT_value(t, param_s);
           //     double next_d = get_JMT_value(t, param_d);
           //     vector<double> xy = getXY(next_s,next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
           //     next_x_vals.push_back(xy[0]);
           //     next_y_vals.push_back(xy[1]);
           // }
           // //----------------------------------------//



           // Generate spline
           //----------------------------------------//
           // ptsx and ptsy are anchore points for apline
           vector<double> ptsx;
           vector<double> ptsy;

           // Reference x, y, yaw states
           double ref_x = car_x;
           double ref_y = car_y;
           double ref_yaw = deg2rad(car_yaw);

           if (prev_size < 2){
               double ref_x_pre = car_x - cos(car_yaw);
               double ref_y_pre = car_y - sin(car_yaw);

               ptsx.push_back(ref_x_pre);
               ptsx.push_back(ref_x);
               ptsy.push_back(ref_y_pre);
               ptsy.push_back(ref_y);
           }else{
               // Redefine reference state as previous path end point
               ref_x = previous_path_x[prev_size-1];
               ref_y = previous_path_y[prev_size-1];

               double ref_x_pre = previous_path_x[prev_size-2];
               double ref_y_pre = previous_path_y[prev_size-2];
               ref_yaw = atan2(ref_y - ref_y_pre, ref_x - ref_x_pre);

               ptsx.push_back(ref_x_pre);
               ptsx.push_back(ref_x);
               ptsy.push_back(ref_y_pre);
               ptsy.push_back(ref_y);
           }

           // Add three evenly spaced points (in Frenet) ahead of starting point
           // TODO: Why use car_s instead of end_path_s?
           double cp_space = 30.0; // m, note: 25 m/s * 1.0 s = 25 m < 30 m
           vector<double> next_wp0 = getXY(car_s+cp_space, lane_to_d(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
           vector<double> next_wp1 = getXY(car_s+2*cp_space, lane_to_d(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
           vector<double> next_wp2 = getXY(car_s+3cp_space, lane_to_d(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

           //
           ptsx.push_back(next_wp0[0]);
           ptsx.push_back(next_wp1[0]);
           ptsx.push_back(next_wp2[0]);
           //
           ptsy.push_back(next_wp0[1]);
           ptsy.push_back(next_wp1[1]);
           ptsy.push_back(next_wp2[1]);

           // Now we have totally 5 points in ptsx and ptsy

           // Change reference coordinate frame
           for (size_t i=0; i < ptsx.size(); ++i){
               double shift_x = ptsx[i] - ref_x;
               double shift_y = ptsy[i] - ref_y;
               ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
               ptsx[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
           }

           // Create a spline
           tk::spline s;
           // Insert anchor points
           s.set_points(ptsx, ptsy)


           // Push the previous_path into next vals
           for (size_t i=0; i < previous_path_x.siize(); ++i){
               next_x_vals.push_back(previous_path_x[i]);
               next_y_vals.push_back(previous_path_y[i]);
           }

           // Caluculate how to sample the spline point for required velocity
           double target_x = 30.0;
           double target_y = s(target_x);
           double target_dist = sqrt( target_x*target_x + target_y*target_y);

           double x_add_on = 0;


           // Fill up the rest of the path after filling up with previous path points.
           // Here we always output 50 points
           for (size_t i=1; i <= (50-previous_path_x.size()); ++i){
               double N = target_dist/(T_sample*ref_vel_mph*mph2mps);
               double x_local = x_add_on + target_x/N;
               double y_local = s(x_local);
               x_add_on = x_local;

               // Coordinate transformation, from local frame to global frame
               double x_point = x_local * cos(ref_yaw) - y_local * sin(ref_yaw);
               double y_point = x_local * sin(ref_yaw) + y_local * cos(ref_yaw);
               // Translation
               x_point += ref_x;
               y_point += ref_y;

               // Add point to path
               next_x_vals.push_back( x_point );
               next_y_vals.push_back( y_point );
           }
           //----------------------------------------//


          //---------------------------------------------------//
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
