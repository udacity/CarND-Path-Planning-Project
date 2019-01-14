#include <cmath>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "util.h"
#include <uWS/uWS.h>

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  std::vector<double> map_waypoints_x, map_waypoints_y, map_waypoints_s,
      map_waypoints_dx, map_waypoints_dy;

  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  const double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x, y;
    float s, d_x, d_y;
    iss >> x >> y >> s >> d_x >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // start in lane 1.
  int lane = 1;

  // Have a reference velocity to target
  double ref_vel = 49.5; // mph

  h.onMessage([&ref_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &lane](
      uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length <= 2 || data[0] != '4' || data[1] != '2')
      return;

    const auto json_str = hasData(data);

    if (json_str == "") {
      // Manual driving
      const std::string msg = "42[\"manual\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      return;
    }

    const auto j = json::parse(json_str);

    if (j[0].get<std::string>() != "telemetry")
      return;

    // j[1] is the data JSON object

    // Main car's localization Data
    const auto car_x = static_cast<double>(j[1]["x"]);
    const auto car_y = static_cast<double>(j[1]["y"]);
    const auto car_s = static_cast<double>(j[1]["s"]);
    const auto car_d = static_cast<double>(j[1]["d"]);
    const auto car_yaw = static_cast<double>(j[1]["yaw"]);
    const auto car_speed = static_cast<double>(j[1]["speed"]);

    // Previous path data given to the Planner
    const auto previous_path_x = j[1]["previous_path_x"];
    const auto previous_path_y = j[1]["previous_path_y"];
    // Previous path's end s and d values
    const auto end_path_s = static_cast<double>(j[1]["end_path_s"]);
    const auto end_path_d = static_cast<double>(j[1]["end_path_d"]);

    // Sensor Fusion Data, a list of all other cars on the same side of
    // the road.
    const auto sensor_fusion = j[1]["sensor_fusion"];

    const auto prev_size = previous_path_x.size();

    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m.
    // Later we will interpolate these waypoints with a pline and fill it in
    // with more points that control speed.

    std::vector<double> ptsx, ptsy;

    auto ref_x = car_x;
    auto ref_y = car_y;
    auto ref_yaw = deg2rad(car_yaw);

    if (prev_size < 2) {
      const auto prev_car_x = car_x - cos(car_yaw);
      const auto prev_car_y = car_y - sin(car_yaw);

      ptsx.push_back(prev_car_x);
      ptsx.push_back(ref_x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(ref_y);
    } else {
      ref_x = previous_path_x[prev_size - 1];
      ref_y = previous_path_y[prev_size - 1];

      const double ref_x_prev = previous_path_x[prev_size - 2];
      const double ref_y_prev = previous_path_y[prev_size - 2];
      ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);
    }

    for (int i = 1; i < 4; ++i) {
      const auto next_wp = getXY(car_s + 30 * i, 2 + 4 * lane, map_waypoints_s,
                                 map_waypoints_x, map_waypoints_y);
      ptsx.push_back(next_wp[0]);
      ptsy.push_back(next_wp[1]);
    }

    for (size_t i = 0; i < ptsx.size(); ++i) {
      const auto shift_x = ptsx[i] - ref_x;
      const auto shift_y = ptsy[i] - ref_y;

      ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
      ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    tk::spline s;
    s.set_points(ptsx, ptsy);

    std::vector<double> next_x_vals, next_y_vals;
    for (size_t i = 0; i < previous_path_x.size(); ++i) {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }

    const auto target_x = 30.0L;
    const auto target_y = s(target_x);
    const auto target_dist = distance(target_x, target_y, 0, 0);

    auto x_add_on = 0.0L;

    for (size_t i = 0; i <= 50 - previous_path_x.size(); ++i) {
      const auto N = target_dist / (0.02 * ref_vel / 2.24);
      const auto x_ref = x_add_on + target_x / N;
      const auto y_ref = s(x_ref);

      x_add_on = x_ref;

      const auto x_point = ref_x + x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
      const auto y_point = ref_y + x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
    }

    json msgJson;
    msgJson["next_x"] = next_x_vals;
    msgJson["next_y"] = next_y_vals;

    const auto msg = "42[\"control\"," + msgJson.dump() + "]";

    // this_thread::sleep_for(chrono::milliseconds(1000));
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest(
      [](uWS::HttpResponse *res, uWS::HttpRequest req, char *, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
          res->end(s.data(), s.length());
        } else {
          // i guess this should be done more gracefully?
          res->end(nullptr, 0);
        }
      });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int, char *, size_t) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  constexpr int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
