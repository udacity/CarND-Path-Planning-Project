#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <uWS/uWS.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  const auto found_null = s.find("null");
  const auto b1 = s.find_first_of("[");
  const auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(const double x1, const double y1, const double x2,
                const double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int closestWaypoint(const double x, const double y,
                    const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y) {

  double closest_len = std::numeric_limits<double>::max();
  int closest_waypoint = 0;

  for (size_t i = 0; i < maps_x.size(); i++) {
    const double dist = distance(x, y, maps_x[i], maps_y[i]);
    if (dist < closest_len) {
      closest_len = dist;
      closest_waypoint = i;
    }
  }

  return closest_waypoint;
}

int nextWaypoint(const double x, const double y, const double theta,
                 const std::vector<double> &maps_x,
                 const std::vector<double> &maps_y) {

  auto closest_waypoint = closestWaypoint(x, y, maps_x, maps_y);

  const auto map_x = maps_x[closest_waypoint];
  const auto map_y = maps_y[closest_waypoint];
  const auto heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closest_waypoint++;
    if (closest_waypoint == maps_x.size()) {
      closest_waypoint = 0;
    }
  }

  return closest_waypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(const double x, const double y,
                              const double theta,
                              const std::vector<double> &maps_x,
                              const std::vector<double> &maps_y) {
  const auto next_wp = nextWaypoint(x, y, theta, maps_x, maps_y);
  const auto prev_wp = (next_wp == 0) ? maps_x.size() - 1 : next_wp - 1;

  const auto n_x = maps_x[next_wp] - maps_x[prev_wp];
  const auto n_y = maps_y[next_wp] - maps_y[prev_wp];
  const auto x_x = x - maps_x[prev_wp];
  const auto x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  const auto proj_norm = (x_x * n_x + x_y * n_y) / (pow(n_x, 2) + pow(n_y, 2));
  const auto proj_x = proj_norm * n_x;
  const auto proj_y = proj_norm * n_y;

  auto frenet_d = distance(x_x, x_y, proj_x, proj_y);

  // see if d value is positive or negative by comparing it to a center point

  const auto center_x = 1000 - maps_x[prev_wp];
  const auto center_y = 2000 - maps_y[prev_wp];
  const auto center_to_pos = distance(center_x, center_y, x_x, x_y);
  const auto center_to_ref = distance(center_x, center_y, proj_x, proj_y);

  if (center_to_pos <= center_to_ref) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(const double s, const double d,
                          const std::vector<double> &maps_s,
                          const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] &&
         prev_wp < static_cast<int>(maps_s.size() - 1)) {
    prev_wp++;
  }

  const auto wp2 = (prev_wp + 1) % maps_x.size();

  const auto heading =
      atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  const auto seg_s = (s - maps_s[prev_wp]);

  const auto seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  const auto seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  const auto perp_heading = heading - pi() / 2;

  const auto x = seg_x + d * cos(perp_heading);
  const auto y = seg_y + d * sin(perp_heading);

  return {x, y};
}

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

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy](
      uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length <= 2 || data[0] != '4' || data[1] != '2')
      return;

    const auto s = hasData(data);

    if (s == "") {
      // Manual driving
      const std::string msg = "42[\"manual\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      return;
    }

    const auto j = json::parse(s);

    if (j[0].get<std::string>() != "telemetry")
      return;

    // j[1] is the data JSON object

    // Main car's localization Data
    const double car_x = j[1]["x"];
    const double car_y = j[1]["y"];
    const double car_s = j[1]["s"];
    const double car_d = j[1]["d"];
    const double car_yaw = j[1]["yaw"];
    const double car_speed = j[1]["speed"];

    // Previous path data given to the Planner
    const auto previous_path_x = j[1]["previous_path_x"];
    const auto previous_path_y = j[1]["previous_path_y"];
    // Previous path's end s and d values
    const double end_path_s = j[1]["end_path_s"];
    const double end_path_d = j[1]["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side of
    // the road.
    const auto sensor_fusion = j[1]["sensor_fusion"];

    json msgJson;

    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    const double dist_inc = 0.5;
    for (int i = 0; i < 50; i++) {
      const auto next_s = car_s + (i + 1) * dist_inc;
      const auto next_d = 6;
      const auto xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x,
                            map_waypoints_y);
      next_x_vals.push_back(xy[0]);
      next_y_vals.push_back(xy[1]);
    }

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
