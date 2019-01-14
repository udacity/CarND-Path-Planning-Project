#pragma once

#include <cmath>
#include <string>
#include <vector>

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

int closestWaypoint(const double x, const double y,
                    const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y) {
  auto closest_len = std::numeric_limits<double>::max();
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
  const auto heading = atan2(map_y - y, map_x - x);

  double angle = fabs(theta - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closest_waypoint = (closest_waypoint + 1) % maps_x.size();
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
  for (int i = 0; i < prev_wp; ++i) {
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
  const auto seg_s = s - maps_s[prev_wp];
  const auto seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  const auto seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  const auto perp_heading = heading - pi() / 2;

  const auto x = seg_x + d * cos(perp_heading);
  const auto y = seg_y + d * sin(perp_heading);

  return {x, y};
}
