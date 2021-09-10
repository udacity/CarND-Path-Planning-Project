#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>

#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(const double x) { return x * pi() / 180; }
double rad2deg(const double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(const double x1, const double y1, const double x2,
                const double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

const double cycleTime = 0.02;
const double factorMilesPhToMperS = 2.24;

double getTravelledDistance(const double velocity,
                            const double time = cycleTime) {
  return time * velocity / factorMilesPhToMperS;
}

enum laneIndex { left = 0, middle = 1, right = 2, unkown = 3 };
double laneWidth = 4.0;
// start in lane 1
auto targetLaneIndex = middle;
double getLaneDisplacement(const laneIndex myLaneIndex,
                           const double laneWidth = laneWidth) {
  double center = laneWidth / 2;
  return (center + laneWidth * static_cast<int>(myLaneIndex));
}
auto targetOffsetLat = getLaneDisplacement(targetLaneIndex);
auto controlOffsetLat = targetOffsetLat;
auto offsetLatStep = 0.3;

// reference velocity to target [miles per hour]
double controlSpeed = 0;
const double maxVelocity = 49.5;
double targetSpeed = maxVelocity;
// 5m/s
double velocityStep = 0.224;

struct points {
  vector<double> x;
  vector<double> y;
};

struct pointXY {
  double x;
  double y;
};

struct poseXY {
  pointXY xy;
  double yaw;
};

struct pointSD {
  double s;
  double d;
};

struct path {
  vector<pointXY> xy;
  vector<pointSD> sd;
};

struct egoVehicle {
  // Main car's localization Data
  pointXY xy;
  pointSD sd;
  double yaw;
  double speed;
  laneIndex currentlaneIndex;

  // Previous path's end s and d values
  pointSD end_path_sd;

  // Previous path data given to the Planner
  vector<double> previous_path_x;
  vector<double> previous_path_y;
};

struct mapWaypoints {
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> dx;
  vector<double> dy;
};

struct object {
  // unique ID
  double id;
  // x and  position in map coordinates
  pointXY xy;
  // s and d position in frenet coordinates
  pointSD sd;
  // x velocity in m/s
  double vx;
  // y velocity in m/s
  double vy;
  // velocity in m/s
  double v;
  // laneIndex
  laneIndex currentlaneIndex;
  // predicted s
  double predS;

  object(vector<double> data) {
    id = data[0];
    xy.x = data[1];
    xy.y = data[2];
    vx = data[3];
    vy = data[4];
    sd.s = data[5];
    sd.d = data[6];
    currentlaneIndex = unkown;
    predS = 0.0;
  }
};

struct lane {
  // object within critical distance
  double isObjectBlocking;
  // max velocity in m/s
  double maxV = maxVelocity;
};

pointXY calcPreviousPoint(const pointXY point, const double yaw) {
  pointXY newPoint;
  newPoint.x = point.x - cos(yaw);
  newPoint.y = point.y - sin(yaw);

  return newPoint;
}

double calcYaw(const pointXY pointA, const pointXY pointB) {
  return atan2(pointA.y - pointB.y, pointA.x - pointB.x);
}

pointXY calcTranslation(const pointXY pointA, const pointXY pointB,
                        bool substract = false) {
  pointXY result;
  if (substract) {
    result.x = pointA.x - pointB.x;
    result.y = pointA.y - pointB.y;
  } else {
    result.x = pointA.x + pointB.x;
    result.y = pointA.y + pointB.y;
  }
  return result;
}

pointXY calcRotation(const pointXY pointA, const double yaw) {
  pointXY result;
  result.x = (pointA.x * cos(yaw) - pointA.y * sin(yaw));
  result.y = (pointA.x * sin(yaw) + pointA.y * cos(yaw));
  return result;
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {
  double closestLen = 100000;  // large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  // see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
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
pointXY getXY(double s, double d, const vector<double> &maps_s,
              const vector<double> &maps_x, const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading =
      atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  pointXY point;
  point.x = seg_x + d * cos(perp_heading);
  point.y = seg_y + d * sin(perp_heading);

  return point;
}

#endif  // HELPERS_H