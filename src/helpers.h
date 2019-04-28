#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <chrono>
#include <ctime>

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
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
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
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
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
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

// Borrowed from: https://knowledge.udacity.com/questions/9577
// Return velocity along s and d in frenet coordinates
vector<double> getFrenetVelocity(double x, double y,
                                 double vx, double vy,
                                 const vector<double> &maps_x, 
                                 const vector<double> &maps_y,
                                 const vector<double> &maps_dx, 
                                 const vector<double> &maps_dy) {
  int wp = ClosestWaypoint(x, y, maps_x, maps_y);
  double dx = maps_dx[wp];
  double dy = maps_dy[wp];
  double vd = vx*dx + vy*dy;
  double vs = -vx*dy + vy*dx;
  return {vs, vd};
}

class Egocar {

  public:
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;

    void update(double car_x, double car_y, double car_s,
                double car_d, double car_yaw, double car_speed) {
      x = car_x;
      y = car_y;
      s = car_s;
      d = car_d;
      yaw = car_yaw;
      speed = car_speed;
    }

};

namespace Lane {
    enum laneNumber {
      LANE_RIGHT,
      LANE_CENTER,
      LANE_LEFT
    };
    enum laneType {
      LANE_EGO,
      LANE_EGO_LEFT,
      LANE_EGO_LEFT_FAR,
      LANE_EGO_RIGHT,
      LANE_EGO_RIGHT_FAR
    };
    const double LANE_WIDTH = 4.0;
    class LaneObject {
      public:
        int num;
        int position;
        double laneSpeed;
        vector<vector<double>> openStretches;
    };
}
class WorldObject {
  public:
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double vs;
    double vd;
    double s;
    double d;
    double d_prev;
    double d_speed;
    enum Lane::laneNumber laneAssignment;
    double relativeVx;
    double relativeVy;
    double relativeVel;
    double distance;
    double ttc;
    double speed;
    vector<vector<double>> prediction;
    vector<vector<double>*> maps;

    WorldObject (vector<vector<double>*> worldMaps) {
      maps = worldMaps;
    }

    void update(WorldObject obj) {
      std::cout << "Updating car " << id << " with new info" << std::endl;
      x = obj.x;
      y = obj.y;
      vx = obj.vx;
      vy = obj.vy;
      s = obj.s;
      d = obj.d;

      if (distance < 10.0){
        if (laneAssignment != obj.laneAssignment) {
          std::cout << "Car "<< id << " has changed lanes from lane "
                    << laneAssignment << " to lane "
                    << obj.laneAssignment << std::endl;;
        }
      }

      laneAssignment = obj.laneAssignment;
      relativeVx = obj.relativeVx;
      relativeVy = obj.relativeVy;
      distance = obj.distance;
      ttc = obj.ttc;
      
      if (distance < 10.0){
        vector<double> frenetVelocities = getFrenetVelocity(x, y, vx, vy,
                                                            *maps[0],
                                                            *maps[1],
                                                            *maps[2],
                                                            *maps[3]);
        vs = frenetVelocities[0];
        vd = frenetVelocities[1];

        prediction = {};
        predict(5, prediction);
        double predictedD = prediction[prediction.size()-1][1];
        std::cout << "lane: " << laneAssignment << " vs: " << vs
                  << " vd: " << vd << " curr d:" << d
                  << " predicted d: " << predictedD
                  << std::endl;
        if ((std::abs(predictedD - d) > 4.0) {
          std::cout << "predicting that car "<<id<<" will change lanes"<<std::endl;
        }
      }
    }

  private:

    void predict(uint8_t duration, vector<vector<double>>& prediction) {
      for (int i = 0; i < duration; i++) {
        prediction.push_back({s + i * vs, d + i * vd});
      }
    }
};

class WorldModel {
  public:
    vector<WorldObject> cars;
    vector<Lane::LaneObject> lanes;
    Egocar* egocar;
    vector<double>maps_x;
    vector<double>maps_y;
    vector<double>maps_s;
    vector<double>maps_dx;
    vector<double>maps_dy;
    vector<vector<double>*> maps;

    WorldModel (Egocar* pointer, const vector<double>maps_x_in,
                                 const vector<double>maps_y_in,
                                 const vector<double>maps_s_in,
                                 const vector<double>maps_dx_in,
                                 const vector<double>maps_dy_in) {
      egocar = pointer;
      maps_x = maps_x_in;
      maps_y = maps_y_in;
      maps_s = maps_s_in;
      maps_dx = maps_dx_in;
      maps_dy = maps_dy_in;
      maps.push_back(&maps_x);
      maps.push_back(&maps_y);
      maps.push_back(&maps_dx);
      maps.push_back(&maps_dy);
    }
    
    void update(const vector<vector<double>> sensor_fusion) {
      // Format for each car: [ id, x, y, vx, vy, s, d]
      for (auto& detection:sensor_fusion) {
        // Copy over the data
        WorldObject obj(maps); 
        obj.id = detection[0];
        obj.x  = detection[1];
        obj.y  = detection[2];
        obj.vx = detection[3];
        obj.vy = detection[4];
        obj.s  = detection[5];
        obj.d  = detection[6];
    
        // Assign each car to a lane
        for (int laneNum = 0; laneNum < 4; laneNum++) { 
          if (obj.d < Lane::LANE_WIDTH*(laneNum+1) && 
              obj.d > Lane::LANE_WIDTH*laneNum) {
            obj.laneAssignment = static_cast<Lane::laneNumber>(laneNum);
            break;
          }
        }
        
        // Update relative properties
        obj.speed = std::pow((obj.vx * obj.vx) + (obj.vy * obj.vy), 0.5);
        obj.relativeVel = egocar->speed - obj.speed;
        obj.distance = distance(egocar->x, egocar->y, obj.x, obj.y);
        obj.ttc = obj.distance/obj.relativeVel;
        
        WorldObject* matchedCar = getWorldObjById(obj.id);
        if (matchedCar){
          matchedCar->update(obj);
        }
        else {
          std::cout << "[" << std::time(0) << "] - "
                    << "New car with id " << obj.id
                    << " in lane " << obj.laneAssignment << std::endl;
          cars.push_back(obj);
        }
      }
    }
    
  private:

    WorldObject* getWorldObjById(double id) {
      for(auto& obj:cars) {
        if (obj.id == id) {
          return &obj;
        }
      }
      return NULL;
    }
};

#endif  // HELPERS_H
