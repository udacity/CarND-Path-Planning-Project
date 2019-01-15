#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <set>
#include "util.h"
#include "astar.hpp"

using namespace util;
// for convenience
using json = nlohmann::json;

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
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  int lane = 1; // start with lane 1
  double ref_vel = 0.0; // mph
  
  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                                       uWS::OpCode opCode) {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                //auto sdata = string(data).substr(0, length);
                //cout << sdata << endl;
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

                      // Sensor Fusion Data, a list of all other cars on the same side of the road.
                      auto sensor_fusion = j[1]["sensor_fusion"];

                      json msgJson;

                      // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                      // build upon Aaron's solution in the video tutorial
                      double max_speed = 48.5;
                      double accl = 0.224;
                      int prev_size = previous_path_x.size();
                      if(prev_size > 0){
                        car_s = end_path_s;
                      }
                      bool too_close = false,  too_close_left = false, too_close_right = false;
                      bool keep_lane = true;

                      AStar::Generator generator;
                      generator.setWorldSize({3, 3}); // 3x3 grid
                      std::vector<AStar::Vec2i> obstacles; // fill the last row with obstables
                      AStar::Vec2i start(2, lane);
                      // find ref_v to use
                      // The data format for each car is: [ id, x, y, vx, vy, s, d]. The id is a unique identifier for that car. The x, y values are in global map coordinates
                      for(int i=0; i<sensor_fusion.size(); i++){
                        int car_id = sensor_fusion[i][0];
                        double x = sensor_fusion[i][1];
                        double y = sensor_fusion[i][2];
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double s = sensor_fusion[i][5];
                        double d = sensor_fusion[i][6];

                        double check_speed = sqrt(vx*vx+vy*vy);
                        double check_car_s = s;
                        // estimate car s position given previous path
                        check_car_s += ((double)prev_size* 0.02 * check_speed);
                        int car_lane = getLane(d);
                        bool is_blocker = ((check_car_s > car_s) && (check_car_s-car_s < 30)) || ((car_s > check_car_s) && (car_s - check_car_s < 10));
                        if (is_blocker){
                          double car_dist = fabs(check_car_s - car_s);
                          if(check_car_s > car_s){
                            obstacles.push_back({0, car_lane});
                            if (car_dist < 10 ){
                              obstacles.push_back({1, car_lane});
                            }
                          }else{
                            if (car_lane != lane){
                              obstacles.push_back({2, car_lane});
                            }
                          }
                        }
                        too_close = (car_lane == lane) && (fabs(check_car_s-car_s) < 30) && (check_car_s > car_s); //too close to the car ahead
                        if (too_close && ref_vel > check_speed){
                          ref_vel -= accl * 2;
                        }
                        if (car_lane !=lane && fabs(check_car_s-car_s) < 10){
                          if (car_lane > lane){
                            too_close_right = true;
                          }
                          if(car_lane < lane){
                            too_close_right = true;
                          }
                        }
                      }
           
                      generator.addCollisionList(obstacles);
                      vector<AStar::Vec2i> targets;
                      for(int i=0; i<3; i++){
                        bool has_obs = false;
                        for(auto obs: obstacles){
                          if(obs.y==i){
                            has_obs = true;
                            break;
                          }
                        }
                        if(!has_obs){
                          targets.push_back({0, i});
                        }
                      }
                      AStar::Vec2i target = start;
                      if(targets.size()==0){ // car blocks in every lane
                        keep_lane = true;
                      }else{
                        int min_cost = 100;
                        for(auto t: targets){
                          //std::cout<<"Search parh to target point ("<<t.x<<","<<t.y<<")"<<std::endl;
                          auto path = generator.findPath(start, t);
                          if (path.size() < min_cost){
                            min_cost = path.size();
                            target = t;
                          }
                        }
                        //std::cout<<"Best path length  "<<min_cost<<" Target ("<<target.x<<","<<target.y<<")"<<std::endl;
                        if(target.y > lane && !too_close_right){
                          keep_lane = false;
                          //std::cout<<"Change to the right lane ["<<target.y<<"]"<<std::endl;
                          lane = target.y;
                        }
                        else if(target.y < lane && !too_close_left){
                          keep_lane = false;
                          //std::cout<<"Change to the left lane ["<<target.y<<"]"<<std::endl;
                          lane = target.y;
                        }
                        else{
                          keep_lane = true;
                        }
                      }
                      /*
                        std::cout<<"Start (2,"<<lane<<"), Target ("<<target.x<<","<<target.y<<")"<<std::endl;
                        std::cout<<"Obstacles: "<<std::endl;
                        for(auto obs : obstacles){
                        std::cout<<"("<<obs.x<<", "<<obs.y<<")"<<std::endl;
                        }*/
                      generator.setSource(start);
                      generator.setTarget(target);
                      generator.drawWorld();
                      if(keep_lane){
                        if(ref_vel < max_speed){
                          ref_vel += accl * 1.5;
                        }
                        else{
                          ref_vel -= accl;
                        }
                      }
                      /* generate path by interpolation and smooth trajectory with spline */
                      vector<double> next_x_vals;
                      vector<double> next_y_vals;

                      vector<vector<double>> interpolated_wps;
                      // in Frenet add evenly 30m spaced points ahead of the starting reference
                      for(int i=1; i<4; i++){
                        vector<double> next_wps = getXY(car_s+30*i, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        interpolated_wps.push_back(next_wps);
                      }
                      constructPath(car_x, car_y, car_yaw, ref_vel, previous_path_x, previous_path_y, interpolated_wps, next_x_vals, next_y_vals);

                      msgJson["next_x"] = next_x_vals;
                      msgJson["next_y"] = next_y_vals;

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
