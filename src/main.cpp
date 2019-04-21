#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;
  static WorldModel world;
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          world.update(sensor_fusion);

        //double dist_inc = 0.5;
        //for (int i = 0; i < 50; ++i) {
        //  next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
        //  next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
        //}



          // Calculate cost

          // Avoid cars
          // predict which maneuver a vehicle is in given a single coordinate 

          // Lesson 5: Trajectory Generation
          // Build trajectory
          // Hybrid A* is good for unstrcutured enviornments (parking lots or mazes)
          // Jerk minimization trajectory (quintic polynomial solver)
          // Just need initial state and goal state
          //
          // 1. Get waypoint

        //next_waypoint = 
        //map_waypoints_x.
        //map_waypoints_y.
        //map_waypoints_s.
        //map_waypoints_dx.
        //map_waypoints_dy.


        //// FSM
        //// inputs to transition function: predictions, map, speed limit, localization, current state
        //switch (state) {
        //  case KEEP:
        //    trajectories = generate_valid_trajectories()
        //    for (auto& trajectory:trajectories) {
        //      calculate_cost(trajectory);
        //    }

        //    // Select state associated with minimum cost
        //    state = get_minimum_cost(trajectories);

        //    // If I'm already in target lane and going speed limit, stay here
        //    // If I'm already in target lane going too slow, consider passing car
        //    //    But only if I have enough time to pull this off smoothly!
        //    // If I'm not in target lane, change lanes
        //    //    If lane change will make me slow down:
        //    //      If I have lots of time, don't change lanes until later
        //    //      If I dont have time, change regardless

        //    break;

        //  case PREP_CHANGE_LEFT:
        //    // DO whatever we can 
        //    // Adjust speed to match gaps in left lane
        //    if (safety_check_lane()) {
        //      state = CHANGE_LEFT;
        //    }
        //    else {
        //      state = KEEP;
        //    }
        //    break;
        //  case CHANGE_LEFT:
        //    while(!perform_lane_change());
        //    state = KEEP;
        //    break;
        //  case PREP_CHANGE_RIGHT:
        //    break;
        //  case CHANGE_RIGHT:
        //    break;
        //  default:
        //    break;
        //}


          // Costs
          // Jerk, distance to obstacle, dist to center lane, time to goal
          // There could be more costs
          // Tricky to balance costs - optimizing one may break another
          // Combine all cost functions into one weighted cost function

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

double calculateCosts() {
  double totalCost = 0; 
  //totalCost = goal_distance_cost() + ineffieciency_cost();
  return totalCost;
}

double goal_distance_cost(int goal_lane, int intended_lane, int final_lane, 
                          double distance_to_goal) {
  // The cost increases with both the distance of intended lane from the goal
  //   and the distance of the final lane from the goal. The cost of being out 
  //   of the goal lane also becomes larger as the vehicle approaches the goal.
  int delta_d = 2.0 * goal_lane - intended_lane - final_lane;
  double cost = 1 - exp(-(std::abs(delta_d) / distance_to_goal));

  return cost;
}

double inefficiency_cost(int target_speed, int intended_lane, int final_lane, 
                         const std::vector<int> &lane_speeds) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than target_speed.
  double speed_intended = lane_speeds[intended_lane];
  double speed_final = lane_speeds[final_lane];
  double cost = (2.0*target_speed - speed_intended - speed_final)/target_speed;

  return cost;
}


vector<double> solveCoeffs(vector<double> &start, vector<double> &end, double T) {
  Eigen::MatrixXd m(3,3);
  m << std::pow(T, 3)  , std::pow(T, 4)   , std::pow(T, 5)  ,
       3*std::pow(T, 2), 4*std::pow(T, 3) , 5*std::pow(T, 4),
       6*T             , 12*std::pow(T, 2), 20*std::pow(T, 3);
  double sf            = end[0];
  double sf_dot        = end[1];
  double sf_double_dot = end[2];
  double si            = start[0];
  double si_dot        = start[1];
  double si_double_dot = start[2]; 
  Eigen::VectorXd right(3);
  right << sf - (si + si_dot*T + 0.5*si_double_dot*T*T),
           sf_dot - (si_dot + si_double_dot*T),
           sf_double_dot - si_double_dot;
  Eigen::VectorXd coefficients(3);
  coefficients = m.colPivHouseholderQr().solve(right);

  double a0 = si;
  double a1 = si_dot;
  double a2 = 0.5*si_double_dot;
  double a3 = coefficients[0];
  double a4 = coefficients[1];
  double a5 = coefficients[2];
  
  return {a0, a1, a2, a3, a4, a5};
}

vector<double> getTrajectoryPoint(double t, vector<double> coeffs) {
  double a_0 = coeffs[0];
  double a_1 = coeffs[1];
  double a_2 = coeffs[2];
  double a_3 = coeffs[3];
  double a_4 = coeffs[4];
  double a_5 = coeffs[5];
  double pos = a_0 + a_1 * t
                   + a_2 * std::pow(t, 2) 
                   + a_3 * std::pow(t, 3)
                   + a_4 * std::pow(t, 4)
                   + a_5 * std::pow(t, 5);
  double velocity = a_1 + 2 * a_2 * t
                        + 3 * a_3 * std::pow(t, 2)
                        + 4 * a_4 * std::pow(t, 3)
                        + 5 * a_5 * std::pow(t, 4);
  double accel = 2 * a_2 + 6 * a_3 * t 
                         + 12 * a_4 * std::pow(t, 2)
                         + 20 * a_5 * std::pow(t, 3);
  return {pos, velocity, accel};
}

vector<double> getValidTrajectories(){
  // 2. Consider waypoint to be goal state
  // 3. Calculate T based on how far waypoint is

//double distance = std:pow(std::pow((next_waypoint_x - car_x), 2) + std::pow((next_waypoint_y - car_y), 2), 0.5); 
//double T = distance/car_speed;

  // 4. Solve JMT coefficients for s and d for given T

//vector<double> s_coeffs = solveCoeffs(start, end, T);
//vector<double> d_coeffs = solveCoeffs(start, end, T);

  // 4. Create multiple trajectories by solving trajectory equations from 0 to T

//vector<double> s_point = getTrajectoryPoint(T, s_coeffs);
//vector<double> d_point = getTrajectoryPoint(T, d_coeffs);

  // 5. Figure out how to apply costs to each trajctory
  // 6. Figure out how to add collision avoidance
  // 7. Choose a trajectory
  return {0.0};
}


