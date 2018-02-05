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
#include "vehicle.h"
#include "helpers.h"

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
    } 
    else if (b1 != string::npos && b2 != string::npos) 
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

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

    // 0 = left
    // 1 = middle
    // 2 = right
    int lane = 1;

    // reference MPH
    double ref_vel = 0.0;

    // Create a new vehicle object here and update global map of waypoints
    Vehicle ego;
    // ego.updateGlobalMap(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);

    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) 
    {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') 
        {
            auto s = hasData(data);

            if (s != "") 
            {
                auto j = json::parse(s);
                
                string event = j[0].get<string>();
        
                if (event == "telemetry") 
                {
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

                    std::cout << "START Vehicle Health: " << std::endl;
                    std::cout << "X: " << car_x << std::endl;
                    std::cout << "Y: " << car_y << std::endl;
                    std::cout << "S: " << car_s << std::endl;
                    std::cout << "D: " << car_d << std::endl;
                    std::cout << "YAW: " << car_yaw << std::endl;
                    std::cout << "SPEED: " << car_speed << std::endl;
                    std::cout << "Length of SF: " << sensor_fusion.size() << std::endl;
                    std::cout << "END Vehicle Health" << std::endl;
                    std::cout << std::endl;
                    // update Vehicle object x, y, s, d, yaw, speed, path
                    // update sensor fusion information

                    // START -- Intermediate Planner
                    int prev_size = previous_path_x.size();

                    // sensor fusion
                    if (prev_size > 0) 
                    {
                      car_s = end_path_s;
                    }

                    bool too_close = false;

                    // find a reference value to use
                    for (int i = 0; i < sensor_fusion.size(); i++) 
                    {
                        // car is in my lane
                        float d = sensor_fusion[i][6];
                        if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) 
                        {
                            double vx = sensor_fusion[i][3];
                            double vy = sensor_fusion[i][4];
                            double check_speed = sqrt(vx*vx + vy*vy);
                            double check_car_s = sensor_fusion[i][5];

                            // get the vehicle in front of you

                            // if using previous points can project s value outward
                            // check s values greater than mine and s gap
                            check_car_s += ((double) prev_size * 0.02 * check_speed);
                            // in front of and within critical zone
                            if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) 
                            {
                                // move to the right/left
                                too_close = true;

                                // initiate a lane change move
                                if (lane == 1) 
                                {
                                    lane = 0;
                                } 
                            }
                        }
                    }

                    // accel. with +/- 5 m/s^2
                    if (too_close) 
                    {
                        ref_vel -= 0.224;
                    }
                    else if (ref_vel < 49.5)
                    {
                        ref_vel += 0.224;
                    }

                    // widely, evenly spaced coordinates
                    // interpolated with a spline
                    vector<double> ptsx;
                    vector<double> ptsy;

                    // either reference the starting point as where the car is
                    // or at the previous path's end point
                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);

                    // use the car as the starting reference
                    if (prev_size < 2) 
                    {
                        // Use two points that make the path tangent to the car
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                    }
                    // use the previous path's end point as starting reference
                    else 
                    {
                        // Redefine reference state as previous end point
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        // Use the two points that make path 
                        // tangent to the previous path's end point
                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }

                    // In Frenet add evenly 30m spaced points ahead of the starting reference
                    vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    // transform to local coordinates
                    for (int i = 0; i < ptsx.size(); i++) 
                    {
                        // shift reference angle to 0 degrees
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
                    }

                    // create a spline and set (x, y) points
                    tk::spline s;
                    s.set_points(ptsx, ptsy);

                    // define points for the planner
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    // start with all of the previous path points from last time
                    // add new points to the existing path points 
                    // smoothen the transition from one path to another path
                    for (int i = 0; i < previous_path_x.size(); i++) 
                    {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    // Implement JMT trajectory here 

                    // calculate how to break up spline points so that we 
                    // travel at our desired reference velociy
                    double target_x = 30.0; // horizon x
                    double target_y = s(target_x); // horizon y
                    double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
                    double x_add_on = 0;

                    for (int i = 1; i <= 50 - previous_path_x.size(); i++) 
                    {
                        // convert to m/s
                        double N = (target_dist / (0.02 * ref_vel / 2.24));
                        double x_point = x_add_on + (target_x) / N; // individual x point
                        double y_point = s(x_point); // individual y point

                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        // shift + rotation -- convert back to global coordinates
                        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                        x_point += ref_x;
                        y_point += ref_y;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                    }

                    json msgJson;

                    // next_x_vals = vehicle.getTrajectoryX();
                    // next_y_vals = vehicle.getTrajectoryY();

            // END -- Intermediate Planner

                    // START -- Basic Planner
                    // double dist_inc = 0.3;
                    // for (int i = 0; i < 50; i++) {
                    //   // i+1 for the next location
                    //   double next_s = car_s+(i+1)*dist_inc;
                    //   double next_d = 6;
                    //   vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                    //   next_x_vals.push_back(xy[0]);
                    //   next_y_vals.push_back(xy[1]);

                    //   // next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
                    //   // next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
                    // }
                    // END -- Basic Planner

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
                }
            } 
            else 
            {
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
        if (req.getUrl().valueLength == 1) 
        {
            res->end(s.data(), s.length());
        } 
        else 
        {
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
    } 
    else 
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
