#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "path_planner.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

MapWayPoint generateMapWayPoints()
{
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line))
    {
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

    MapWayPoint mapWayPoint;
    mapWayPoint.map_waypoints_dx = map_waypoints_dx;
    mapWayPoint.map_waypoints_dy = map_waypoints_dy;
    mapWayPoint.map_waypoints_s = map_waypoints_s;
    mapWayPoint.map_waypoints_x = map_waypoints_x;
    mapWayPoint.map_waypoints_y = map_waypoints_y;

    return mapWayPoint;
}

int main()
{
    uWS::Hub h;
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    MapWayPoint mapWayPoint = generateMapWayPoints();
    PathPlanner pathPlanner;
    pathPlanner.mapWayPoint = mapWayPoint;

    h.onMessage([&mapWayPoint, &pathPlanner]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode)
                {
                    // "42" at the start of the message means there's a websocket message event.
                    // The 4 signifies a websocket message
                    // The 2 signifies a websocket event
                    if (length && length > 2 && data[0] == '4' && data[1] == '2')
                    {
                        auto s = hasData(data);

                        if (s != "")
                        {
                            std::cout << s << "\n\n";
                            auto j = json::parse(s);

                            string event = j[0].get<string>();

                            if (event == "telemetry")
                            {
                                // j[1] is the data JSON object

                                CarInfo carInfo;
                                // Main car's localization Data
                                double car_x = j[1]["x"];
                                double car_y = j[1]["y"];
                                double car_s = j[1]["s"];
                                double car_d = j[1]["d"];
                                double car_yaw = j[1]["yaw"];
                                double car_speed = j[1]["speed"];


                                carInfo.car_x = j[1]["x"];
                                carInfo.car_y = j[1]["y"];
                                carInfo.car_s = j[1]["s"];
                                carInfo.car_d = j[1]["d"];
                                carInfo.car_yaw = j[1]["yaw"];
                                carInfo.car_speed = j[1]["speed"];

                                pathPlanner.carInfo = carInfo;

                                PreviousPathInfo previousPathInfo;
                                // Previous path data given to the Planner
                                auto previous_path_x = j[1]["previous_path_x"];
                                auto previous_path_y = j[1]["previous_path_y"];
                                // Previous path's end s and d values
                                double end_path_s = j[1]["end_path_s"];
                                double end_path_d = j[1]["end_path_d"];

                                // Previous path data given to the Planner
                                previousPathInfo.previous_path_x = j[1]["previous_path_x"];
                                previousPathInfo.previous_path_y = j[1]["previous_path_y"];
                                // Previous path's end s and d values
                                previousPathInfo.end_path_s = j[1]["end_path_s"];
                                previousPathInfo.end_path_d = j[1]["end_path_d"];

                                pathPlanner.previousPathInfo = previousPathInfo;

                                // Sensor Fusion Data, a list of all other cars on the same side
                                //   of the road.
                                auto sensor_fusion = j[1]["sensor_fusion"];
                                pathPlanner.sensor_fusion = j[1]["sensor_fusion"];

                                json msgJson;


                                vector<double> next_x_vals;
                                vector<double> next_y_vals;

                                /**
                                 * TODO: define a path made up of (x,y) points that the car will visit
                                 *   sequentially every .02 seconds
                                 */

                                pathPlanner.generate_next_paths(next_x_vals, next_y_vals);

                                msgJson["next_x"] = next_x_vals;
                                msgJson["next_y"] = next_y_vals;

                                auto msg = "42[\"control\"," + msgJson.dump() + "]";

                                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                            }  // end "telemetry" if
                        }
                        else
                        {
                            // Manual driving
                            std::string msg = "42[\"manual\",{}]";
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }
                    }  // end websocket if
                }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
                   {
                       std::cout << "Connected!!!" << std::endl;
                   });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length)
                      {
                          ws.close();
                          std::cout << "Disconnected" << std::endl;
                      });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}