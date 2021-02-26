#include <uWS/uWS.h>
#include <fstream>
#include "Eigen-3.3/Eigen/Core"
#include "helpers.h"
#include "json.hpp"
#include "path_planner.h"

using namespace path_planning;

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

static const std::string WAYPOINT_MAP_FILE = "../data/highway_map.csv";

vector<MapWayPoint> readMapWayPoints(const std::string &mapFile);

SimulatorRequest extractSimulatorRequestData(const nlohmann::json &);

int main()
{
    uWS::Hub h;
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;
    vector<MapWayPoint> mapWayPoints = readMapWayPoints(WAYPOINT_MAP_FILE);
    PathPlanner pathPlanner(mapWayPoints);

    h.onMessage([&pathPlanner]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode)
                {
                    // "42" at the start of the message means there's a websocket message event.
                    // The 4 signifies a websocket message
                    // The 2 signifies a websocket event
                    if (length > 2 && data[0] == '4' && data[1] == '2')
                    {
                        auto s = hasData(data);

                        if (!s.empty())
                        {
                            auto j = json::parse(s);

                            string event = j[0].get<string>();

                            if (event == "telemetry")
                            {
                                SimulatorRequest simulatorRequest = extractSimulatorRequestData(j);
                                auto res = pathPlanner.planPath(simulatorRequest);

                                json msgJson;
                                msgJson["next_x"] = res.first;
                                msgJson["next_y"] = res.second;

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


vector<MapWayPoint> readMapWayPoints(const std::string &mapFile)
{
    std::vector<MapWayPoint> waypoints;

    std::ifstream inStreamMap(mapFile.c_str(), std::ifstream::in);

    std::string line;
    while (getline(inStreamMap, line))
    {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float normX;
        float normY;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> normX;
        iss >> normY;
        waypoints.push_back({x, y, s, normX, normY});
    }
    return waypoints;
}

SimulatorRequest extractSimulatorRequestData(const nlohmann::json &j)
{
    // j[1] is the data JSON object
    // Main car's localization Data
    MainCar mainCar{
            .x =  j[1]["x"],
            .y =  j[1]["y"],
            .s = j[1]["s"],
            .d = j[1]["d"],
            .yaw = j[1]["yaw"],
            .speed = j[1]["speed"]
    };

    // Previous path data given to the Planner
    std::vector<double> previous_path_x = j[1]["previous_path_x"];
    std::vector<double> previous_path_y = j[1]["previous_path_y"];
    // Previous path's end s and d values
    double end_path_s = j[1]["end_path_s"];
    double end_path_d = j[1]["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side
    //   of the road.
    std::vector<OtherCar> otherCars;
    for (const std::vector<double> &otherCarVector : j[1]["sensor_fusion"])
    {
        otherCars.push_back({
                                    .id = otherCarVector[0],
                                    .x = otherCarVector[1],
                                    .y = otherCarVector[2],
                                    .dx = otherCarVector[3],
                                    .dy = otherCarVector[4],
                                    .s = otherCarVector[5],
                                    .d = otherCarVector[6]
                            });
    }


    SimulatorRequest simulatorRequest{
            .mainCar = mainCar,
            .previous_path_x = previous_path_x,
            previous_path_y = previous_path_y,
            .end_path_s = end_path_s,
            .end_path_d = end_path_d,
            .otherCars = otherCars
    };

    return simulatorRequest;
}