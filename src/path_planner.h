//
// Created by sajith on 2/26/21.
//

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

struct CarInfo
{
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;
};

struct MapWayPoint
{
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
};

struct PreviousPathInfo
{
    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;
    double end_path_s;
    double end_path_d;
};

class PathPlanner
{
public:
    PathPlanner();

    virtual ~PathPlanner();

    std::pair<std::vector<double>, std::vector<double >> generate_next_paths();

private:
    std::vector<double> sensor_fusion;
};

#endif //PATH_PLANNER_H
