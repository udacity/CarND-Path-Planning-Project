//
// Created by sajith on 2/26/21.
//

#include <cmath>
#include "path_planner.h"

using namespace path_planning;

PathPlanner::PathPlanner(std::vector<MapWayPoint> &wayPoints) : m_wayPoints(wayPoints) {}

PathPlanner::~PathPlanner() {}

std::pair<std::vector<double>, std::vector<double >> PathPlanner::planPath(
        const path_planning::SimulatorRequest &simReqData)
{

    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    double dist_inc = 0.5;
    for (int i = 0; i < 50; ++i)
    {
        next_x_vals.push_back(simReqData.mainCar.x + (dist_inc * i) * cos(simReqData.mainCar.yaw * 180 / M_PI));
        next_y_vals.push_back(simReqData.mainCar.y + (dist_inc * i) * sin(simReqData.mainCar.yaw * 180 / M_PI));
    }

    return std::make_pair(next_x_vals, next_y_vals);
}

