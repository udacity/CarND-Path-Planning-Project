//
// Created by sajith on 2/26/21.
//

#include "path_planner.h"

using namespace path_planning;

PathPlanner::PathPlanner(std::vector<MapWayPoint> &wayPoints) : m_wayPoints(wayPoints) {}

PathPlanner::~PathPlanner() {}

std::pair<std::vector<double>, std::vector<double >> PathPlanner::planPath(
        const path_planning::SimulatorRequest &simReqData)
{

    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    return std::make_pair(next_x_vals, next_y_vals);
}

