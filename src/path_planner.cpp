//
// Created by sajith on 2/26/21.
//

#include <cmath>
#include <limits>
#include "path_planner.h"

using namespace path_planning;

// How many nodes from a trajectory to keep in history
const int TRAJECTORY_HISTORY_LENGTH = 100;
// Speed limit
const double SPEED_LIMIT_METRES_PER_SECOND = 50.0 * 0.9;
const double LANE_SPEED_FORWARD_SCAN_RANGE = SPEED_LIMIT_METRES_PER_SECOND * 3.0;
const double D_LEFT_LANE = 2.0;
const double D_MIDDLE_LANE = 6.0;
const double D_RIGHT_LANE = 6.0;

const std::array<double, 3> lanes{D_LEFT_LANE, D_MIDDLE_LANE, D_RIGHT_LANE};


PathPlanner::PathPlanner(std::vector<MapWayPoint> &wayPoints) : m_wayPoints(wayPoints) {}

PathPlanner::~PathPlanner() {}

std::pair<std::vector<double>, std::vector<double >> PathPlanner::planPath(
        const path_planning::SimulatorRequest &simReqData)
{

    // Update trajectory history
    updateTrajectoryHistory(simReqData);

    std::array<double, 3> laneSpeeds = getLaneSpeeds(simReqData.mainCar, simReqData.otherCars);
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

/**
 * Update trajectory history
 * @param simReqData : Simulator sending request's data
 */
void PathPlanner::updateTrajectoryHistory(const path_planning::SimulatorRequest &simReqData)
{
    auto executedCommands = m_lastX.size() - simReqData.previous_path_x.size();

    for (auto itr = m_lastX.begin(); itr != m_lastX.begin() + executedCommands; ++itr)
    {
        m_historyMainX.push_front(*itr);
    }
    if (m_historyMainX.size() > TRAJECTORY_HISTORY_LENGTH)
        m_historyMainX.resize(TRAJECTORY_HISTORY_LENGTH);

    for (auto itr = m_lastY.begin(); itr != m_lastY.begin() + executedCommands; ++itr)
    {
        m_historyMainY.push_front(*itr);
    }

    if (m_historyMainY.size() > TRAJECTORY_HISTORY_LENGTH)
        m_historyMainY.resize(TRAJECTORY_HISTORY_LENGTH);
}


std::array<double, 3> PathPlanner::getLaneSpeeds(path_planning::MainCar mainCar,
                                                 const std::vector<OtherCar> &sensorFusions) const
{

    std::array<double, 3> speeds{SPEED_LIMIT_METRES_PER_SECOND, SPEED_LIMIT_METRES_PER_SECOND,
                                 SPEED_LIMIT_METRES_PER_SECOND};
    for (int i = 0; i < lanes.size(); ++i)
    {
        mainCar.d = lanes[i];
        int carAheadIndx = getCarAhead(mainCar, sensorFusions);
        if (carAheadIndx != -1 && sensorFusions[carAheadIndx].s - mainCar.s <= LANE_SPEED_FORWARD_SCAN_RANGE)
        {
            const auto &carAhead = sensorFusions[carAheadIndx];
            speeds[i] = std::sqrt(pow(carAhead.dx, 2) + pow(carAhead.dy, 2));
        }
    }

    return speeds;
}


int PathPlanner::getCarAhead(const path_planning::MainCar &mainCar,
                             const std::vector<path_planning::OtherCar> &sensorFusions) const
{
    int carAheadIndx = -1;
    double minDist = std::numeric_limits<double>::max();
    int i = 0;
    for (auto const &otherCar: sensorFusions)
    {
        if (std::abs(otherCar.d - mainCar.d) < 1.0 && otherCar.s >= mainCar.s)
        {
            double s_dist = otherCar.s - mainCar.s;
            if (s_dist < minDist)
            {
                minDist = s_dist;
                carAheadIndx = i;
            }
        }

        ++i;
    }
    return carAheadIndx;
}