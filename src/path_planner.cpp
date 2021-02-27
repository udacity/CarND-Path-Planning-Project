//
// Created by sajith on 2/26/21.
//

#include <cmath>
#include <limits>
#include "path_planner.h"
#include "spline.h"

using namespace path_planning;

// How many nodes from a trajectory to keep in history
const int TRAJECTORY_HISTORY_LENGTH = 100;
// Speed limit
const double SPEED_LIMIT_METRES_PER_SECOND = 50.0 * 0.9;
const double LANE_SPEED_FORWARD_SCAN_RANGE = SPEED_LIMIT_METRES_PER_SECOND * 3.0;
const double D_LIMIT_FOR_LANE_CHANGE_PENALTY = 0.5;
const int LANE_CHANGE_PENALTY = 5;
const double LANE_CHANGE_CLEAR = SPEED_LIMIT_METRES_PER_SECOND * 0.5;
const double LANE_CHANGE_COST = 1.0;

const std::array<double, 3> lanes{D_LEFT_LANE, D_MIDDLE_LANE, D_RIGHT_LANE};


PathPlanner::PathPlanner(std::vector<MapWayPoint> &wayPoints) : m_wayPoints(wayPoints) {}

PathPlanner::~PathPlanner() = default;

std::pair<std::vector<double>, std::vector<double >> PathPlanner::planPath(
        const path_planning::SimulatorRequest &simReqData)
{

    // Update trajectory history
    updateTrajectoryHistory(simReqData);

    // Get the lane speeds
    std::array<double, 3> laneSpeeds = getLaneSpeeds(simReqData.mainCar, simReqData.otherCars);
    // Scheduling the lane's changes
    scheduleLaneChange(simReqData.mainCar, laneSpeeds, simReqData.otherCars);

    // generate Spiline x and y trajectories
    auto xy_trajectories = generateTrajectorySplines(simReqData.mainCar, laneSpeeds[m_targetLaneIndex],
                                                     simReqData.previous_path_x, simReqData.previous_path_y);

    m_lastX = xy_trajectories.first;
    m_lastY = xy_trajectories.second;
    return xy_trajectories;
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
    double minSDist = std::numeric_limits<double>::max();
    int i = 0;
    for (auto const &otherCar: sensorFusions)
    {
        if (std::abs(otherCar.d - mainCar.d) < 1.0 && otherCar.s >= mainCar.s)
        {
            double s_dist = otherCar.s - mainCar.s;
            if (s_dist < minSDist)
            {
                minSDist = s_dist;
                carAheadIndx = i;
            }
        }

        ++i;
    }
    return carAheadIndx;
}

void PathPlanner::scheduleLaneChange(const path_planning::MainCar &mainCar, const std::array<double, 3> &laneSpeeds,
                                     const std::vector<path_planning::OtherCar> &sensorFusions)
{
    if (std::abs(mainCar.d - m_targetLaneD) > D_LIMIT_FOR_LANE_CHANGE_PENALTY)
    {
        m_laneChangeDelay = LANE_CHANGE_PENALTY;
    }
    else if (m_laneChangeDelay != 0)
    {
        --m_laneChangeDelay;
    }
    else
    {
        if (m_targetLaneD == D_LEFT_LANE && laneSpeeds[1] - LANE_CHANGE_COST > laneSpeeds[0]
            && !isLaneBlocked(D_MIDDLE_LANE, mainCar, sensorFusions))
        {
            m_targetLaneD = D_MIDDLE_LANE;
            m_targetLaneIndex = 1;
            m_laneChangeDelay = LANE_CHANGE_PENALTY;
        }
        else if (m_targetLaneD == D_RIGHT_LANE && laneSpeeds[1] - LANE_CHANGE_COST > laneSpeeds[2]
                 && !isLaneBlocked(D_MIDDLE_LANE, mainCar, sensorFusions))
        {
            m_targetLaneD = D_MIDDLE_LANE;
            m_targetLaneIndex = 1;
            m_laneChangeDelay = LANE_CHANGE_PENALTY;
        }
        else if (m_targetLaneD == D_MIDDLE_LANE &&
                 (laneSpeeds[0] - LANE_CHANGE_COST > laneSpeeds[1] || laneSpeeds[2] - LANE_CHANGE_COST > laneSpeeds[1]))
        {
            if (laneSpeeds[0] > laneSpeeds[2] && !isLaneBlocked(D_LEFT_LANE, mainCar, sensorFusions))
            {
                m_targetLaneD = D_LEFT_LANE;
                m_targetLaneIndex = 0;
                m_laneChangeDelay = LANE_CHANGE_PENALTY;
            }
            else if (!isLaneBlocked(D_RIGHT_LANE, mainCar, sensorFusions))
            {
                m_targetLaneD = D_RIGHT_LANE;
                m_targetLaneIndex = 2;
                m_laneChangeDelay = LANE_CHANGE_PENALTY;
            }
        }

    }

}


bool PathPlanner::isLaneBlocked(const double &targetLaneD, const path_planning::MainCar &mainCar,
                                const std::vector<path_planning::OtherCar> &sensorFusions) const
{
    for (const auto &otherCar: sensorFusions)
    {
        if (std::abs(otherCar.d - targetLaneD) < 1.0 && std::abs(otherCar.s - mainCar.s) <= LANE_CHANGE_CLEAR)
        {
            return true;
        }
    }

    return false;
}

std::pair<std::vector<double>, std::vector<double >> PathPlanner::generateTrajectorySplines(
        const path_planning::MainCar &mainCar, const double &max_lane_speed, const std::vector<double> &previous_path_x,
        const std::vector<double> &previous_path_y)
{


}