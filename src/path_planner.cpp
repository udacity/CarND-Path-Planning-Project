//
// Created by sajith on 2/26/21.
//

#include <cmath>
#include <limits>
#include <tuple>
#include "path_planner.h"
#include "helpers.h"

using namespace path_planning;

// How many nodes from a trajectory to keep in history
static constexpr unsigned TRAJECTORY_HISTORY_LENGTH = 100u;

// Speed limit
static constexpr double MPH_TO_METRES_PER_SECOND(const double mph)
{
    return mph / 3600.0 * 1609.344;
}

static constexpr double SPEED_LIMIT_METRES_PER_SECOND = MPH_TO_METRES_PER_SECOND(50.0 * 0.85);
static constexpr double LANE_SPEED_FORWARD_SCAN_RANGE = SPEED_LIMIT_METRES_PER_SECOND * 3.0;
static constexpr double D_LIMIT_FOR_LANE_CHANGE_PENALTY = 0.5;
static constexpr int LANE_CHANGE_PENALTY = 5;
static constexpr double LANE_CHANGE_CLEAR = SPEED_LIMIT_METRES_PER_SECOND * 0.5;
static constexpr double LANE_CHANGE_COST = 1.0;
static constexpr double PATH_DURATION_SECONDS = 2.5;
static constexpr double NODE_TRAVERSAL_RATE_SECONDS = 0.02;
static constexpr double SPEED_CHANGE = 0.1;

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
    std::pair<std::vector<double>, std::vector<double>> xy_trajectories =
            generateTrajectorySplines(simReqData.mainCar, laneSpeeds[m_targetLaneIndex], simReqData.previous_path_x,
                                      simReqData.previous_path_y);

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
    {
        m_historyMainX.resize(TRAJECTORY_HISTORY_LENGTH);
    }

    for (auto itr = m_lastY.begin(); itr != m_lastY.begin() + executedCommands; ++itr)
    {
        m_historyMainY.push_front(*itr);
    }

    if (m_historyMainY.size() > TRAJECTORY_HISTORY_LENGTH)
    {
        m_historyMainY.resize(TRAJECTORY_HISTORY_LENGTH);
    }
}


std::array<double, 3> PathPlanner::getLaneSpeeds(path_planning::MainCar mainCar,
                                                 const std::vector<OtherCar> &sensorFusions) const
{
    std::array<double, 3> speeds{};
    for (int i = 0; i < lanes.size(); ++i)
    {
        mainCar.d = lanes[i];
        int carAheadIndx = getCarAhead(mainCar, sensorFusions);
        speeds[i] = SPEED_LIMIT_METRES_PER_SECOND;
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
    for (int i = 0; i < sensorFusions.size(); ++i)
    {
        const OtherCar &otherCar = sensorFusions[i];
        if (std::abs(otherCar.d - mainCar.d) < 1.0 && otherCar.s >= mainCar.s)
        {
            double s_dist = otherCar.s - mainCar.s;
            if (s_dist < minSDist)
            {
                minSDist = s_dist;
                carAheadIndx = i;
            }
        }
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
    const double d_difference = m_targetLaneD - mainCar.d;
    double beforeEndS = mainCar.s + max_lane_speed * PATH_DURATION_SECONDS * 0.5;
    double beforeEndD = mainCar.d + d_difference * 0.5;

    double endS = mainCar.s + max_lane_speed * PATH_DURATION_SECONDS;
    double endD = m_targetLaneD;

    double beforeEndX, beforeEndY, endX, endY;
    std::tie(beforeEndX, beforeEndY) = getXY(beforeEndS, beforeEndD, m_wayPoints);
    std::tie(endX, endY) = getXY(endS, endD, m_wayPoints);

    std::vector<double> x_points, y_points;

    double x_reference = mainCar.x;
    double y_reference = mainCar.y;

    auto x_histItr = m_historyMainX.begin();
    auto y_histItr = m_historyMainY.begin();

    for (; x_histItr != m_historyMainX.end() && y_histItr != m_historyMainY.end(); ++x_histItr, ++y_histItr)
    {
        double histX = *x_histItr;
        double histY = *y_histItr;
        const double distToRef = distance(histX, histY, x_reference, y_reference);

        if (distToRef > 1.5)
        {
            x_points.insert(x_points.begin(), histX);
            y_points.insert(y_points.begin(), histY);

            x_reference = histX;
            y_reference = histY;
        }
    }

    // Push starting position
    x_points.emplace_back(mainCar.x);
    y_points.emplace_back(mainCar.y);

    if (previous_path_x.size() >= 5)
    {
        x_points.emplace_back(previous_path_x[4]);
        y_points.emplace_back(previous_path_y[4]);
    }

    if (previous_path_x.size() >= 10)
    {
        x_points.emplace_back(previous_path_x[9]);
        y_points.emplace_back(previous_path_y[9]);
    }

    //  Push endpoints
    x_points.emplace_back(beforeEndX);
    x_points.emplace_back(endX);

    y_points.emplace_back(beforeEndY);
    y_points.emplace_back(endY);

    std::vector<double> x_carPoints, y_carPoints;
    std::tie(x_carPoints, y_carPoints) = worldCoordinatesToVehicleCoordinates(mainCar, x_points, y_points);

    tk::spline spl;
    spl.set_points(x_carPoints, y_carPoints);

    auto numExecutedCommands = static_cast<int>(m_lastX.size() - previous_path_x.size());

    std::pair<std::vector<double>, std::vector<double>> xyPath = splineToPath(spl,
                                                                              mainCar,
                                                                              max_lane_speed,
                                                                              numExecutedCommands);

    return xyPath;
}

// https://www.mathworks.com/help/driving/ug/coordinate-systems.html

std::pair<double, double> PathPlanner::worldCoordinateToVehicleCoordinate(
        const path_planning::MainCar &mainCar, const double &worldX, const double &worldY)
{
    const double car_yawRadius = M_PI * mainCar.yaw / 180.0;

    double shiftX = worldX - mainCar.x;
    double shiftY = worldY - mainCar.y;

    double carX = shiftX * cos(-car_yawRadius) - shiftY * sin(-car_yawRadius);
    double carY = shiftX * sin(-car_yawRadius) + shiftY * cos(-car_yawRadius);

    return std::make_pair(carX, carY);
}

std::pair<std::vector<double>, std::vector<double>> PathPlanner::worldCoordinatesToVehicleCoordinates(
        const path_planning::MainCar &mainCar, const std::vector<double> &worldX, const std::vector<double> &worldY)
{
    std::vector<double> xCoords(worldX.size()), yCoords(worldY.size());

    for (int i = 0; i < worldX.size(); ++i)
    {
        double x, y;
        std::tie(x, y) = worldCoordinateToVehicleCoordinate(mainCar, worldX[i], worldY[i]);
        xCoords[i] = x;
        yCoords[i] = y;
    }

    return std::make_pair(xCoords, yCoords);
}


std::pair<std::vector<double>, std::vector<double>> PathPlanner::splineToPath(const tk::spline &spl,
                                                                              const path_planning::MainCar &mainCar,
                                                                              const double &maxLaneSpeed,
                                                                              const int &numCommandsExecuted)
{
    const double previousEndSpeed = m_prevSegmentSpeeds[numCommandsExecuted];

    std::vector<double> x_carPoints, y_carPoints;
    m_prevSegmentSpeeds.clear();
    double previousSpeed = previousEndSpeed;
    double t = NODE_TRAVERSAL_RATE_SECONDS;
    while (t < PATH_DURATION_SECONDS)
    {
        const double speedDiff = maxLaneSpeed - previousSpeed;
        const double speedChangeSign = speedDiff >= 0 ? 1.0 : -1.0;

        double newSpeed = previousSpeed + speedChangeSign * SPEED_CHANGE;

        if (newSpeed > SPEED_LIMIT_METRES_PER_SECOND)
        {
            newSpeed = SPEED_LIMIT_METRES_PER_SECOND;
        }
        else if (newSpeed < 0.0)
        {
            newSpeed = 0.0;
        }

        previousSpeed = newSpeed;
        m_prevSegmentSpeeds.emplace_back(previousSpeed);
        const double x = newSpeed * t;

        x_carPoints.emplace_back(x);
        y_carPoints.emplace_back(spl(x));

        t += NODE_TRAVERSAL_RATE_SECONDS;
    }


    std::pair<std::vector<double>, std::vector<double>> xyPoints = carCoordinatesToWorldCoordinates(mainCar,
                                                                                                    x_carPoints,
                                                                                                    y_carPoints);

    return xyPoints;
}

std::pair<double, double> PathPlanner::carCoordinateToWorldCoordinate(const path_planning::MainCar &mainCar,
                                                                      const double &carX, const double &carY)
{
    const double carYawRadius = M_PI * mainCar.yaw / 180.0;

    double worldX = carX * cos(carYawRadius) - carY * sin(carYawRadius);
    double worldY = carX * sin(carYawRadius) + carY * cos(carYawRadius);

    worldX += mainCar.x;
    worldY += mainCar.y;

    return std::make_pair(worldX, worldY);
}

std::pair<std::vector<double>, std::vector<double>> PathPlanner::carCoordinatesToWorldCoordinates(
        const path_planning::MainCar &mainCar, const std::vector<double> &carX, const std::vector<double> &carY)
{
    vector<double> xWorld(carX.size()), yWorld(carY.size());

    for (int i = 0; i < carX.size(); ++i)
    {
        double x, y;
        std::tie(x, y) = carCoordinateToWorldCoordinate(mainCar, carX[i], carY[i]);
        xWorld[i] = x;
        yWorld[i] = y;
    }

    return std::make_pair(xWorld, yWorld);
}


