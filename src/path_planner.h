//
// Created by sajith on 2/26/21.
//

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <array>
#include <deque>
#include "spline.h"
#include "common_utils.h"


namespace path_planning
    {
            const double D_LEFT_LANE = 2.0;
            const double D_MIDDLE_LANE = 6.0;
            const double D_RIGHT_LANE = 6.0;

            class PathPlanner
            {
            public:
                explicit PathPlanner(std::vector<MapWayPoint> &map_wayPoints);

                virtual ~PathPlanner();

                std::pair<std::vector<double>, std::vector<double>> planPath(const SimulatorRequest &simReqData);

            private:
                const std::vector<MapWayPoint> m_wayPoints;

                void updateTrajectoryHistory(const SimulatorRequest &simReqData);

                std::array<double, 3> getLaneSpeeds(MainCar mainCar, const std::vector<OtherCar> &sensorFusions) const;

                int getCarAhead(const MainCar &mainCar, const std::vector<OtherCar> &sensorFusions) const;

                void scheduleLaneChange(const MainCar &mainCar, const std::array<double, 3> &laneSpeeds,
                                        const std::vector<OtherCar> &sensorFusions);

                bool isLaneBlocked(const double &targetLaneD, const MainCar &mainCar,
                                   const std::vector<OtherCar> &sensorFusions) const;

                std::pair<std::vector<double>, std::vector<double >> generateTrajectorySplines(const MainCar &mainCar,
                                                                                               const double &max_lane_speed,
                                                                                               const std::vector<double> &previous_path_x,
                                                                                               const std::vector<double> &previous_path_y);

                std::pair<double, double> worldCoordinateToVehicleCoordinate(const MainCar &mainCar,
                                                                             const double &worldX,
                                                                             const double &worldY);

                std::pair<std::vector<double>, std::vector<double>> worldCoordinatesToVehicleCoordinates(
                        const MainCar &mainCar,
                        const std::vector<double> &worldX,
                        const std::vector<double> &worldY);

                std::pair<std::vector<double>, std::vector<double>> splineToPath(const tk::spline &spl,
                                                                                 const MainCar &mainCar,
                                                                                 const double &maxLaneSpeed,
                                                                                 const int numCommandsExecuted);

                std::pair<double, double> carCoordinateToWorldCoordinate(const MainCar &mainCar,
                                                                         const double &carX,
                                                                         const double &carY);

                std::pair<std::vector<double>, std::vector<double>> carCoordinatesToWorldCoordinates(
                        const MainCar &mainCar,
                        const std::vector<double> &carX,
                        const std::vector<double> &carY);


                std::vector<double> m_lastX;
                std::vector<double> m_lastY;

                // History
                std::deque<double> m_historyMainX;
                std::deque<double> m_historyMainY;

                double m_targetLaneD{D_MIDDLE_LANE};
                unsigned int m_laneChangeDelay = 5u;
                int m_targetLaneIndex{1};

                // Holds previous speeds, including previous starting speed
                std::vector<double> m_prevSegmentSpeeds{{0.0}};
            };

    }
#endif //PATH_PLANNER_H