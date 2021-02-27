//
// Created by sajith on 2/26/21.
//

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <array>
#include <deque>


namespace path_planning
    {
            const double D_LEFT_LANE = 2.0;
            const double D_MIDDLE_LANE = 6.0;
            const double D_RIGHT_LANE = 6.0;

            struct MapWayPoint
            {
                double x;
                double y;
                float s;
                float normX;
                float normY;
            };

            // Our car
            struct MainCar
            {
                double x;
                double y;
                double s;
                double d;
                double yaw;
                double speed;
            };

            // Sensor fusion cars
            struct OtherCar
            {
                double id;
                double x;
                double y;
                double dx;
                double dy;
                double s;
                double d;
            };

            // Simulator request data
            struct SimulatorRequest
            {
                // Main car
                MainCar mainCar;

                // Previous path data given to the Planner
                std::vector<double> previous_path_x;
                std::vector<double> previous_path_y;

                // Previous path's end s and d values
                double end_path_s;
                double end_path_d;

                // Sensor fusion data
                std::vector<OtherCar> otherCars;
            };

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

                bool isLaneBlocked(const double targetLaneD, const MainCar &mainCar,
                                   const std::vector<OtherCar> &sensorFusions) const;

                std::vector<double> m_lastX;
                std::vector<double> m_lastY;

                // History
                std::deque<double> m_historyMainX;
                std::deque<double> m_historyMainY;

                double m_targetLaneD{D_MIDDLE_LANE};
                unsigned int m_laneChangeDelay = 5u;
                int m_targetLaneIndex{1};
            };

    }
#endif //PATH_PLANNER_H