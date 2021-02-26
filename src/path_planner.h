//
// Created by sajith on 2/26/21.
//

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

namespace path_planning
    {

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
            };

    }
#endif //PATH_PLANNER_H