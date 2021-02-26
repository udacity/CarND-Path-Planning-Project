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

            class PathPlanner
            {
            public:
                PathPlanner(std::vector<MapWayPoint> &map_wayPoints);

                virtual ~PathPlanner();

            private:
                const std::vector<MapWayPoint> m_wayPoints;
            };

    }
#endif //PATH_PLANNER_H