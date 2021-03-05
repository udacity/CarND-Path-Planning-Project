#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "common_utils.h"

// for convenience
using std::string;
using std::vector;

namespace path_planning
    {


            //
            // Helper functions related to waypoints and converting from XY to Frenet
            //   or vice versa
            //

            // For converting back and forth between radians and degrees.
            constexpr double pi() { return M_PI; }

            double deg2rad(double x) { return x * pi() / 180; }

            double rad2deg(double x) { return x * 180 / pi(); }

            // Calculate distance between two points
            double distance(double x1, double y1, double x2, double y2)
            {
                return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
            }

            // Calculate closest waypoint to current x, y position
            int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                                const vector<double> &maps_y)
            {
                double closestLen = 100000; //large number
                int closestWaypoint = 0;

                for (int i = 0; i < maps_x.size(); ++i)
                {
                    double map_x = maps_x[i];
                    double map_y = maps_y[i];
                    double dist = distance(x, y, map_x, map_y);
                    if (dist < closestLen)
                    {
                        closestLen = dist;
                        closestWaypoint = i;
                    }
                }

                return closestWaypoint;
            }

            // Returns next waypoint of the closest waypoint
            int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                             const vector<double> &maps_y)
            {
                    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

                double map_x = maps_x[closestWaypoint];
                double map_y = maps_y[closestWaypoint];

                double heading = atan2((map_y - y), (map_x - x));

                double angle = fabs(theta - heading);
                angle = std::min(2 * pi() - angle, angle);

                if (angle > pi() / 2)
                {
                    ++closestWaypoint;
                    if (closestWaypoint == maps_x.size())
                    {
                        closestWaypoint = 0;
                    }
                }

                return closestWaypoint;
            }

            // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
            vector<double> getFrenet(double x, double y, double theta,
                                     const vector<double> &maps_x,
                                     const vector<double> &maps_y)
            {
                int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

                int prev_wp;
                prev_wp = next_wp - 1;
                if (next_wp == 0)
                {
                    prev_wp = maps_x.size() - 1;
                }

                double n_x = maps_x[next_wp] - maps_x[prev_wp];
                double n_y = maps_y[next_wp] - maps_y[prev_wp];
                double x_x = x - maps_x[prev_wp];
                double x_y = y - maps_y[prev_wp];

                // find the projection of x onto n
                double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
                double proj_x = proj_norm * n_x;
                double proj_y = proj_norm * n_y;

                double frenet_d = distance(x_x, x_y, proj_x, proj_y);

                //see if d value is positive or negative by comparing it to a center point
                double center_x = 1000 - maps_x[prev_wp];
                double center_y = 2000 - maps_y[prev_wp];
                double centerToPos = distance(center_x, center_y, x_x, x_y);
                double centerToRef = distance(center_x, center_y, proj_x, proj_y);

                if (centerToPos <= centerToRef)
                {
                    frenet_d *= -1;
                }

                // calculate s value
                double frenet_s = 0;
                for (int i = 0; i < prev_wp; ++i)
                {
                    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
                }

                frenet_s += distance(0, 0, proj_x, proj_y);

                return {frenet_s, frenet_d};
            }


            std::pair<double, double> getXY(double s, double d, const std::vector<MapWayPoint> &waypoints)
            {
                int prevWpIndex = -1;
                while (s > waypoints[prevWpIndex + 1].s && (prevWpIndex < (int) (waypoints.size() - 1)))
                {
                    ++prevWpIndex;
                }
                assert(prevWpIndex != -1);

                const MapWayPoint &prevWp = waypoints[prevWpIndex];
                const MapWayPoint &nextWp = waypoints[(prevWpIndex + 1) % waypoints.size()];

                double heading = atan2(nextWp.y - prevWp.y, nextWp.x - prevWp.x);

                // the x,y,s along the segment
                double segS = (s - prevWp.s);
                double segX = prevWp.x + segS * cos(heading);
                double segY = prevWp.y + segS * sin(heading);

                double perpHeading = heading - pi() / 2;

                double x = segX + d * cos(perpHeading);
                double y = segY + d * sin(perpHeading);

                return std::make_pair(x, y);
            }

    }
#endif  // HELPERS_H