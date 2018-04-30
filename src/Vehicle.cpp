#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"
#include "spline.h"
#include "helpers.h"

const float REACH_GOAL = pow(10, 6);
const float EFFICIENCY = pow(10, 5);
const float SAFE_DISTANCE = 30.0;


Vehicle::Vehicle(int lane, double refVelocity)
{
    std::cout << "New ego vehicle" << std::endl;
    std::cout << "Lane: " << lane << std::endl;
    std::cout << "Ref Velocity: " << refVelocity << std::endl;
    this->lane = lane;
    this->refVelocity = refVelocity;
}


Vehicle::~Vehicle() {}


void Vehicle::updateGlobalMap(std::vector<double> mapWaypointsX, std::vector<double> mapWaypointsY, std::vector<double> mapWaypointsS, std::vector<double> mapWaypointsDX, std::vector<double> mapWaypointsDY)
{
    this->mapWaypointsX = mapWaypointsX;
    this->mapWaypointsY = mapWaypointsY;
    this->mapWaypointsS = mapWaypointsS;
    this->mapWaypointsDX = mapWaypointsDX;
    this->mapWaypointsDY = mapWaypointsDY;
}


void Vehicle::updateVehicleState(double x, double y, double s, double d, double yaw, double speed, std::vector<double> previousPathX, std::vector<double> previousPathY, double endPathS, double endPathD, std::vector<std::vector<double>> sensorFusion)
{
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
    this->speed = speed;
    this->previousPathX = previousPathX;
    this->previousPathY = previousPathY;
    this->endPathS = endPathS;
    this->endPathD = endPathD;
    this->sensorFusion = sensorFusion;
}


/* Trajectory Generation */
std::vector<std::vector<double>> Vehicle::getSmoothSplineTrajectory()
{
    // START -- Intermediate Planner
    int prev_size = previousPathX.size();

    // sensor fusion
    if (prev_size > 0) 
    {
        s = endPathS;
    }

    bool too_close = false;
    // find a reference value to use
    for (int i = 0; i < sensorFusion.size(); i++) 
    {
        // car is in my lane
        double d = sensorFusion[i][6];
        if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) 
        {
            double vx = sensorFusion[i][3];
            double vy = sensorFusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensorFusion[i][5];

            // car's unique ID
            // car's x position in map coordinates
            // car's y position in map coordinates
            // car's x velocity in m/s
            // car's y velocity in m/s
            // car's s position in frenet coordinates
            // car's d position in frenet coordinates

            // predict s' for next vehicle in this lane
            check_car_s += ((double) prev_size * 0.02 * check_speed);

            // in front of and within critical zone
            if ((check_car_s > s) && ((check_car_s - s) < SAFE_DISTANCE)) 
            {
                // vehicle is too close
                too_close = true;

                // initiate a lane change move 
                bool maneuver_complete = false;

                // look to the right
                // lane change can complete if there is no vehicle within the +/- car length
                if (lane - 1 >= 0)
                {
                    // check to the front within L
                    // check to the side 
                    // check to the back within L
                    std::cout << "Checking left" << std::endl;
                    for (int j = 0; j < sensorFusion.size(); j++) 
                    {
                        double vx_left = sensorFusion[j][3];
                        double vy_left = sensorFusion[j][4];
                        double speed_left = sqrt(vx_left*vx_left + vy_left*vy_left);
                        double s_left = sensorFusion[j][5];
                        double d_left = sensorFusion[j][6];

                        // d_left is between the next lane
                        if ( (d_left < 2 + 4 * (lane-1) + 2) && (d_left > 2 + 4 * (lane-1) - 2) && (abs(s_left - s) < SAFE_DISTANCE) ) 
                        {
                            std::cout << "There is a vehicle to the left!" << std::endl;
                            maneuver_complete = false;
                            break;
                        }
                        else 
                        {
                            std::cout << "ABS S: " << abs(s_left - s) << std::endl;
                            std::cout << "There is not a vehicle to the left!" << std::endl;
                            maneuver_complete = true;
                            lane = lane - 1;
                            break;
                        }
                    }
                }
                else if (lane + 1 <= 2)
                {
                    std::cout << "Checking right" << std::endl;
                    // check to the front within L
                    // check to the side 
                    // check to the back within L
                    for (int j = 0; j < sensorFusion.size(); j++) 
                    {

                        double vx_right = sensorFusion[j][3];
                        double vy_right = sensorFusion[j][4];
                        double speed_right = sqrt(vx_right*vx_right + vy_right*vy_right);
                        double s_right = sensorFusion[j][5];
                        double d_right = sensorFusion[j][6];

                        // d_right is between the next lane
                        if ( (d_right < 2 + 4 * (lane+1) + 2) && (d_right > 2 + 4 * (lane+1) - 2) && (abs(s_right - s) < SAFE_DISTANCE) )
                        {
                            std::cout << "There is a vehicle to the right!" << std::endl;
                            maneuver_complete = false;
                            break;
                        }
                        else 
                        {
                            std::cout << "ABS S: " << abs(s_right - s) << std::endl;
                            std::cout << "There is not a vehicle to the right!" << std::endl;
                            maneuver_complete = true;
                            lane = lane + 1;
                            break;
                        }
                    }
                }
                else if (!maneuver_complete)
                {
                    std::cout << "We stay in the same lane: " << lane << std::endl;
                }

                /*
                if (lane + 1 <= 2)
                {
                    // check to the front within L
                    // check to the side 
                    // check to the back within L
                    for (int j = 0; j < sensorFusion.size(); j++) 
                    {
                        double vx_right = sensorFusion[j][3];
                        double vy_right = sensorFusion[j][4];
                        double speed_right = sqrt(vx_right*vx_right + vy_right*vy_right);
                        double s_right = sensorFusion[j][5];
                        double d_right = sensorFusion[j][6];

                        // d_right is between the next lane
                        if ( (d_right < 2 + 4 * (lane+1) + 2) && (d_right > 2 + 4 * (lane+1) - 2) && (abs(s_right - s) < 5) )
                        {
                            std::cout << "There is a vehicle to the right!" << std::endl;
                            maneuver_complete = false;
                            break;
                        }
                        else 
                        {
                            std::cout << "There is not a vehicle to the right!" << std::endl;
                            maneuver_complete = true;
                            lane = lane + 1;
                            std::cout << "Next lane (move to the right): " << lane << std::endl;
                            break;
                        }
                    }
                } 
                // look to the left
                else if (!maneuver_complete && lane - 1 >= 0)
                {
                    // check to the front within L
                    // check to the side 
                    // check to the back within L

                    for (int j = 0; j < sensorFusion.size(); j++) 
                    {
                        double vx_left = sensorFusion[j][3];
                        double vy_left = sensorFusion[j][4];
                        double speed_left = sqrt(vx_left*vx_left + vy_left*vy_left);
                        double s_left = sensorFusion[j][5];
                        double d_left = sensorFusion[j][6];

                        // d_right is between the next lane
                        if ( (d_left < 2 + 4 * (lane-1) + 2) && (d_left > 2 + 4 * (lane-1) - 2) && (abs(s_left - s) < 5) ) 
                        {
                            std::cout << "There is a vehicle to the left!" << std::endl;
                            maneuver_complete = false;
                            break;
                        }
                        else 
                        {
                            std::cout << "There is not a vehicle to the left!" << std::endl;
                            maneuver_complete = true;
                            lane = lane - 1;
                            std::cout << "Next lane (move to the left): " << lane << std::endl;
                            break;
                        }
                    }
                }
                else if (!maneuver_complete)
                {
                    std::cout << "We stay in the same lane: " << lane << std::endl;
                }
                */
            }
        }
    }

    // accel. with +/- 5 m/s^2
    if (too_close) 
    {
        refVelocity -= 0.224;
    }
    else if (refVelocity < 49.5)
    {
        refVelocity += 0.224;
    }

    // widely, evenly spaced coordinates
    // interpolated with a spline
    std::vector<double> ptsx;
    std::vector<double> ptsy;

    // either reference the starting point as where the car is
    // or at the previous path's end point
    double ref_x = x;
    double ref_y = y;
    double ref_yaw = deg2rad(yaw);

    // use the car as the starting reference
    if (prev_size < 2) 
    {
        // Use two points that make the path tangent to the car
        double prev_car_x = x - cos(yaw);
        double prev_car_y = y - sin(yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(y);
    }
    // use the previous path's end point as starting reference
    else 
    {
        // Redefine reference state as previous end point
        ref_x = previousPathX[prev_size - 1];
        ref_y = previousPathY[prev_size - 1];

        double ref_x_prev = previousPathX[prev_size - 2];
        double ref_y_prev = previousPathY[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // Use the two points that make path 
        // tangent to the previous path's end point
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // In Frenet add evenly 30m spaced points ahead of the starting reference
    std::vector<double> next_wp0 = getXY(s + 30, (2 + 4*lane), mapWaypointsS, mapWaypointsX, mapWaypointsY);
    std::vector<double> next_wp1 = getXY(s + 60, (2 + 4*lane), mapWaypointsS, mapWaypointsX, mapWaypointsY);
    std::vector<double> next_wp2 = getXY(s + 90, (2 + 4*lane), mapWaypointsS, mapWaypointsX, mapWaypointsY);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // transform to local coordinates
    for (int i = 0; i < ptsx.size(); i++) 
    {
        // shift reference angle to 0 degrees
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // create a spline and set (x, y) points
    tk::spline s;
    s.set_points(ptsx, ptsy);

    // define points for the planner
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    // start with all of the previous path points from last time
    // add new points to the existing path points 
    // smoothen the transition from one path to another path
    for (int i = 0; i < previousPathX.size(); i++) 
    {
        next_x_vals.push_back(previousPathX[i]);
        next_y_vals.push_back(previousPathY[i]);
    }

    // Implement JMT trajectory here 

    // calculate how to break up spline points so that we 
    // travel at our desired reference velociy
    double target_x = 30.0; // horizon x
    double target_y = s(target_x); // horizon y
    double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
    double x_add_on = 0;

    for (int i = 1; i <= 50 - previousPathY.size(); i++) 
    {
        // convert to m/s
        double N = (target_dist / (0.02 * refVelocity / 2.24));
        double x_point = x_add_on + (target_x) / N; // individual x point
        double y_point = s(x_point); // individual y point

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // shift + rotation -- convert back to global coordinates
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    return {
        next_x_vals,
        next_y_vals
    };
}

std::vector<std::vector<double>> Vehicle::getJerkMinimizedTrajectory()
{
    std::vector<std::vector<double>> jerk;

    return jerk; 
}

int Vehicle::getVehicleFromSensorFusion() 
{
    // get the vehicle from 

    return 0;
}

void Vehicle::printVehicleHealth()
{
    std::cout << "START Vehicle Health: " << std::endl;
    std::cout << "X: " << x << std::endl;
    std::cout << "Y: " << y << std::endl;
    std::cout << "S: " << s << std::endl;
    std::cout << "D: " << d << std::endl;
    std::cout << "YAW: " << yaw << std::endl;
    std::cout << "SPEED: " << speed << std::endl;
    std::cout << "Length of SF: " << sensorFusion.size() << std::endl;
    std::cout << "END Vehicle Health" << std::endl;
    std::cout << std::endl;
}

void Vehicle::setLane(int lane) { this->lane = lane; }
void Vehicle::setX(double x) { this->x = x; }
void Vehicle::setY(double y) { this->y = y; }
void Vehicle::setS(double s) { this->s = s; }
void Vehicle::setD(double d) { this->d =  d; }
void Vehicle::setYaw(double yaw) { this->yaw = yaw; }
void Vehicle::setSpeed(double speed) { this->speed = speed; }


int Vehicle::getLane() const { return lane; }
double Vehicle::getS() const { return s; }
double Vehicle::getD() const { return d; }
double Vehicle::getX() const { return x; }
double Vehicle::getY() const { return y; }
double Vehicle::getYaw() const { return yaw; }
double Vehicle::getSpeed() const { return speed; }
std::vector<std::vector<double>> Vehicle::getSF() const { return sensorFusion; }


