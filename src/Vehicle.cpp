#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"
#include "helpers.h"
#include "spline.h"

const float REACH_GOAL = pow(10, 6);
const float EFFICIENCY = pow(10, 5);
const float SAFE_DISTANCE = 30.0;
const int NUM_POINTS = 100;

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
    this->safeSpeed = speed;
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

    int vehicleFrontIndex = -1;
    int vehicleLeftIndex = -1;
    int vehicleRightIndex = -1;
    int vehicleBackIndex = -1;

    bool vehicleFront = false;
    bool vehicleLeft = false;
    bool vehicleRight = false;
    bool vehicleBack = false;

    for (int i = 0; i < sensorFusion.size(); i++) 
    {
        // car is in my lane
        double d = sensorFusion[i][6];
        double vx = sensorFusion[i][3];
        double vy = sensorFusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensorFusion[i][5];
        // predict s' for next vehicle in this lane
        check_car_s += ((double) prev_size * 0.02 * check_speed);

        if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) 
        {
            // vehicle in front and violating SAFE_DISTANCE
            if ((check_car_s > this->s) && ((check_car_s - this->s) < SAFE_DISTANCE)) 
            {
                // vehicle is too close
                // std::cout << "Vehicle " << i << " is too close." << std::endl;
                vehicleFront = true;
                vehicleFrontIndex = i;
            }
            else if ((check_car_s > this->s) && ((check_car_s - this->s) < SAFE_DISTANCE/2))
            {
                vehicleBackIndex = i;
                vehicleBack = true;
            }
        }
        else if ( (d < (2 + 4 * (lane - 1) + 2)) && (d > (2 + 4 * (lane - 1) - 2)) )
        {
            // Vehicle to the left 
            if ((check_car_s > this->s) && ((check_car_s - this->s) < SAFE_DISTANCE))
            {
                // std::cout << "Vehicle " << i << " is in lane " << (lane - 1) << " and it is too close to complete lane change maneuver." << std::endl;
                vehicleLeftIndex = i;
                vehicleLeft = true;
            }
        }
        else if ( (d < (2 + 4 * (lane + 1) + 2)) && (d > (2 + 4 * (lane + 1) - 2)) )
        {
            // Vehicle to the right 
            if ((check_car_s > this->s) && ((check_car_s - this->s) < SAFE_DISTANCE))
            {
                // std::cout << "Vehicle " << i << " is in lane " << (lane + 1) << " and it is too close to complete lane change maneuver." << std::endl;
                vehicleRightIndex = i;
                vehicleRight = true;
            }
        }

        // std::cout << "Front Index: " << vehicleFrontIndex << std::endl;
        // std::cout << "Back Index: " << vehicleBackIndex << std::endl;
        // std::cout << "Left Index: " << vehicleLeftIndex << std::endl;
        // std::cout << "Right Index: " << vehicleRightIndex << std::endl;
        // std::cout << std::endl;

        if (vehicleFront)
        {
            // left is free and possible to move to left
            if (!vehicleLeft && (lane - 1) >= 0)
            {
                std::cout << "Moving to the left." << std::endl;
                lane = lane - 1;
            }   
            // right is free and possible to move to right
            else if (!vehicleRight && (lane + 1) <= 2)
            {
                std::cout << "Moving to the right." << std::endl;
                lane = lane + 1;
            }
            // not possible to move either way
            else 
            {
                std::cout << "Stay put." << std::endl;
            }
            break;
        }
    }

    // accel. with +/- 5 m/s^2
    if (vehicleFront) 
    {
        refVelocity -= 0.224;
    }
    else if (refVelocity < 49.5)
    {
        refVelocity += 0.224;
    }
                // bool move_to_left = false;
                // bool move_to_right = false;

                // for (int j = 0; j < sensorFusion.size(); j++)
                // {
                //     if (i != j)
                //     {
                //         double vx_adj = sensorFusion[j][3];
                //         double vy_adj = sensorFusion[j][4];
                //         double speed_adj = sqrt(vx_adj*vx_adj + vy_adj*vy_adj);
                //         double s_adj = sensorFusion[j][5];
                //         double d_adj = sensorFusion[j][6];
                //         double s_adj_predicted = s_adj + 0.02 * s_adj;

                //         // look to the left
                //         if (lane - 1 >= 0)
                //         {
                //             if ( ((d_adj < 2 + 4 * (lane-1) + 2) && (d_adj > 2 + 4 * (lane-1) - 2)) 
                //                 && (s_adj_predicted > s && s_adj_predicted - s < SAFE_DISTANCE) )
                //             {
                //                 std::cout << "D_Adj: " << d_adj << " is in between " << (2 + 4 * (lane - 1) + 2) << " and " << (2 + 4 * (lane - 1) - 2) << std::endl;
                //                 move_to_left = false;
                //             } 
                //             else 
                //             {
                //                 move_to_left = true;
                //                 break;
                //             }
                //         }

                //         // if the left is blocked, look to the right
                //         if (!move_to_left && lane + 1 <= 2)
                //         {
                //             if ( ((d_adj < 2 + 4 * (lane+1) + 2) && (d_adj > 2 + 4 * (lane+1) - 2)) 
                //                 && (s_adj_predicted > s && s_adj_predicted - s < SAFE_DISTANCE) )
                //             {
                //                 std::cout << "D_Adj: " << d_adj << " is in between " << (2 + 4 * (lane + 1) + 2) << " and " << (2 + 4 * (lane + 1) - 2) << std::endl;
                //                 move_to_right = false;
                //                 break;
                //             }
                //             else 
                //             {
                //                 move_to_right = true;
                //                 break;
                //             }
                //         }
                //     }
                // }

                // if (move_to_left)
                // {
                //     lane = lane - 1;
                // } 
                // else if (move_to_right)
                // {
                //     lane = lane + 1;
                // }
                // else 
                // {
                //     std::cout << "Stay in the current lane." << std::endl;
                // }



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
    std::vector<double> next_wp0 = getXY(s + SAFE_DISTANCE, (2 + 4*lane), mapWaypointsS, mapWaypointsX, mapWaypointsY);
    std::vector<double> next_wp1 = getXY(s + SAFE_DISTANCE*2, (2 + 4*lane), mapWaypointsS, mapWaypointsX, mapWaypointsY);
    std::vector<double> next_wp2 = getXY(s + SAFE_DISTANCE*3, (2 + 4*lane), mapWaypointsS, mapWaypointsX, mapWaypointsY);

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
    // double target_x = 30.0; // horizon x
    double target_x = (double)SAFE_DISTANCE;
    double target_y = s(target_x); // horizon y
    double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
    double x_add_on = 0;

    for (int i = 1; i <= NUM_POINTS - previousPathY.size(); i++) 
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