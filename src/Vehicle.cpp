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
const float SPEED_LIMIT = 49.5;
const float ACCEL = 0.224;
const float SAFE_DISTANCE = 30.0;
const int NUM_POINTS = 75;

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

    bool vehicleFront = false;
    bool vehicleLeft = false;
    bool vehicleRight = false;

    float minDistanceToVehicleInRightLane = 9999.0;
    float minDistanceToVehicleInLeftLane = 9999.0;

    int indexOfVehicleInRightLaneWithMinDistance = -1;
    int indexOfVehicleInLeftLaneWithMinDistance = -1;

    for (int i = 0; i < sensorFusion.size(); i++) 
    {
        double vehicle_vx = sensorFusion[i][3];
        double vehicle_vy = sensorFusion[i][4];
        double vehicle_speed = sqrt(vehicle_vx*vehicle_vx + vehicle_vy*vehicle_vy);
        double vehicle_s = sensorFusion[i][5];
        double vehicle_d = sensorFusion[i][6];

        int vehicle_lane = -1;
        // predict s' for next vehicle in this lane
        vehicle_s += ((double) prev_size * 0.02 * vehicle_speed);

        // figure out what lane vehicle is lane
        if (vehicle_d < (2 + 4 * lane + 2) && vehicle_d > (2 + 4 * lane - 2))
        {
            vehicle_lane = lane;
        }
        else if ( (vehicle_d < (2 + 4 * (lane - 1) + 2)) && (vehicle_d > (2 + 4 * (lane - 1) - 2)) )
        {
            vehicle_lane = lane - 1;
        }
        else if ( (vehicle_d < (2 + 4 * (lane + 1) + 2)) && (vehicle_d > (2 + 4 * (lane + 1) - 2)) )
        {
            vehicle_lane = lane + 1;
        } 

        // is the vehicle within a SAFE_DISTANCE 
        if (vehicle_lane == lane)
        {
            if ( (vehicle_s > this->s) && (vehicle_s - this->s < SAFE_DISTANCE) )
            {
                vehicleFront = true;
            }
        }
        else if (vehicle_lane - lane == -1)
        {
            if (vehicle_s - this->s > 0 && vehicle_s - this->s < minDistanceToVehicleInLeftLane)
            {
                minDistanceToVehicleInLeftLane = vehicle_s - this->s;
            }

            if (this->s - SAFE_DISTANCE < vehicle_s && this->s + SAFE_DISTANCE > vehicle_s)
            {
                vehicleLeft = true;
            }

        }
        else if (vehicle_lane - lane == 1)
        {
            if (vehicle_s - this->s > 0 && vehicle_s - this->s < minDistanceToVehicleInRightLane)
            {
                minDistanceToVehicleInRightLane = vehicle_s - this->s;
            }

            if (this->s - SAFE_DISTANCE < vehicle_s && this->s + SAFE_DISTANCE > vehicle_s)
            {
                vehicleRight = true;
            }
        }
    }

    if (vehicleFront)
    {
        if (!vehicleLeft && lane > 0 && minDistanceToVehicleInLeftLane > minDistanceToVehicleInRightLane)
        {
            lane = lane - 1;
            std::cout << "lane change left" << std::endl;
        } 
        else if (!vehicleRight && lane < 2 && minDistanceToVehicleInRightLane > minDistanceToVehicleInLeftLane) 
        {
            lane = lane + 1;
            std::cout << "lane change right" << std::endl;
        }
        else {
            refVelocity -= ACCEL/2.0;
        }
    }
    else 
    {
        // original
        // have a tendency to move to the middle lane if you can
        if ( (!vehicleLeft && lane == 2) || (!vehicleRight && lane == 0) )
        {
            lane = 1;
        }

        if (refVelocity < SPEED_LIMIT)
        {
            refVelocity += ACCEL;
        }
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

// Debug
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