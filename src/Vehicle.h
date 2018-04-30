#pragma once 

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

// #include "helpers.h"

class Vehicle 
{
public:
    /* Constructor */
    Vehicle(int lane, double refVelocity);
    
    /* Destructor */
    virtual ~Vehicle();

    /* Set Global coordiantes */
    void updateGlobalMap(std::vector<double> mapWaypointsX, std::vector<double> mapWaypointsY, std::vector<double> mapWaypointsS, std::vector<double> mapWaypointsDX, std::vector<double> mapWaypointsDY);

    /* Update Vehicle State */
    void updateVehicleState(double x, double y, double s, double d, double yaw, double speed, std::vector<double> previousPathX, std::vector<double> previousPathY, double endPathS, double endPathD, std::vector<std::vector<double>> sensorFusion);

    /* Trajectory Generation */
    std::vector<std::vector<double>> getSmoothSplineTrajectory();
    std::vector<std::vector<double>> getJerkMinimizedTrajectory();

    /* Getters and Setters */
    void setLane(int lane);
    void setX(double x);
    void setY(double y);
    void setS(double s);
    void setD(double d);
    void setYaw(double yaw);
    void setSpeed(double speed);

    int getLane() const;
    double getX() const;
    double getY() const;
    double getS() const;
    double getD() const;
    double getYaw() const;
    double getSpeed() const;
    std::vector<std::vector<double>> getSF() const;

    void printVehicleHealth();

private: 
    // 0 = left
    // 1 = middle
    // 2 = right
    int lane;
    double refVelocity;

    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;

    std::vector<double> previousPathX;
    std::vector<double> previousPathY;

    double endPathS;
    double endPathD;

    // car's unique ID
    // car's x position in map coordinates
    // car's y position in map coordinates
    // car's x velocity in m/s
    // car's y velocity in m/s
    // car's s position in frenet coordinates
    // car's d position in frenet coordinates
    std::vector<std::vector<double>> sensorFusion;

    std::vector<double> trajectoryX;
    std::vector<double> trajectoryY;

    std::vector<double> mapWaypointsX;
    std::vector<double> mapWaypointsY;
    std::vector<double> mapWaypointsS;
    std::vector<double> mapWaypointsDX;
    std::vector<double> mapWaypointsDY;
};