#pragma once 
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

#include "TrajectoryPlanner.h"
#include "BehaviorPlanner.h"
#include "helpers.h"

class Vehicle 
{
public:
    /* Constructor */
    Vehicle();
    Vehicle(int lane, float s, float v, float a, std::string state="CS");
    
    /* Destructor */
    virtual ~Vehicle();

    /* Getters and Setters */
    void setLane(int lane);
    void setS(float s);
    void setV(float v);
    void setA(float a);
    void setTargetSpeed(float targetSpeed);
    void setMaxAcceleration(float maxAcceleration);
    void setString(std::string state);

    int getLane();
    int getS();
    int getV();
    int getA();
    int getTargetSpeed();
    int getMaxAcceleration();
    std::string getState();

private: 
    int L = 1;

    int preferred_buffer = 6;

    int lane;

    float s;

    float v;

    float a;

    float targetSpeed;

    int lanesAvailable;

    float maxAcceleration;

    // int goal_lane;
    // float goal_s;

    std::string state;

    TrajectoryPlanner tp;
    BehaviorPlanner bp;

};