#pragma once 
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

#include "costs.h"
#include "TrajectoryPlanner.h"
#include "helpers.h"

class Vehicle 
{
public:
    /* Constructor */
    Vehicle();
    Vehicle(int lane, float s, float v, float a, std::string state="CS");
    
    /* Destructor */
    virtual ~Vehicle();

    /* FSM */
    vector<Vehicle> chooseNextState(map<int, vector<Vehicle>> predictions);
    vector<std::string> successorStates();

    vector<Vehicle> generateTrajectory(std::string, map<int, vector<Vehicle>> predictions);
    vector<float> getKinematics(map<int, vector<Vehicle>> predictions, int lane);

    vector<Vehicle> constantSpeedTrajectory();
    vector<Vehicle> keepLaneTrajectory(map<int, vector<Vehicle>> predictions);
    vector<Vehicle> laneChangeTrajectory(std::string state, map<int, vector<Vehicle>> predictions);
    vector<Vehicle> prepLaneChangeTrajectory(std::string state, map<int, vector<Vehicle>> predictions);

    void increment(int dt);
    float positionAt(int t);

    bool getVehicleBehind(map<int, vector<Vehicle>> predictions, int lane);
    bool getVehicleAhead(map<int, vector<Vehicle>> predictions, int lane);
    bool getVehicleRight(map<int, vector<Vehicle>> predictions, int lane);
    bool getVehicleLeft(map<int, vector<Vehicle>> predictions, int lane);

    vector<Vehicle> generatePredictions(int horizon);
    void realizeNextState(vector<Vehicle> trajectory);
    void configure(vector<int> roadData);

    /* FSM Cost Functions */
    float calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory);
    float goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data)
    float inefficiency_cost(const Vehicle & vehicl, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data);
    float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane);
    map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions);

    /* Trajectory Generation */
    vector<double> getTrajectoryX();
    vector<double> getTrajectoryY();

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
    // BehaviorPlanner bp;

};