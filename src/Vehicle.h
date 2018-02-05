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
    Vehicle();
    Vehicle(int lane, float s, float v, float a, std::string state="CS");
    
    /* Destructor */
    virtual ~Vehicle();

    /* FSM */
    std::vector<Vehicle> chooseNextState(std::map<int, std::vector<Vehicle>> predictions);
    std::vector<std::string> successorStates();

    std::vector<Vehicle> generateTrajectory(std::string, std::map<int, std::vector<Vehicle>> predictions);
    std::vector<float> getKinematics(std::map<int, std::vector<Vehicle>> predictions, int lane);

    std::vector<Vehicle> constantSpeedTrajectory();
    std::vector<Vehicle> keepLaneTrajectory(std::map<int, std::vector<Vehicle>> predictions);
    std::vector<Vehicle> laneChangeTrajectory(std::string state, std::map<int, std::vector<Vehicle>> predictions);
    std::vector<Vehicle> prepLaneChangeTrajectory(std::string state, std::map<int, std::vector<Vehicle>> predictions);

    void increment(int dt);
    float positionAt(int t);

    bool getVehicleBehind(std::map<int, std::vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
    bool getVehicleAhead(std::map<int, std::vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
    bool getVehicleRight(std::map<int, std::vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
    bool getVehicleLeft(std::map<int, std::vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

    // float calculate_cost(const Vehicle & vehicle, const std::map<int, std::vector<Vehicle>> & predictions, const std::vector<Vehicle> & trajectory);
    // float goal_distance_cost(const Vehicle & vehicle, const std::vector<Vehicle> & trajectory, const std::map<int, std::vector<Vehicle>> & predictions, std::map<std::string, float> & data);
    // float inefficiency_cost(const Vehicle & vehicl, const std::vector<Vehicle> & trajectory, const std::map<int, std::vector<Vehicle>> & predictions, std::map<std::string, float> & data);
    // float lane_speed(const std::map<int, std::vector<Vehicle>> & predictions, int lane);
    // std::map<std::string, float> get_helper_data(const Vehicle & vehicle, const std::vector<Vehicle> & trajectory, const std::map<int, std::vector<Vehicle>> & predictions);

    std::vector<Vehicle> generatePredictions(int horizon);
    void realizeNextState(std::vector<Vehicle> trajectory);
    void configure(std::vector<int> roadData);

    void updateGlobalMap(std::vector<double> map_waypoints_x, std::vector<double> map_waypoints_y, std::vector<double> map_waypoints_s, std::vector<double> map_waypoints_dx, std::vector<double> map_waypoints_dy);

    /* Trajectory Generation */
    std::vector<double> getTrajectoryX();
    std::vector<double> getTrajectoryY();

    /* Getters and Setters */
    void setLane(int lane);
    void setGoalLane(int goalLane);
    void setS(float s);
    void setGoalS(float goalS);
    void setV(float v);
    void setA(float a);
    void setTargetSpeed(float targetSpeed);
    void setMaxAcceleration(float maxAcceleration);
    void setString(std::string state);

    int getLane() const;
    int getGoalLane() const;
    int getS() const;
    int getGoalS() const;
    int getV() const;
    int getA() const;
    int getTargetSpeed() const;
    int getMaxAcceleration() const;
    std::string getState() const;

private: 
    std::map<std::string, int> lane_direction = {
        {"PLCL", 1}, 
        {"LCL", 1}, 
        {"LCR", -1}, 
        {"PLCR", -1}
    };

    int L = 1;

    int preferredBuffer = 6;

    int lane;

    float s;

    float v;

    float a;

    float targetSpeed;

    int lanesAvailable;

    float maxAcceleration;

    int goal_lane;

    float goal_s;

    std::string state;

    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;

    //TrajectoryPlanner tp;
    // BehaviorPlanner bp;

};