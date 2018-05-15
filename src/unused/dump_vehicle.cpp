#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

const float REACH_GOAL = pow(10, 6);
const float EFFICIENCY = pow(10, 5);

Vehicle::Vehicle() {
    std::cout << "New ego vehicle" << std::endl;
}

Vehicle::Vehicle(int lane, float s, float v, float a, std::string state)
{
    this->lane = lane;
    this->s = s;
    this->v = s;
    this->a = a;
    this->state = state;
    maxAcceleration = -1;
}

Vehicle::~Vehicle() {}

std::vector<Vehicle> Vehicle::chooseNextState(std::map<int, std::vector<Vehicle>> predictions)
{
    /*
        INPUT: 
            predictions map of vehicle id keys with predicted 
            vehicle trajectories of values; trajectories are a std::vector 
            of Vehicle objects representing vehicle at current timestep 
            and one timestep in the future

        OUTPUT: 
            the best (lowest cost) trajectory 
            corresponding to the next state
    */

    // generate successor states
    std::vector<std::string> states = successorStates();
    float cost;
    std::vector<float> costs;
    std::vector<std::string> final_states;
    std::vector<std::vector<Vehicle>> final_trajectories;

    for (auto it = states.begin(); it != states.end(); ++it) 
    {
        // generate trajectory for each successor
        std::vector<Vehicle> trajectory = generateTrajectory(*it, predictions);
        if (trajectory.size() != 0)
        {
            // store cost of trajectory to this successor
            // cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);

            // store generated trajectory
            final_trajectories.push_back(trajectory);
        }
    }

    // get minimum cost
    std::vector<float>::iterator best_cost = min_element(std::begin(costs), std::end(costs));
    
    // get and return trajectory of minimum cost
    int best_idx = distance(std::begin(costs), best_cost);
    return final_trajectories[best_idx];
}


std::vector<std::string> Vehicle::successorStates()
{
        /* 
        Provide possible next states given the current state 
        for the FSM, with exception that lane changes
        happen instantaneously, so Lane Change Left and 
        Lane Change Right can only transition back to Keep Lane
     */

    std::vector<std::string> states;
    states.push_back("KL");
    std::string state = this->state;

    // State == KEEP_LANE
    if (state.compare("KL") == 0)
    {
        // could move to either left or right lane
        // in the next timestep
        states.push_back("PLCL");
        states.push_back("PLCR");
    }
    // STATE == PREPARE_LANE_CHANGE_LEFT
    else if (state.compare("PLCL") == 0)
    {
        // if current lane is not left most lane
        if (lane != lanesAvailable - 1)
        {
            // change to left lane
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    }
    // STATE == PREPARE_LANE_CHANGE_RIGHT
    else if (state.compare("PLCR") == 0) 
    {
        // if current lane is not right most lane
        if (lane != 0)
        {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }

    // if LCL or LCR, return KL
    return states;
}


std::vector<Vehicle> Vehicle::generateTrajectory(std::string, std::map<int, std::vector<Vehicle>> predictions)
{
    /*
        INPUT: 
            next state

        OUTPUT:
            std::vector of appropriate trajectories to realize next state
     */

    std::vector<Vehicle> trajectory;
    // state == CONSTANT_SPEED
    if (state.compare("CS") == 0)
    {
        trajectory = constantSpeedTrajectory();
    }
    else if (state.compare("KL") == 0)
    {
        trajectory = keepLaneTrajectory(predictions);
    }
    else if (state.compare("LCL") == 0 || state.compare("LCR") == 0)
    {
        trajectory = laneChangeTrajectory(state, predictions);
    }
    else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0)
    {
        trajectory = prepLaneChangeTrajectory(state, predictions);
    }
    return trajectory;
}


std::vector<float> Vehicle::getKinematics(std::map<int, std::vector<Vehicle>> predictions, int lane)
{
    /*
        INPUT: 
            map of predictions
            current lane

        OUTPUT:
            gets next timestep kinematics (position, velocity, acceleration)
            for a given lane; tries to choose the maximum velocity and acceleration
            given other vehicle positions and accel/velocity constraints
     */

    // velocity = a/t + v, t = 1
    float max_velocity_accel_limit = this->maxAcceleration + this->v;
    float new_position;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    // if there's a vehicle ahead
    if (getVehicleAhead(predictions, lane, vehicle_ahead))
    {
        // if there's a vehicle behind us
        if (getVehicleBehind(predictions, lane, vehicle_behind))
        {
            // must travel at the speed of traffic
            // regardless of preferred buffer
            new_velocity = vehicle_ahead.v;
        }
        // there's no vehicle behind us
        else 
        {
            // accelerate to a safe distance behind the lead 
            // and stay within speed limit
            // (ahead_s - s - buffer) + ahead_v - 0.5 * a --> pos + pos/t - 0.5 * pos/t^2, t = 1 --> pos + pos + pos
            float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferredBuffer) + vehicle_ahead.v - 0.5 * this->a;
            new_velocity = std::min(std::min(max_velocity_in_front, max_velocity_accel_limit), this->targetSpeed);
        }
    }
    // there's no vehicle ahead
    else 
    {
        // accelerate to a safe speed
        new_velocity = std::min(max_velocity_accel_limit, this->targetSpeed);
    }

    // accleration = (v1 - v0) / t, t = 1
    new_accel = new_velocity - this->v; 
    new_position = this->s + new_velocity + new_accel / 2.0;
    return 
    {
        new_position,
        new_velocity, 
        new_accel
    };
}


std::vector<Vehicle> Vehicle::constantSpeedTrajectory()
{
    /* 
        INPUT: 

        OUTPUT: 
            constant speed trajectory
     */

    float next_pos = positionAt(1);

    // update position, acceleration = 0, state, velocity, lane stay the same
    std::vector<Vehicle> trajectory = {
        Vehicle(this->lane, this->s, this->v, this->a, this->state),
        Vehicle(this->lane, next_pos, this->v, 0, this->state)
    };

    return trajectory;
}


std::vector<Vehicle> Vehicle::keepLaneTrajectory(std::map<int, std::vector<Vehicle>> predictions)
{
    /* 
        INPUT: 
            map of predictions

        OUTPUT: 
            keep lane trajectory
     */

    std::vector<Vehicle> trajectory = {
        Vehicle(lane, this->s, this->v, this->a, state)
    };

    std::vector<float> kinematics = getKinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    return trajectory;
}


std::vector<Vehicle> Vehicle::laneChangeTrajectory(std::string state, std::map<int, std::vector<Vehicle>> predictions)
{
    /*
        INPUT: 
            current state
            map of predictions

        OUTPUT:
            lane change trajectory
     */

    int new_lane = this->lane + lane_direction[state];
    std::vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;

    // check if a lane change is possible (spot is not occupied)
    for (auto it = predictions.begin(); it != predictions.end(); ++it)
    {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane)
        {
            // if not possible, return empty trajectory
            return trajectory;
        }
    }

    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    std::vector<float> kinematics = getKinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    return trajectory;
}


std::vector<Vehicle> Vehicle::prepLaneChangeTrajectory(std::string state, std::map<int, std::vector<Vehicle>> predictions)
{
    /* 
        INPUT: 
            current state
            map of predictions

        OUTPUT: 
            prepare for lane change trajectory
     */

    float new_s;
    float new_v;
    float new_a;

    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    std::vector<Vehicle> trajectory = {
        Vehicle(this->lane, this->s, this->v, this->a, this->state)
    };
    std::vector<float> curr_lane_new_kinematics = getKinematics(predictions, this->lane);

    // if there's a vehicle behind us
    if (getVehicleBehind(predictions, this->lane, vehicle_behind))
    {
        // keep speed to the current lane to not collide with car behind us
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
    }
    // there is no car behind us
    // get ready to start moving into the other lane, but just not quite yet
    // actual lane change occurs in lane_change_trajectory()
    else 
    {
        std::vector<float> best_kinematics;
        std::vector<float> next_lane_new_kinematics = getKinematics(predictions, new_lane);

        // choose kinematics with lowest velocity
        // do I need to speed up for the next lane
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1])
        {
            best_kinematics = next_lane_new_kinematics;
        }
        // or can I keep going at the same speed
        else 
        {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    return trajectory;
}


void Vehicle::increment(int dt = 1)
{
    this->s = positionAt(dt);
}


float Vehicle::positionAt(int t)
{
    return this->s + this->v*t + this->a*t*t/2.0;
}


bool Vehicle::getVehicleBehind(std::map<int, std::vector<Vehicle>> predictions, int lane, Vehicle & rVehicle)
{
    /*
        INPUT
            map of predictions
            current lane
            vehicle object to update

        OUTPUT
            boolean 
     */

    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;

    for (auto it = predictions.begin(); it != predictions.end(); ++it)
    {
        temp_vehicle = it->second[0];
        // same lane, s is less than our s, and s is closest to us so far
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s)
        {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}


bool Vehicle::getVehicleAhead(std::map<int, std::vector<Vehicle>> predictions, int lane, Vehicle & rVehicle)
{
    /*
        INPUT
            map of predictions
            current lane
            vehicle object to update

        OUTPUT
            boolean
     */

    int min_s = this->goal_s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;

    for (auto it = predictions.begin(); it != predictions.end(); ++it)
    {
        temp_vehicle = it->second[0];
        // same lane, s is greater than our s, and s is closest to us so far
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s)
        {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}


bool Vehicle::getVehicleRight(std::map<int, std::vector<Vehicle>> predictions, int lane, Vehicle & rVehicle)
{
    return false;
}


bool Vehicle::getVehicleLeft(std::map<int, std::vector<Vehicle>> predictions, int lane, Vehicle & rVehicle)
{
    return true;
}

// float Vehicle::goal_distance_cost(const Vehicle & vehicle, const std::vector<Vehicle> & trajectory, const std::map<int, std::vector<Vehicle>> & predictions, std::map<std::string, float> & data)
// {
//     /* 
//         INPUT
//             vehicle
//             trajectory
//             predictions
//             data

//         OUTPUT
//             distance to goal cost
//      */

//     float cost;
//     float distance = data["distance_to_goal"];

//     if (distance > 0)
//     {
//         // cost increases based on distance of intended lane and final lane
//         // cost of being of goal lane also becomes larger as vehicle approaches goal distance
//         cost = 1 - 2 * std::exp(-(std::abs(2.0 * vehicle.getGoalLane() - data["intended_lane"] - data["final_lane"]) / distance));
//     }
//     else 
//     {
//         cost = 1;
//     }
//     return cost;
// }


// float Vehicle::inefficiency_cost(const Vehicle & vehicle, const std::vector<Vehicle> & trajectory, const std::map<int, std::vector<Vehicle>> & predictions, std::map<std::string, float> & data)
// {
//     /*
//         INPUT
//             vehicle
//             trajectory
//             predictions
//             data

//         OUTPUT
//             inefficiency cost
//      */

//     float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
//     if (proposed_speed_intended < 0) 
//     {
//         // no vehicle in intended lane, move at target speed
//         proposed_speed_intended = vehicle.getTargetSpeed();
//     }

//     float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
//     if (proposed_speed_final < 0) 
//     {
//         // no vehicle in final lane, move at target speed
//         proposed_speed_final = vehicle.getTargetSpeed();
//     }

//     // cost becomes higher for trajectories with intendend lane and final that have 
//     // traffic slower than vehicle's target speed
//     float cost = (2.0 * vehicle.getTargetSpeed() - proposed_speed_intended - proposed_speed_final) / vehicle.getTargetSpeed();

//     return cost;
// }


// float Vehicle::lane_speed(const std::map<int, std::vector<Vehicle>> & predictions, int lane)
// {
//     /*
//         INPUT
//             map of predictions
//             current lane

//         OUTPUT
//             speed limit of a lane (equals the speed of any non-ego vehicle in that lane)
//      */

//     for (auto it = predictions.begin(); it != predictions.end(); ++it) 
//     {
//         int key = it->first;
//         Vehicle vehicle = it->second[0];
//         if (vehicle.getLane() == lane && key != -1)
//         {
//             return vehicle.getV();
//         }
//     }

//     return -1.0; // no vehicle in lane
// }


// float Vehicle::calculate_cost(const Vehicle & vehicle, const std::map<int, std::vector<Vehicle>> & predictions, const std::vector<Vehicle> & trajectory)
// {
//     /* 
//         INPUT
//             vehicle
//             map of predictions
//             trajectory

//         OUTPUT
//             sum weighted cost functions to get total cost for trajectory
//      */

//     std::map<std::string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
//     float cost = 0.0;

//     std::vector<std::function<float(const Vehicle & , const std::vector<Vehicle> & , const std::map<int, std::vector<Vehicle>> , std::map<std::string, float> &)>> cf_list = {
//         goal_distance_cost,
//         inefficiency_cost
//     };

//     std::vector<float> weight_list = {
//         REACH_GOAL, 
//         EFFICIENCY
//     };

//     for (int i = 0; i < cf_list.size(); i++) 
//     {
//         float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
//         cost += new_cost;
//     }

//     return cost;
// }


// std::map<std::string, float> Vehicle::get_helper_data(const Vehicle & vehicle, const std::vector<Vehicle> & trajectory, const std::map<int, std::vector<Vehicle>> & predictions)
// {
//     /*
//         INPUT
//             vehicle
//             trajectory
//             map of predictions

//         OUTPUT
//             map of data for use in cost functions
//                 intended_lane: current lane +/- 1 if vehicle is planning/executing a lane change
//                 final_lane: lane of the vehicle at the end of the trajectory
//                 distance_to_goal: distance of the vehicle to the goal
//      */

//     std::map<std::string, float> trajectory_data;
//     Vehicle trajectory_last = trajectory[1];
//     float intended_lane;

//     if (trajectory_last.getState().compare("PLCL") == 0)
//     {
//         intended_lane = trajectory_last.getLane() + 1;
//     }  
//     else if (trajectory_last.getState().compare("PLCR") == 0)
//     {
//         intended_lane = trajectory_last.getLane() - 1;
//     }
//     else 
//     {
//         intended_lane = trajectory_last.getLane(); // KL
//     }

//     float distance_to_goal = vehicle.getGoalS() - trajectory_last.getS();
//     float final_lane = trajectory_last.getLane();
//     trajectory_data["intended_lane"] = intended_lane;
//     trajectory_data["final_lane"] = final_lane;
//     trajectory_data["distance_to_goal"] = distance_to_goal;
//     return trajectory_data;
// }



std::vector<Vehicle> Vehicle::generatePredictions(int horizon)
{
    /* 
        INPUT
            horizon value

        OUTPUT
            predictions for non-ego vehicles for trajectory generation
     */

    std::vector<Vehicle> predictions;
    for (int i = 0; i < horizon; i++) 
    {
        float next_s = positionAt(i);
        float next_v = 0;
        if (i < horizon - 1) 
        {
            next_v = positionAt(i + 1) - this->s;
        }
        predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
    }
    return predictions;
}


void Vehicle::realizeNextState(std::vector<Vehicle> trajectory)
{
    /*
        INPUT
            trajectory

        OUTPUT
     */

    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v = next_state.v;
    this->a = next_state.a;
}


void Vehicle::configure(std::vector<int> roadData)
{
    targetSpeed = roadData[0];
    lanesAvailable = roadData[1];
    goal_s = roadData[2];
    goal_lane = roadData[3];
    maxAcceleration = roadData[4];
}

void Vehicle::updateGlobalMap(std::vector<double> map_waypoints_x, std::vector<double> map_waypoints_y, std::vector<double> map_waypoints_s, std::vector<double> map_waypoints_dx, std::vector<double> map_waypoints_dy)
{
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
    this->map_waypoints_dx = map_waypoints_dx;
    this->map_waypoints_dy = map_waypoints_dy;
}


/* Trajectory Generation */
std::vector<double> Vehicle::getTrajectoryX()
{
    std::vector<double> points;
    return points;
}


std::vector<double> Vehicle::getTrajectoryY()
{
    std::vector<double> points;
    return points;
}

void Vehicle::setLane(int lane) { this->lane = lane; }
void Vehicle::setGoalLane(int goalLane) { this->goal_lane = goalLane; }
void Vehicle::setS(float s) { this->s = s; }
void Vehicle::setX(float x) { this->x = x; }
void Vehicle::setY(float y) { this->y = y; }
void Vehicle::setGoalS(float goalS) { this->goal_s = goalS; }
void Vehicle::setV(float v) { this->v = v; }
void Vehicle::setA(float a) { this->a = a; }
void Vehicle::setTargetSpeed(float targetSpeed) { this->targetSpeed = targetSpeed; }
void Vehicle::setMaxAcceleration(float maxAcceleration) { this->maxAcceleration = maxAcceleration; }
void Vehicle::setString(std::string state) { this->state = state; }

int Vehicle::getLane() const { return lane; }
int Vehicle::getGoalLane() const { return goal_lane; }
int Vehicle::getS() const { return s; }
int Vehicle::getGoalS() const { return goal_s; }
int Vehicle::getV() const { return v; }
int Vehicle::getA() const { return a; }
int Vehicle::getTargetSpeed() const { return targetSpeed; }
int Vehicle::getMaxAcceleration() const { return maxAcceleration; }
std::string Vehicle::getState() const { return state; }
