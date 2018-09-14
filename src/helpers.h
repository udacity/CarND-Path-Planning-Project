#ifndef _HELPERS_H_
#define _HELPERS_H_

#include <iostream>
#include <math.h>
#include <vector>
#include <map>
#include "cost_functions.h"

using namespace std;

/**
 *  (time_diff_cost,    1),
    (s_diff_cost,       20),
    (d_diff_cost,       20),
    (efficiency_cost,   1),
    (max_jerk_cost,     1),
    (total_jerk_cost,   1),
    (collision_cost,    20),
    (buffer_cost,       1),
    (max_accel_cost,    1),
    (total_accel_cost,  1),
*/
const vector<float> WEIGHTS = {1, 1, 1, 20, 1, 1, 20, 1, 1, 1};
const vector<float> SIGMA_SD = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}; //{10., 4., 2., 1., 1., 1.};
const float VEHICLE_RADIUS = 1.5;
const double HORIZON = 5.;
const double MAX_SPEED = 50 * 0.44704;
const double MAX_ACC = 10.0;
const double MAX_JERK = 10.0;

struct Vehicle
{
    map<string, int> lane_direction = {{"LCL", -1}, {"KL", 0}, {"LCR", +1}};

    // state: s, s_dot, s_ddot, d, d_dot, d_ddot
    vector<double> state;

    // s,d
    vector<double> s;
    vector<double> d;

    // lane state: LK, LKL, LKR
    string lstate;
    // current lane
    int lane;
    // total available lanes
    int lanes_available;

    double speed;
    double target_speed;

    bool follow_front;

    Vehicle()
    {
        state.resize(6);
        lane = 1;
        lanes_available = 3;
        lstate = "KL";
        speed = 0.;
        target_speed = MAX_SPEED * 0.98;
        follow_front = false;
    }
    Vehicle(const vector<double> &s)
    {
        if (s.size() == 6)
        {
            state = s;
        }
        else
        {
            cerr << "Invalid input vector.\n";
        }
    }

    Vehicle(const vector<double> &s, const string &st) : lstate(st)
    {
        if (s.size() == 6)
        {
            state = s;
        }
        else
        {
            cerr << "Invalid input vector.\n";
        }
    }

    vector<double> state_in(double t) const
    {
        return {state[0] + state[1] * t + state[2] * t * t / 2.,
                state[1] + state[2] * t,
                state[2],
                state[3] + state[4] * t + state[5] * t * t / 2.,
                state[4] + state[5] * t,
                state[5]};
    }

    void updateTraj()
    {
        double t, st, dt;
        for (int i = 0; i < 101; ++i)
        {
            t = float(i) / 100. * HORIZON;
            st = state[0] + state[1] * t + state[2] * t * t / 2.;
            dt = state[3] + state[4] * t + state[5] * t * t / 2.;
            s.push_back(st);
            d.push_back(dt);
        }
    }

    void updateLane()
    {
        lane = floor(state[3] / 4);
        if (lane < 0)
            lane = 0;
        if (lane > lanes_available - 1)
            lane = lanes_available - 1;
        cout << "lane = " << lane << endl;
    }
    vector<double> choose_next_state(const vector<Vehicle> &predictions);
    vector<double> choose_next_state_v2(const vector<Vehicle> &predictions);
    vector<string> successor_states();
    vector<double> generate_trajectory(string state, const vector<Vehicle> &predictions);
    vector<double> keep_lane_trajectory(const vector<Vehicle> &predictions);
    vector<double> lane_change_trajectory(string state, const vector<Vehicle> &predictions);
    vector<double> free_lane_trajectory();
    bool get_vehicle_ahead(const vector<Vehicle> &predictions, int &idx);
};

std::vector<double> JMT(std::vector<double> start, std::vector<double> end,
                        double T);

double logistic(double x);

/**
 * Path trajectory generation
 * 
 * return: s_coeffs(6), d_coeffs(6), cost(1)
*/
vector<double> PTG(const vector<double> &start_s, const vector<double> &start_d, const int &target_vehicle,
                   const vector<double> &delta, const double &T, const vector<Vehicle> &predictions);

vector<double> PTG_free(const vector<double> &start, const vector<double> &target, const double &T);

vector<double> VecAdd(const vector<double> &v1, const vector<double> &v2);

vector<double> VecSub(const vector<double> &v1, const vector<double> &v2);

vector<double> perturb_goal(const vector<double> &sd);

double polyval(const vector<double> &coeffs, double x);

vector<double> polyval(const vector<double> &coeffs, const vector<double> &x);

vector<double> evalState(const vector<double> &coeffs, const double t);

vector<double> differntiate(const vector<double> &coeffs);

double nearest_approach_to_any_vehicle(const vector<double> &traj, const vector<Vehicle> &vehicles);

double nearest_approach(const vector<double> &traj, const Vehicle &vehicle);

int getLane(double d);

void printState(const vector<double> &x);

void printVec(const vector<double> &x);

void toVehicleFrame(vector<double> &x_v, vector<double> &y_v, const vector<double> &x_w, const vector<double> &y_w, double x_ref, double y_ref, double yaw_ref);

void toWorldFrame(vector<double> &x_w, vector<double> &y_w, const vector<double> &x_v, const vector<double> &y_v, double x_ref, double y_ref, double yaw_ref);

struct TwoVect
{
    double v1;
    double v2;
    TwoVect(double a, double b) : v1(a), v2(b) {}
};

bool compareVec(TwoVect &a, TwoVect &b);

void sortVecs(vector<double> &v1, vector<double> &v2);

double meanVecAbs(const vector<double> &x);

double maxVelocity(const vector<double> &vx, const vector<double> &vy);

#endif // HELPERS_H_