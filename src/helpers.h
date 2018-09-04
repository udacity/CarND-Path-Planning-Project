#ifndef _HELPERS_H_
#define _HELPERS_H_

#include <iostream>
#include <math.h>
#include <vector>
#include <map>

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
const vector<float> WEIGHTS = {1, 20, 20, 1, 1, 1, 20, 1, 1, 1};
const vector<float> SIGMA_SD = {10., 4., 2., 1., 1., 1.};
const float VEHICLE_RADIUS = 1.5;

struct Vehicle
{
    map<string, int> lane_direction = { {"LCL", 1}, {"LCR", -1}};

    vector<double> state;
    string lstate;
    int lane;
    int lanes_available;

    Vehicle()
    {
        state.resize(6);
        lane = 1;
        lanes_available = 3;
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

    vector<double> state_in(double t) const
    {
        return {state[0] + state[1] * t + state[2] * t * t / 2.,
                state[1] + state[2] * t,
                state[2],
                state[3] + state[4] * t + state[5] * t * t / 2.,
                state[4] + state[5] * t,
                state[5]};
    }

    vector<string> successor_states();
};

std::vector<double> JMT(std::vector<double> start, std::vector<double> end,
                        double T);

double logistic(double x);

void PTG(vector<double> start_s, vector<double> start_d, int target_vehicle,
         vector<double> delta, double T, vector<Vehicle> predictions);

vector<double> VecAdd(const vector<double> &v1, const vector<double> &v2);

vector<double> perturb_goal(const vector<double> &sd);

double polyval(const vector<double> &coeffs, double x);

vector<double> differntiate(const vector<double> &coeffs);

double nearest_approach_to_any_vehicle(const vector<double> &traj, const vector<Vehicle> &vehicles);

double nearest_approach(const vector<double> &traj, const Vehicle &vehicle);

int getLane(double d);
#endif // HELPERS_H_