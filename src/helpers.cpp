#include "helpers.h"

#include "Dense"
#include <random>
#include <algorithm>    // std::copy

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

static const int N_SAMPLES = 10;
static std::random_device rd{};
static std::mt19937 gen{rd()};
static const vector<float> SIGMA_SD{
    10.,
    4.,
    2.,
    1.,
    1.,
    1.,
};

void PTG(vector<double> start_s, vector<double> start_d, int target_vehicle,
         vector<double> delta, double T, vector<Vehicle> predictions)
{
    Vehicle target = predictions[target_vehicle];
    vector<vector<double>> all_goals; // s,d,t
    float timestep = 0.5;
    float t = T - 4 * timestep;
    while (t <= (T + 4 * timestep))
    {
        vector<double> target_state = VecAdd(target.state, delta);
        vector<double> goals = target_state;
        goals.push_back(t);
        all_goals.push_back(goals);
        for (int i = 0; i < N_SAMPLES; ++i)
        {
            goals = perturb_goal(target_state);
            goals.push_back(t);
            all_goals.push_back(goals);
        }
        t += timestep;
    }

    vector<vector<double>> trajectories;
    vector<double> traj(13);
    for (size_t i = 0; i < all_goals.size(); ++i)
    {
        vector<double> s_goal(all_goals[i].begin(), all_goals[i].begin() + 3);
        vector<double> d_goal(all_goals[i].begin() + 3, all_goals[i].begin() + 6);
        double t = all_goals[i][6];
        vector<double> s_coeffs = JMT(start_s, s_goal, t);
        vector<double> d_coeffs = JMT(start_d, d_goal, t);
        copy(s_coeffs.begin(),s_coeffs.end(), traj.begin.end());
        copy(d_coeffs.begin(),d_coeffs.end(), traj.begin.end()+6);
        traj[12] = t;
        trajectories.push_back(traj);
    }
}

vector<double> JMT(vector<double> start, vector<double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    vector<double> coeffs(6);
    coeffs[0] = start[0];
    coeffs[1] = start[1];
    coeffs[2] = start[2] * 0.5;

    double t2 = T * T;
    double t3 = t2 * T;
    double t4 = t3 * T;
    double t5 = t4 * T;

    MatrixXd A(3, 3);
    A << t3, t4, t5,
        3 * t2, 4 * t3, 5 * t4,
        6 * T, 12 * t2, 20 * t3;
    VectorXd b(3);
    b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * t2),
        end[1] - (start[1] + start[2] * T),
        end[2] - start[2];
    VectorXd x = A.colPivHouseholderQr().solve(b);
    for (int i = 0; i < 3; ++i)
    {
        coeffs[3 + i] = x[i];
    }
    return coeffs;
}

vector<float> VecAdd(const vector<float> &v1, const vector<float> &v2)
{
    vector<float> r;
    for (size_t i = 0; i < v1.size(); ++i)
    {
        r.push_back(v1[i] + v2[i]);
    }
    return r;
}

double logistic(double x)
{
    return 2.0 / (1. + exp(-x)) - 1.;
}

vector<float> perturb_goal(const vector<float> &sd)
{
    vector<float> r;
    for (size_t i = 0; i < sd.size(); ++i)
    {
        std::normal_distribution<> d{sd[i], SIGMA_SD[i]};
        r.push_back(d(gen));
    }
}