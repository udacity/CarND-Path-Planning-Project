#include "helpers.h"

#include "Dense"
#include <algorithm> // std::copy
#include <random>

using Eigen::MatrixXd;
using Eigen::VectorXd;

static const int N_SAMPLES = 10;
static std::random_device rd{};
static std::mt19937 gen{rd()};

vector<double> Vehicle::choose_next_state(const vector<Vehicle> &predictions)
{
    vector<string> possible_successor_states = successor_states();
    //cout << "Hello 2 .. tr\n";
    vector<double> costs;
    vector<vector<double>> trajectories;
    for (size_t i = 0; i < possible_successor_states.size(); ++i)
    {
        vector<double> traj_states = generate_trajectory(possible_successor_states[i], predictions);
        //cout << "Hello 2 .. traj_states.size() = " << traj_states.size();
        costs.push_back(traj_states[13]);
        trajectories.push_back(traj_states);
    }
    int idx = distance(costs.begin(), min_element(costs.begin(), costs.end()));

    lstate = possible_successor_states[idx];
    state = evalState(trajectories[idx], trajectories[idx][12]);
    lane += lane_direction[lstate];
    return trajectories[idx];
}

vector<string> Vehicle::successor_states()
{
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    string state = this->lstate;
    if (state.compare("KL") == 0)
    {
        states.push_back("LCL");
        states.push_back("LCR");
    }
    else if (state.compare("LCL") == 0)
    {
        if (lane != lanes_available - 1)
        {
            states.push_back("LCL");
        }
    }
    else if (state.compare("LCR") == 0)
    {
        if (lane != 0)
        {
            states.push_back("LCR");
        }
    }

    return states;
}

vector<double> Vehicle::generate_trajectory(string state_, const vector<Vehicle> &predictions)
{
    //cout << "state " << state_ << "\n";
    if (state_.compare("KL") == 0)
    {
        return keep_lane_trajectory(predictions);
    }
    else
    {
        return lane_change_trajectory(state_, predictions);
    }
}

vector<double> Vehicle::keep_lane_trajectory(const vector<Vehicle> &predictions)
{
    cout << "___lane keep traj____\n";
    int idx = 0;
    if (get_vehicle_ahead(predictions, idx))
    {
        if (predictions[idx].state[0] - state[0] > 60)
        {
            return free_lane_trajectory();
        }
        vector<double> start_s(state.begin(), state.begin() + 3);
        vector<double> start_d(state.begin() + 3, state.begin() + 6);
        vector<double> delta = {-30, 0, 0, 0, 0, 0};

        return PTG(start_s, start_d, idx, delta, HORIZON, predictions);
    }
    else
    {
        return free_lane_trajectory();

        vector<double> start_s(state.begin(), state.begin() + 3);
        vector<double> start_d(state.begin() + 3, state.begin() + 6);

        double max_avail_speed = (state[1] + MAX_ACC * HORIZON);
        max_avail_speed = max_avail_speed > MAX_SPEED ? MAX_SPEED : max_avail_speed;
        double target_speed = max_avail_speed - state[1];
        vector<double> delta = {50, target_speed, 0, 0, 0, 0};

        vector<Vehicle> tmp;
        tmp.push_back(*this);
        return PTG(start_s, start_d, 0, delta, HORIZON, tmp);
    }
}

vector<double> Vehicle::lane_change_trajectory(string states, const vector<Vehicle> &predictions)
{
    cout << "___lane change traj____\n";
    int idx = 0;
    int new_lane = lane + lane_direction[states];
    if (get_vehicle_ahead(predictions, idx))
    {
        vector<double> start_s(state.begin(), state.begin() + 3);
        vector<double> start_d(state.begin() + 3, state.begin() + 6);
        vector<double> delta = {-30, 0, 0, 4.0 * lane_direction[states], 0, 0};

        return PTG(start_s, start_d, idx, delta, HORIZON, predictions);
    }
    else
    {
        vector<double> rst(14, 0);
        rst[13] = 1e9;
        return rst;
    }
}

vector<double> Vehicle::free_lane_trajectory()
{
    cout << "free lane ...\n";
    vector<double> start_s(state.begin(), state.begin() + 3);
    vector<double> start_d(state.begin() + 3, state.begin() + 6);
    double max_avail_speed = (state[1] + MAX_ACC * HORIZON);
    double target_speed = MAX_SPEED - 0.2;

    max_avail_speed = max_avail_speed > target_speed ? target_speed : max_avail_speed;
    double delta_speed = max_avail_speed - state[1];
    double a = MAX_ACC; //delta_speed / HORIZON;
    double ta = (target_speed - state[1]) / MAX_ACC;
    double s0 = state[0] + state[1] * HORIZON + ta * ta * 0.5 * a;

    vector<double> target_state = {s0, target_speed, 0, 2. + 4. * lane, 0, 0};

    printState(state);
    printState(target_state);

    return PTG_free(state, target_state, HORIZON);
}

bool Vehicle::get_vehicle_ahead(const vector<Vehicle> &predictions, int &idx)
{
    bool found = false;
    double d = 1e9;

    for (size_t i = 0; i < predictions.size(); ++i)
    {
        if (getLane(predictions[i].state[3]) == lane)
        {
            if (this->state[0] < predictions[i].state[0])
            {
                if (predictions[i].state[0] < d)
                {
                    idx = i;
                    d = predictions[i].state[0];
                }
                found = true;
            }
        }
    }
    return found;
}

vector<double> PTG(const vector<double> &start_s, const vector<double> &start_d, const int &target_vehicle,
                   const vector<double> &delta, const double &T, const vector<Vehicle> &predictions)
{
    Vehicle target = predictions[target_vehicle];
    vector<vector<double>> all_goals; // s,d,t
    float timestep = 0.5;
    float t = T - 4 * timestep;
    while (t <= (T + 4 * timestep))
    {
        vector<double> target_state = VecAdd(target.state_in(t), delta);
        vector<double> goals = target_state;
        goals.push_back(t);
        all_goals.push_back(goals);
        for (int i = 0; i < N_SAMPLES; ++i)
        {
            //cout << " " << goals.size();
            goals = perturb_goal(target_state);
            goals.push_back(t);
            all_goals.push_back(goals);
        }
        //cout << " \n";
        t += timestep;
    }
    // cout << "perturb ..." << all_goals.size() << "\n";
    vector<vector<double>> trajectories;
    vector<double> traj(13);
    for (size_t i = 0; i < all_goals.size(); ++i)
    {
        vector<double> s_goal(all_goals[i].begin(), all_goals[i].begin() + 3);
        vector<double> d_goal(all_goals[i].begin() + 3, all_goals[i].begin() + 6);
        double t = all_goals[i][6];
        vector<double> s_coeffs = JMT(start_s, s_goal, t);
        vector<double> d_coeffs = JMT(start_d, d_goal, t);
        copy(s_coeffs.begin(), s_coeffs.end(), traj.begin());
        copy(d_coeffs.begin(), d_coeffs.end(), traj.begin() + 6);
        traj[12] = t;
        trajectories.push_back(traj);
        //cout << "perturb ..." << i;
    }
    vector<double> costs;
    for (size_t i = 0; i < trajectories.size(); ++i)
    {
        costs.push_back(calculate_cost(trajectories[i], target_vehicle, delta, T, predictions));
        //costs.push_back(i);
    }
    int idx = distance(costs.begin(), min_element(costs.begin(), costs.end()));

    vector<double> rst;
    rst = trajectories[idx];
    rst.push_back(costs[idx]);

    return rst;
}

vector<double> PTG_free(const vector<double> &start, const vector<double> &target, const double &T)
{
    vector<double> s_start(start.begin(), start.begin() + 3);
    vector<double> d_start(start.begin() + 3, start.begin() + 6);

    vector<double> s_goal(target.begin(), target.begin() + 3);
    vector<double> d_goal(target.begin() + 3, target.begin() + 6);

    vector<double> s_coeffs = JMT(s_start, s_goal, T);
    vector<double> d_coeffs = JMT(d_start, d_goal, T);

    auto rst = s_coeffs;
    rst.insert(rst.end(), d_coeffs.begin(), d_coeffs.end());
    rst.push_back(T);
    rst.push_back(0.);

    return rst;
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
an array of length 6, each value corresponding to a coefficent in the
polynomial s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 *
t**5

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
    A << t3, t4, t5, 3 * t2, 4 * t3, 5 * t4, 6 * T, 12 * t2, 20 * t3;
    VectorXd b(3);
    b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * t2),
        end[1] - (start[1] + start[2] * T), end[2] - start[2];
    VectorXd x = A.colPivHouseholderQr().solve(b);
    for (int i = 0; i < 3; ++i)
    {
        coeffs[3 + i] = x[i];
    }
    return coeffs;
}

vector<double> VecAdd(const vector<double> &v1, const vector<double> &v2)
{
    if (v1.size() != v2.size())
    {
        cerr << "Invalid input\n";
    }
    vector<double> r;
    for (size_t i = 0; i < v1.size(); ++i)
    {
        r.push_back(v1[i] + v2[i]);
    }
    return r;
}

vector<double> VecSub(const vector<double> &v1, const vector<double> &v2)
{
    if (v1.size() != v2.size())
    {
        cerr << "Invalid input\n";
    }
    vector<double> r;
    for (size_t i = 0; i < v1.size(); ++i)
    {
        r.push_back(v1[i] - v2[i]);
    }
    return r;
}

double logistic(double x) { return 2.0 / (1. + exp(-x)) - 1.; }

vector<double> perturb_goal(const vector<double> &sd)
{
    vector<double> r;
    for (size_t i = 0; i < sd.size(); ++i)
    {
        std::normal_distribution<> d(sd[i], SIGMA_SD[i]);
        r.push_back(d(gen));
    }
    return r;
}

double polyval(const vector<double> &coeffs, double x)
{
    double s = 0.;

    for (size_t i = 0; i < coeffs.size(); ++i)
    {
        s += coeffs[i] * pow(x, i);
    }
    return s;
}

vector<double> polyval(const vector<double> &coeffs, const vector<double> &x)
{
    vector<double> r;
    double s;
    for (size_t i = 0; i < x.size(); ++i)
    {
        s = 0.;
        for (size_t j = 0; j < coeffs.size(); ++j)
        {
            s += coeffs[j] * pow(x[i], j);
        }
        r.push_back(s);
    }
    return r;
}

vector<double> evalState(const vector<double> &coeffs, const double t)
{
    vector<double> s(coeffs.begin(), coeffs.begin() + 6);
    vector<double> d(coeffs.begin() + 6, coeffs.begin() + 12);

    auto s1 = differntiate(s);
    auto s2 = differntiate(s1);

    auto d1 = differntiate(d);
    auto d2 = differntiate(d1);

    return {polyval(s, t),
            polyval(s1, t),
            polyval(s2, t),
            polyval(d, t),
            polyval(d1, t),
            polyval(d2, t)};
}

vector<double> differntiate(const vector<double> &coeffs)
{
    if (coeffs.size() > 0)
    {
        vector<double> r;
        for (size_t i = 1; i < coeffs.size(); ++i)
        {
            r.push_back(coeffs[i] * i);
        }
        return r;
    }
    else
    {
        cerr << "Coeffs(vector) must have at least one element.\n";
    }
}

double nearest_approach_to_any_vehicle(const vector<double> &traj,
                                       const vector<Vehicle> &vehicles)
{
    double closest = 1e9, d;
    for (size_t i = 0; i < vehicles.size(); ++i)
    {
        d = nearest_approach(traj, vehicles[i]);
        if (d < closest)
        {
            closest = d;
        }
    }
    return closest;
}

double nearest_approach(const vector<double> &traj, const Vehicle &vehicle)
{
    double closest = 1e9;
    vector<double> s(traj.begin(), traj.begin() + 3);
    vector<double> d(traj.begin() + 3, traj.begin() + 6);
    double T = traj[12];
    double t, cur_s, cur_d, target_s, target_d, dist;
    vector<double> states;
    for (int i = 0; i < 100; ++i)
    {
        t = float(i) / 100 * T;
        cur_s = polyval(s, t);
        cur_d = polyval(d, t);
        states = vehicle.state_in(t);
        target_s = states[0];
        target_d = states[3];

        dist = pow(cur_s - target_s, 2) + pow(cur_d - target_d, 2);
        if (dist < closest * closest)
        {
            closest = sqrt(dist);
        }
    }
    return closest;
}

int getLane(double d)
{
    return d / 4;
}

void printState(const vector<double> &x)
{
    cout << "state: ";
    for (int i = 0; i < 6; ++i)
    {
        cout << x[i] << ", ";
    }
    cout << "\n";
}

void printVec(const vector<double> &x)
{
    for (int i = 0; i < x.size(); ++i)
    {
        cout << x[i] << ", ";
    }
    cout << "\n";
}