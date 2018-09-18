#include <chrono>
#include <iostream>
#include <algorithm>
#include <thread>
#include <vector>
#include <map>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#include "helpers.h"
#include "cost_functions.h"

int main()
{

    Vehicle v1;
    v1.state = {72 + 50, MAX_SPEED * 0.6, 0, 6, 0, 0};
    v1.updateTraj();
    Vehicle ego;
    ego.state = {72, MAX_SPEED * 0.8, 0, 6, 0, 0};
    vector<Vehicle> vehs;
    vehs.push_back(v1);

    //auto traj = ego.choose_next_state(vehs);
    std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
    auto traj2 = ego.choose_next_state_v3(vehs);
    std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() << std::endl;

    ego.prev_traj = traj2;
    //printVec(traj2.s);
    //cout << endl;
    //printVec(ego.prev_traj.s);
    return 0;
}