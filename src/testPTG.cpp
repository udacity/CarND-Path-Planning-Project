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
    v1.state = {72+50, 3.3, 0, 6, 0, 0};
    v1.updateTraj();
    Vehicle ego;
    ego.state = {72, 48.5*0.224, 0, 6, 0, 0};
    vector<Vehicle> vehs;
    vehs.push_back(v1);

    auto traj = ego.choose_next_state(vehs);
    auto traj2 = ego.choose_next_state_v2(vehs);

    return 0;
}