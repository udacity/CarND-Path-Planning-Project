#pragma once 
#include <vector>
#include <cmath>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
    TrajectoryPlanner
        Purpose: interpolates JMT and spline curves given 
                 start and waypoints of trajectory
 */
class TrajectoryPlanner 
{
public: 
    TrajectoryPlanner();
    
    virtual ~TrajectoryPlanner();

    std::vector<double> JMT(std::vector<double> start, std::vector<double> end, double T);

private: 
};