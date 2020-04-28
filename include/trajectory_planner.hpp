#include <iostream>
#include <string>
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"
#include "Eigen/Dense"
#include "helpers.h"



std::vector<double> JMT(const std::vector<double> &start, const std::vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
    std::vector<double> T_order;
    T_order.push_back(1.0);
    for (size_t i=1; i <= 5; ++i){
        T_order.push_back( T_order[i-1] * T);
    }

    Eigen::MatrixXd A(3,3);
    A << T_order[3], T_order[4], T_order[5],
        3*T_order[2], 4*T_order[3], 5*T_order[4],
        6*T_order[1], 12*T_order[2], 20*T_order[3];
    Eigen::VectorXd b(3);
    b << (end[0] - (start[0] + start[1]*T_order[1] + 0.5*start[2]*T_order[2])),
        (end[1] - (start[1] + start[2]*T_order[1])),
        (end[2] - start[2] );
    Eigen::VectorXd x = A.inverse() * b;
  return {start[0], start[1], 0.5*start[2], x[0], x[1], x[2]};
}
