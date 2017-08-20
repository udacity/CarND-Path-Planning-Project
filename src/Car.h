//
// Created by Joey Liu on 2017/08/17.
//

#ifndef PATH_PLANNING_CARSTATE_H
#define PATH_PLANNING_CARSTATE_H

struct Car {
  double pos_s = 0.0, vel_s = 0.0, acc_s = 0.0, pos_d = 0.0, vel_d = 0.0, acc_d = 0.0;
  vector<vector<double>> past_states = {{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},
                                {0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0}};
};

#endif //PATH_PLANNING_CARSTATE_H
