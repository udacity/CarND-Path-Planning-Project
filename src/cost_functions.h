//
// Created by Joey Liu on 2017/08/23.
//

#ifndef PATH_PLANNING_COST_FUNCTIONS_H
#define PATH_PLANNING_COST_FUNCTIONS_H

#include <map>
#include <string>
#include "polynomial.h"
#include "car.h"

namespace {
  namespace cf {

    std::map<std::string, double> cost_weights = {
            {"tr_dist_cost", 140.0},
            {"acc_s_cost", 11.0},
            {"acc_d_cost", 10.0},
            {"jerk_cost", 10.0},
            {"busy_cost", 8.0},
    };

    inline bool check_exceed_speed_limit(const pair<Polynomial, Polynomial> &traj,
                                           const int time_step, const double max_vel) {
      for (int i = 0; i < time_step; i++) {
        if (traj.first.eval(i, "first") + traj.second.eval(i, "first") > max_vel) {
          return true;
        }
      }
      return false;
    }

    inline bool check_exceed_accel_limit(const pair<Polynomial, Polynomial> &traj,
                                           const int time_step, const double max_acc) {
      for (int i = 0; i < time_step; i++) {
        if (traj.first.eval(i, "second") + traj.second.eval(i, "second") > max_acc)
          return true;
      }
      return false;
    }

    inline bool check_exceed_jerk_limit(const pair<Polynomial, Polynomial> &traj,
                                          const int time_step, const double max_jerk) {
      for (int i = 0; i < time_step; i++) {
        if (traj.first.eval(i, "third") + traj.second.eval(i, "third") > max_jerk)
          return true;
      }
      return false;
    }

    inline bool check_collision(const pair<Polynomial, Polynomial> &traj,
                                 const int time_step, const vector<Car> &other_cars,
                                 const double car_critical_width, const double car_critical_length) {
      for (int i = 0; i < time_step; i++) {
        double my_car_latest_s = traj.first.eval(i);
        double my_car_latest_d = traj.second.eval(i);

        for (auto other_car:other_cars) {
          double other_car_latest_pos_s = other_car.pos_s + i*other_car.vel_s;
          double other_car_latest_pos_d = other_car.pos_d;
          double diff_s = abs(other_car_latest_pos_s - my_car_latest_s);
          double diff_d = abs(other_car_latest_pos_d - my_car_latest_d);

          if ((diff_s <= car_critical_length*5.5) && (diff_d <= car_critical_width*2.5)) {
            return true;
          }
        }
      }
      return false;
    }

    inline double traffic_distance_cost(const pair<Polynomial, Polynomial> &traj,
                                        const int time_step, const vector<Car> &other_cars,
                                        const double car_safe_width, const double car_safe_length) {
      double cost = 0.0;

      for (auto other_car:other_cars) {
        for (int i = 0; i < time_step; i++) {
          double my_car_latest_s = traj.first.eval(i);
          double my_car_latest_d = traj.second.eval(i);
          double other_car_latest_pos_s = other_car.pos_s + i*other_car.vel_s;
          double diff_s = other_car_latest_pos_s - my_car_latest_s;

          if (diff_s < -10) break;

          diff_s = abs(diff_s);
          double diff_d = abs(other_car.pos_d - my_car_latest_d);

          if ((diff_s <= car_safe_length) && (diff_d <= car_safe_width)) {
            cost += ut::logistic(1 - (diff_s / car_safe_length)) / time_step;
          }
        }
      }
      return cost*cost_weights["tr_dist_cost"];
    }

    inline double total_accel_s_cost(const pair<Polynomial, Polynomial> &traj, const double time_step) {
      double cost = 0.0;
      for (int i = 0; i < time_step; i++) {
        cost += abs(traj.first.eval(i, "second"));
      }
      return ut::logistic(cost)*cost_weights["acc_s_cost"];
    }

    inline double total_accel_d_cost(const pair<Polynomial, Polynomial> &traj, const double time_step) {
      double cost = 0.0;
      for (int i = 0; i < time_step; i++) {
        cost += abs(traj.second.eval(i, "second"));
      }
      return ut::logistic(cost)*cost_weights["acc_d_cost"];
    }

    inline double total_jerk_cost(const pair<Polynomial, Polynomial> &traj, const double time_step) {
      double cost = 0.0;
      for (int i = 0; i < time_step; i++) {
        cost += traj.first.eval(i, "third");
        cost += traj.second.eval(i, "third");
      }
      return ut::logistic(cost)*cost_weights["jerk_cost"];
    }

    inline double busy_lane_cost(const pair<Polynomial, Polynomial> &traj,
                             const double time_step, const vector<Car> &other_cars) {
      double my_car_pos_s = traj.first.eval(0);
      double my_car_pos_d = traj.second.eval(0);
      double my_car_pos_d_end = traj.second.eval(time_step);

      int future_lane = h::get_lane_id(my_car_pos_d_end);
      int future_closest_car_id = h::closest_car_in_lane(future_lane, my_car_pos_s, other_cars);

      if (future_closest_car_id != -1) {
        double future_diff_s = other_cars[future_closest_car_id].pos_s - my_car_pos_s;
        double future_other_car_vel_s = other_cars[future_closest_car_id].vel_s;

        if (future_diff_s < time_step*0.8) {
          int current_lane = h::get_lane_id(my_car_pos_d);
          int closest_car_id = h::closest_car_in_lane(current_lane, my_car_pos_s, other_cars);
          if ((closest_car_id != -1) && (future_diff_s < time_step*0.8/2.0)) {
            double other_car_vel_s = other_cars[closest_car_id].vel_s;
            if (future_other_car_vel_s < other_car_vel_s * 0.95) {
              return 1000*cost_weights["busy_cost"];
            }
          }
          return ut::logistic(1 - (future_diff_s / time_step))*cost_weights["busy_cost"];
        }
      }
      return 0.0;
    }
  } //namespace cf
} // namespace

#endif //PATH_PLANNING_COST_FUNCTIONS_H
