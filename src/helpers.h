//
// Created by Joey Liu on 2017/08/23.
//

#ifndef PATH_PLANNING_HELPERS_H
#define PATH_PLANNING_HELPERS_H

#include "Car.h"

namespace {
  namespace h {
    int get_lane_id(double d) {
      int lane_id = 0;
      if (d > 8) {
        lane_id = 2;
      } else if (d > 4) {
        lane_id = 1;
      }
      return lane_id;
    }

    int closest_vehicle_in_lane(int target_lane, const Car my_car, const vector<Car> &other_cars) {
      int closest_id = -1;
      double min_s_diff = numeric_limits<double>::infinity();

      for (int i = 0; i < other_cars.size(); i++) {
        int other_lane = get_lane_id(other_cars[i].pos_d);

        if (target_lane == other_lane) {
          double diff_s =  other_cars[i].pos_s - my_car.pos_s;
          if ((diff_s > 0.0) && (diff_s < min_s_diff)) {
            min_s_diff = diff_s;
            closest_id = i;
          }
        }
      }
      return closest_id;
    }

    vector<int> closest_vehicle_in_lanes(const Car my_car, const vector<Car> &other_cars) {
      vector<int> closest_cars(3);
      for (int target_lane = 0; target_lane < 3; target_lane++) {
        closest_cars[target_lane] = closest_vehicle_in_lane(target_lane, my_car, other_cars);
      }
      return closest_cars;
    }
  } //namespace h
} // namespace

#endif //PATH_PLANNING_HELPERS_H
