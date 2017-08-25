//
// Created by Joey Liu on 2017/08/04.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <cmath>

namespace {

  namespace ut {
    double deg2rad(double x) { return x * M_PI / 180; }

    double rad2deg(double x) { return x * 180 / M_PI; }

    double distance(double x1, double y1, double x2, double y2) {
      return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    double logistic(double x) {
      return (2.0 / (1 + exp(-x)) - 1.0);
    }
  } //namespace ut
} // namespace
#endif //PATH_PLANNING_UTILS_H
