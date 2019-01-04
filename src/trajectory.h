#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include "telemetry.h"
#include "map.h"

struct Trajectory {
  std::vector<double> x;
  std::vector<double> y;
};

struct Position {
  double x;
  double y;
  double yaw;
};

class TrajectoryUtil {

  private:
    std::array<Position, 2> get_ref_state(Telemetry tl);
    std::vector<double> row_to_vector(Eigen::MatrixXd m, int row);
    bool is_busy;
    int preferred_lane;
  public:
    TrajectoryUtil(){is_busy=false;};
    Trajectory generate(Telemetry tl, Map map, unsigned int target_lane, double target_vel);
};
#endif
