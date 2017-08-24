//
// Created by Joey Liu on 2017/08/16.
//
#include "planner.h"

using namespace std;

Polynomial Planner::jmt(vector<double> const &start, vector<double> const &end, const int t) {
  double T = t;
  double t_2 = pow(T, 2);
  double t_3 = pow(T, 3);
  double t_4 = pow(T, 4);
  double t_5 = pow(T, 5);

  Eigen::MatrixXd A(3, 3);
  Eigen::VectorXd b(3, 1);
  A << t_3,   t_4,    t_5,
       3*t_2, 4*t_3,  5*t_4,
       6*t,   12*t_2, 20*t_3;

  b << end[0] - (start[0] + start[1]*t + 0.5*start[2]*t_2),
       end[1] - (start[1] + start[2]*t),
       end[2] - start[2];

  Eigen::VectorXd c = A.inverse() * b;

  Polynomial result({start[0], start[1], 0.5*start[2], c[0], c[1], c[2]});
  return result;

}

void Planner::perturb_end(vector<double> &end_vals, vector<vector<double>> &end_points, bool change_left) {
  double std_dev = 0.1;
  std::normal_distribution<double> dist(0.0, std_dev);
  vector<double> point(6);
  for (int i = 0; i < kNPerturbSample; i++) {
    double multiplier = dist(rand_generator_);
    if (change_left && (multiplier > 0.0)) {
      multiplier *= -1.0;
    }
    point[0] = end_vals[0] + (max_delta_s_ * multiplier);
    point[1] = end_vals[1] + (max_vel_ * multiplier);
    point[2] = 0.0;

    multiplier = dist(rand_generator_);
    point[3] = end_vals[3] + multiplier;
    point[4] = 0.0;
    point[5] = 0.0;
    end_points.push_back(point);
  }
}

double Planner::calculate_cost(const pair<Polynomial, Polynomial> &traj,
                             const vector<double> &ends, vector<vector<double>> &costs) {

  bool exceed_speed_limit = cf::exceeds_speed_limit_cost(traj, time_step_, kMaxVel);
  bool exceeds_accel_limit = cf::exceeds_accel_limit_cost(traj, time_step_, kMaxAcc);
  bool exceeds_jerk_limit = cf::exceeds_jerk_limit_cost(traj, time_step_, kMaxJerk);
  bool collision = cf::collision_cost(traj,time_step_, other_cars_, kCarCriticalWidth, kCarCriticalLength);

  bool critical_cost = (exceed_speed_limit || exceeds_accel_limit || exceeds_jerk_limit|| collision);

  if (critical_cost) {
    costs.emplace_back(kInf);
    return kInf;
  }

  double traffic_distance_cost_val = cf::traffic_distance_cost(traj, time_step_, other_cars_, kCarSafeWidth, kCarSafeLength);
  double accel_s_cost_val = cf::accel_s_cost(traj, time_step_);
  double accel_d_cost_val = cf::accel_d_cost(traj, time_step_);
  double total_jerk_cost_val = cf::total_jerk_cost(traj, time_step_);
  double busy_lane_cost_val = cf::busy_lane_cost(traj, time_step_, other_cars_);

  vector<double> cost_vals = {traffic_distance_cost_val, accel_s_cost_val, accel_d_cost_val, total_jerk_cost_val, busy_lane_cost_val};
  costs.push_back(cost_vals);

  double cost = traffic_distance_cost_val + accel_s_cost_val + accel_d_cost_val + total_jerk_cost_val + busy_lane_cost_val;
  return cost;

}

void Planner::preprocess(double car_s, double car_d, const vector<double> &previous_path_x,
                         const vector<double> &previous_path_y, vector<vector<double>> &sensor_fusion) {
  // Clear the container
  traj_s_.clear();
  traj_d_.clear();
  next_x_.clear();
  next_y_.clear();
  other_cars_.clear();

  prev_path_size_ = (int)previous_path_x.size();

  // Check to update the planned path or not.
  do_update = prev_path_size_ < (time_step_ - interval_);
  if (do_update) {
    // Set the private variables
    prev_path_x_ = previous_path_x;
    prev_path_y_ = previous_path_y;
    current_lane_ = h::get_lane_id(car_d);
    way_points_.fit_spline_segment(car_s);
    my_car_.pos_s = way_points_.get_local_s(car_s);
    my_car_.pos_d = car_d;

    // Check whether the speed is too fast or not.
    // If the speed is too fast, then scale down the speed limit.
    double dx0 = way_points_.spline_x_s(my_car_.pos_s);
    double dx1 = way_points_.spline_x_s(my_car_.pos_s + 100);
    double diff_dx = abs(dx1 - dx0);
    speed_limit_ = kDefaultSpeedLimit;
    double factor_ = 1.0;
    if (diff_dx > 0.25) {
      if (current_lane_ == 0) {
        factor_ = (1 - ut::logistic(diff_dx)*.5) * .10 + .90;
      } else if (current_lane_ == 1) {
        factor_ = (1 - ut::logistic(diff_dx)*.5) * .15 + .85;
      } else {
        factor_ = (1 - ut::logistic(diff_dx)*.5) * .20 + .80;
      }
    }
    speed_limit_ *= factor_;

    // Collect the information of other cars
    for (auto data:sensor_fusion) {
      Car other_car;
      other_car.pos_s = way_points_.get_local_s(data[5]);
      other_car.pos_d = data[6];

      double vx = data[3];
      double vy = data[4];
      other_car.vel_s = sqrt(vx*vx + vy*vy) / 50.0;
      other_cars_.push_back(other_car);
    }

    // Set the velocity and acceleration of both s and d for planning the path
    my_car_.vel_s = my_car_.past_states[0];
    my_car_.acc_s = my_car_.past_states[1];
    my_car_.vel_d = my_car_.past_states[2];
    my_car_.acc_d = my_car_.past_states[3];

    // Set max velocity and max delta s for planning the path
    max_vel_ = kConversion * speed_limit_;
    // make my car starts smoothly without exceeding acceleration or jerk limit
    if (my_car_.vel_s < max_vel_ / 1.7) {
      double vel_diff = max_vel_ - my_car_.vel_s;
      max_vel_ -= vel_diff * 0.70;
    }
    max_delta_s_ = time_step_ * max_vel_;
  }
}

void Planner::plan() {

  cout << "my car's local s: " << my_car_.pos_s << " vel s: " << my_car_.vel_s << " d: " << my_car_.pos_d << " lane: " << current_lane_ << endl;

  vector<vector<double>> end_points;
  vector<vector<double>> traj_ends;
  vector<double> traj_costs;

  // Define the strategies for the action
  bool go_straight = true;
  bool follow_lead = false;
  bool change_left = false;
  bool change_right = false;

  vector<int> closest_cars = h::closest_vehicle_in_lanes(my_car_, other_cars_);

  int closest_car_id = closest_cars[current_lane_];
  if (closest_car_id != -1) {
    double diff_s = abs(other_cars_[closest_car_id].pos_s - my_car_.pos_s);
    if (diff_s < 100) {
      change_left = true;
      change_right = true;
    }
    if (diff_s < kCarSafeLength) {
      go_straight = false;
      follow_lead = true;
      change_left = true;
      change_right = true;
    }
  }

  if (go_straight) {
    double end_pos_s = my_car_.pos_s + max_delta_s_;
    double end_vel_s = max_vel_;
    double end_acc_s = 0.0;
    double end_pos_d = 2 + 4 * current_lane_;
    double end_vel_d = 0.0;
    double end_acc_d = 0.0;

    vector<double> end_vals = {end_pos_s, end_vel_s, end_acc_s, end_pos_d, end_vel_d, end_acc_d};
    vector<vector<double>> end_points_straight = {end_vals};
    perturb_end(end_vals, end_points_straight);
    end_points.reserve(end_points.size() + end_points_straight.size());
    end_points.insert(end_points.end(), end_points_straight.begin(), end_points_straight.end());
  }

  if (follow_lead) {
    Car leading_car = other_cars_[closest_car_id];
    if ((leading_car.pos_s - my_car_.pos_s < kCarSafeLength*0.7) && (leading_car.vel_s < my_car_.vel_s*0.75)) {
      cout << "EMERGENCY" << endl;
      current_action_ = "emergency";
      time_step_ = 120;
      change_left = false;
      change_right = false;
    }

    double end_pos_s = my_car_.pos_s + leading_car.vel_s * time_step_;
    double end_vel_s = leading_car.vel_s;
    double end_acc_s = 0.0;
    double end_pos_d = 2 + 4 * current_lane_;
    double end_vel_d = 0.0;
    double end_acc_d = 0.0;

    vector<double> end_vals = {end_pos_s, end_vel_s, end_acc_s, end_pos_d, end_vel_d, end_acc_d};
    vector<vector<double>> end_points_follow = {end_vals};
    perturb_end(end_vals, end_points_follow);
    end_points.reserve(end_points.size() + end_points_follow.size());
    end_points.insert(end_points.end(), end_points_follow.begin(), end_points_follow.end());
  }

  if (change_left && (current_lane_ != 0)) {
    double end_pos_s = my_car_.pos_s + max_delta_s_;
    double end_vel_s = max_vel_;
    if (follow_lead) {
      Car leading_car = other_cars_[closest_car_id];
      if (leading_car.pos_s - my_car_.pos_s < kCarSafeLength*0.7) {
        end_pos_s = my_car_.pos_s + leading_car.vel_s*time_step_;
        end_vel_s = leading_car.vel_s;
      }
    }

    double end_acc_s = 0.0;
    double end_pos_d = (2 + 4 * current_lane_) - 4;
    double end_vel_d = 0.0;
    double end_acc_d = 0.0;

    vector<double> end_vals = {end_pos_s, end_vel_s, end_acc_s, end_pos_d, end_vel_d, end_acc_d};
    vector<vector<double>> end_points_left = {end_vals};
    perturb_end(end_vals, end_points_left, true);
    end_points.reserve(end_points.size() + end_points_left.size());
    end_points.insert(end_points.end(), end_points_left.begin(), end_points_left.end());
  }

  if (change_right && (current_lane_ != 2)) {
    double end_pos_s = my_car_.pos_s + max_delta_s_;
    double end_vel_s = max_vel_;
    if (follow_lead) {
      Car leading_car = other_cars_[closest_car_id];
      if (leading_car.pos_s - my_car_.pos_s < kCarSafeLength*0.7) {
        end_pos_s = my_car_.pos_s + leading_car.vel_s * time_step_;
        end_vel_s = leading_car.vel_s;
      }
    }

    double end_acc_s = 0.0;
    double end_pos_d = (2 + 4 * current_lane_) + 4;
    double end_vel_d = 0.0;
    double end_acc_d = 0.0;

    vector<double> end_vals = {end_pos_s, end_vel_s, end_acc_s, end_pos_d, end_vel_d, end_acc_d};
    vector<vector<double>> end_points_right = {end_vals};
    perturb_end(end_vals, end_points_right);
    end_points.reserve(end_points.size() + end_points_right.size());
    end_points.insert(end_points.end(), end_points_right.begin(), end_points_right.end());
  }

  cout << "POSSIBLE ACTIONS: ";
  if (go_straight)
    cout << " GO STRAIGHT ";
  if (follow_lead)
    cout << " FOLLOW LEAD ";
  if (change_left)
    cout << " CHANGE LEFT ";
  if (change_right)
    cout << " CHANGE RIGHT ";
  cout << endl;

  const vector<double> start_s = {my_car_.pos_s, my_car_.vel_s, my_car_.acc_s};
  const vector<double> start_d = {my_car_.pos_d, my_car_.vel_d, my_car_.acc_d};

  // Generate the polynomial of s and d for generating the planned path
  vector<pair<Polynomial, Polynomial>> traj_coeffs;
  for (auto end_point: end_points) {
    if (end_point[3] > 1.0 && end_point[3] < 11.0) {
      vector<double> end_s = {end_point[0], end_point[1], end_point[2]};
      vector<double> end_d = {end_point[3], end_point[4], end_point[5]};
      Polynomial traj_s = jmt(start_s, end_s, time_step_);
      Polynomial traj_d = jmt(start_d, end_d, time_step_);
      traj_coeffs.emplace_back(traj_s, traj_d);
      traj_ends.emplace_back(initializer_list<double>{end_point[0], end_point[1], end_point[2],
                                                      end_point[3], end_point[4], end_point[5]});
    }
  }

  // Calculate the costs of each possible path
  double min_cost = kInf;
  int min_cost_id = 0;
  vector<vector<double>> costs;
  for (int i = 0; i < traj_coeffs.size(); i++) {
    double cost = calculate_cost(traj_coeffs[i], traj_ends[i], costs);
    traj_costs.push_back(cost);
    if (cost < min_cost) {
      min_cost = cost;
      min_cost_id = i;
    }
  }

  // rare edge case: vehicle is stuck in infeasible trajectory (usually stuck close behind other car)
  if (min_cost == kInf) {
    double min_s = traj_coeffs[0].first.eval(time_step_);
    int min_s_id = 0;
    // find trajectory going straight with minimum s
    for (int i = 1; i < kNPerturbSample; i++) {
      if (traj_coeffs[i].first.eval(time_step_) < min_s) {
        min_s = traj_coeffs[i].first.eval(time_step_);
        min_s_id = i;
      }
    }
    min_cost_id = min_s_id;
  }

  current_action_ = "straight";
  if (min_cost_id > kNPerturbSample) {
    current_action_ = "lane_change";
  }

  for (int t = 0; t < time_step_; t++) {
    traj_s_.push_back(traj_coeffs[min_cost_id].first.eval(t));
    traj_d_.push_back(traj_coeffs[min_cost_id].second.eval(t));
  }
}

void Planner::postprocess() {
  // update information for next cycle
  time_step_ = kDefaultTimeStep;
  interval_ = kDefaultInterval;
  if (current_action_ == "lane_change") {
    interval_ = time_step_ - 55;
  } else if (current_action_ == "emergency") {
    time_step_ = 120;
    interval_ = time_step_ - 80;
  }
  double s0 = traj_s_[interval_ + 1];
  double s1 = traj_s_[interval_ + 2];
  double s2 = traj_s_[interval_ + 3];
  double d0 = traj_d_[interval_ + 1];
  double d1 = traj_d_[interval_ + 2];
  double d2 = traj_d_[interval_ + 3];
  double s_v1 = s1 - s0;
  double s_v2 = s2 - s1;
  double s_a = s_v2 - s_v1;
  double d_v1 = d1 - d0;
  double d_v2 = d2 - d1;
  double d_a = d_v2 - d_v1;

  my_car_.past_states = {s_v1, s_a, d_v1, d_a};

  // generate planned path
  bool smooth_path = prev_path_size_ > 0;
  double new_x = 0.0, new_y = 0.0;
  int smooth_range = 30;
  int reuse_prev_range = 20;
  vector<double> prev_xy_planned;

  for (int i = 0; i < reuse_prev_range; i++) {
    prev_xy_planned = way_points_.get_spline_xy(traj_s_[i], traj_d_[i]);

    if (smooth_path) {
      new_x = prev_path_x_[i];
      new_y = prev_path_y_[i];
      next_x_.push_back(new_x);
      next_y_.push_back(new_y);
    } else {
      next_x_.push_back(prev_xy_planned[0]);
      next_y_.push_back(prev_xy_planned[1]);
    }
  }
  for (int i = reuse_prev_range; i < traj_s_.size(); i++) {
    vector<double> xy_planned = way_points_.get_spline_xy(traj_s_[i], traj_d_[i]);
    if (smooth_path) {
      double x_dif_planned = xy_planned[0] - prev_xy_planned[0];
      double y_dif_planned = xy_planned[1] - prev_xy_planned[1];
      new_x = new_x + x_dif_planned;
      new_y = new_y + y_dif_planned;

      double smooth_scale_fac = (smooth_range - (i - reuse_prev_range)) / smooth_range;
      if (i > smooth_range) {
        smooth_scale_fac = 0.0;
      }
      double smooth_x = (prev_path_x_[i] * smooth_scale_fac) + (new_x * (1 - smooth_scale_fac));
      double smooth_y = (prev_path_y_[i] * smooth_scale_fac) + (new_y * (1 - smooth_scale_fac));

      next_x_.push_back(smooth_x);
      next_y_.push_back(smooth_y);
      prev_xy_planned = xy_planned;
    } else {
      next_x_.push_back(xy_planned[0]);
      next_y_.push_back(xy_planned[1]);
    }
  }
}

vector<vector<double>> Planner::get_planned_result() {
  return {next_x_, next_y_};
}