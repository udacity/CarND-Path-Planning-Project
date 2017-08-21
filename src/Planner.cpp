//
// Created by Joey Liu on 2017/08/16.
//
#include <vector>
#include "Planner.h"
#include "utils.h"

using namespace std;

int Planner::get_lane_id(const double d) {
  if (d > 8) {
    return 2;
  } else if (d > 4) {
    return 1;
  } else {
    return 0;
  }
}

int Planner::closest_vehicle_in_lane(int target_lane) {
  int closest_id = -1;
  double min_s_diff = inf_value;

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

vector<int> Planner::closest_vehicle_in_lanes() {
  vector<int> closest_cars(3);
  for (int target_lane = 0; target_lane < 3; target_lane++) {
    closest_cars[target_lane] = closest_vehicle_in_lane(target_lane);
  }
  return closest_cars;
}

Polynomial Planner::jmt(vector<double> const &start, vector<double> const &end, const int t) {
  double t_2 = pow((double)t, 2);
  double t_3 = pow((double)t, 3);
  double t_4 = pow((double)t, 4);
  double t_5 = pow((double)t, 5);

  Eigen::Matrix3d A;
  A << t_3,   t_4,    t_5,
       3*t_2, 4*t_3,  5*t_4,
       6*t,   12*t_2, 20*t_3;

  double b_0 = start[0] + start[1]*t + 0.5*start[2]*t_2;
  double b_1 = start[1] + start[2]*t;
  double b_2 = start[2];

  Eigen::MatrixXd b(3,1);
  b << end[0] - b_0, end[1] - b_1, end[2] - b_2;

  Eigen::VectorXd c = A.inverse() * b;

  Polynomial result({start[0], start[1], 0.5*start[2], c[0], c[1], c[2]});
  return result;

}

void Planner::perturb_end(vector<double> &end_vals, vector<vector<double>> &end_points, bool no_ahead = false) {
  double percentage_std_deviation = 0.1;
  std::normal_distribution<double> distribution_10_percent(0.0, percentage_std_deviation);
  vector<double> point(6);
  for (int i = 0; i < number_perturb_sample_; i++) {
    double multiplier = distribution_10_percent(rand_generator_);
    if (no_ahead && (multiplier > 0.0)) {
      multiplier *= -1.0;
    }
    point[0] = end_vals[0] + (ref_delta_s_ * multiplier);
    point[1] = end_vals[1] + (ref_vel_ * multiplier);
    point[2] = 0.0;

    multiplier = distribution_10_percent(rand_generator_);
    point[3] = end_vals[3] + multiplier;
    point[4] = 0.0;
    point[5] = 0.0;
    end_points.push_back(point);
  }
}

double Planner::exceeds_speed_limit_cost(const pair<Polynomial, Polynomial> &traj) {
  for (int i = 0; i < global_interval_; i++) {
    if (traj.first.eval(i, "first") + traj.second.eval(i, "first") > hard_max_vel_per_timestep_) {
      return 1.0;
    }
  }
  return 0.0;
}

double Planner::exceeds_accel_limit_cost(const pair<Polynomial, Polynomial> &traj) {
  for (int i = 0; i < global_interval_; i++) {
    if (traj.first.eval(i, "second") + traj.second.eval(i, "second") > hard_max_acc_per_timestep_)
      return 1.0;
  }
  return 0.0;
}

double Planner::exceeds_jerk_limit_cost(const pair<Polynomial, Polynomial> &traj) {
  for (int i = 0; i < global_interval_; i++) {
    if (traj.first.eval(i, "third") + traj.second.eval(i, "third") > hard_max_jerk_per_timestep_)
      return 1.0;
  }
  return 0.0;
}

double Planner::collision_cost(const pair<Polynomial, Polynomial> &traj) {
  for (int i = 0; i < global_interval_; i++) {
    double my_car_latest_s = traj.first.eval(i);
    double my_car_latest_d = traj.second.eval(i);
    for (int ii = 0; ii < other_cars.size(); ii++) {
      // calculate other cars' state at i
      double other_car_latest_pos_s = other_cars[i].pos_s + i*other_cars[i].vel_s;
      double other_car_latest_pos_d = other_cars[i].pos_d;
      double diff_s = abs(other_car_latest_pos_s - my_car_latest_s);
      double diff_d = abs(other_car_latest_pos_d - my_car_latest_d);

      if ((diff_s <= car_critical_length_*5.0) && (diff_d <= car_critical_width_*3.0)) {
        return 1.0;
      }
    }
  }
  return 0.0;
}

double Planner::traffic_distance_cost(const pair<Polynomial, Polynomial> &traj) {
  double cost = 0.0;

  for (auto other_car:other_cars) {
    for (int i = 0; i < global_interval_; i++) {
      double my_car_latest_s = traj.first.eval(i);
      double my_car_latest_d = traj.second.eval(i);
      double other_car_latest_pos_s = other_car.pos_s + i*other_car.vel_s;
      double diff_s = other_car_latest_pos_s - my_car_latest_s;

      if (diff_s < -10) break;

      diff_s = abs(diff_s);
      double diff_d = abs(other_cars[i].pos_d - my_car_latest_d);

      if ((diff_s <= car_safe_length_) && (diff_d <= car_safe_width_)) {
        cost += ut::logistic(1 - (diff_s / car_safe_length_)) / global_interval_;
      }
    }
  }
  return cost;
}

double Planner::accel_s_cost(const pair<Polynomial, Polynomial> &traj) {
  double cost = 0.0;
  for (int i = 0; i < global_interval_; i++) {
    cost += abs(traj.first.eval(i, "second"));
  }
  return ut::logistic(cost);
}

double Planner::accel_d_cost(const pair<Polynomial, Polynomial> &traj) {
  double cost = 0.0;
  for (int i = 0; i < global_interval_; i++) {
    cost += abs(traj.second.eval(i, "second"));
  }
  return ut::logistic(cost);
}

double Planner::total_jerk_cost(const pair<Polynomial, Polynomial> &traj) {
  double cost = 0.0;
  for (int i = 0; i < global_interval_; i++) {
    cost += traj.first.eval(i, "third");
    cost += traj.second.eval(i, "third");
  }
  return ut::logistic(cost);
}

double Planner::efficiency_cost(const pair<Polynomial, Polynomial> &traj, const vector<double> &ends) {
  double s_dist = ends[0] - traj.first.eval(0);
  return abs(ut::logistic((ref_delta_s_ - s_dist) / ref_delta_s_)); // abs() because going faster is actually bad
}

double Planner::compute_cost(const pair<Polynomial, Polynomial> &traj,
                             const vector<double> &ends, vector<vector<double>> &costs) {
  double cost;

  double exceeds_speed_limit_cost_val = exceeds_speed_limit_cost(traj);
  double exceeds_accel_limit_cost_val = exceeds_accel_limit_cost(traj);
  double exceeds_jerk_limit_cost_val = exceeds_jerk_limit_cost(traj);
  double collision_cost_val = collision_cost(traj);

  double critical_cost = exceeds_speed_limit_cost_val + exceeds_accel_limit_cost_val + exceeds_jerk_limit_cost_val + collision_cost_val;

  if (critical_cost > 0.0) {
    costs.push_back({inf_value});
    return inf_value;
  }

  double traffic_distance_cost_val = traffic_distance_cost(traj)*cost_weights_["tr_dist_cost"];
  double accel_s_cost_val = accel_s_cost(traj)*cost_weights_["acc_s_cost"];
  double accel_d_cost_val = accel_d_cost(traj)*cost_weights_["acc_d_cost"];
  double total_jerk_cost_val = total_jerk_cost(traj)*cost_weights_["jerk_cost"];
  double efficiency_cost_val = efficiency_cost(traj, ends)*cost_weights_["eff_cost"];

  vector<double> cost_vals = {traffic_distance_cost_val, accel_s_cost_val, accel_d_cost_val, total_jerk_cost_val, efficiency_cost_val};
  costs.push_back(cost_vals);

  cost = traffic_distance_cost_val + accel_s_cost_val + accel_d_cost_val + total_jerk_cost_val + efficiency_cost_val;
  return cost;

}

void Planner::preprocess(vector<double> &car_state, vector<vector<double>> &previous_path, vector<vector<double>> &sensor_fusion) {
  speed_limit_ = default_speed_limit_;

  current_lane = get_lane_id(car_state[1]);

  way_points.fit_spline_segment(car_state[0]);

  my_car.pos_s = way_points.get_local_s(car_state[0]);
  my_car.pos_d = car_state[1];

  other_cars.clear();
  for (int i = 0; i < sensor_fusion.size(); i++) {
    Car other_car;
    other_car.pos_s = way_points.get_local_s(sensor_fusion[i][5]);
    other_car.pos_d = sensor_fusion[i][6];

    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    other_car.vel_s = sqrt(vx*vx + vy*vy) / 50.0;
    other_cars.push_back(other_car);
  }

  int lag = global_interval_ - local_interval_ - (int)previous_path[0].size();
  if (lag > 9) lag = 0;
  my_car.vel_s = my_car.past_states[lag][0];
  my_car.acc_s = my_car.past_states[lag][1];
  my_car.vel_d = my_car.past_states[lag][2];
  my_car.acc_d = my_car.past_states[lag][3];

}

vector<vector<double>> Planner::plan(vector<double> &current_s_d, vector<vector<double>> &previous_path,
                                     vector<vector<double>> &sensor_fusion) {

  vector<double> next_x;
  vector<double> next_y;

  int previous_path_size = (int) previous_path[0].size();

  bool smooth_path = previous_path_size > 0;

  if (previous_path_size < global_interval_ - local_interval_) {

    cout << "update path" << endl;

    preprocess(current_s_d, previous_path, sensor_fusion);

    const vector<double> start_s = {my_car.pos_s, my_car.vel_s, my_car.acc_s};
    const vector<double> start_d = {my_car.pos_d, my_car.vel_d, my_car.acc_d};

    ref_vel_ = conversion_ * speed_limit_;

    if (my_car.vel_s < ref_vel_ / 1.8) {
      double ref_vel_diff = ref_vel_ - my_car.vel_s;
      ref_vel_ -= ref_vel_diff * 0.70;
    }

    ref_delta_s_ = global_interval_ * ref_vel_;

    cout << "my car's local s: " << my_car.pos_s << " vel s: " << my_car.vel_s << " d: " << my_car.pos_d << " lane: " << current_lane << endl;

    vector<vector<double>> end_points;
    // s, s_dot, s_double_dot, d, d_dot, d_double_dot
    vector<vector<double>> traj_ends;
    vector<double> traj_costs;

    // #########################################
    // FIND FEASIBLE NEXT STATES FROM:
    // - go straight
    // - go straight following leading vehicle
    // - lane change left
    // - lane change right
    // #########################################
    bool go_straight = true;
    bool follow_lead = false;
    bool change_left = false;
    bool change_right = false;

    vector<int> closest_cars = closest_vehicle_in_lanes();

    int closest_car_id = closest_cars[current_lane];
    cout << "closest_car_id: " << closest_car_id << endl;
    if (closest_car_id != -1) {

      double diff_s = abs(other_cars[closest_car_id].pos_s - my_car.pos_s);

      if (diff_s < 100) {
        change_left = true;
        change_right = true;
      }
      if (diff_s < car_safe_length_) {
        go_straight = false;
        follow_lead = true;
        change_left = true;
        change_right = true;
      }
    }
    /*
    bool prefer_mid_lane = false;

    if ((current_lane == 0) || (current_lane == 2)) {
      Car closest_car_current_lane = other_cars[closest_cars[current_lane]];
      Car closest_car_mid_lane = other_cars[closest_cars[1]];
      Car closest_car_op_lane = other_cars[closest_cars[abs(current_lane - 2)]];

      if (abs(my_car.pos_s - closest_car_current_lane.pos_s) < 2*car_safe_length_) {
        if (abs(my_car.pos_s - closest_car_mid_lane.pos_s) < 2*car_safe_length_) {
          if (abs(my_car.pos_s - closest_car_op_lane.pos_s) > 4*car_safe_length_) {
            prefer_mid_lane = true;
            if (current_lane == 0) {
              change_right = true;
            } else {
              change_left = true;
            }
          }
        }
      }
    }
  */
    if (go_straight) {
      // end_pos_s, end_vel_s, end_acc_s, end_pos_d, end_vel_d, end_acc_d
      vector<double> end_vals = {my_car.pos_s + ref_delta_s_, ref_vel_, 0.0, 2. + 4. * current_lane, 0.0, 0.0};
      vector<vector<double>> end_points_straight = {end_vals};
      perturb_end(end_vals, end_points_straight);
      end_points.reserve(end_points.size() + end_points_straight.size());
      end_points.insert(end_points.end(), end_points_straight.begin(), end_points_straight.end());
    }

    if (follow_lead) {
      Car leading_car = other_cars[closest_car_id];
      if ((leading_car.pos_s - my_car.pos_s < car_safe_length_*0.5) && (leading_car.vel_s < my_car.vel_s*0.8)) {
        cout << "EMERGENCY" << endl;
        current_action = "emergency";
        global_interval_ = 120;
        change_left = false;
        change_right = false;
      }
      // end_pos_s, end_vel_s, end_acc_s, end_pos_d, end_vel_d, end_acc_d
      vector<double> end_vals = {my_car.pos_s + leading_car.vel_s * global_interval_, leading_car.vel_s, 0.0,
                                 2. + 4. * current_lane, 0.0, 0.0};
      vector<vector<double>> end_points_follow = {end_vals};
      perturb_end(end_vals, end_points_follow);

      end_points.reserve(end_points.size() + end_points_follow.size());
      end_points.insert(end_points.end(), end_points_follow.begin(), end_points_follow.end());
    }

    if (change_left && (current_lane != 0)) {
      double end_pos_s = my_car.pos_s + ref_delta_s_;
      double end_vel_s = ref_vel_;
      if (follow_lead) {
        Car leading_car = other_cars[closest_car_id];
        if (leading_car.pos_s - my_car.pos_s < car_safe_length_*0.5) {
          end_pos_s = my_car.pos_s + leading_car.vel_s*global_interval_;
          end_vel_s = leading_car.vel_s;
        }
      }
      // end_pos_s, end_vel_s, end_acc_s, end_pos_d, end_vel_d, end_acc_d
      vector<double> end_vals = {end_pos_s, end_vel_s, 0.0, (2. + 4.*current_lane) - 4, 0.0, 0.0};
      vector<vector<double>> end_points_left = {end_vals};
      perturb_end(end_vals, end_points_left);
      end_points.reserve(end_points.size() + end_points_left.size());
      end_points.insert(end_points.end(), end_points_left.begin(), end_points_left.end());
    }

    if (change_right && (current_lane != 2)) {
      double end_pos_s = my_car.pos_s + ref_delta_s_;
      double end_vel_s = ref_vel_;
      if (follow_lead) {
        Car leading_car = other_cars[closest_car_id];
        if (leading_car.pos_s - my_car.pos_s < car_safe_length_*0.5) {
          end_pos_s = my_car.pos_s + leading_car.vel_s * global_interval_;
          end_vel_s = leading_car.vel_s;
        }
      }
      // end_pos_s, end_vel_s, end_acc_s, end_pos_d, end_vel_d, end_acc_d
      vector<double> end_vals = {end_pos_s, end_vel_s, 0.0, (2. + 4. * current_lane) + 4, 0.0, 0.0};
      vector<vector<double>> end_points_right = {end_vals};
      perturb_end(end_vals, end_points_right);
      end_points.reserve(end_points.size() + end_points_right.size());
      end_points.insert(end_points.end(), end_points_right.begin(), end_points_right.end());
    }

    cout << "PLAN: ";
    if (go_straight)
      cout << " :GO STRAIGHT: ";
    if (follow_lead)
      cout << " :FOLLOW LEAD: ";
    if (change_left)
      cout << " :CHANGE LEFT: ";
    if (change_right)
      cout << " :CHANGE RIGHT: ";
    cout << endl;

    vector<pair<Polynomial, Polynomial>> traj_coeffs;
    for (auto end_point: end_points) {
      vector<double> end_s = {end_point[0], end_point[1], end_point[2]};
      vector<double> end_d = {end_point[3], end_point[4], end_point[5]};

      if (end_point[3] > 1.0 && end_point[3] < 11.0) {
        Polynomial traj_s_poly = jmt(start_s, end_s, global_interval_);
        Polynomial traj_d_poly = jmt(start_d, end_d, global_interval_);
        traj_coeffs.push_back(std::make_pair(traj_s_poly, traj_d_poly));
        traj_ends.push_back({end_point[0], end_point[1], end_point[2], end_point[3], end_point[4], end_point[5]});
      }
    }



    vector<vector<double>> costs;

    for (int i = 0; i < traj_coeffs.size(); i++) {
      double cost = compute_cost(traj_coeffs[i], traj_ends[i], costs);
      //if (prefer_mid_lane && (abs(6 - traj_coeffs[i].second.eval(global_interval_)) < 1.0))
      //  cost *= 0.5;
      traj_costs.push_back(cost);
    }

    cout << "traj_costs.size: " << traj_costs.size() << endl;

    double min_cost = traj_costs[0];
    int min_cost_id = 0;
    for (int i = 1; i < traj_costs.size(); i++) {
      if (traj_costs[i] < min_cost) {
        min_cost = traj_costs[i];
        min_cost_id = i;
      }
    }

    // rare edge case: vehicle is stuck in infeasible trajectory (usually stuck close behind other car)
    if (min_cost == inf_value) {
      double min_s = traj_coeffs[0].first.eval(global_interval_);
      int min_s_id = 0;
      // find trajectory going straight with minimum s
      for (int i = 1; i < number_perturb_sample_; i++) {
        if (traj_coeffs[i].first.eval(global_interval_) < min_s) {
          min_s = traj_coeffs[i].first.eval(global_interval_);
          min_s_id = i;
        }
      }
      min_cost_id = min_s_id;
    }
    current_action = "straight";
    if (min_cost_id > number_perturb_sample_) {
      current_action = "lane_change";
    }


    cout << "min_cost_id: " << min_cost_id << endl;

    vector<double> traj_s(global_interval_);
    vector<double> traj_d(global_interval_);
    for (int t = 0; t < global_interval_; t++) {
      traj_s[t] = traj_coeffs[min_cost_id].first.eval(t);
      traj_d[t] = traj_coeffs[min_cost_id].second.eval(t);
    }

    global_interval_ = default_global_interval_;
    local_interval_ = default_local_interval_;
    if (current_action == "lane_change") {
      cout << "LANE CHANGE" << endl;
      local_interval_ = global_interval_ - 50;
    } else if (current_action == "emergency") {
      cout << "EMERGENCY" << endl;
      global_interval_ = 120;
      local_interval_ = global_interval_ - 80;
    }

    for (int i = 0; i < 10; i++) {
      double s0 = traj_s[i + local_interval_];
      double s1 = traj_s[i + local_interval_ + 1];
      double s2 = traj_s[i + local_interval_ + 2];
      double d0 = traj_d[i + local_interval_];
      double d1 = traj_d[i + local_interval_ + 1];
      double d2 = traj_d[i + local_interval_ + 2];
      double s_v1 = s1 - s0;
      double s_v2 = s2 - s1;
      double s_a = s_v2 - s_v1;
      double d_v1 = d1 - d0;
      double d_v2 = d2 - d1;
      double d_a = d_v2 - d_v1;

      my_car.past_states[i] = {s_v1, s_a, d_v1, d_a};
    }

    double new_x, new_y;
    int smooth_range = 20;
    int reuse_prev_range = 15;
    vector<double> prev_xy_planned;

    for (int i = 0; i < reuse_prev_range; i++) {
      prev_xy_planned = way_points.getXY_splines(traj_s[i], traj_d[i]);

      if (smooth_path) {
        new_x = previous_path[0][i];
        new_y = previous_path[1][i];
        next_x.push_back(new_x);
        next_y.push_back(new_y);
      } else {
        next_x.push_back(prev_xy_planned[0]);
        next_y.push_back(prev_xy_planned[1]);
      }
    }

    for (int i = reuse_prev_range; i < traj_s.size(); i++) {
      vector<double> xy_planned = way_points.getXY_splines(traj_s[i], traj_d[i]);
      if (smooth_path) {
        double x_dif_planned = xy_planned[0] - prev_xy_planned[0];
        double y_dif_planned = xy_planned[1] - prev_xy_planned[1];
        new_x = new_x + x_dif_planned;
        new_y = new_y + y_dif_planned;

        double smooth_scale_fac = (smooth_range - (i - reuse_prev_range)) / smooth_range;
        if (i > smooth_range)
          smooth_scale_fac = 0.0;
        double smooth_x = (previous_path[0][i] * smooth_scale_fac) + (new_x * (1 - smooth_scale_fac));
        double smooth_y = (previous_path[1][i] * smooth_scale_fac) + (new_y * (1 - smooth_scale_fac));

        next_x.push_back(smooth_x);
        next_y.push_back(smooth_y);
        prev_xy_planned = xy_planned;
      } else {
        next_x.push_back(xy_planned[0]);
        next_y.push_back(xy_planned[1]);
      }
    }
  } else {
    for(int i = 0; i < previous_path_size; i++) {
      next_x.push_back(previous_path[0][i]);
      next_y.push_back(previous_path[1][i]);
    }
  }
  return {next_x, next_y};
}