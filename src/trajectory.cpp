#include <vector>
#include <iostream>
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"
#include "telemetry.h"
#include "trigs.h"
#include "frenet.h"
#include "state.h"
#include "map.h"

#include "trajectory.h"

using namespace std;
Trajectory TrajectoryUtil::generate(Telemetry tl, Map map, unsigned int target_lane, double target_vel){

  //State& cst = State::Instance();

  Eigen::MatrixXd anchors(3,5);
  /* Initialize reference state */
  array<Position, 2> state = get_ref_state(tl);
  anchors(0,0) = state[1].x;
  anchors(1,0) = state[1].y;
  anchors(0,1) = state[0].x;
  anchors(1,1) = state[0].y;

  double ref_yaw = state[0].yaw;

  double dx = anchors(0, 1);
  double dy = anchors(1, 1);

  /* Initialize anchor points */
  // In Frenet add evenly 30m spaced points ahead of the starting reference
  for(unsigned int i=0; i<3; i++){
    vector<double> next_wp0 = Frenet::getXY(tl.s + 30*(i+1) + 10, (2+4*target_lane), map);
    anchors(0, 2+i) = next_wp0[0];
    anchors(1, 2+i) = next_wp0[1];
  }

  anchors = anchors.colwise() + Trigs::homo();
  Eigen::MatrixXd car_pos = Trigs::rigid_reverse(anchors, -ref_yaw, -dx, -dy);
  
  vector<double> ptsx = row_to_vector(car_pos, 0);
  vector<double> ptsy = row_to_vector(car_pos, 1);

  // Create a spline
  tk::spline s;

  // set (x, y) points to the spline
  s.set_points(ptsx, ptsy);

  //Define the actual (x, y) points we will use for the planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  //Start with all of the previous path points from last time
  for(unsigned int i=0; i< tl.previous_path_x.size(); i++) {
    next_x_vals.push_back(tl.previous_path_x[i]);
    next_y_vals.push_back(tl.previous_path_y[i]);
  }

  //Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0; // Horizon value
  double target_y = s(target_x); //Value of horizon from the spline
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;
  
  cout<<" ---------------- "<<endl;
  cout<<"car x=" << tl.x <<"; car y="<<tl.y<<endl;
  for(unsigned int i=0; i<tl.previous_path_x.size(); i++){
    cout << "x=" << tl.previous_path_x[i]<<"; y="<< tl.previous_path_y[i];
    if(i>0) {
      double px = tl.previous_path_x[i-1];
      double cx = tl.previous_path_x[i];
      double dx = cx - px;

      double py = tl.previous_path_y[i-1];
      double cy = tl.previous_path_y[i];
      double dy = cy - py;

      double magn = sqrt(dx*dx + dy*dy);
      cout <<"; magn="<<magn;

    }
    cout << endl;
  }

  //Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
  double total_speed;

  unsigned int pp_size = tl.previous_path_x.size();
  if(pp_size < 2){
     total_speed = tl.speed/2.237;
  } else {
    double px = tl.previous_path_x[pp_size-2];
    double cx = tl.previous_path_x[pp_size-1];
    double py = tl.previous_path_y[pp_size-2];
    double cy = tl.previous_path_y[pp_size-1];
    double dx = cx-px;
    double dy = cy-py;
    double ds = sqrt(dx*dx+ dy*dy);
    cout << "DS " << ds <<endl;
    total_speed = ds/0.02;
  }

  double target_vel_mps = target_vel/2.237;
  double total_x = 0;
  Eigen::MatrixXd new_points(3, 50-tl.previous_path_x.size());
  cout << "Current speed mps " << total_speed<<"; Limit "<<target_vel_mps<<endl;
  for (unsigned int i = 0; i<new_points.cols(); i++) {
    if(total_speed < target_vel_mps){
      total_speed += 0.1;
    } else if (total_speed > target_vel_mps) {
      total_speed -= 0.1;
    }
    cout << "  total speed miles/s "<< total_speed<<endl;
    double dx = total_speed * 0.02;
    total_x += dx;
    double x_ref = total_x * cos(ref_yaw);
    double y_ref = s(x_ref);
    new_points(0, i) = x_ref;
    new_points(1, i) = y_ref;
    new_points(2, i) = 1.0;//Homogenous
    // double N = (target_dist / (.02 * target_vel/ 2.24));
    // double x_point = x_add_on + (target_x) /N; //Project points from hypotenuse to x-axis
    // double y_point = s(x_point);

    // x_add_on = x_point;

    // double x_ref = x_point;
    // double y_ref = y_point;

    // new_points(0, i) = x_ref;
    // new_points(1, i) = y_ref;
    // new_points(2, i) = 1.0;//Homogenous
  }

  new_points = Trigs::rigid(new_points, ref_yaw, dx, dy);

  vector<double> new_points_x = row_to_vector(new_points, 0);
  vector<double> new_points_y = row_to_vector(new_points, 1);


  //if(tl.previous_path_x.size()==0){
  next_x_vals.insert(next_x_vals.end(), new_points_x.begin(), new_points_x.end());
  next_y_vals.insert(next_y_vals.end(), new_points_y.begin(), new_points_y.end());
  //}

  Trajectory t;
  t.x =  next_x_vals;
  t.y = next_y_vals;

  return t;
}

array<Position, 2> TrajectoryUtil::get_ref_state(Telemetry tl) {

  int prev_size = tl.previous_path_x.size();
  Position last;
  Position prev;
  // if previous size is almost empty, use the car as starting reference
  if(prev_size < 2) {
    //Use two points that make the path tangent to the car
    last.yaw = Trigs::deg2rad(tl.yaw);
    double prev_car_x = tl.x - cos(last.yaw);
    double prev_car_y = tl.y - sin(last.yaw);
    prev.x = prev_car_x;
    prev.y = prev_car_y;

    last.x = tl.x;
    last.y = tl.y;
  }
  //use the previous path's endpoint as starting reference
  else {
    // Redefine reference state as previous path and point
    double ref_x = tl.previous_path_x[prev_size - 1];
    double ref_y = tl.previous_path_y[prev_size - 1];

    double ref_x_prev = tl.previous_path_x[prev_size-2];
    double ref_y_prev = tl.previous_path_y[prev_size-2];
    double ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    //Use two points that make path tangent to the previous path's end point
    prev.x = ref_x_prev;
    last.x = ref_x;
    prev.y = ref_y_prev;
    last.y = ref_y;
    last.yaw = ref_yaw;
  }

  array<Position, 2> state = {last, prev};
  return state;
}

vector<double> TrajectoryUtil::row_to_vector(Eigen::MatrixXd m, int row){
  Eigen::VectorXd v  = m.row(row);
  vector<double> result(v.data(), v.data() + v.size());
  return result;
}
