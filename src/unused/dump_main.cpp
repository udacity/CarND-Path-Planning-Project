// update Vehicle object x, y, s, d, yaw, speed, path
// update sensor fusion information

// START -- Intermediate Planner
// int prev_size = previous_path_x.size();

// // sensor fusion
// if (prev_size > 0) 
// {
//   car_s = end_path_s;
// }

// bool too_close = false;

// // find a reference value to use
// for (int i = 0; i < sensor_fusion.size(); i++) 
// {
//     // car is in my lane
//     float d = sensor_fusion[i][6];
//     if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) 
//     {
//         double vx = sensor_fusion[i][3];
//         double vy = sensor_fusion[i][4];
//         double check_speed = sqrt(vx*vx + vy*vy);
//         double check_car_s = sensor_fusion[i][5];

//         // get the vehicle in front of you

//         // if using previous points can project s value outward
//         // check s values greater than mine and s gap
//         check_car_s += ((double) prev_size * 0.02 * check_speed);
//         // in front of and within critical zone
//         if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) 
//         {
//             // move to the right/left
//             too_close = true;

//             // initiate a lane change move
//             if (lane == 1) 
//             {
//                 lane = 0;
//             } 
//         }
//     }
// }

// // accel. with +/- 5 m/s^2
// if (too_close) 
// {
//     ref_vel -= 0.224;
// }
// else if (ref_vel < 49.5)
// {
//     ref_vel += 0.224;
// }

// // widely, evenly spaced coordinates
// // interpolated with a spline
// vector<double> ptsx;
// vector<double> ptsy;

// // either reference the starting point as where the car is
// // or at the previous path's end point
// double ref_x = car_x;
// double ref_y = car_y;
// double ref_yaw = deg2rad(car_yaw);

// // use the car as the starting reference
// if (prev_size < 2) 
// {
//     // Use two points that make the path tangent to the car
//     double prev_car_x = car_x - cos(car_yaw);
//     double prev_car_y = car_y - sin(car_yaw);

//     ptsx.push_back(prev_car_x);
//     ptsx.push_back(car_x);

//     ptsy.push_back(prev_car_y);
//     ptsy.push_back(car_y);
// }
// // use the previous path's end point as starting reference
// else 
// {
//     // Redefine reference state as previous end point
//     ref_x = previous_path_x[prev_size - 1];
//     ref_y = previous_path_y[prev_size - 1];

//     double ref_x_prev = previous_path_x[prev_size - 2];
//     double ref_y_prev = previous_path_y[prev_size - 2];
//     ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

//     // Use the two points that make path 
//     // tangent to the previous path's end point
//     ptsx.push_back(ref_x_prev);
//     ptsx.push_back(ref_x);

//     ptsy.push_back(ref_y_prev);
//     ptsy.push_back(ref_y);
// }

// // In Frenet add evenly 30m spaced points ahead of the starting reference
// vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
// vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
// vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

// ptsx.push_back(next_wp0[0]);
// ptsx.push_back(next_wp1[0]);
// ptsx.push_back(next_wp2[0]);

// ptsy.push_back(next_wp0[1]);
// ptsy.push_back(next_wp1[1]);
// ptsy.push_back(next_wp2[1]);

// // transform to local coordinates
// for (int i = 0; i < ptsx.size(); i++) 
// {
//     // shift reference angle to 0 degrees
//     double shift_x = ptsx[i] - ref_x;
//     double shift_y = ptsy[i] - ref_y;

//     ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
//     ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
// }

// // create a spline and set (x, y) points
// tk::spline s;
// s.set_points(ptsx, ptsy);

// // define points for the planner
// vector<double> next_x_vals;
// vector<double> next_y_vals;

// // start with all of the previous path points from last time
// // add new points to the existing path points 
// // smoothen the transition from one path to another path
// for (int i = 0; i < previous_path_x.size(); i++) 
// {
//     next_x_vals.push_back(previous_path_x[i]);
//     next_y_vals.push_back(previous_path_y[i]);
// }

// // Implement JMT trajectory here 

// // calculate how to break up spline points so that we 
// // travel at our desired reference velociy
// double target_x = 30.0; // horizon x
// double target_y = s(target_x); // horizon y
// double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
// double x_add_on = 0;

// for (int i = 1; i <= 50 - previous_path_x.size(); i++) 
// {
//     // convert to m/s
//     double N = (target_dist / (0.02 * ref_vel / 2.24));
//     double x_point = x_add_on + (target_x) / N; // individual x point
//     double y_point = s(x_point); // individual y point

//     x_add_on = x_point;

//     double x_ref = x_point;
//     double y_ref = y_point;

//     // shift + rotation -- convert back to global coordinates
//     x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
//     y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

//     x_point += ref_x;
//     y_point += ref_y;

//     next_x_vals.push_back(x_point);
//     next_y_vals.push_back(y_point);
// }


// next_x_vals = vehicle.getTrajectoryX();
// next_y_vals = vehicle.getTrajectoryY();

// END -- Intermediate Planner

// START -- Basic Planner
// double dist_inc = 0.3;
// for (int i = 0; i < 50; i++) {
//   // i+1 for the next location
//   double next_s = car_s+(i+1)*dist_inc;
//   double next_d = 6;
//   vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

//   next_x_vals.push_back(xy[0]);
//   next_y_vals.push_back(xy[1]);

//   // next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
//   // next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
// }
// END -- Basic Planner
