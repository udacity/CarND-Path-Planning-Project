#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

void print(const char* input)
{
    std::cout << input << std::endl;
}

void print(double input)
{
    std::cout << input << std::endl;
}

void print(const char* input, int val) {
  std::cout << input << val << std::endl;
}

void print(const char* input, double val) {
  std::cout << input << val << std::endl;
}

bool collide(vector<double> point, vector<double> car_point, vector<double> old_car_point, int lane_check) {
  double temp_d = point[0];
  double back_s = point[1] - old_car_point[1];
  double temp_s = point[1] - car_point[1];
  //print(temp_d);
  //print(temp_s);
  //print(car_point[0]);

  double front_distance = 5.0;//min_s
  double back_distance = -2.0;//max_s
  double lane_width = 4.0;

  // If it is in the lane we are looking at.
  if (temp_d < lane_check * lane_width + lane_width && temp_d > lane_width * lane_check){
      // If it is close enough to worry about..
      if (back_s > back_distance && temp_s < front_distance) {
        //print(temp_d);
        //print(temp_s);
        //print("lane_check ",lane_check);
        return true;
      }
  } else {
    return false;
  }
  return false;
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);




  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  bool debug = false;
  // The current lane.
  int lane = 1;
  // Our target speed.
  double ref_vel = 0.0; // In miles per hour(mph).

  auto last_lane_shift = std::chrono::system_clock::now();


  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &debug, &last_lane_shift](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

            // Length of the previous path.
            int previous_size = previous_path_x.size();

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


            double old_car_s;
            if (previous_size > 0) {
              old_car_s = car_s;
              car_s = end_path_s;
            }

            bool too_close = false;
            double other_car_speed = 0.0;
            double dist_to_car = 0.0;

            //double lane_width = 4.0;
            //double distance_ahead = 10;
            //double distance_behind = -10;
            int num_lanes = 3;
            vector<bool> safe_lanes;// = {true, true, true};
            for (int j = 0; j < num_lanes; j++) {
              safe_lanes.push_back(true);
            }
            // Find ref_v to use.
            int telemetry_size = sensor_fusion.size();
            //print("telemetry", telemetry_size);
            for (int i = 0; i < sensor_fusion.size(); i++) {
              // Car is in my lane.
              float d = sensor_fusion[i][6];
              float s = sensor_fusion[i][5];
              for (int j = 0; j < num_lanes; j++) {
                if (collide({d, s}, {car_d, car_s}, {car_d, old_car_s}, j)) {safe_lanes[j] = false;}
              }
              if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s = sensor_fusion[i][5];
                //print(check_car_s - car_s);

                // If using previous points can project s value out.
                check_car_s += ((double)previous_size * 0.02 * check_speed);
                // Check s values greater than mine and the gap between us.
                if ((check_car_s > car_s) && ((check_car_s-car_s) < 30)) {
                  // Lower reference velocity so we don't crash into the car in
                  // front of us. Could also flag to try to change lanes.
                  //ref_vel = 29.5; // In miles per hour(mps).
                  too_close = true;
                  other_car_speed = check_speed;
                  dist_to_car = check_car_s - car_s;
                }
              }
            }
            if (debug) {
              print(too_close);
              print(car_d);
            }

            bool try_lane_shift = false;
            if (too_close && ref_vel > other_car_speed) {
              ref_vel -= 0.224 + 1 / dist_to_car;
              if (ref_vel < other_car_speed) {
                ref_vel = other_car_speed;
              }
              if ((std::chrono::system_clock::now() - last_lane_shift).count() > 1.5) {
                try_lane_shift = true;
                last_lane_shift = std::chrono::system_clock::now();
              }
              //print("try lane shift");
            } else if (ref_vel < 49.5) {
              ref_vel+= 0.224;
              if (ref_vel > 49.5) {
                ref_vel = 49.5;
              }
            }

            if (try_lane_shift) {
              //print("Trying to shift lanes.");
              for (int j = 0; j < num_lanes; j++) {
                //print(safe_lanes[j]);
                if (j != lane) {
                  if (abs(j - lane) == 1) {
                    // We are right next to the lane being considered.
                    if (safe_lanes[j] && try_lane_shift) {
                      lane = j;
                      //print("Changed to lane ", lane);
                      try_lane_shift = false;
                    }
                  }
                }
              }
              //print("");
              }


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            // Spline based system.

            /*
            if (prev_size > 0) {
              car_s = end_path_s
            }
            */

            // List of widely spaced points.
            vector<double> ptsx;
            vector<double> ptsy;

            // reference x, y, and yaw states.
            // either refrence from the start point or at the previous path's
            // end point.
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            if (debug == true){
              print("Reference points:");
              print("Ref X:", ref_x);
              print("Ref Y:", ref_y);
              print("Ref Yaw:", ref_yaw);
            }

            // if previous size is almost empty, use the car as starting
            // reference.
            if (previous_size < 2) {
              double prev_car_x = car_x - cos(ref_yaw);
              double prev_car_y = car_y - sin(ref_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            } else {
              // redefine reference state as previous path end point.
              ref_x = previous_path_x[previous_size - 1];
              if (debug == true) {print("Previouis path last:", ref_x);}
              ref_y = previous_path_y[previous_size - 1];

              double ref_x_prev = previous_path_x[previous_size - 2];
              double ref_y_prev = previous_path_y[previous_size - 2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              // Use two points that make the path tangent to the previous
              // path's end point.
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }

            // Variables to decide the next waypoints.
            int lane_d = 2.0 + 4.0 * lane;
            double step_size = 30.0;
            int num_waypoints = 3;
            /*
            for (int i = 0; i < num_waypoints; i++) {
              vector<double> next_waypoint = getXY(car_s + step_size*i, lane_d,
                                                   map_waypoints_s,
                                                   map_waypoints_x,
                                                   map_waypoints_y
                                                 );
              ptsx.push_back(next_waypoint[0]);
              ptsy.push_back(next_waypoint[1]);
            }
            */
            //vector<double> ref_s_d = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp0 = getXY(car_s + 30, (2.0 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        		vector<double> next_wp1 = getXY(car_s + 60, (2.0 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        		vector<double> next_wp2 = getXY(car_s + 90, (2.0 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            if (debug == true) {
              print("Ref X:", ref_x);
              print("Next waypoint 1-x:", next_wp0[0]);
              print("Next waypoint 1-y:", next_wp0[1]);
              print("Next waypoint 2-x:", next_wp1[0]);
              print("Next waypoint 2-y:", next_wp1[1]);
              print("Next waypoint 3-x:", next_wp2[0]);
              print("Next waypoint 3-y:", next_wp2[1]);
            }

        		ptsx.push_back(next_wp0[0]);
        		ptsx.push_back(next_wp1[0]);
        		ptsx.push_back(next_wp2[0]);

        		ptsy.push_back(next_wp0[1]);
        		ptsy.push_back(next_wp1[1]);
        		ptsy.push_back(next_wp2[1]);


            for (int i = 0; i < ptsx.size(); i++) {
              // Shift reference angle to zero.
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = (shift_x * cos(0 - ref_yaw)
                          - shift_y * sin(0 - ref_yaw));
              ptsy[i] = (shift_x * sin(0 - ref_yaw)
                          + shift_y * cos(0 - ref_yaw));
              if (debug == true) {
                //std::cout << ptsx[i] << "\n" << ptsy[i] << std::endl;
                print("ptsx[i]:", ptsx[i]);
              }
            }



            // Create base spline.
            tk::spline s;

            // Set points of the spline.
            s.set_points(ptsx, ptsy);

            // Define the actual points to be used in the planner.
            //vector<double> next_x_vals;
            //vector<double> next_y_vals;

            // Start with all the previous path points from the last time.
            for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate how to break up the spline so that we travel at the
            // proper speed.
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)
                                   +(target_y)*(target_y));
            double x_add_on = 0;

            // Fill the rest of our path planner after filling it with previous
            // points. Always output num_points points.
            int num_points = 50;
            for (int i = 1; i <= num_points - previous_path_x.size(); i++) {
              double N = (target_dist/(0.02*ref_vel/2.24));
              double x_point = x_add_on + (target_x)/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // rotate back to normal to reset to actual angle.
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }


            /*
            // First examples.
            double dist_inc = 0.25;
            double next_s, next_d;
            double next_x, next_y;
            for (int i = 0; i < 50; i++) {
              next_s = car_s + (dist_inc * i + 1);
              next_d = 6;
              vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              next_x_vals.push_back(xy[0]);
              next_y_vals.push_back(xy[1]);
              // Straight line.
              //next_x_vals.push_back(car_s + (dist_inc * i + 1) * cos(deg2rad(car_yaw)));
              //next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));

            }
            */
            // END
            if (debug == true) {print(next_x_vals.size());}
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
