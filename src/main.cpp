#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <deque>
#include <queue>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
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
vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

    Eigen::MatrixXd t(3,3);
    t << pow(T, 3), pow(T, 4), pow(T, 5),
         3*pow(T, 2), 4*pow(T, 3), 5*pow(T, 4),
         6*T, 12*pow(T, 2), 20*pow(T, 3);

    Eigen::MatrixXd t_inv = t.inverse();

    Eigen::VectorXd s(3);
    s << end[0] - (start[0] + start[1] * T + .5 * start[2]*T*T),
         end[1] - (start[1] + start[2] * T),
         end[2] - start[2];

    Eigen::VectorXd a(6);
    a << start[0], start[1], .5 * start[2], t_inv * s;

    vector<double> vec(a.data(), a.data() + a.rows() * a.cols());
    return vec;

}

const int LEFT = -1;
const int RIGHT = 1;
void laneChange(const int direction, queue<double> &lane_change_offsets, int &current_lane) {

    double current_position = current_lane*4.0 + 2.0;
    double new_position = current_position + direction * 4.0;

    current_lane += direction;

    vector<double> current_position_horiz = {current_position,0.0,0.0};
    vector<double> next_position_horiz = {new_position, 0.0, 0.0};

    vector<double> jmt = JMT(current_position_horiz, next_position_horiz, 2);

    for (double t = .02; t <= 2.0; t+= .02) {
      double offset = 0;
      for (int i = 0; i < jmt.size(); i++) {
	offset += jmt[i] * pow(t, i);
      }
      lane_change_offsets.push(offset);
    }
}


double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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

  deque<double> middle_line_trajectory_x, middle_line_trajectory_y;
  int trajectory_points_inserted = 0;

  queue<double> lane_change_offsets;
  queue<double> slow_down_offsets;
  int current_lane = 0;
  double current_slowdown = 0;


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


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &middle_line_trajectory_x, &middle_line_trajectory_y, &trajectory_points_inserted, &lane_change_offsets, &slow_down_offsets, &current_lane, &current_slowdown](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;


		double car_theta = deg2rad(car_yaw);

		int previous_path_length = previous_path_x.size();
		double end_path_x, end_path_y;
		double end_path_theta;
		if (previous_path_length < 2) {
		    end_path_s = car_s;
		    end_path_d = car_d;
		    end_path_theta = car_theta;
		    end_path_x = car_x;
		    end_path_y = car_y;
		} else {
		    end_path_x = middle_line_trajectory_x.back();
		    end_path_y = middle_line_trajectory_y.back();
		    double second_end_path_x = *(++middle_line_trajectory_x.rbegin());
		    double second_end_path_y = *(++middle_line_trajectory_y.rbegin());

		    end_path_theta = atan2(end_path_y - second_end_path_y, end_path_x - second_end_path_x);

		}

		// Add enough points to get our number of points to 50
		double goal_miles_per_hour = 45;
		const double seconds_per_hour = 3600;
		const double meters_per_mile = 1609.34;

		int trajectory_points_traveled = trajectory_points_inserted - previous_path_length;
		for (int i = 0; i < trajectory_points_traveled; i++) {
		  middle_line_trajectory_x.pop_front();
		  middle_line_trajectory_y.pop_front();
		  trajectory_points_inserted--;
		}

		vector<double> next_x_vals;
		vector<double> next_y_vals;
		for (int i = 0; i < previous_path_length; i++) {
		    next_x_vals.push_back(previous_path_x[i]);
		    next_y_vals.push_back(previous_path_y[i]);
		}

		tk::spline s;

		vector<double> spline_x, spline_y;

		int num_waypoints = map_waypoints_x.size();

		// Get info about other cars
		for (int i = 0; i < sensor_fusion.size(); i++) {
		  int other_car_number = sensor_fusion[i][0];
		  double other_car_x = sensor_fusion[i][1];
		  double other_car_y = sensor_fusion[i][2];
		  double other_car_vx = sensor_fusion[i][3];
		  double other_car_vy = sensor_fusion[i][4];
		  double other_car_s = sensor_fusion[i][5];
		  double other_car_d = sensor_fusion[i][6];
		  double other_car_theta = atan2(other_car_vy, other_car_vx);

		  bool in_same_lane = ((current_lane*4+2) - 2 <= other_car_d && other_car_d <= (current_lane*4+2) + 2);
		  double other_car_speed = other_car_vx * cos(-other_car_theta) - other_car_vy * sin(-other_car_theta);

		  other_car_speed *= (seconds_per_hour / meters_per_mile);
		  
		  
		  double distance = (double)sensor_fusion[i][5] - car_s;
		  if (0 <= distance && distance <= 30 && lane_change_offsets.empty() && in_same_lane) {
		    cout << other_car_speed << endl;
		    goal_miles_per_hour = other_car_speed;
		    //laneChange(RIGHT, lane_change_offsets, current_lane);
		  }
		}

		// Activate lane change demo
		//if (lane_change_offsets.empty()) {
		//  laneChange(RIGHT, lane_change_offsets, current_lane);
		//  laneChange(LEFT, lane_change_offsets, current_lane);
		//  laneChange(0, lane_change_offsets, current_lane);
		//}

		int starting_waypoint = ClosestWaypoint(end_path_x, end_path_y, map_waypoints_x, map_waypoints_y) - 4;
		starting_waypoint = starting_waypoint % num_waypoints;
		starting_waypoint += (starting_waypoint < 0) * num_waypoints;

		for (int i = 0; i < 9; i++) {
		    int waypoint_num = (starting_waypoint + i) % num_waypoints;
		    waypoint_num += (waypoint_num < 0) * num_waypoints;
		    double shifted_x = map_waypoints_x[waypoint_num] - end_path_x;
		    double shifted_y = map_waypoints_y[waypoint_num] - end_path_y;

		    double rotated_x = shifted_x * cos(-end_path_theta) - shifted_y * sin(-end_path_theta);
		    double rotated_y = shifted_x * sin(-end_path_theta) + shifted_y * cos(-end_path_theta);

		    spline_x.push_back(rotated_x);
		    spline_y.push_back(rotated_y);
		}

		s.set_points(spline_x, spline_y);

		double goal_meters_per_second = goal_miles_per_hour * meters_per_mile / seconds_per_hour;

		const double seconds_per_iteration = .02;
		const double goal_meters_per_iteration = goal_meters_per_second * seconds_per_iteration;

		for (int i = 1; i <= 100 - previous_path_length; i++) {
		    double rotated_x = goal_meters_per_iteration * i;
		    double rotated_y = s(rotated_x);

		    double offset = current_lane * 4 + 2;
		    if (!lane_change_offsets.empty()) {
		      offset = lane_change_offsets.front();
		      lane_change_offsets.pop();
		    }
		    double rotated_y_offset = rotated_y - offset;

		    double shifted_x = rotated_x * cos(end_path_theta) - rotated_y * sin(end_path_theta);
		    double shifted_y = rotated_x * sin(end_path_theta) + rotated_y * cos(end_path_theta);
		    double shifted_x_offset = rotated_x * cos(end_path_theta) - rotated_y_offset * sin(end_path_theta);
		    double shifted_y_offset = rotated_x * sin(end_path_theta) + rotated_y_offset * cos(end_path_theta);


		    double x = shifted_x + end_path_x;
		    double y = shifted_y + end_path_y;
		    double x_offset = shifted_x_offset + end_path_x;
		    double y_offset = shifted_y_offset + end_path_y;

		    next_x_vals.push_back(x_offset);
		    next_y_vals.push_back(y_offset);
		    middle_line_trajectory_x.push_back(x);
		    middle_line_trajectory_y.push_back(y);
		    trajectory_points_inserted++;

		}


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
