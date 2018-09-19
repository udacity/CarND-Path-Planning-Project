#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <thread>
#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#include "helpers.h"
#include "cost_functions.h"

using namespace std;

// for convenience
using json = nlohmann::json;

const double TARGET_SPEED = 49.5;
const double DT = 0.02;
const double d2r = M_PI / 180.;
const double tomps = 0.224;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = fabs(theta - heading);
	angle = min(2 * pi() - angle, angle);

	if (angle > pi() / 4)
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
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
		prev_wp = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x, y};
}

int main()
{
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
	while (getline(in_map_, line))
	{
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

	// start in lane 1;
	int lane = 1;
	// reference velocity
	double ref_vel = 0; //mph
	Vehicle ego;
	bool initialized = false;

	h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &lane, &ref_vel, &ego, &initialized](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
																																				  uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		// auto sdata = string(data).substr(0, length);
		// cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{

			auto s = hasData(data);

			if (s != "")
			{
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry")
				{
					std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
					// j[1] is the data JSON object

					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					auto prev_path_x = j[1]["previous_path_x"];
					auto prev_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					size_t prev_size = prev_path_x.size();
					cout << "prev size = " << prev_size << endl;
					cout << "car_s = " << car_s << endl;
					vector<double> previous_path_x;
					vector<double> previous_path_y;

					size_t idx_path_start = 0;
					if (prev_size > 10)
					{
						prev_size = 5;

						previous_path_x.resize(prev_size);
						previous_path_y.resize(prev_size);
						for (size_t i = 0; i < prev_size; ++i)
						{
							previous_path_x[i] = prev_path_x[i];
							previous_path_y[i] = prev_path_y[i];
						}
						auto tmp = getFrenet(previous_path_x[prev_size - 1], previous_path_y[prev_size - 1], car_yaw * d2r, map_waypoints_x, map_waypoints_y);
						//cout << "end path s = " << end_path_s << " tmp[0] = " << tmp[0] << endl;
						end_path_s = tmp[0];
						end_path_d = tmp[1];
						car_s = end_path_s;
						car_d = end_path_d;
					}

					vector<double> prev_d, prev_s;
					for (size_t i = 0; i < previous_path_x.size(); ++i)
					{
						auto tmp = getFrenet(previous_path_x[i], previous_path_y[i], car_yaw * d2r, map_waypoints_x, map_waypoints_y);
						prev_s.push_back(tmp[0]);
						prev_d.push_back(tmp[1]);
					}
					cout << "previous s = ";
					printVec(prev_s);

					car_speed *= tomps;
					cout << "car: speed = " << car_speed << "m/s, (x,y) = (" << car_x << ", " << car_y << "), (s, d) = (" << car_s << ", " << car_d << ")\n";

					// use ref_vel instead of car_speed
					car_speed = ref_vel;
					ego.state = {car_s, car_speed, 0, car_d, 0, 0};
					double dt = 0.02;
					int N = 50;
					size_t idx_prev_end = 0;

					if (!initialized)
					{
						ego.lstate = "KL";
						ego.lanes_available = 3;
						ego.state = {car_s, 0, 0, car_d, 0, 0};
						ego.buf_states.resize(N);
						initialized = true;
					}

					ego.updateLane();

					vector<Vehicle> predictions;
					for (size_t i = 0; i < sensor_fusion.size(); ++i)
					{ // car's unique ID, car's x position in map coordinates,
						// car's y position in map coordinates, car's x velocity in m/s,
						// car's y velocity in m/s, car's s position in frenet coordinates,
						// car's d position in frenet coordinates
						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double v_speed = sqrt(vx * vx + vy * vy);
						//v_speed = abs(v_speed);
						//v_speed *= tomps;
						vector<double> veh = {sensor_fusion[i][5], v_speed /*+ car_speed*/, 0, sensor_fusion[i][6], 0, 0};
						predictions.push_back(Vehicle(veh));
						predictions[i].updateTraj();
					}

					//vector<double> ego_rst = ego.choose_next_state_v2(predictions);
					auto ego_rst = ego.choose_next_state_v3(predictions);
					ego.prev_traj = ego_rst;
					cout << "target speed = " << ego.target_speed << ", ref_vel = " << ref_vel << endl;
					//vector<double> ego_rst = ego.free_lane_trajectory();
					//ego.state = evalState(ego_rst, dt * (N - prev_size));

					//cout << "cost = " << ego_rst[13] << ", lane = " << ego.lane << ", state = " << ego.lstate << "\n";

					printState(ego.state);

					vector<double> t_vec;
					for (int i = 0; i < (N - prev_size); i++)
					{
						t_vec.push_back(dt * (1 + i));
					}
					vector<double> t_fit = {2, 3, 4, 5};
					vector<double> traj_s, traj_d, fit_s, fit_d;
					ego_rst.pos(t_vec, traj_s, traj_d);
					ego_rst.pos(t_fit, fit_s, fit_d);

					cout << "fit_s = ";
					printVec(fit_s);
					//cout << "traj_s = ";
					//printVec(traj_s);
					vector<double> traj_x, traj_y, fit_x, fit_y;

					for (size_t i = 0; i < traj_s.size(); ++i)
					{
						auto tmp = getXY(traj_s[i], traj_d[i], map_waypoints_s, map_waypoints_x, map_waypoints_y);
						traj_x.push_back(tmp[0]);
						traj_y.push_back(tmp[1]);
					}
					for (size_t i = 0; i < t_fit.size(); ++i)
					{
						auto tmp = getXY(fit_s[i], fit_d[i], map_waypoints_s, map_waypoints_x, map_waypoints_y);
						fit_x.push_back(tmp[0]);
						fit_y.push_back(tmp[1]);
					}

					// a list of waypoints, evenly spaced at 30m
					// then interpolate with a spline and fill more points
					vector<double> ptsx, ptsy;
					// vehicle frame
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);
					// add 'previous' points to smooth the waypoints
					if (prev_size < 2)
					{
						// add tangents to smooth the path
						double prev_car_x = car_x - cos(ref_yaw);
						double prev_car_y = car_y - sin(ref_yaw);
						ptsx.push_back(prev_car_x);
						ptsx.push_back(car_x);
						ptsy.push_back(prev_car_y);
						ptsy.push_back(car_y);
					}
					else
					{
						ref_x = previous_path_x[prev_size - 1];
						ref_y = previous_path_y[prev_size - 1];

						double ref_x_prev = previous_path_x[prev_size - 5];
						double ref_y_prev = previous_path_y[prev_size - 5];

						ptsx.push_back(ref_x_prev);
						ptsx.push_back(ref_x);
						ptsy.push_back(ref_y_prev);
						ptsy.push_back(ref_y);
					}
					/*for (size_t i = 0; i < fit_x.size(); ++i)
					{
						if (fit_x[i] > ptsx[1])
						{
							ptsx.push_back(fit_x[i]);
							ptsy.push_back(fit_y[i]);
						}
					}*/
					ptsx.insert(ptsx.end(), fit_x.begin(), fit_x.end());
					ptsy.insert(ptsy.end(), fit_y.begin(), fit_y.end());

					vector<double> fx_v, fy_v, tmpx, tmpy;
					toVehicleFrame(fx_v, fy_v, ptsx, ptsy, ref_x, ref_y, ref_yaw);
					toVehicleFrame(tmpx, tmpy, traj_x, traj_y, ref_x, ref_y, ref_yaw);
					traj_x = tmpx;
					traj_y = tmpy;

					tk::spline s;
					sortVecs(fx_v, fy_v);
					s.set_points(fx_v, fy_v);
					double dist_x = 30;
					double dist_y = s(dist_x);
					double slope = dist_x / sqrt(dist_x * dist_x + dist_y * dist_y);

					vector<double> next_x, next_y;

					if (abs(ref_vel - ego.target_speed) > 0.224)
					{
						if (ref_vel < ego.target_speed)
							ref_vel += 0.224 * 2.5;
						else
							ref_vel -= 0.224;
					}
					else if (abs(ref_vel - ego.target_speed) > 0.01)
					{
						ref_vel = ego.target_speed;
					}
					/*
					if (ref_vel > ego.target_speed + 0.225)
					{
						ref_vel -= 0.112 * 2;
					}
					else if (ref_vel < ego.target_speed)
					{
						ref_vel += 0.112 * 2;
					}*/
					//ref_vel = ego.target_speed;
					for (size_t i = 0; i < traj_x.size(); ++i)
					{
						traj_x[i] = (i + 1) * 0.02 * ref_vel * slope;
						traj_y[i] = s(traj_x[i]);
					}
					toWorldFrame(next_x, next_y, traj_x, traj_y, ref_x, ref_y, ref_yaw);

					std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
					std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() << std::endl;

					next_x.insert(next_x.begin(), previous_path_x.begin(), previous_path_x.end());
					next_y.insert(next_y.begin(), previous_path_y.begin(), previous_path_y.end());
					cout << "next_x size = " << next_x.size() << endl;
					// END
					json msgJson;
					msgJson["next_x"] = next_x;
					msgJson["next_y"] = next_y;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
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
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
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
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
