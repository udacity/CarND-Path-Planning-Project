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

double distance(double x1, double y1, double x2, double y2)
{
  // calculating Euclidean distance between two points
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  // Verify what is the closest waypoint on the highway map

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

  //start in lane 1
  int lane = 1;
  double ref_vel = 0; //mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	auto sensor_fusion = j[1]["sensor_fusion"]; // Format: [id,x,y,vx,vy,s,d] // captures vehicles front and back

            int prev_size = previous_path_x.size();

            //std::cout<<"S Data = "<<car_s<<endl;
            //std::cout<<"Sensor Fusion Data = "<<sensor_fusion<<endl;

            /*
            std::cout<<"Car_x = "<<car_x<<endl;
            std::cout<<"Car_y = "<<car_y<<endl;

            std::cout<<"Car_s = "<<car_s<<endl;
            std::cout<<"Car_d = "<<car_d<<endl;
            std::cout<<"Previous Path X= "<<previous_path_x<<endl;
            std::cout<<"Previous Path Y= "<<previous_path_y<<endl;
            std::cout<<"End Path S= "<<end_path_s<<endl;
            std::cout<<"End Path D= "<<end_path_d<<endl;
            */
            //std::cout<<"Car_s BEFORE = "<<car_s<<endl;

            double car_s_x = car_s;




          	json msgJson;

            // take that previous path's end s value as current s value
            // seems to have an effect where the car performs smoother
            // lane changes

            if(prev_size > 0)
            {
              car_s = end_path_s; 
            }



            bool too_close = false;


            // When the car is in the centre of the right or left lane

            if(lane !=1 && ((car_d > 8.5) or (car_d < 3.5))) 
            {
              bool m_occupied = false;
              for(int i = 0; i < sensor_fusion.size(); i++){    // scan through all sensor
                float d = sensor_fusion[i][6];                  // fusion data
                if(d < (8) && d > (4)){
                  double vx = sensor_fusion[i][3];        // x velocity of detected car
                  double vy = sensor_fusion[i][4];        // y velocity of detected car 
                  double check_speed = sqrt(vx*vx+vy*vy); // velocity of detected car
                  double check_car_s = sensor_fusion[i][5]; // s distance

                  check_car_s += ((double)prev_size*.02*check_speed); // if using previous points can project s value outwards in time
                  //check s values greater than mine and s gap

                  // if the s distance of the middle car and our car is less than 17 metres
                  // or if the car in middle is going more than 7 mph fast than us 
                  // don't go in to middle aka middle is occupied
                  if (abs(check_car_s - car_s) < 15 or car_speed < 30)
                  {
                    m_occupied = true;

                  }
                }

                else if(d < (2+4*lane+2) && d > (2+4*lane-2))  // if detected car is in our lane
                {
                  double vx = sensor_fusion[i][3];        // x velocity of detected car
                  double vy = sensor_fusion[i][4];        // y velocity of detected car 
                  double check_speed = sqrt(vx*vx+vy*vy); // velocity of detected car
                  double check_car_s = sensor_fusion[i][5]; // s distance

                  check_car_s += ((double)prev_size*.02*check_speed); // if using previous points can project s value outwards in time
                  //check s values greater than mine and s gap
                  if((check_car_s > car_s) && ((check_car_s-car_s) < 25)) { // if the car's path extrapolated in the future is greater than ours
                                                                            // (ahead of us) and the gap is less than 30

                    // Do some logic here, lower reference velocity so we don't crash into the car infront of us, could
                    // also flag to try to change lanes.

                    //ref_vel = 29.5; //mph
                    too_close = true;
                  }
                }

              }

              if (m_occupied == false){
                lane = 1;
              }

              if(too_close){
                ref_vel -= .624;
              }
              else if(ref_vel < 49.5)
              {
                ref_vel += .424;
              }
            }

            else {
                            // REAR END MODULE
              for(int i = 0; i < sensor_fusion.size(); i++){
                float d = sensor_fusion[i][6];
                if(d < (2+4*lane+2) && d > (2+4*lane-2))  // if detected car is in our lane
                {
                  double vx = sensor_fusion[i][3];        // x velocity of detected car
                  double vy = sensor_fusion[i][4];        // y velocity of detected car 
                  double check_speed = sqrt(vx*vx+vy*vy); // velocity of detected car
                  double check_car_s = sensor_fusion[i][5]; // s distance

                  check_car_s += ((double)prev_size*.02*check_speed); // if using previous points can project s value outwards in time
                  //check s values greater than mine and s gap
                  if((check_car_s > car_s) && ((check_car_s-car_s) < 25)) { // if the car's path extrapolated in the future is greater than ours
                                                                            // (ahead of us) and the gap is less than 20

                    // Do some logic here, lower reference velocity so we don't crash into the car infront of us, could
                    // also flag to try to change lanes.

                    //ref_vel = 29.5; //mph
                    too_close = true;
                    // LANE CHANGE FROM MIDDLE
                    
                    bool l_occupied = false;
                    bool r_occupied = false;

                    for (int j = 0; j < sensor_fusion.size(); j++){
                      float d_side = sensor_fusion[j][6];

                      // Checking for occupancy of left lane
                      if (d_side < (2+4*(lane-1)+3) && d_side > (4*(lane-1))) // if d is 0 to 5
                      {
                        std::cout<<"L Lane Check = "<<endl;
                        vx = sensor_fusion[j][3];          // x velocity of detected car
                        vy = sensor_fusion[j][4];          // y velocity of detected car 
                        check_speed = sqrt(vx*vx+vy*vy);   // velocity of detected car
                        check_car_s = sensor_fusion[j][5]; // s distance

                        check_car_s += ((double)prev_size*.02*check_speed); // if using previous points can project s value outwards in time
                        //check s values greater than mine and s gap
                        if ((abs(check_car_s - car_s) < 20) or car_speed < 30)//((check_car_s > car_s) && ((check_car_s-car_s) < 20))
                        {
                          l_occupied = true;
                        }
                      }
                      // Checking for occupancy of right lane
                      else if (d_side < (2+4*(lane+1)+2) && d_side > (2+4*(lane+1)-3))
                      {
                        std::cout<<"R Lane Check = "<<endl;
                        vx = sensor_fusion[j][3];          // x velocity of detected car
                        vy = sensor_fusion[j][4];          // y velocity of detected car 
                        check_speed = sqrt(vx*vx+vy*vy);   // velocity of detected car
                        check_car_s = sensor_fusion[j][5]; // s distance

                        check_car_s += ((double)prev_size*.02*check_speed); // if using previous points can project s value outwards in time
                        //check s values greater than mine and s gap
                        if((abs(check_car_s - car_s) < 20) or car_speed < 30) //((check_car_s > car_s) && ((check_car_s-car_s) < 30))
                        {
                          r_occupied = true;
                        }
                      }
                    }

                    if (l_occupied == true && r_occupied == true)
                    {
                      lane = lane;
                      too_close = true;
                    }
                    else if (r_occupied == false)
                    {
                      if (lane != 2){
                        lane = lane + 1;
                      }
                    }
                    else if (l_occupied == false)
                    {
                      if (lane != 0){
                        lane = lane - 1;
                      }
                    } 
                    
                  }
                }
              }


              if(too_close){
                ref_vel -= .324;
              }
              else if(ref_vel < 49.5)
              {
                ref_vel += .424;
              }
            }

            // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
            // Later we will interpolate these waypoints with a spline and fill it in with more points that control

          	vector<double> ptsx;
          	vector<double> ptsy;

            // reference x,y, yaw states
            // either we will reference the starting point as where the car is or at the previous paths end point

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // ADDING A POINT FOR SPLINE THAT IS THE END POINT OF PREVIOUS PATH OR STARTING POSITION

            // if previous size is almost empty, use the car as starting reference
            if(prev_size < 2){

              //use two points that make the path tangent to the car
              // THESE POINTS ARE A UNIT VECTOR AWAY FROM CURRENT POSE 
              double prev_car_x = car_x - cos(car_yaw);  
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            }

            // use the previous path's end point as starting reference
            else{

              //Redefine reference state as previous path end point
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];

              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);


            }
            //std::cout<<"Car_s AFTER = "<<car_s<<endl;

            //car_s = car_s_x;
            // ADDING MORE POINTS FOR SPLINE THAT ARE 30 

            //In Frenet add evenly 30m spaced points ahead of the starting reference
            vector<double> next_wp0 = getXY(car_s+30, (2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+50, (2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+70, (2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            // if previous size is almost empty, use the car as starting reference

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            //  cout<<(ptsx.size())<<endl;

            for(int i = 0; i < ptsx.size(); i++) // Ptsx size is 5
            {
              //double next_s = car_s+(i+1)*dist_inc;
              //double next_d = 6;
              double shift_x = ptsx[i]-ref_x; // distance from sample point of path with current position
              double shift_y = ptsy[i]-ref_y;

              ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw)); // shift in rotation from global to local car
              ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw)); // coordinate

            }

            // create a spline
            tk::spline s;

            // set(x,y) points to the spline
            s.set_points(ptsx,ptsy);

            // Defin1e the actual (x,y) points we will use for the planner
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // Start with all of the previous path points from last time
            for(int i = 0; i < previous_path_x.size(); i++){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate how to break up spline points so that we travel at our desired reference velocity
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

            double x_add_on = 0;

            // Fill up the rest of our path planner after filling it with previous points, here we will always 
            //output 50 points

            for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
              double N = (target_dist/(.02*ref_vel/2.24)); // 2.24 convert from mph to mps
              double x_point = x_add_on + (target_x)/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // rotate points back to global coordinates
              // to reverse earlier translate/rotate to local car coordinates
              x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
              y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

              // translate points
              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }



            // END
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
