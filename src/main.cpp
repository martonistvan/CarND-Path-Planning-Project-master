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
#include "helper_functions.h"
#include "vehicle.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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

  // define lane number
  int ego_lane;

  //define reference velocity
  double ref_vel = 0.0; //mph

  //create ego vehicle
  Vehicle ego_car = Vehicle(0,0,0,0,0);

  h.onMessage([&ref_vel, &ego_lane, &ego_car, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	//set horizon and reference s, d
          	int prev_size = previous_path_x.size();
          	int horizon = 20;
          	if(prev_size > 0) {
          		car_s = end_path_s;
          		car_d = end_path_d;
          		horizon = prev_size;
          	}

          	//get current lane for ego car
          	ego_lane = GetLane(car_d);

          	//store ego car loc. data
          	ego_car.update_vehicle(ego_lane, car_s, car_speed, 0, car_d, car_x, car_y, car_yaw);

          	bool too_close = false;

          	map<int ,vector<Vehicle::prediction>>  pred_other_cars;

          	//I. PREDICTION
          	//Predict other car's future state
          	// The data format for each car is: [ id, x, y, vx, vy, s, d].

          	for(int i = 0; i < sensor_fusion.size(); i++) {
          		Vehicle other_car = Vehicle(GetLane(sensor_fusion[i][6]), sensor_fusion[i][5], GetSpeed(sensor_fusion[i][3],sensor_fusion[i][4]),0,sensor_fusion[i][6]);
          		vector<Vehicle::prediction> pred = other_car.generate_predictions(horizon);
          		pred_other_cars[sensor_fusion[i][0]] = pred;
          	}

          	//II. Behavioral planning

          	States target_state = KL;

          	//get closest car in current lane ahead of ego
          	int closest_car_inlane = CarInMinDistInLane(ego_lane, pred_other_cars, car_s, car_d, prev_size);

          	double speed_diff = 0;
          	int target_lane = ego_lane;
          	double closest_car_speed = 0;
          	if(closest_car_inlane >= 0) {
          		too_close = true;

          		closest_car_speed = pred_other_cars.at(closest_car_inlane)[pred_other_cars.at(closest_car_inlane).size()-1].v;

          		//search for optimal lane to drive
          		target_state = SearchForLane(ego_lane, pred_other_cars, car_s, car_d, car_speed, prev_size);
          		if(target_state != KL) {too_close = false;}
          		if(target_state == LCL) {target_lane = ego_lane-1;}
          		if(target_state == LCR) {target_lane = ego_lane+1;}

          	}


          	json msgJson;

          	//III. Generate trajectory according to the new state

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
          	vector<vector<double>> trajectory;

          	//trajectory = GenerateTrajectory(car_x, car_y, car_yaw, previous_path_x, previous_path_y, car_s, target_lane, map_waypoints_x, map_waypoints_y, map_waypoints_s, too_close, ref_vel);
          	trajectory = GenerateTrajectory(ego_car, previous_path_x, previous_path_y, target_lane, map_waypoints_x, map_waypoints_y, map_waypoints_s, too_close, closest_car_speed, ref_vel);
          	next_x_vals = trajectory[0];
          	next_y_vals = trajectory[1];

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
















































































