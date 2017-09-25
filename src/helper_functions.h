/*
 * helper_functions.h
 * Some helper functions for the 2D particle filter.
 *  Created on: Dec 13, 2016
 *      Author: Tiffany Huang
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <iterator>
#include "vehicle.h"
#include "spline.h"

using namespace std;


//double allowed_dist_ahead = 20;
//double allowed_dist_behind = 10;
double max_speed = 50;

enum States {KL, LCL, LCR, PLCL, PLCR};

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int GetLane(double d) {
	//get current lane id
	return (int)(round((d-2)/4));
}

double GetSpeed(double vx, double vy) {
	//calculate speed
	return sqrt(pow(vx,2)+pow(vy,2));
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

bool CheckLaneFree(int lane, map<int ,vector<Vehicle::prediction>> other_cars, double ego_s, double ego_d, int prev_size, double ahead, double behind) {
	bool isfree = true;

	map<int ,vector<Vehicle::prediction>>::iterator it = other_cars.begin();
	//check if any lane is free to safely change to
	while(it != other_cars.end()) {
		float oc_d= it->second[it->second.size()-1].d;
		int oc_lane = GetLane(oc_d);
		if(oc_lane == lane) {
			double oc_v = it->second[it->second.size()-1].v;
			double oc_s = it->second[it->second.size()-1].s;

			if(oc_s < (ego_s + ahead) && oc_s > (ego_s - behind)) {
				isfree = false;
			}
		}
		it++;
	}

	return isfree;
}


int CarInMinDistInLane(int lane, map<int ,vector<Vehicle::prediction>> other_cars, double ego_s, double ego_d, int prev_size) {
	int closest_car_id = -2;
	double closest_dist = 500;

	double min_range_d = 2+4*lane - 2;
	double max_range_d = 2+4*lane + 2;

	map<int ,vector<Vehicle::prediction>>::iterator it = other_cars.begin();

	//check all the other cars ahead of us and return its id
	while(it != other_cars.end()) {
		float oc_d= it->second[it->second.size()-1].d;
		if(oc_d > min_range_d && oc_d < max_range_d) {
			double oc_s = it->second[it->second.size()-1].s;
			if(oc_s > ego_s && (oc_s - ego_s) < 30 && (oc_s - ego_s) < closest_dist) {
				closest_car_id = it->first;
			}
		}
		it++;
	}

	return closest_car_id;
}

/*int SearchForLane(int ego_lane, map<int ,vector<Vehicle::prediction>> other_cars, double ego_s, double ego_d, double ego_speed, int prev_size) {
	int target_lane = ego_lane;
	map<int,double> cost_per_lane;
	bool lane_free;
	for(int i = 0; i<=2; i++) {
		//cout << "lane: " << i << endl;
		double lane_cost = 0;
		int closest_car_inlane = -2;
		lane_free = true;
		if(i!=ego_lane){
			lane_free = CheckLaneFree(i, other_cars, ego_s, ego_d, prev_size, 20, 10);
			if(!lane_free) {lane_cost += 10000;} //cost for lane with cars within range (+20, - 20) -> no lane change
			if (abs(i - ego_lane) > 1) {lane_cost += 5000; } //cost for more than one lane change
			if (ego_speed < 20.0) {lane_cost += 2000;} //

		}

		lane_cost += pow(max_speed-ego_speed,2)*500; //driving close to limit

		//check for cars ahead to see which lane is the best
		closest_car_inlane = CarInMinDistInLane(i, other_cars, ego_s, ego_d, prev_size);

		if(closest_car_inlane > -1) {
			//cost for lanes with slow cars that are within range of 30
			lane_cost += 2000 * (1/pow(other_cars.at(closest_car_inlane)[other_cars.at(closest_car_inlane).size()-1].v - ego_speed,2));
			lane_cost += 500 * (1/pow(other_cars.at(closest_car_inlane)[other_cars.at(closest_car_inlane).size()-1].s - ego_s,2));

		}
		//cout << "lane cost: " << lane_cost << endl;
		cost_per_lane[i] = lane_cost;
	}


	map<int, double>::iterator it = cost_per_lane.begin();
	int mincost = it->second;
	target_lane = it->first;
	it++;
	while(it != cost_per_lane.end()) {
		if(it->second < mincost) {
			mincost = it->second;
			target_lane = it->first;
		}
		it++;
	}
	//cout << "target lane: " << target_lane << endl;
	return target_lane;
}*/

States SearchForLane(int ego_lane, map<int ,vector<Vehicle::prediction>> other_cars, double ego_s, double ego_d, double ego_speed, int prev_size) {
	int target_lane = ego_lane;
	map<int,double> cost_per_lane;
	bool lane_free;
	States new_state;

	//loop over all the lanes and calculate cost to decide
	for(int i = 0; i<=2; i++) {
		//cout << "lane: " << i << endl;
		double lane_cost = 0;
		int closest_car_inlane = -2;
		lane_free = true;
		if(i!=ego_lane){
			lane_free = CheckLaneFree(i, other_cars, ego_s, ego_d, prev_size, 20, 10);
			if(!lane_free) {lane_cost += 10000;} //cost for lane with cars within range (+20, - 10) -> no lane change
			if (abs(i - ego_lane) > 1) {lane_cost += 5000; } //cost for more than one lane change
			if (ego_speed < 20.0) {lane_cost += 2000;} //cost for low speed

		}

		lane_cost += pow(max_speed-ego_speed,2)*500; //driving close to limit

		//check for cars ahead to see which lane is the best
		closest_car_inlane = CarInMinDistInLane(i, other_cars, ego_s, ego_d, prev_size);

		if(closest_car_inlane > -1) {
			//cost for lanes with slow cars that are within range of 30
			lane_cost += 1500 * (1/pow(other_cars.at(closest_car_inlane)[other_cars.at(closest_car_inlane).size()-1].v - ego_speed,2));
			lane_cost += 200 * (1/pow(other_cars.at(closest_car_inlane)[other_cars.at(closest_car_inlane).size()-1].s - ego_s,2));

		}
		//cout << "lane cost: " << lane_cost << endl;
		cost_per_lane[i] = lane_cost;
	}

	//pick lane with lowest cost
	map<int, double>::iterator it = cost_per_lane.begin();
	int mincost = it->second;
	target_lane = it->first;
	it++;
	while(it != cost_per_lane.end()) {
		if(it->second < mincost) {
			mincost = it->second;
			target_lane = it->first;
		}
		it++;
	}

	//set new state based on lowest cost
	if(ego_lane == target_lane) {
		new_state = KL;
	} else {
		if(target_lane > ego_lane) {
			new_state = LCR;
		} else {
			new_state = LCL;
		}
	}

	return new_state;
}

vector<vector<double>> GenerateTrajectory(Vehicle car, vector<double> previous_path_x, vector<double> previous_path_y, int target_lane, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, bool too_close, double closest_car_speed, double& ref_vel) {
	//create a list of widely spaced waypoints
	vector<double> ptsx;
	vector<double> ptsy;

	//reference state: x,y, yaw

	int prev_size = previous_path_x.size();
	double ref_x = car.x;
	double ref_y = car.y;
	double ref_yaw = deg2rad(car.yaw);

	if(prev_size < 2) {
		//use car's current position
		double prev_car_x = car.x - cos(car.yaw);
		double prev_car_y = car.y - sin(car.yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(car.x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(car.y);

	}
	else {
		//use previous path's end point
		ref_x = previous_path_x[prev_size-1];
		ref_y = previous_path_y[prev_size-1];

		double ref_x_prev = previous_path_x[prev_size-2];
		double ref_y_prev = previous_path_y[prev_size-2];
		ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);

	}

	//define 3 points 30 m spaced from reference points

	vector<double> next_wp0 = getXY(car.s + 30, (2+4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = getXY(car.s + 60, (2+4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(car.s + 90, (2+4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	for(int i = 0; i < ptsx.size(); i++) {
		//convert to the car's coordinate
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
		ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
	}


	//define spline
	tk::spline s;

	//set (x,y) points
	s.set_points(ptsx, ptsy);

	vector<double> next_x_vals;
	vector<double> next_y_vals;


	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

	//keep remaining points from previous path, helps with transition
	for(int i = 0; i < prev_size; i++) {
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	//break up spline points
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt(pow(target_x,2)+pow(target_y,2));

	double x_add_on = 0;

	double speed_diff = car.v - closest_car_speed;

	for(int i = 1; i <= 50 - prev_size; i++) {
		//incorporate velovity information in calculation to avoid high acceleration and jerk
		if(too_close) {
			if(speed_diff < .2) {
				ref_vel = closest_car_speed;
			} else {
				ref_vel -= .224;
			}
		} else if(ref_vel < 49) {
			ref_vel += .224;
		}

		double N = target_dist / (.02 * ref_vel / 2.24);
		double x_point = x_add_on + target_x / N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
		y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);

	}

	vector<vector<double>> trajectory;
	trajectory.push_back(next_x_vals);
	trajectory.push_back(next_y_vals);

	return trajectory;
}

#endif /* HELPER_FUNCTIONS_H_ */
