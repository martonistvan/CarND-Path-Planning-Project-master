#ifndef COST_FUNCTIONS_H_
#define COST_FUNCTIONS_H_

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"


struct TrajectoryData{
	int proposed_lane;
	double avg_speed;
	double max_acceleration;
	double rms_acceleration;
	double closest_approach;
	double end_distance_to_goal;
	int end_lanes_from_goal;
	bool collides;
};

// priority levels for costs
double COLLISION  = pow(10,6);
double DANGER     = pow(10,5);
double REACH_GOAL = pow(10,5);
double COMFORT    = pow(10,4);
double EFFICIENCY = pow(10,2);

double DESIRED_BUFFER = 1.5; // timesteps
int PLANNING_HORIZON = 2;

double calculate_cost(Vehicle vehicle, vector<Vehicle::snapshot> trajectory, map<int,vector< vector<int> > > predictions) {

	//TrajectoryData trajectory_data = get_helper_data(vehicle, trajectory, predictions);
	return 0;
}

TrajectoryData get_helper_data(Vehicle vehicle, vector<Vehicle::snapshot> trajectory, map<int,vector< vector<int> > > predictions) {
	return {0,0,0,0,0,0,0,false};
}

#endif /* COST_FUNCTIONS_H_ */
