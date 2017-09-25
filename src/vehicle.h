#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>


using namespace std;

class Vehicle {
public:

	struct collider{

		bool collision ; // is there a collision?
		int  time; // time collision happens

	};

	struct snapshot{
		int lane;
		int s;
		int v;
		int a;
		string state;
	};

	struct prediction {
		int lane;
		double s;
		double v;
		double a;
		double d;
	};

	int L = 1;

	int preferred_buffer = 6; // impacts "keep lane" behavior.

	int lane;

	double s;

	double v;

	double a;

	double d;

	double x;

	double y;

	double yaw;

	double target_speed;

	int lanes_available;

	int max_acceleration;

	int goal_lane;

	double goal_s;

	string state;


	/**
	 * Constructor
	 */
	Vehicle(int lane, double s, double v, double a, double d);

	/**
	 * Destructor
	 */
	virtual ~Vehicle();

	void update_state(map<int, vector <vector<int> > > predictions);

	void configure(vector<int> road_data);

	string display();

	void increment(int dt);

	prediction state_at(int t);

	void update_vehicle(int lane, double s, double v, double a, double d, double x, double y, double yaw);

/*	bool collides_with(Vehicle other, int at_time);

	collider will_collide_with(Vehicle other, int timesteps);

	void realize_state(map<int, vector < vector<int> > > predictions);

	void realize_constant_speed();

	int _max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s);

	void realize_keep_lane(map<int, vector< vector<int> > > predictions);

	void realize_lane_change(map<int,vector< vector<int> > > predictions, string direction);

	void realize_prep_lane_change(map<int,vector< vector<int> > > predictions, string direction);*/

	vector<prediction> generate_predictions(int horizon);

	/*string get_next_state(map<int,vector< vector<int> > > predictions);

	vector<snapshot> trajectory_for_state(string state, map<int,vector< vector<int> > > predictions, int horizon);

	snapshot self_snapshot();

	void restore_state_from_snapshot(snapshot snapshot);
*/
};

#endif
