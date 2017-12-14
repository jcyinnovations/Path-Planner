/*
 * Trajectory.h
 *
 *  Created on: Oct 16, 2017
 *      Author: jyarde
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>
#include "common.h"
#include <stdio.h>
#include <cmath>
#include <ostream>
#include <iostream>

using namespace std;

/**
 * Hybrid A* Trajectory planner.
 * Updates the route based on requested actions of the Behavior module
 * Uses a 5th order polynomial to reduce jerk.
 *
 */

class PathNode {
public:
	vector<FSM> allowed_moves = {FSM::KE, FSM::CL, FSM::KB, FSM::CR};
	vector<string> move_annotation = {"^", "<", "v", ">"};
	vector<double> move_costs = {1.0, 1.0, 1.0, 1.0};
	vector<vector<int>> delta = {{-1, 0}, {0,-1}, {1, 0}, {0, 1}};

	vector<vector<double>> heuristic;
	FSM 		move;
	VehiclePose pos;
	vector<VehiclePose> path;
	vector<FSM> turns;
	double 		cost;

	PathNode(VehiclePose start, vector<vector<double>> heuristic);

	PathNode(FSM& move, PathNode start);

	~PathNode();

	void executeMove(FSM move);

	double calc_g();

	VehiclePose calc_move(FSM move, VehiclePose start);

	VehiclePose calc_move(FSM move);

	void expand(vector<vector<int>>grid, queue<PathNode> &que);

	vector<vector<string>> map_path(vector<vector<int>> grid);

	bool valid_move(VehiclePose pos, vector<vector<int>> grid);
};

ostream& operator<<(ostream& out, const PathNode& node);


class TrajectoryPlanner {

public:

	/**
	 * Map section near the car (closest N waypoints)
	 */
	vector<double> map_x;
	vector<double> map_y;
	vector<double> map_s;
	vector<double> map_dx;
	vector<double> map_dy;

	/**
	 * Index of the nearest waypoint
	 */
	int closestWaypoint;

	/**
	 * Grid within sensor range
	 * 0: empty, 1: occupied, 2: expanded
	 */
	vector<vector<int>> map_grid;

	/**
	 * Chosen lane based on instructions from the behaviour planner
	 */
	int target_lane;

	/**
	 * Trajectory
	 */
	Trajectory trajectory;

	int track_length;

	/**
	 * Frenet position of starting waypoint
	 */
	double grid_start_s;

	/**
	 * Current location (grid cell, lane, coordinates)
	 */
	VehiclePose location;

	/**
	 * Splines of the route
	 */
	tk::spline s_x;
	tk::spline s_y;
	tk::spline s_dir;

	TrajectoryPlanner(
			vector<double> x,
			vector<double> y,
			vector<double> s,
			vector<double> dx,
			vector<double> dy);

	TrajectoryPlanner();

	~TrajectoryPlanner();

	/**
	 * Take current environment (sensor fusion) and vehicle state
	 * and convert to a grid form usable by Hybrid A*
	 */
	void state_update(
			VehiclePose vehicle,
			vector<vector<double>> sensor_fusion,
			int closestWaypoint,
			double max_s);

	/**
	 * Generate trajectories based on surrounding traffic and desired next state
	 * state - requested state change
	 * pose  - vehicle parameters
	 * sorted_traffic - nearby traffic sorted by distance and lane
	 * remainder - trajectory left of previous iteration
	 */
	Trajectory plan_trajectory(
			FSM state,
			VehiclePose pose,
			vector<vector<VehiclePose>> sorted_traffic,
			int remainder,
			double end_s,
			double end_d);

	/**
	 * Only regenerate if something changes
	 */
	Trajectory plan_trajectory2(
			FSM state,
			VehiclePose pose,
			vector<vector<VehiclePose>> sorted_traffic,
			int remainder,
			double end_s,
			double end_d,
			double end_x,
			double end_y);

	/**
	 *Ensure proper spacing of points
	 */
	void smooth_trajectory(
			vector<double> px,
			vector<double> py,
			VehiclePose ego_car,
			vector<vector<VehiclePose>> sorted_traffic,
			int rem,
			double end_x,
			double end_y);

	/**
	 * Find the best route forward. Based on Hybrid A* search
	 *
	 */
	void find_best_route();

	/**
	 * Update the nominal route based on new information from sensor fusion and current position of the vehicle.
	 * Uses Hybrid A* search to search for the best route
	 */
	void update_route();
};



class AStar {
	vector<vector<int>> grid;
	vector<vector<double>> heuristic;

public:
    AStar();

    ~AStar();

    bool arrived(VehiclePose goal, VehiclePose current);

    /**
     * Step cost heuristic
     */
    void init_heuristic(vector<vector<int>> grid, VehiclePose goal);

    void print_world();

    void print_heuristic();

    void search(VehiclePose init, VehiclePose goal);
};



/**
 * Route path on map
 */
inline vector<vector<string>> PathNode::map_path(vector<vector<int>> grid) {
	vector<vector<string>> path_on_map( grid.size(), vector<string>(grid[0].size(), " ") );
	for(int i=0; i < this->turns.size(); i++) {
		path_on_map[this->path[i].grid_x][this->path[i].grid_y] = this->move_annotation[(int)this->turns[i]];
	}
	return path_on_map;
}

/**
 * Print a vector of primitives
 */
template <class T>
ostream& operator<< (ostream& out, const vector<T>& list) {
	out << "[";
    for (int i=0; i < list.size(); i++) {
    	out << list[i] << ", ";
    }
    out << "]";
    return out;
}

inline ostream& operator<< (ostream& out, const VehiclePose& pose) {
	out << "[" << pose.grid_x << ", " << pose.grid_y << "]";
	if (pose.id != -1)
		out << pose.leading;
    return out;
}

/**
 * Print a PathNode
 */
inline ostream& operator<< (ostream & out, const PathNode& node) {
    out << "\n\tLocation: " << node.pos << "Cost: " << node.cost << " Path:" << node.path;
    return out;
}

#endif /* TRAJECTORY_H_ */
