/*
 * Behavior.h
 *
 *  Created on: Oct 17, 2017
 *      Author: jyarde
 */

#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>
#include "Trajectory.h"

using namespace std;

/**
 * Finite State Machine behavior planner.
 * Decides next best move based on inputs from localization and prediction
 * Uses cost functions to prioritize the possible actions
 *
 */
class Behavior {

public:
	TrajectoryPlanner trajectory_planner;
	double weight_speed;
	double weight_lane_keep;
	double weight_acceleration;
	double weight_lane_target;
	double weight_on_road;

	double v_target;
	double v_limit;
	double v_buffer;
	double cost_stop;

	double a_max;

	Behavior();

	virtual ~Behavior();

	/**
	 * Naive Bayes Estimation of future state
	 */
	FSM transition_function(
			vector<int>predictions,
			FSM current_state,
			VehiclePose pose,
			vector<vector<VehiclePose>> sorted_traffic);

	double cost_function(VehiclePose state);

	double cost_speed(VehiclePose state);

	double cost_lane_keep(VehiclePose state);

	double cost_on_road(VehiclePose state);

	double cost_acceleration(VehiclePose state);

	double cost_lane_target(VehiclePose state);
};


#endif /* BEHAVIOR_H_ */
