/*
 * Behavior.cpp
 *
 *  Created on: Oct 17, 2017
 *      Author: jyarde
 */

#include "Behavior.h"

Behavior::Behavior() :
	trajectory_planner(trajectory_planner),
	weight_speed(0.2),
	weight_lane_keep(0.2),
	weight_acceleration(0.2),
	weight_lane_target(0.2),
	weight_on_road(0.2),
	v_limit(mph_to_mps(SPEED_LIMIT)),
	v_buffer(mph_to_mps(5)),
	v_target(v_limit - v_buffer),
	cost_stop(0.75),
	a_max(MAX_ACCELERATION)
{}

vector<FSM> available_sucessor_states(FSM current_state) {
	FSM next_state;
	switch(current_state) {
	case(FSM::KE):
		return {FSM::KE, FSM::CL, FSM::CR, FSM::PL, FSM::PR};
	break;

	case(FSM::CL):
		return {FSM::CL, FSM::KE};
	break;

	case(FSM::CR):
		return {FSM::CR, FSM::KE};
	break;

	case(FSM::PL):
		return {FSM::CL, FSM::KE, FSM::PL};
	break;

	case(FSM::PR):
		return {FSM::CR, FSM::KE, FSM::PR};
	break;

	default:
		return {FSM::KE};
	}
}

FSM Behavior::transition_function(vector<int>predictions, FSM current_state, VehiclePose pose, vector<vector<VehiclePose>> sorted_traffic) {
    //only consider states which ca n be reached from current FSM state.
    vector<FSM> possible_successor_states = available_sucessor_states(current_state);
    FSM next_state = FSM::KE;

    //keep track of the total cost of each state.
    vector<double> costs;

    for (auto const& state : possible_successor_states) {
        //generate resulting trajectory for the target state
        Trajectory state_trajectory ;//.= trajectory_planner.plan_trajectory(state, pose, sorted_traffic);

        //calculate the "cost" of that trajectory.
        double cost_for_state = cost_function(pose);
        costs.push_back(cost_for_state);
    }

    //Find the minimum cost state.
    double min_cost = 9999999;
    for (int i=0; i < possible_successor_states.size(); i++) {
        FSM state = possible_successor_states[i];
        double cost  = costs[i];
        if (cost < min_cost) {
            min_cost = cost;
            next_state = state;
        }
    }
    return next_state;
}

double Behavior::cost_function(VehiclePose pose) {
	double cost = 0;
	cost = cost_speed(pose) +
			cost_lane_keep(pose) +
			cost_acceleration(pose) +
			cost_on_road(pose) +
			cost_lane_target(pose);
	return cost;
}

double Behavior::cost_speed(VehiclePose pose) {
	double cost = 0.0;
	if (pose.v < this->v_target) {
		//Accelerate when under speed limit
		cost = this->cost_stop * (this->v_target - pose.v) / this->v_target;
	} else if (pose.v > this->v_limit) {
		cost = 1.0;
	} else {
		// Slow down when approaching speed limit
		cost = (pose.v - this->v_limit)/this->v_buffer;
	}
	return cost * this->weight_speed;
}

double Behavior::cost_lane_keep(VehiclePose pose) {
	double cost = 0.0;
	double center = lane_center(pose.lane);
	cost = 1/ (1 + exp(-pow(pose.d - center, 2.0)));
	return cost * this->weight_lane_keep;
}

double Behavior::cost_acceleration(VehiclePose pose) {
	double cost = 0.0;
	return cost * this->weight_acceleration;
}

double Behavior::cost_on_road(VehiclePose pose) {
	double cost = 0.0;
	if (pose.d > ROAD_MAX or pose.d < ROAD_MIN) {
		cost = 1.0;
	}
	return cost * this->weight_on_road;
}

double Behavior::cost_lane_target(VehiclePose pose) {
	double cost = 0.0;
	return cost * this->weight_lane_target;
}

