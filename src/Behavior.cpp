/*
 * Behavior.cpp
 *
 *  Created on: Oct 17, 2017
 *      Author: jyarde
 */

#include "Behavior.h"

/**
 * First priority is staying on the road
 */
Behavior::Behavior()
    : trajectory_planner(trajectory_planner),
      weight_speed(0.15),
      weight_lane_keep(0.15),
      weight_acceleration(0.30),
      weight_lane_target(0.05),
      weight_on_road(0.35),
      cost_stop(0.75),
      a_max(MAX_ACCELERATION) {
  v_buffer = mph_to_mps(10);
  v_limit  = SPEED_LIMIT_MPS;
  v_target = v_limit - v_buffer;
}

vector<FSM> available_sucessor_states(FSM current_state) {
  FSM next_state;
  switch (current_state) {
    case (FSM::KE):
      return {FSM::KE, FSM::CL, FSM::CR};//, FSM::PL, FSM::PR};
      break;

    case (FSM::CL):
      return {FSM::CL, FSM::KE};
      break;

    case (FSM::CR):
      return {FSM::CR, FSM::KE};
      break;

    case (FSM::PL):
      return {FSM::CL, FSM::KE, FSM::PL};
      break;

    case (FSM::PR):
      return {FSM::CR, FSM::KE, FSM::PR};
      break;

    default:
      return {FSM::KE};
  }
}

Behavior::~Behavior() {
}

/**
 * State transition function. Only consider states which can be reached from current FSM state.
 * Takes current state, vehicle state, traffic and current trajectory as input to compute
 * next best state.
 */
void Behavior::transition_function(SharedData shared, vector<int> predictions,
                                   VehiclePose ego_car,
                                   vector<Limit> limits,
                                   double end_path_s, double end_path_d,
                                   vector<double> previous_path_x,
                                   vector<double> previous_path_y,
                                   Trajectory &trajectory) {

  vector<FSM> possible_successor_states = available_sucessor_states(trajectory.target_state);
  int chosen = 0;

  double min_cost = 999999;      //Minimum cost found
  std::cout << std::endl;
  int current_target_lane = trajectory.target_lane;
  /**
   * Cache the end 'd' position
   */
  double end_d = trajectory.end_d;
  bool pause_planning = trajectory.in_progress;
  /**
   * Update the current state to see if its cost-effective
   */
  std::cout << "*State:" << state_label(trajectory.target_state) << ": ";
  trajectory_planner.plan_trajectory(shared, trajectory.target_state,
                                     ego_car, limits, end_path_s,
                                     end_path_d, previous_path_x,
                                     previous_path_y, trajectory);
  trajectory.cost = cost_function(trajectory);
  min_cost = trajectory.cost;
  std::cout << ", Cost: " << min_cost << std::endl;

  /**
   * Halt planning until the lane-change is complete
   */
  if (pause_planning)
    return;

  //Find the minimum cost state.
  for (auto const& state : possible_successor_states) {
    //Skip the current state
    if (state == trajectory.target_state)
      continue;

    //generate resulting trajectory for the target state
    Trajectory state_trajectory;
    state_trajectory.end_d = end_d;
    state_trajectory.end_v = SPEED_LIMIT_MPS;

    std::cout << "State: " << state_label(state) << ": ";
    trajectory_planner.plan_trajectory(shared, state, ego_car, limits,
                                       end_path_s, end_path_d, previous_path_x,
                                       previous_path_y, state_trajectory);

    //calculate the "cost" of that trajectory.
    double cost_for_state = cost_function(state_trajectory);
    state_trajectory.cost = cost_for_state;

    std::cout << ", Cost: " << cost_for_state << std::endl;

    if (cost_for_state < min_cost) {
      trajectory = state_trajectory;
      min_cost = cost_for_state;
    }
  }

  std::cout << " Decision: " << state_label(trajectory.target_state) << ": " << trajectory.cost << std::endl;
  //<< std::flush;
}

/**
 * Total cost function
 */
double Behavior::cost_function(const Trajectory& trajectory) {
  double cost = 0;
  cost = cost_speed(trajectory) + cost_lane_keep(trajectory)
      + cost_acceleration(trajectory) + cost_on_road(trajectory)
      + cost_lane_target(trajectory);
  return cost;
}

/**
 * Cost of maintaining the target speed
 */
double Behavior::cost_speed(const Trajectory& trajectory) {
  double cost = 0.0;
  double v = trajectory.target_v;
  if (v < v_target) {
    //Accelerate when under speed limit
    cost = cost_stop * (v_target - v) / v_target;
  } else if (v > v_limit) {
    cost = 1.0;
  } else {
    // Slow down when approaching speed limit
    cost = max(v - v_limit, 0.0) / v_buffer;
  }
  return cost * weight_speed;
}

/**
 * Cost of lane-keeping
 */
double Behavior::cost_lane_keep(const Trajectory& trajectory) {
  double cost = 0.0;
  double d = trajectory.end_d;
  double center = lane_center(trajectory.target_lane);
  cost = 1 / (1 + exp(-pow(d - center, 2.0)));
  return cost * weight_lane_keep;
}

/**
 * Acceleration costs
 */
double Behavior::cost_acceleration(const Trajectory& trajectory) {
  double cost = 0.0;
  double acc = trajectory.target_acc;
  //Heavily penalize slowing down
  if (acc >= MAX_ACCELERATION)
    cost = 1.0;
  else if (acc < 0)
    cost = 0.75;

  return cost * weight_acceleration;
}

/**
 * Cost of staying on the road
 */
double Behavior::cost_on_road(const Trajectory& trajectory) {
  double cost = 0.0;
  double d = lane_center(trajectory.target_lane);

  if (d > ROAD_MAX or d < ROAD_MIN) {
    cost = 1.0;
  }
  return cost * weight_on_road;
}

/**
 * Cost of the target lane. Preference for center lane
 */
double Behavior::cost_lane_target(const Trajectory& trajectory) {
  double cost = 0.0;
  if (trajectory.target_lane == 2)
    cost = 0.75;
  else
    cost = 1.0;
  return cost * weight_lane_target;
}

