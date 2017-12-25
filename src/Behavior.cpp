/*
 * Behavior.cpp
 *
 *  Created on: Oct 17, 2017
 *      Author: jyarde
 */

#include "Behavior.h"

Behavior::Behavior()
    : trajectory_planner(trajectory_planner),
      weight_speed(0.2),
      weight_lane_keep(0.2),
      weight_acceleration(0.2),
      weight_lane_target(0.2),
      weight_on_road(0.2),
      v_buffer(0.0),
      v_target(0.0),
      cost_stop(0.75),
      a_max(MAX_ACCELERATION) {
  v_buffer = mph_to_mps(5);
  v_limit  = SPEED_LIMIT_MPS;
  v_target = v_limit - v_buffer;
}

vector<FSM> available_sucessor_states(FSM current_state) {
  FSM next_state;
  switch (current_state) {
    case (FSM::KE):
      return {FSM::KE, FSM::CL, FSM::CR, FSM::PL, FSM::PR};
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
 * State transition function. Only consider states which ca n be reached from current FSM state.
 * Takes current state, vehicle state, traffic and current trajectory as input to compute
 * next best state
 */
void Behavior::transition_function(SharedData shared, vector<int> predictions,
                                   VehiclePose ego_car,
                                   vector<Limit> limits,
                                   double end_path_s, double end_path_d,
                                   vector<double> previous_path_x,
                                   vector<double> previous_path_y,
                                   Trajectory &trajectory) {

  vector<FSM> possible_successor_states = available_sucessor_states(
      trajectory.target_state);
  bool change_state = false;
  vector<Trajectory> potentials;
  int chosen = 0;

  vector<double> costs;                 //Track cost by successor state
  Trajectory state_trajectory;          //Planned trajectory for given state
  double min_cost = trajectory.cost;    //Minimum cost found
  int idx = 0;
  //Find the minimum cost state.
  for (auto const& state : possible_successor_states) {
    //generate resulting trajectory for the target state
    state_trajectory = trajectory;
    /**
    state_trajectory.t = trajectory.t;
    state_trajectory.s = trajectory.s;
    state_trajectory.d = trajectory.d;
    state_trajectory.x = trajectory.x;
    state_trajectory.y = trajectory.y;
    state_trajectory.end_v        = trajectory.end_v;
    state_trajectory.target_lane  = trajectory.target_lane;
    state_trajectory.target_v     = trajectory.target_v;
    state_trajectory.target_state = trajectory.target_state;
    state_trajectory.end_d        = trajectory.end_d;
    state_trajectory.cost         = trajectory.cost;
    state_trajectory.target_acc   = trajectory.target_acc;
    **/
    trajectory_planner.plan_trajectory(shared, state, ego_car, limits,
                                       end_path_s, end_path_d, previous_path_x,
                                       previous_path_y, state_trajectory);

    //calculate the "cost" of that trajectory.
    double cost_for_state = cost_function(state_trajectory);
    state_trajectory.cost = cost_for_state;
    costs.push_back(cost_for_state);          //Save state costs
    potentials.push_back(state_trajectory);   //Save state trajectory
    std::cout << state_label(state) << ": " << cost_for_state << ", ";

    if (cost_for_state < min_cost) {
      min_cost = cost_for_state;
      change_state = true;
      chosen = idx;
    }
    idx++;
  }
  if (change_state) {
    trajectory = potentials[chosen];
    trajectory.plan = potentials[chosen].plan;
    trajectory.s = potentials[chosen].s;
    trajectory.d = potentials[chosen].d;
    trajectory.x = potentials[chosen].x;
    trajectory.y = potentials[chosen].y;
  } else {
    /**
     * Update current trajectory
     * **/
    trajectory_planner.plan_trajectory(shared, trajectory.target_state,
                                       ego_car, limits, end_path_s,
                                       end_path_d, previous_path_x,
                                       previous_path_y, trajectory);
  }
  std::cout << " =: " << state_label(trajectory.target_state) << ": " << trajectory.cost << std::endl;
  //<< std::flush;
}

/**
 * Total cost function
 */
double Behavior::cost_function(Trajectory trajectory) {
  double cost = 0;
  cost = cost_speed(trajectory) + cost_lane_keep(trajectory)
      + cost_acceleration(trajectory) + cost_on_road(trajectory)
      + cost_lane_target(trajectory);
  return cost;
}

/**
 * Cost of maintaining the target speed
 */
double Behavior::cost_speed(Trajectory trajectory) {
  double cost = 0.0;
  double v = trajectory.target_v;
  if (v < v_target) {
    //Accelerate when under speed limit
    cost = cost_stop * (v_target - v) / v_target;
  } else if (v > v_limit) {
    cost = 1.0;
  } else {
    // Slow down when approaching speed limit
    cost = (v - v_limit) / v_buffer;
  }
  return cost * weight_speed;
}

/**
 * Cost of lane-keeping
 */
double Behavior::cost_lane_keep(Trajectory trajectory) {
  double cost = 0.0;
  double d = trajectory.end_d;
  double center = lane_center(trajectory.target_lane);
  cost = 1 / (1 + exp(-pow(d - center, 2.0)));
  return cost * this->weight_lane_keep;
}

/**
 * Acceleration costs
 */
double Behavior::cost_acceleration(Trajectory trajectory) {
  double cost = 0.0;
  double acc = trajectory.target_acc;
  //Heavily penalize slowing down
  if (acc < 0)
    acc = fabs(acc) * 2;
  return cost * this->weight_acceleration;
}

/**
 * Cost of staying on the road
 */
double Behavior::cost_on_road(Trajectory trajectory) {
  double cost = 0.0;
  double d = trajectory.end_d;

  if (d > ROAD_MAX or d < ROAD_MIN) {
    cost = 1.0;
  }
  return cost * this->weight_on_road;
}

/**
 * Cost of the target lane
 */
double Behavior::cost_lane_target(Trajectory trajectory) {
  double cost = 0.0;
  return cost * this->weight_lane_target;
}

