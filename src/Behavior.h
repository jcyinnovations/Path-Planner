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
 * Uses cost functions to prioritize the possible actions.
 *
 * If a lane-change state is still in progress, avoids making an additional state
 * change until the current one is completed.
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

  ~Behavior();

  void transition_function(SharedData shared, vector<int> predictions,
                          VehiclePose ego_car, vector<Limit> limits,
                          double end_path_s, double end_path_d,
                          vector<double> previous_path_x,
                          vector<double> previous_path_y,
                          Trajectory &trajectory);

  double cost_function(const Trajectory& trajectory);

  double cost_speed(const Trajectory& trajectory);

  double cost_lane_keep(const Trajectory& trajectory);

  double cost_on_road(const Trajectory& trajectory);

  double cost_acceleration(const Trajectory& trajectory);

  double cost_lane_target(const Trajectory& trajectory);
};

#endif /* BEHAVIOR_H_ */
