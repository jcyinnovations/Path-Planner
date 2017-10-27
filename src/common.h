/*
 * common.h
 *
 *  Created on: Oct 17, 2017
 *      Author: jyarde
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <vector>
#include <sstream>
#include <queue>
#include <ostream>
#include <stdio.h>
#include <cmath>

using namespace std;


/**
 * KE - Keep lane
 * CR - Change Right
 * CL - Change Left
 * PR - Prepare to CR
 * PL - Prepare to CL
 */

namespace {
	int TOTAL_LANES = 3;
	int LANE_WIDTH  = 4;
	double COMFORTABLE_ACCELERATION = 10.0;		//meters per second squared
	double SPEED_LIMIT = 50.0; 					//miles per hour
	double ROAD_MAX	 = TOTAL_LANES * LANE_WIDTH;
	double ROAD_MIN	 = 0.0;
	int SENSOR_RANGE = 20;						// construct grid from N waypoints
	double CELL_SIDE = LANE_WIDTH;				// size of grid cell (m)
}
/**
 * Find the lane center
 */
inline double get_lane_center(int lane) {
	return LANE_WIDTH/2 + (lane-1) * LANE_WIDTH;
}

/**
 * Convert mph to meters per second
 */
inline double mph_to_mps(double speed) {
	return 0.44704 * speed;
}

enum class FSM { KE, CL, KB, CR, PR, PL };

//typedef typename vector<vector<FSM>> PathGrid;
//typedef typename vector<vector<int>> MapGrid;
//typedef typename vector<vector<double>> CostGrid;

/**
 * Look for an entry in a vector
 */
template <class T>
inline bool vector_contains(T v, vector<T> target) {
	if( std::find(v.begin(), v.end(), target) != v.end() )
		return true;
	else
		return false;
}

/**
 * Extract a subset of a vector
 */
template <class U>
inline vector<U> subset(vector<U> source, int start, int length) {
	int count = length;
	int remaining = source.size() - start;
	if (start >= source.size()) {
		return vector<U>();
	}
	if (remaining < length) {
		count = remaining;
	}
	//vector<U>::const_iterator first = source.begin() + start;
	//vector<U>::const_iterator last = source.begin() + start + count;
	vector<U> _subset(source.begin()+start, source.begin()+start+count);
	return _subset;
}

struct Trajectory {
	vector<float> x;
	vector<float> y;
	FSM target_state;
};

struct VehiclePose {
  	double x;
  	double y;
  	double s;
  	double d;
  	double yaw;
  	double v;

  	int lane;
  	//Search grid position
  	int id;
  	int grid_x;
  	int grid_y;
};


#endif /* COMMON_H_ */
