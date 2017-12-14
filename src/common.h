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
#include <algorithm>
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * KE - Keep lane
 * CR - Change Right
 * CL - Change Left
 * PR - Prepare to CR
 * PL - Prepare to CL
 */

/**
 * Convert mph to meters per second
 */
inline double mph_to_mps(double speed) {
	return 0.44704 * speed;
}

namespace {
	int TOTAL_LANES = 3;
	int LANE_WIDTH  = 4;
	double MAX_ACCELERATION = 0.5*10.0;			//meters per second squared
	double SPEED_LIMIT 		= 50.0; 			//miles per hour
	double SPEED_LIMIT_MPS 	= mph_to_mps(SPEED_LIMIT);
	double ROAD_MAX	 = TOTAL_LANES * LANE_WIDTH;
	double ROAD_MIN	 = 0.0;
	int SENSOR_RANGE = 10;						// construct grid from 2*N waypoints
	double CELL_SIDE = LANE_WIDTH;				// size of grid cell (m)
	double INTERVAL  = 0.02;					// update interval
	int HORIZON	 	 = 50;						// number of planning intervals
	double PLAN_AHEAD= 30;						// plan-ahead distance
	double max_s	 = 6945.554;				// The max s value before wrapping around the track back to 0
}
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

/**
 * Uses waypoint splines to convert from Frenet to Cartesian coordinates
 * ,
 */
inline vector<double> getXY2(
		double s,
		double d,
		const tk::spline &spline_x,
		const tk::spline &spline_y,
		const tk::spline &spline_dir)
{
	double seg_x = spline_x(s);
	double seg_y = spline_y(s);

	double perp_heading = spline_dir(s);

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}


inline double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

inline int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

inline int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
	angle = min(2*pi() - angle, angle);

	if(angle > pi()/4)
	{
		closestWaypoint++;
		if (closestWaypoint == maps_x.size())
		{
			closestWaypoint = 0;
		}
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
inline vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
inline vector<double> getXY(
		double s,
		double d,
		const vector<double> &maps_s,
		const vector<double> &maps_x,
		const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));

	//heading = min(2*pi() - heading, heading);

	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

/**
 * Find the lane center
 */
inline double lane_center(int lane) {
	return LANE_WIDTH/2 + (lane-1) * LANE_WIDTH;
}

/**
 * Lane 1: is far left, Lane 2 is center, Lane 3 is far right
 */
inline int current_lane(double d) {
	int lane = 0;
	double distance_from_center = 99999;
	for (int l=1; l <= TOTAL_LANES; l++) {
		double distance = fabs(d-lane_center(l));
		if (distance < distance_from_center) {
			distance_from_center = distance;
			lane = l;
		}
	}
	return lane;
}

/**
 * Check if the value is within tolerance (%) of the target
 */
inline bool in_range(double value, double target, double tolerance) {
	double lower = target * (1-tolerance);
	double upper = target;// * (1+tolerance);
	return value >= lower && value <= upper;
}

enum class FSM { KE, CL, KB, CR, PR, PL, START };

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
	Trajectory() :
		t(0.0),
		s(0.0),
		d(0.0),
		x(0.0),
		y(0.0),
		sf_dot(0.0),
		target_lane(2),
		target_v(0.0),
		target_state(FSM::START),
		state_possible(true) {}

	vector<double> s;		//Trajectory s
	vector<double> d;		//Trajectory d
	vector<double> x;		//Trajectory x
	vector<double> y;		//Trajectory y
	FSM target_state;		//Requested state
	int target_lane;		//Based on state
	double target_v;		//Target speed
	double t;				//Planner time

	bool state_possible;	//can the state change be made
	//Current Trajectory parameters
	VectorXd a = VectorXd(6);
	VectorXd a_s = VectorXd(6);
	VectorXd b = VectorXd(6);
	double sf_dot;			//Trajectory end speed
};

struct VehiclePose {
	VehiclePose() :
		x(0.0),
		y(0.0),
		s(0.0),
		d(0.0),
		yaw(0.0),
		v(0.0),
		lane(2),
		vx(0.0),
		vy(0.0),
		waypoint(0),
		distance(0.0),
		leading(false),
		id(0),
		grid_x(0),
		grid_y(0) {}
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

  	//Useful for traffic only
  	double vx;
  	double vy;
  	int waypoint;	//closest waypoint
  	double distance;//distance to ego car
  	bool leading;	//In front of ego car
};

/**
 * Sort traffic
 */
inline bool compare_traffic(VehiclePose v1, VehiclePose v2) {
	return (v1.distance < v2.distance) && v1.leading;
}

/**
 * Sort traffic by lane and distance to ego car
 */
inline vector<vector<VehiclePose>> sort_traffic(
		VehiclePose vehicle,
		vector<vector<double>> sensor_fusion) {

	vector<vector<VehiclePose>> traffic(TOTAL_LANES,vector<VehiclePose>());
  	for (auto sf : sensor_fusion) {
  		VehiclePose p;
  		p.id = sf[0];
  		p.x = sf[1];
  		p.y = sf[2];
  		p.vx = sf[3];
  		p.vy = sf[4];
  		p.yaw= sf[7];
  		p.v  = sf[4] / sin(p.yaw);
  		p.s  = sf[5];
  		p.d  = sf[6];
  		p.lane = current_lane(sf[6]);
  		p.distance = sf[5] - vehicle.s; //distance(vehicle.x, vehicle.y, sf[1], sf[2]);
  		p.grid_x = p.lane;
  		p.grid_y = p.distance;

  		//TODO: fix bug in this formula when vehicle is near start
  		double s_distance = sf[5] - vehicle.s;
  		p.leading = s_distance > 0;

  		traffic[p.lane-1].push_back(p);
  	}
  	for (int i=0; i < TOTAL_LANES; i++) {
  		std::sort(traffic[i].begin(), traffic[i].end(), compare_traffic);
  	}
	return traffic;
}

#endif /* COMMON_H_ */
