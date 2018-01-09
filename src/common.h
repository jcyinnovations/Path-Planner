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
#include <iomanip>

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

struct Coord {
  double s;
  double d;
  double x;
  double y;
  double v;

  Coord() :
    s(0.0),
    d(0.0),
    x(0.0),
    y(0.0),
    v(0.0) {}

  Coord(const Coord& src) :
    d(src.d),
    s(src.s),
    x(src.x),
    y(src.y),
    v(src.v) {}
};

namespace {
  int TOTAL_LANES   = 3;
  int LANE_WIDTH    = 4;
  double MAX_ACCELERATION = 0.5 * 10.0;	//meters per second squared
  double SPEED_LIMIT      = 50.0; 			      //miles per hour
  double SPEED_LIMIT_MPS  = mph_to_mps(SPEED_LIMIT) * 0.90;
  double ROAD_MAX   = TOTAL_LANES * LANE_WIDTH;
  double ROAD_MIN   = 0.0;
  int SENSOR_RANGE  = 10;						// construct grid from 2*N waypoints
  double CELL_SIDE  = LANE_WIDTH;		// size of grid cell (m)
  double INTERVAL   = 0.02;					// update interval
  int HORIZON       = 40;						// number of planning intervals
  double PLAN_AHEAD = 50;						// plan-ahead distance in meters (roughly 10 car lengths and distance for comfortable acceleration to speed limit)
  double max_s      = 6945.554;	    // The max s value before wrapping around the track back to 0
  queue<Coord> EMPTY_Q;
  double CLEARANCE  = 10.0;          //Used to specify the acceptable gap to trailing vehicle for lane change
}

struct Limit {
  double v;
  double gap;
  double clearance;

  Limit() :
    v(SPEED_LIMIT_MPS),
    gap(PLAN_AHEAD),
    clearance(PLAN_AHEAD) {};
};


/**
 * Print a PathNode
 */
inline ostream& operator<< (ostream & out, const Limit& l) {
    out << "\n\t Limit speed: " << l.v << "\t clearance: " << l.clearance << "\t gap:" << l.gap;
    return out;
}


// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}
inline double deg2rad(double x) {
  return x * pi() / 180;
}
inline double rad2deg(double x) {
  return x * 180 / pi();
}

/**
 * Uses waypoint splines to convert from Frenet to Cartesian coordinates
 * ,
 */
inline vector<double> getXY2(double s, double d, const tk::spline &spline_x,
                             const tk::spline &spline_y,
                             const tk::spline &spline_dir) {
  double seg_x = spline_x(s);
  double seg_y = spline_y(s);

  double perp_heading = spline_dir(s);

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x,y};
}

inline double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                           const vector<double> &maps_y) {

  double closestLen = 100000;  //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;

}

inline int NextWaypoint(double x, double y, double theta,
                        const vector<double> &maps_x,
                        const vector<double> &maps_y) {

  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = min(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
inline vector<double> getFrenet(double x, double y, double theta,
                                const vector<double> &maps_x,
                                const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
inline vector<double> getXY(double s, double d, const vector<double> &maps_s,
                            const vector<double> &maps_x,
                            const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                         (maps_x[wp2] - maps_x[prev_wp]));

  //heading = min(2*pi() - heading, heading);

  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x,y};

}

/**
 * Find the lane center
 */
inline double lane_center(int lane) {
  return LANE_WIDTH / 2 + (lane - 1) * LANE_WIDTH;
}

/**
 * Lane 1: is far left, Lane 2 is center, Lane 3 is far right
 */
inline int current_lane(double d) {
  int lane = 0;
  if (d < 0 || d > TOTAL_LANES*LANE_WIDTH)
    return lane;  //Not on this side of the road

  double min_distance = 99999;
  for (int l = 1; l <= TOTAL_LANES; l++) {
    double distance = fabs(d - lane_center(l));
    if (distance > LANE_WIDTH/2)
      continue; //Doesn't match this lane

    if (distance < min_distance) {
      min_distance = distance;
      lane = l;
    }
  }
  return lane;
}

/**
 * Check if the value is within tolerance (%) of the target
 */
inline bool in_range(double value, double target, double tolerance) {
  double lower = target * (1 - tolerance);
  double upper = target;  // * (1+tolerance);
  return value >= lower && value <= upper;
}

/**
 *KE: Keep Lane (cruise in current lane at target speed)
 *CL: Change Left
 *KB: Keep Back (Slow down or reverse as needed)
 *CR: Change Right
 *PR: Prepare for right turn
 *PL: Prepare for left turn
 *START: Initial state
 */
enum class FSM {
  KE,
  CL,
  KB,
  CR,
  PR,
  PL,
  START
};

//typedef typename vector<vector<FSM>> PathGrid;
//typedef typename vector<vector<int>> MapGrid;
//typedef typename vector<vector<double>> CostGrid;

inline string state_label(FSM state) {
  switch (state) {
    case (FSM::KE):
      return "KE";
      break;

    case (FSM::CL):
      return "CL";
      break;

    case (FSM::CR):
      return "CR";
      break;

    case (FSM::PL):
      return "PL";
      break;

    case (FSM::PR):
      return "PR";
      break;

    case (FSM::KB):
      return "KB";
      break;
    default:
      return "START";
  }
}

/**
 * Look for an entry in a vector
 */
template<class T>
inline bool vector_contains(T v, vector<T> target) {
  if (std::find(v.begin(), v.end(), target) != v.end())
    return true;
  else
    return false;
}

/**
 * Extract a subset of a vector
 */
template<class U>
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
  vector<U> _subset(source.begin() + start, source.begin() + start + count);
  return _subset;
}

struct SharedData {
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  tk::spline s_x;
  tk::spline s_y;
  tk::spline s_dir;
};

struct Trajectory {
  Trajectory()
      : t(0.0),
        x(0.0),
        y(0.0),
        end_v(0.0),
        end_d(0.0),
        end_s(0.0),
        final_v(0.0),
        final_d(0.0),
        final_s(0.0),
        target_lane(2),
        target_v(0.0),
        target_state(FSM::KE),
        cost(99999999),
        target_acc(0.0),
        gap(PLAN_AHEAD),
        clearance(CLEARANCE),
        points_rem(0.0),
        in_progress(false) {
  }

  FSM target_state;		//Requested state
  int target_lane;		//Based on state
  double target_v;		//Target speed
  double t;				    //Planner time
  bool in_progress;   //Signal lane-change in progress to Behavior Planner

  //Current Trajectory parameters
  VectorXd a    = VectorXd(6);
  VectorXd a_s  = VectorXd(6);
  VectorXd b    = VectorXd(6);

  vector<double> x;   //x points of trajectory
  vector<double> y;   //y points of trajectory

  double end_v;				//cache end speed
  double end_d;				//cache end lateral position
  double end_s;       //cache end forward position

  double final_v;     //Trajectory end speed
  double final_d;     //Trajectory end lateral position
  double final_s;     //Trajectory end forward position

  double cost;        //Trajectory cost
  double target_acc;	//Trajectory acceleration

  double gap;         //Gap ahead of the ego_car
  double clearance;   //Space between ego car and closest trailing vehicle
  int    points_rem;  //Number of intervals remaining to complete the trajectory
};

struct VehiclePose {
  VehiclePose()
      : x(0.0),
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
        grid_y(0) { }

  VehiclePose(const VehiclePose& o)
      : x(o.x),
        y(o.y),
        s(o.s),
        d(o.d),
        yaw(o.yaw),
        v(o.v),
        lane(o.lane),
        vx(o.vx),
        vy(o.vy),
        waypoint(o.waypoint),
        distance(o.distance),
        leading(o.leading),
        id(o.id),
        grid_x(o.grid_x),
        grid_y(o.grid_y) { }

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
  int waypoint;  //closest waypoint
  double distance;  //distance to ego car
  bool leading;  //In front of ego car
};

/**
 * Sort traffic
 */
inline bool compare_traffic(VehiclePose v1, VehiclePose v2) {
  return v1.leading && (v1.distance > v2.distance);
}

/**
 * Update traffic position to future state using model-based prediction
 * Future time: INTERVAL * HORIZON (set to 1 second)
 */
inline void predict_traffic(const VehiclePose& vehicle,
                            double future,
                            vector<vector<double>>& sensor_fusion) {
  for (int i=0; i < sensor_fusion.size(); i++) {
    double v = sqrt(sensor_fusion[i][3]*sensor_fusion[i][3] + sensor_fusion[i][4]*sensor_fusion[i][4]);
    //v = v - vehicle.v;  //use relative speed to calculate change in location
    sensor_fusion[i][5] = sensor_fusion[i][5] + future*v;  //s
  }
}

/**
 * Find the speed of the slowest vehicle in each lane, ahead of ego_car
 */
inline vector<Limit> lane_limits(const VehiclePose& vehicle,
                                 vector<vector<double>> sensor_fusion) {

  /**
   * Where will traffic be in the future
   */
  double future = INTERVAL * HORIZON;
  predict_traffic(vehicle, future, sensor_fusion);
  double ego_s = vehicle.s + vehicle.v*future;

  Limit l;
  vector<Limit> limits(3, l);

  for (auto sf : sensor_fusion) {
    double v = sqrt(sf[3]*sf[3] + sf[4]*sf[4]);
    int lane = current_lane(sf[6]);
    //Check that its on the same side of the road
    if (lane > 0 && lane <= TOTAL_LANES) {
      int idx = lane - 1;
      double gap = sf[5] - ego_s;
      //Get the closest vehicles in front
      if (gap > 0) {
        // Leading vehicle
        if (gap < limits[idx].gap) {
          limits[idx].gap = gap;
          //Only limit speed if the gap is smaller than plan-ahead distance
          if (gap < PLAN_AHEAD)
            limits[idx].v   = v;
        }
      } else {
        //Trailing vehicle
        if (fabs(gap) < limits[idx].clearance) {
          limits[idx].clearance = fabs(gap);
        }
      }
    }
  }
  return limits;
}

/**
 * Sort traffic by lane and relative speed
 */
inline vector<vector<VehiclePose>> sort_traffic(
    VehiclePose vehicle, vector<vector<double>> sensor_fusion) {

  vector<vector<VehiclePose>> traffic(TOTAL_LANES, vector<VehiclePose>());
  for (auto sf : sensor_fusion) {
    VehiclePose p;
    p.id = sf[0];
    p.x = sf[1];
    p.y = sf[2];
    p.vx = sf[3];
    p.vy = sf[4];
    p.yaw = sf[7];
    p.v = sqrt(sf[3] * sf[3] + sf[4] * sf[4]);
    p.s = sf[5];
    p.d = sf[6];
    p.lane = current_lane(sf[6]);
    double s_distance = sf[5] - vehicle.s;
    p.distance = fabs(s_distance);
    p.grid_x = p.lane;
    p.grid_y = p.distance;
    //TODO: fix bug in this formula when vehicle is near start
    p.leading = s_distance > 0;
    traffic[p.lane - 1].push_back(p);
  }
  for (int i = 0; i < TOTAL_LANES; i++) {
    std::sort(traffic[i].begin(), traffic[i].end(), compare_traffic);
  }
  return traffic;
}

#endif /* COMMON_H_ */
