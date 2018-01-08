/*
 * Trajectory.cpp
 *
 *  Created on: Oct 16, 2017
 *      Author: jyarde
 */

#include "Trajectory.h"

/**
 * Created by the expand() method
 */
PathNode::PathNode(VehiclePose start, vector<vector<double>> heuristic) :
	move(FSM::KE),
	heuristic(heuristic),
	pos(start),
	path(),
	cost(0),
	turns()
	{}

/**
 * start is either a x,y coordinates of starting point or Entry object
 */
PathNode::PathNode(FSM& move, PathNode start) :
	move(move),
	pos(start.calc_move(move)),
	heuristic(start.heuristic),
	path(start.path),
	turns(start.turns)
{
	this->path.push_back(this->pos);
	this->turns.push_back(move);
	this->cost = (start.cost + move_costs[(int)move] + this->calc_g());
}

PathNode::~PathNode(){

}

/**
 * Is this move allowed?
 */
bool PathNode::valid_move(VehiclePose pos, vector<vector<int>> grid) {
	int x = pos.grid_x;
	int y = pos.grid_y;
	bool x_in_range = x >= 0  && x < grid.size();
	bool y_in_range = y >= 0  && y < grid[0].size();
	return x_in_range && y_in_range && grid[x][y] == 0;
}

/**
 * Adjust cost for heuristic
 **/
double PathNode::calc_g() {
	if (this->heuristic.empty())
		return 0.0;
	return this->heuristic[this->pos.grid_x][this->pos.grid_y];
}

VehiclePose PathNode::calc_move(FSM move) {
	VehiclePose start = this->pos;
	return PathNode::calc_move(move, start);
}

VehiclePose PathNode::calc_move(FSM move, VehiclePose start) {
	int x = start.grid_x + this->delta[(int)move][0];
	int y = start.grid_y + this->delta[(int)move][1];
	VehiclePose pose;
	pose.grid_x = x;
	pose.grid_y = y;
	return pose;
}

/**
 * Generate a list of Entry objects that represent the possible new positions from current
 */
void PathNode::expand(vector<vector<int>>grid, queue<PathNode> &que) {
	cout << "Expanding: " << this->pos << endl;
	vector<PathNode> expansion;
	double min_cost = 9999999999;
	for (int m = 0; m < this->allowed_moves.size(); m++) {
		VehiclePose p = this->calc_move(this->allowed_moves[m]);
		if (this->valid_move(p, grid)) {
			PathNode new_entry(allowed_moves[m], *this);
			if (new_entry.cost == min_cost) {
				expansion.push_back(new_entry);
			}
			else if (new_entry.cost < min_cost) {
				/**
				 * clear the list and set the new min cost
				 */
				min_cost = new_entry.cost;
				cout << "\tNew Min Cost at: " << this->pos << " Cost: " << min_cost << " " << new_entry.pos;
				expansion.clear();
				expansion.push_back(new_entry);
			}
		}
	}
	cout << "\nExpansion: \n" << expansion << endl;
	/**
	 * Now add them to the queue
	 **/
	for (auto e : expansion) {
		que.push(e);
	}
}

/**
 * A* SEARCH
 */
AStar::AStar() {
}

AStar::~AStar() {
}

bool AStar::arrived(VehiclePose goal, VehiclePose current) {
	return goal.grid_x == current.grid_x && goal.grid_y == current.grid_y;
}

/**
 * Step cost heuristic
 */
void AStar::init_heuristic(vector<vector<int>> grid, VehiclePose goal) {
	int r = grid.size();
	int c = grid[0].size();
	cout << "x width: " << r << "y height: " << c << "\n";
	this->grid = grid;
	this->heuristic = vector<vector<double>>(r, vector<double>(c, 0.0));
	for(int i=0; i < r; i++) {
		for(int j=0; j < c; j++) {
			this->heuristic[i][j] = ( abs(goal.grid_x - i) + abs(goal.grid_y - j) );
		}
	}
}

void AStar::print_world() {
	printf("World Map: \n");
	for (auto r : this->grid) {
		for (auto c : r) {
			printf("%4i, ", c);
		}
		printf("\n");
	}
	printf("\n");
}

void AStar::print_heuristic() {
	printf("Heuristic function: \n");
	for (auto r : this->heuristic) {
		for (auto c : r) {
			printf("%6.2f, ", c);
		}
		printf("\n");
	}
	printf("\n");
}

void AStar::search(VehiclePose init, VehiclePose goal) {
	init_heuristic(grid, goal);
	print_world();
	print_heuristic();

	cout << "Starting from: " << init << " destination: " << goal << endl;
	//Pass the heuristic for the first Entry
	PathNode start(init, this->heuristic);
	queue<PathNode> que;
	que.push(start);
	PathNode e = start;
	while  (!que.empty()) {
		e = que.front();
		que.pop();
		if (this->arrived(goal, e.pos)) {
			cout << "Best Route cost: " << e.cost << endl;
			cout << e.path;
			vector<vector<string>> route = e.map_path(grid);
			printf("Route:\n");
			for (auto& m : route) {
				cout << "\t" << m << endl;
			}
			break;
		}
		if (grid[e.pos.grid_x][e.pos.grid_y] == 0) {
			//Only expand un-expanded or drivable locations
			e.expand(grid, que);
			//Update expanded list
			grid[e.pos.grid_x][e.pos.grid_y] = 2;
		}
	}
	print_world();
}


/**
 * TRAJECTORY PLANNER
 */

TrajectoryPlanner::TrajectoryPlanner(vector<double> x, vector<double> y, vector<double> s,
									 vector<double> dx, vector<double> dy) :
											 map_x(x),
											 map_y(y),
											 map_s(s),
											 map_dx(dx),
											 map_dy(dy),
											 closestWaypoint(0),
											 grid_start_s(0) {
	this->track_length = map_x.size();
}

TrajectoryPlanner::TrajectoryPlanner() :
										 map_x(),
										 map_y(),
										 map_s(),
										 map_dx(),
										 map_dy(),
										 closestWaypoint(0),
										 grid_start_s(0),
										 track_length(0) {
}


TrajectoryPlanner::~TrajectoryPlanner(){}

/**
 * Create a grid over the next N waypoints starting at an the vehicle.
 * interpolate a waypoint for the vehicle from the closest waypoint.
 * Place all detected vehicles on the grid fully covering all cells they touch.
 * Cells are square with side equal to lane width.
 * N set to cover sensor range from vehicle.
 * Must include closest waypoint behind the vehicle.
 */
void TrajectoryPlanner::state_update(VehiclePose vehicle, vector<vector<double>> sensor_fusion, int closestWaypoint, double max_s) {
	this->location = vehicle;
	this->closestWaypoint = closestWaypoint;
	int grid_width = ROAD_MAX / CELL_SIDE;	//width in cells
	/**
	 * Start the grid at SENSOR_RANGE waypoints behind the closest waypoint to the egocar
	 * Then count forward 2*SENSOR_RANGE waypoints and that is the bounds of the grid
	 */
	int start = track_length + (closestWaypoint - SENSOR_RANGE) % track_length;

	if (closestWaypoint > 5)
		start = closestWaypoint - 5;		//Map at least 5 cells behind the ego car
	int grid_length = (int) 2*SENSOR_RANGE*(max_s/181)/4;		//2*SENSOR_RANGE is in waypoints which are max_s/181 apart;
	map_grid = vector<vector<int>>(grid_width, vector<int>(grid_length, 0));	//init map with all 0's
	this->grid_start_s = map_s[start];

	/**
	 * Now map all traffic onto the grid
	 */
	//traffic.clear();
	//Structure: [ 0, 1, 2,  3,  4, 5, 6]
	//Structure: [id, x, y, vx, vy, s, d]
  	for (auto sf : (vector<vector<double>>)sensor_fusion) {
  		//TODO: Account for vehicles changing lanes (straddling cells)
  		//TODO: Make an assumption about vehicle dimensions to do this
  		//Vehicle is in range (in the grid of concern)
  		double distance = sf[5] - grid_start_s;
  		if (distance <= 0)
  			distance += max_s;

		int grid_x = 0;
		if (sf[6] > LANE_WIDTH) {
			if (sf[6] <= 2*LANE_WIDTH)
				grid_x = 1;
			else
				grid_x = 2;
		}

		int grid_y = (int) distance/CELL_SIDE;
		//place vehicle in the grid
		map_grid[grid_x][grid_y] = 1;
  	}

	/**
	 * Map the ego car
	 */
	double distance = vehicle.s - grid_start_s;
	if (distance <= 0)
		distance += max_s;
	int grid_x = 0;
	if (vehicle.d > LANE_WIDTH) {
		if (vehicle.d <= 2*LANE_WIDTH)
			grid_x = 1;
		else
			grid_x = 2;
	}
	int grid_y = (int) distance/CELL_SIDE;
	map_grid[grid_x][grid_y] = 4;


	cout << "Map Grid: \n";
  	for (auto row : map_grid)
  		cout << row << endl;
}

/**
 * Generate target d based on requested state. Adjust requested speed based on
 * traffic.
 */
inline void apply_requested_state (
		FSM state,
		const VehiclePose& ego_car,
		const vector<Limit>& limits,
		Trajectory& new_trajectory) {

  new_trajectory.target_v = SPEED_LIMIT_MPS; //Always unless conditions dictate otherwise
  new_trajectory.gap          = PLAN_AHEAD;
	cout << limits << endl;
	/**
	 * Update final state position. Conditions for update:
	 * 1. Time horizon is length of the trajectory (50 points or 1 second)
	 * 2. Target speed: speed limit or speed of overtaking traffic (behind) in the target lane
	 * 3. Acceleration: 0
	 * 4. Sf: the distance of travel over the time horizon (equivalent to speed for 1 second)
	 *
	 * To accommodate slow moving vehicles in the target lane, change the state to KB if
	 * current state is the same as previous
	 */
	if (state == FSM::KE) {
		//Keep Lane
	  new_trajectory.target_lane = ego_car.lane;
	}
	else if (state == FSM::CL) {
		//Change to left lane if possible
	  if (!new_trajectory.in_progress)
	    new_trajectory.target_lane = ego_car.lane - 1;
	}
	else if (state == FSM::CR) {
		//Change to right lane if possible
    if (!new_trajectory.in_progress)
      new_trajectory.target_lane = ego_car.lane + 1;
	}
	else if (state == FSM::PL) {
		/**
		 * To Keep Lane: develop a trajectory that keeps you behind cars ahead
		 * So match speed of cars ahead if they are lower than the speed limit
		 */
	  new_trajectory.target_lane = ego_car.lane;
	}
  //new_trajectory.end_d = lane_center(new_trajectory.target_lane);

	/**
	 * Adjust speed for target lane
	 */
	if (new_trajectory.target_lane > 0 &&
	    new_trajectory.target_lane <= TOTAL_LANES) {

	  int idx = new_trajectory.target_lane - 1;
	  //Target lane speed set to 95% of the car ahead to ensure we keep back
	  double target_lane_speed = limits[idx].v * 0.99;
	  new_trajectory.gap = limits[idx].gap;
    double clearance = limits[idx].clearance;
    /**
     * Accommodate close vehicles (within PLAN_AHEAD distance)
     * by matching their speed in the gap distance
     */
    if (new_trajectory.gap < PLAN_AHEAD) {
      /**
       * Match the speed of the lane in the PLAN_AHEAD or gap distance;
       * Whichever is less
       */
      if (target_lane_speed < SPEED_LIMIT_MPS) {
        //Under speed limit, set this speed as the lane limit
        new_trajectory.target_v = target_lane_speed;
      }
    } else {
      //ignore larger gaps
      new_trajectory.gap = PLAN_AHEAD;
    }
    /**
     * If the car is behind or next to ego car drastically reduce speed to
     * hinder the lane change
     */
    if (clearance <= CLEARANCE && state != FSM::KE) {
      new_trajectory.target_v = 2*target_lane_speed;
      new_trajectory.clearance= clearance;
    }

	} else {
	  //Adjust car speed if it is below speed limit
	  if (ego_car.v < SPEED_LIMIT_MPS)
	    new_trajectory.target_v = SPEED_LIMIT_MPS;
	}

}

/**
 * Generate parameters for s quintic
 * gap - target distance
 */
inline void solve_s_quintic(const VehiclePose& ego_car,
                            Trajectory& trajectory,
                            int& horizon) {
	/**
	 * Initial State 's'
	 */
	MatrixXd T(3,3);
	MatrixXd T_inverse(3,3);
	double s 		 = ego_car.s;
	double s_dot = ego_car.v;
  double sf_dot    = trajectory.target_v;//final speed
  double sf_dotdot = 0.0;                //final acceleration

	double s_dotdot  = (sf_dot*sf_dot - s_dot*s_dot)/(2*trajectory.gap);
	/**
	 * Can control max acceleration but not deceleration
	 */
	//default to time to cover gap at constant speed
	double t = trajectory.gap/s_dot;
	//Unless speed is not constant over the gap
	if (s_dotdot != 0) {
	  t = (sf_dot - s_dot)/s_dotdot;
	}
	//Invert time when slowing down (negative acceleration)
  if (t < 0) {
	  s_dotdot = -1*s_dotdot;
	  t = -1*t;
	}

	cout << " Gap: " << trajectory.gap << " Time: " << t << " Acceleration: " << s_dotdot
	     << " Target Speed: " << trajectory.target_v;

	double sf	= s + s_dot*t + s_dotdot * t*t/2;

	/**
	 * Setup the acceleration trajectory. Ignore if less than half second of trajectory
	 */
	if ( !in_range(s_dot, trajectory.target_v, 0.01) ) {
		/**
		 * solve for quintic trajectory coefficients
		 */
		T << pow(t, 3), 	pow(t, 4), 		pow(t, 5),
			 3*pow(t, 2), 	4*pow(t, 3), 	5*pow(t, 4),
			 6*t, 			12*pow(t, 2), 	20*pow(t, 3);
		T_inverse = T.inverse();

		VectorXd Sf = VectorXd(3);
		Sf << sf - (s + s_dot*t + s_dotdot * pow(t,2)/2),
				sf_dot - (s_dot + s_dotdot * t),
				sf_dotdot - s_dotdot;
		VectorXd A = T_inverse * Sf;
		trajectory.a << s, s_dot, s_dotdot/2, A[0], A[1], A[2];
		trajectory.target_acc = s_dotdot;
	} else {
		// Already at required speed
		trajectory.a << s, s_dot, 0, 0, 0, 0;
		t = 4.0;	//Set horizon to 4 seconds if already at speed limit
	}
	trajectory.a_s << trajectory.a[1], 	2*trajectory.a[2], 	3*trajectory.a[3],
					  4*trajectory.a[4], 5*trajectory.a[5], 0;
	horizon = fabs(t/INTERVAL);
}


void solve_d_quintic(const VehiclePose& ego_car,
                     Trajectory& trajectory,
                     int horizon) {
  double t_horizon = horizon*INTERVAL;
	double t 		     = 3.5;    //time to complete a lane-change
	if (t_horizon < t)         //For short horizons based on s, used s time-line
	  t = t_horizon;

	MatrixXd T(3,3);
	MatrixXd T_inverse(3,3);
	/**
	 * Initial State 'd'
	 */
	double d 		    = ego_car.d;
	double d_dot 	  = 0.0;
	double d_dotdot	= 0.05;

	/**
	 * Final State 'd'
	 */
	double df		      = lane_center(trajectory.target_lane);	//center of lane chosen below
	double df_dotdot  = 0.0; 									//final acceleration (any adjustment done in time horizon 't')
	double df_dot	    = 0.0; 									//Adjustment over so no lateral movement necessary

	double distance   = df - d;
	d_dotdot = 2 * distance / (t*t);
	/**
	 * Setup coefficients
	 */
	//if (fabs(distance) > 0.04) {
    T << pow(t, 3), 	pow(t, 4), 		pow(t, 5),
      3*pow(t, 2), 	4*pow(t, 3), 	5*pow(t, 4),
      6*t, 			12*pow(t, 2), 	20*pow(t, 3);
    T_inverse = T.inverse();

    VectorXd Df = VectorXd(3);
    Df << df - (d + d_dot*t + d_dotdot * pow(t,2)/2),
        df_dot - (d_dot + d_dotdot * t),
        df_dotdot - d_dotdot;
    VectorXd B = T_inverse * Df;
    trajectory.b << d, d_dot, d_dotdot/2, B[0], B[1], B[2];
	//} else
	//  trajectory.b << d, 0, 0, 0, 0, 0;

  double fwd_acc = trajectory.target_acc;
  //Combined acceleration
  trajectory.target_acc = sqrt(fwd_acc*fwd_acc + d_dotdot*d_dotdot);
  cout << "\n solve_d_quintic: car d: " << ego_car.d << " target lane: " << trajectory.target_lane
      << " acc: " << d_dotdot << " car lane: " << ego_car.lane << " Total acc: " << trajectory.target_acc << endl;

}

/**
 * Generate the trajectory based on the requested state and current conditions.
 *
 * Send the full trajectory to the car and re-plan when the current plot is
 * less than the required horizon (1 second).
 *
 * If a new 'state' is requested or conditons change (slow car ahead), discard remainder
 * and plot a new trajectory
 *
 */
void TrajectoryPlanner::plan_trajectory(
    const SharedData& shared,
		FSM state,
		const VehiclePose& ego_car,
		const vector<Limit>& limits,
		double end_s,
		double end_d,
		const vector<double>& previous_path_x,
		const vector<double>& previous_path_y,
		Trajectory &trajectory) {

	int    rem      = previous_path_x.size();//Number of trajectory points remaining on the car's queue
	trajectory.gap  = PLAN_AHEAD;            //Safe gap between ego car and car ahead
  int horizon     = 0;                     //Actual length of plan in trajectory points (used for generation phase)

  VehiclePose car = ego_car;

  /**
	 * Assess how to obey the state request and change df and sf_dot accordingly
	 */
	apply_requested_state(state, car, limits, trajectory);

  /**
   * Signal a lane-change is in progress.
   * Forces Behavior Planner to wait before requesting a state change
   */
  trajectory.in_progress = (state == FSM::CL || state == FSM::CR) &&
      (!in_range(lane_center(trajectory.target_lane), end_d, 0.01));

  double st = 0.0;
	double dt = 0.0;
	double xt = 0.0;
	double yt = 0.0;

  trajectory.s.clear();
  trajectory.d.clear();
  trajectory.x.clear();
  trajectory.y.clear();

	/**
	 * Plan a new trajectory when requested state or speed have changed,
	 * or the remainder of the old trajectory is less than the HORIZON limit (1 second or 50 intervals)
	 *
	 * NOTE: State machine of the Behaviour Planner controls the following states: KE, CL, CR, PL, PR
	 *  The Trajectory Planner controls the KB states.
	 * 	KB is used when the vehicle needs to slow down.
	 */

	if ( trajectory.target_state != state ||
			(trajectory.target_state == state && trajectory.plan.size() <= HORIZON-rem) ) {

	  if (rem > 0) {
	    //Reuse drive cache if available
      car.s = end_s;
      car.d = end_d;
      car.d = trajectory.end_d;
	  }
    car.v = trajectory.end_v;
		if (trajectory.target_state == state && !trajectory.plan.empty()) {
		  /**
		   * Just updating trajectory for existing state;
		   * starting from end of the current plan or
		   * Vehicle location if plan is empty
		   */
	      Coord c = trajectory.plan.back();
        car.s = c.s;
        car.d = c.d;
		}

    //Ignore the KB state, its just used to trigger this update on a target speed change
    if (state == FSM::KB) state = FSM::KE;
    trajectory.target_state = state;    //New state requested

    //Plan new trajectory
		solve_s_quintic(car, trajectory, horizon);
		solve_d_quintic(car, trajectory, horizon);

		/**
		 * Generate and cache the trajectory and endpoints
		 */
		VectorXd DT(6);
		double ti = 0.0;
		dt = 0.0;
		st = 0.0;
		for (int i = 1; i <= horizon; i++) {
			Coord c;
      ti = ti + INTERVAL;
      DT << 1, ti, pow(ti,2), pow(ti,3), pow(ti,4), pow(ti,5);

			st = trajectory.a.transpose() * DT;
			c.v = trajectory.a_s.transpose() * DT;
			trajectory.s.push_back(st);
			c.s = st;

			/**
			 * Stop adjusting d when close to center of lane
       */
			if ( in_range(dt, lane_center(trajectory.target_lane), 0.01) ) {
				trajectory.b << lane_center(trajectory.target_lane), 0, 0, 0, 0, 0;
			} else {
				dt = trajectory.b.transpose() * DT;
			}

			//dt = trajectory.b.transpose() * DT;
			trajectory.d.push_back(dt);
			c.d = dt;
			trajectory.t = ti;
			/**
			 * Uses a Spline to smooth over the Frenet to
			 * Cartesian conversion for the waypoints
			 */
			vector<double> xy = getXY2(
					trajectory.s.back(),
					trajectory.d.back(),
					shared.s_x,
					shared.s_y,
					shared.s_dir);

			c.x = xy[0];
			c.y = xy[1];
			trajectory.plan.push(c);
		} //END PLAN GENERATION
	}//END OF PLANNING

	/**
	 * Reuse previous path if available
	 */
	if (rem > 0) {
		for (int i=0; i<rem; i++) {
			trajectory.x.push_back(previous_path_x[i]);
			trajectory.y.push_back(previous_path_y[i]);
		}
	}

	/**
	 * Append the new plan.
	 */
	for (int i = 0; i < HORIZON-rem; i++) {
    Coord c = trajectory.plan.front();
    trajectory.plan.pop();
    trajectory.x.push_back(c.x);
    trajectory.y.push_back(c.y);
    /**
     * Record the speed at the end of this section for
     * recalculation on a state change later
     */
    trajectory.end_v = c.v;
    trajectory.end_d = c.d;
    if (trajectory.plan.empty()) {
      break;
    }
	}
}


