/*
 * Trajectory.cpp
 *
 *  Created on: Oct 16, 2017
 *      Author: jyarde
 */

#include "Trajectory.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
											 grid_start_s(0),
											 target_lane(1),
											 startup(true){
	this->track_length = map_x.size();
	car_t0.d 		= 0;
	car_t0.d_dot 	= 0;
	car_t0.d_dotdot = 0;
	car_t0.s 		= 0;
	car_t0.s_dot 	= 0;
	car_t0.s_dotdot = 0;
}

TrajectoryPlanner::TrajectoryPlanner() :
										 map_x(),
										 map_y(),
										 map_s(),
										 map_dx(),
										 map_dy(),
										 closestWaypoint(0),
										 grid_start_s(0),
										 track_length(0),
										 target_lane(1),
										 startup(true){
	car_t0.d 		= 0;
	car_t0.d_dot 	= 0;
	car_t0.d_dotdot = 0;
	car_t0.s 		= 0;
	car_t0.s_dot 	= 0;
	car_t0.s_dotdot = 0;
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
 * Generates trajectories to execute the instructions of the BehaviourPlanner
 * Trajectories are for the next 50 timesteps (1 second of travel).
 *
 * Assumes that the BehaviorPlanner sends a snapshot of the world as it will be at the start of the
 * next timestep of the trajectory.
 *
 * Operations:
 * 		1. Translate traffic predictions into a working grid.
 * 		2. Generate a destination cell based on requested state and 50 timesteps ahead
 * 			(e.g KeepLane: 50 timesteps ahead in the same lane, TurnLeft: 50 timesteps ahead in the left lane
 * 		3. Hybrid A* search to the destination (5th order polynomial for each expansion
 */
Trajectory TrajectoryPlanner::plan_trajectory(
		FSM state,
		VehiclePose ego_car,
		vector<vector<VehiclePose>> sorted_traffic) {

	Trajectory new_trajectory;
	new_trajectory.target_state = state;
	double t   = INTERVAL * HORIZON;	//time horizon for trajectory
	double TARGET_DISTANCE = 50;
	MatrixXd T = MatrixXd(3, 3);
	T << pow(t, 3), pow(t, 4), pow(t, 5),
		3*pow(t, 2), 4*pow(t, 3), 5*pow(t, 4),
		6*t, 12*pow(t, 2), 20*pow(t, 3);

	/**
	 * Initial State
	 */
	VectorXd a = VectorXd(6);
	double s 		= ego_car.s;
	double s_dot 	= ego_car.v; //Derive speed
	double s_dotdot	= (startup) ? 0.0 : (s_dot - car_t0.s_dot); //Derive acceleration

	VectorXd b = VectorXd(6);
	double d 		= ego_car.d;
	double d_dot 	= (startup) ? 0.0 : (ego_car.d - car_t0.d);
	double d_dotdot	= (startup) ? 0.0 : (d_dot - car_t0.d_dot);	//use previous final acceleration

	ego_car.s_dot 	= s_dot;
	ego_car.s_dotdot= s_dotdot;
	ego_car.d_dot 	= d_dot;
	ego_car.d_dotdot= d_dotdot;

	/**
	 * Final State
	 */
	double sf_dot	= min(SPEED_LIMIT_MPS, s_dot + 0.50*MAX_ACCELERATION*t);	//final speed
	double sf		= ego_car.s + t * sf_dot;	//(target distance for limited acceleration)
	double sf_dotdot= (sf_dot-s_dot)/t;	//final acceleration

	double df		= 0.0;	//center of lane chosen below
	double df_dot	= (state == FSM::KE) ? 0.0 : LANE_WIDTH/5;	//0.0 for keep lane, otherwise change lanes in 5 seconds
	double df_dotdot= (state == FSM::KE) ? 0.0 : 0.10*MAX_ACCELERATION;	//final acceleration
	startup = not(startup);	//Pass the iniital phase

	/**
	 * Update final state position. Conditions for update:
	 * 1. Time horizon is length of the trajectory (50 points or 1 second)
	 * 2. Target speed: speed limit or speed of overtaking traffic (behind) in the target lane
	 * 3. Acceleration: 0
	 * 4. Sf: the distance of travel over the time horizon (equivalent to speed for 1 second)
	 */
	if (state == FSM::CL) {
		if (ego_car.lane > 1) {
			//Change to left lane if possible
			new_trajectory.state_possible = true;
			/**
			 * Check for close traffic in the next lane since we can only travel
			 * as far as traffic will allow
			 */
			target_lane = ego_car.lane - 1;
			if (sorted_traffic[target_lane-1].size() > 0) {
				for (VehiclePose current_car : sorted_traffic[target_lane-1]) {
					if (current_car.leading) {
						if (current_car.v < SPEED_LIMIT_MPS){
							sf_dot = current_car.v * cos(current_car.yaw);
						}
						//Traffic is sorted by leading and distance so quit loop after first
						//'leading' hit
						break;
					}
				}
			}
			//Update motion parameters
			df = lane_center(target_lane);
		} else
			new_trajectory.state_possible = false;
	} else

	if (state == FSM::CR) {
		if (ego_car.lane < 3) {
			//Change to right lane if possible
			new_trajectory.state_possible = true;
			/**
			 * Check for close traffic in the next lane since we can only travel
			 * as far as traffic will allow
			 */
			target_lane = ego_car.lane + 1;
			if (sorted_traffic[target_lane-1].size() > 0) {
				for (VehiclePose current_car : sorted_traffic[target_lane-1]) {
					if (current_car.leading) {
						if (current_car.v < SPEED_LIMIT_MPS){
							sf_dot = current_car.v * cos(current_car.yaw);
						}
						//Traffic is sorted by leading and distance so quit loop after first
						//'leading' hit
						break;
					}
				}
			}
			//Update motion parameters
			df = lane_center(target_lane);
		} else
			new_trajectory.state_possible = false;
	} else
	if (state == FSM::KE) {
		/**
		 * To Keep Lane: develop a trajectory that keeps you behind cars ahead
		 * So match speed of cars ahead if they are lower than the speed limit
		 */
		target_lane = ego_car.lane;
		if (sorted_traffic[target_lane-1].size() > 0) {
			for (VehiclePose current_car : sorted_traffic[target_lane-1]) {
				if (current_car.leading) {
					if (current_car.v < ego_car.v){
						sf_dot = current_car.v;
					}
					//Traffic is sorted by leading and distance so quit loop after first
					//'leading' hit
					break;
				}
			}
		}
		//Update motion parameters
		df = lane_center(target_lane);
	}

	/**
	 * solve for coefficients
	 */
	VectorXd Sf = VectorXd(3);
	Sf << sf - (s + s_dot*t + s_dotdot * pow(t,2)/2),
			sf_dot - (s_dot + s_dotdot * t),
			sf_dotdot - s_dotdot;

	VectorXd A = T.inverse() * Sf;

	VectorXd Df = VectorXd(3);

	Df << df - (d + d_dot*t + d_dotdot * pow(t,2)/2),
			df_dot - (d_dot + d_dotdot * t),
			df_dotdot - d_dotdot;

	VectorXd B = T.inverse() * Df;

	/**
	 * Setup coefficients
	 */
	a << s, s_dot, s_dotdot/2, A[0], A[1], A[2];
	b << d, d_dot, d_dotdot/2, B[0], B[1], B[2];
	/**
	 * Save the current vehicle pose
	 */
	car_t0 = ego_car;

	/**
	 * Generate the trajectory. First, the endpoint
	 */
	VectorXd DT(6);
	DT << 1, t, pow(t,2), pow(t,3), pow(t,4), pow(t,5);
	double s_final = a.transpose() * DT;
	double d_final = b.transpose() * DT;
	double s_theta = atan2(s_final, t);
	double d_theta = atan2(d_final, t);

	for (int i = 1; i < HORIZON; i++) {
		double ti = i * s_final * cos(s_theta) / HORIZON;
		//double t_d = i * d_final * cos(d_theta) / HORIZON;
		VectorXd DT(6);
		DT << 1, ti, pow(ti,2), pow(ti,3), pow(ti,4), pow(ti,5);
		new_trajectory.s.push_back( a.transpose() * DT );
		new_trajectory.d.push_back( b.transpose() * DT );
	}
	new_trajectory.s.push_back(s_final);
	new_trajectory.d.push_back(d_final);
	cout << "_____________________>" << endl;
	cout << "Generated Trajectory:" << endl;
	cout << new_trajectory.s << endl;
	cout << new_trajectory.d << endl;
	cout << "_____________________|" << endl;
	return new_trajectory;
}

/**
 * plot a trajectory for the target speed and path
 */
Trajectory TrajectoryPlanner::generate_trajectory(
		FSM state,
		VehiclePose ego_car,
		vector<vector<VehiclePose>> sorted_traffic) {
	Trajectory trajectory;
	//TODO:
	return trajectory;
}


