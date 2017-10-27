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
			printf("%6.2f", c);
		}
		printf("\n");
	}
	printf("\n");
}

void AStar::search(VehiclePose init, VehiclePose goal) {
	this->init_heuristic(grid, goal);
	this->print_world();
	this->print_heuristic();

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
			//cout << route << endl;
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
											 grid_start_s(0){}

TrajectoryPlanner::TrajectoryPlanner(){};

TrajectoryPlanner::~TrajectoryPlanner(){}

/**
 * Create a grid over the next N waypoints starting at an the vehicle.
 * interpolate a waypoint for the vehicle from the closest waypoint.
 * Place all detected vehicles on the grid fully covering all cells they touch.
 * Cells are square with side equal to lane width.
 * N set to cover sensor range from vehicle.
 * Must include closest waypoint behind the vehicle.
 */
void TrajectoryPlanner::state_update(VehiclePose vehicle, vector<vector<double>> sensor_fusion, int closestWaypoint) {
	this->location = vehicle;
	this->closestWaypoint = closestWaypoint;
	int grid_width = ROAD_MAX / CELL_SIDE;	//width in cells
	int start = closestWaypoint;
	if (closestWaypoint > 5)
		start = closestWaypoint - 5;		//Map at least 5 cells behind the ego car
	int grid_length = SENSOR_RANGE;			//(int) (map_s[SENSOR_RANGE+start] - map_s[start])/CELL_SIDE;
	map_grid = vector<vector<int>>(grid_length, vector<int>(grid_width, 0));	//init map with all 0's
	this->grid_start_s = map_s[start];

	/**
	 * Now map all traffic onto the grid
	 */
	traffic.clear();
	//Structure: [ 0, 1, 2,  3,  4, 5, 6]
	//Structure: [id, x, y, vx, vy, s, d]
  	for (auto sf : (vector<vector<double>>)sensor_fusion) {
  		//TODO: Account for vehicles changing lanes (straddling cells)
  		//TODO: Make an assumption about vehicle dimensions to do this
  		//Vehicle is in range (in the grid of concern)
  		if (sf[5] >= grid_start_s && sf[5] < SENSOR_RANGE*CELL_SIDE+grid_start_s) {
  			int grid_y = 0;
  			if (sf[6] > LANE_WIDTH) {
  				if (sf[6] <= 2*LANE_WIDTH)
  					grid_y = 1;
  				else
  					grid_y = 2;
  			}
  			int grid_x = (int) (sf[5]-grid_start_s)/CELL_SIDE;
  			//place vehicle in the grid
  			map_grid[grid_x][grid_y] = 1;
  		}
  	}

	/**
	 * Map the ego car
	 */
	if (vehicle.s >= grid_start_s && vehicle.s < SENSOR_RANGE*CELL_SIDE+grid_start_s) {
		int grid_y = 0;
		if (vehicle.d > LANE_WIDTH) {
			if (vehicle.d <= 2*LANE_WIDTH)
				grid_y = 1;
			else
				grid_y = 2;
		}
		int grid_x = (int) (vehicle.s-grid_start_s)/CELL_SIDE;
		//place vehicle in the grid
		map_grid[grid_x][grid_y] = 3;
	}

	cout << "Map Grid: \n";
  	for (auto row : map_grid)
  		cout << row << endl;
}

Trajectory TrajectoryPlanner::get_trajectory(FSM state, VehiclePose pose) {
	Trajectory new_trajectory;
	//TODO:
	return new_trajectory;
}

