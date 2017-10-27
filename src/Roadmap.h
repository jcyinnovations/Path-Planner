/*
 * Roadmap.h
 *
 *  Created on: Oct 14, 2017
 *      Author: jyarde
 */

#ifndef ROADMAP_H_
#define ROADMAP_H_

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Roadmap {

public:

	int lanes;
	double speed_limit;
	double LANE_WIDTH = 4.0;
	double CELL_SIZE  = 4.0;	//Set the dimensions of a cell to lane width
	double route_length;		///Distance between start and end waypoints
	/**
	 * Roadway grid. Length is route_length/CELL_SIZE. Width is LANE_WIDTH*lanes/CELL_SIZE.
	 * The starting location is assumed to be closed since you don't want to go backwards.
	 */
	vector< vector<int> > GRID;

	Roadmap();

	virtual ~Roadmap();

};


#endif /* ROADMAP_H_ */
