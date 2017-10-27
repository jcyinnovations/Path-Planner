#include "gtest/gtest.h"
#include "Trajectory.h"
#include <iostream>
#include <fstream>

using namespace std;

namespace {
	class TrajectoryTest : public testing::Test {
		public:
		    vector<vector<int>> grid = {{0, 0, 1, 0, 0, 0},
										{0, 0, 1, 0, 0, 0},
										{0, 0, 1, 0, 0, 0},
										{0, 0, 0, 0, 1, 0},
										{0, 0, 1, 1, 1, 0},
										{0, 0, 0, 0, 1, 0}};
			AStar astar;

			TrajectoryPlanner planner; //(845.3767, 1124.909, 60.8172, 9.999032, 9.999032, 0, 800.00);

			vector<vector<double>> sensor_fusion = {
			    {0,845.3767,1124.909,23.41313,-0.04853955,60.8172,9.999032},
				{1,816.2174,1124.919,26.40998,-0.2857875,31.63062,10.01023},
				{2,845.5734,1132.909,23.37145,-0.04845416,60.9868,1.998778},
				{3,775.8,1432.9,0,0,6713.911,-285.7268},
				{4,775.8,1436.3,0,0,6711.566,-288.1896},
				{5,775.8,1441.7,0,0,6661.772,-291.7797},
				{6,762.1,1421.6,0,0,6711.778,-268.0964},
				{7,762.1,1425.2,0,0,6709.296,-270.7039},
				{8,762.1,1429,0,0,6663.543,-273.1828},
				{9,762.1,1432.9,0,0,6660.444,-275.5511},
				{10,762.1,1436.3,0,0,6657.743,-277.6157},
				{11,762.1,1441.7,0,0,6653.453,-280.8947}
			};

			TrajectoryTest() {}

			void SetUp( ) {
				//Search Setup
			    VehiclePose init;
			    init.grid_x = 0;
			    init.grid_y = 0;
			    VehiclePose goal;
				goal.grid_x = grid.size()    - 1;
				goal.grid_y = grid[0].size() - 1;
				this->astar.init_heuristic(grid, goal);
			    astar.search(init, goal);

			    //TrajectoryPlanner Setup
			    VehiclePose ego_car;
			    //910.48, 1128.67, 125.834, 6.17348, 0, 55.9234
	          	ego_car.x = 910.48;
				ego_car.y = 1128.67;
				ego_car.s = 125.834;
				ego_car.d = 6.17348;
				ego_car.yaw = 0;
				ego_car.v = 55.924;
				ego_car.id =-1;

				// Load up map values for waypoint's x,y,s and d normalized normal vectors
				vector<double> map_x;
				vector<double> map_y;
				vector<double> map_s;
				vector<double> map_dx;
				vector<double> map_dy;

				// Waypoint map to read from
				string map_file_ = "/home/jyarde/Udacity/CarND-Path-Planning-Project/data/highway_map.csv";
				// The max s value before wrapping around the track back to 0
				double max_s = 6945.554;
				std::ifstream in_map_(map_file_.c_str(), ifstream::in);
				string line;
				while (getline(in_map_, line)) {
					std::istringstream iss(line);
					double x;
					double y;
					float s;
					float d_x;
					float d_y;
					iss >> x;
					iss >> y;
					iss >> s;
					iss >> d_x;
					iss >> d_y;
					map_x.push_back(x);
					map_y.push_back(y);
					map_s.push_back(s);
					map_dx.push_back(d_x);
					map_dy.push_back(d_y);
				}
				planner = TrajectoryPlanner(map_x, map_y, map_s, map_dx, map_dy);
			    planner.state_update(ego_car, sensor_fusion, 1);

				//this->astar.init_heuristic(grid, goal);
			    //astar.search(init, goal);
			}

			void TearDown( ) {
			}

	};

	TEST_F(TrajectoryTest, Trajectory) {
		ASSERT_TRUE((int)FSM::KE == 0);
		//ASSERT_TRUE(classifier.label_index("left") == 0);
		//ASSERT_TRUE(classifier.label_index("right") == 2);
	}

}
