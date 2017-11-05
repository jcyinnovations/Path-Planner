#include "gtest/gtest.h"
#include "Trajectory.h"
#include <iostream>
#include <fstream>
#include "matplotlibcpp.h"

using namespace std;

namespace plt = matplotlibcpp;

inline void plot_vector(vector<double> v, string title) {
	plt::subplot(1,1,1);
	plt::title(title);
	plt::plot(v);
	plt::show();
}

namespace {
	class TrajectoryTest : public testing::Test {
		public:
		    vector<vector<int>> grid = {{0, 0, 1, 0, 0, 0, 0},
										{0, 0, 1, 0, 0, 0, 0},
										{0, 0, 1, 0, 0, 0, 0},
										{0, 0, 0, 0, 1, 0, 0},
										{0, 0, 1, 1, 1, 0, 0},
										{0, 0, 0, 0, 1, 0, 0}};
			AStar astar;

			TrajectoryPlanner planner; //(845.3767, 1124.909, 60.8172, 9.999032, 9.999032, 0, 800.00);

		    //	[id,	    x,	     y,vx, vy,	       s,	        d]
			vector<vector<double>> sensor_fusion = {
			{ 0,     887.235,   1128.789,   19.75101,   -0.02290,   102.6436,   6.01552},
			{ 1,    1061.183,   1161.814,   14.89583,    6.13705,   278.8251,   9.98521},
			{ 2,    1063.721,   1171.479,   17.44818,    7.19984,   284.8947,   2.04750},
			{ 3,     857.7584,  1132.914,   20.76746,    0.03553,    73.1717,   1.95211},
			{ 4,    1058.058,   1164.845,   16.41346,    6.70840,   277.1107,   5.98276},
			{ 5,     840.9315,  1132.841,    8.86841,    0.05004,    56.3394,   2.07203},
			{ 6,     871.342,   1128.833,   11.52896,   -0.04238,    86.7690,   5.98748},
			{ 7,     911.0454,  1132.809,   23.40757,    0.10243,   126.4348,   2.03984},
			{ 8,    1079.099,   1173.556,   15.91902,    6.30091,   299.8836,   6.06347},
			{ 9,     866.6768,  1128.851,   14.48248,   -0.0561,     82.1038,   5.98522},
			{10,     896.8539,  1132.786,   14.29458,    0.00285,   112.2613,   2.01557},
			{11,     897.9222,  1124.785,   22.16800,    0.00744,   113.3319,  10.01624}
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
			    //909.48, 1128.67, 124.834, 6.16483, 0, 0,

	          	ego_car.x 	=  910.48;
				ego_car.y 	= 1128.67;
				ego_car.s 	=  124.834;
				ego_car.d 	=    6.16483;
				ego_car.yaw = 	 0;
				ego_car.v 	= 	55.924;
				ego_car.id 	=	-1;

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
			    planner.state_update(ego_car, sensor_fusion, 1, 6945.554);

				goal.grid_x = 1;
				goal.grid_y = 190;
				VehiclePose start;
				start.grid_x = 1;
				start.grid_y = 134;
				planner.map_grid[1][134] = 0;
				this->astar.init_heuristic(planner.map_grid, goal);
			    astar.search(start, goal);

			    /**
			     * Sort traffic
			     */
			    vector<vector<VehiclePose>> traffic = sort_traffic(ego_car, sensor_fusion);
			    cout << "Traffic Sorted by Lane: \n";
			    for (vector<VehiclePose> t : traffic) {
			    	cout << t << endl;
			    }

			    /**
			     *Distance from ego car
			     */
			    vector<double> d = vector<double>(sensor_fusion.size(), 0);
			    cout << "Traffic distance: " << sensor_fusion.size() << endl;
			    for (int s=0; sensor_fusion.size(); s++) {
			    	d[s] = sensor_fusion[s][5] - ego_car.s;
				    cout << "s: " << sensor_fusion[s][5] << " distance: " << d[s] << endl;
			    }

			    plot_vector(d, "Traffic Distance");
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
