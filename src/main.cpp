#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "common.h"
#include "Trajectory.h"
#include "Behavior.h"

using namespace std;

// for convenience
using json = nlohmann::json;

TrajectoryPlanner trajectory_planner;
Behavior behavior_planner;
Trajectory trajectory;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  vector<double> map_waypoints_dir;
  vector<double> map_waypoints_cosdir;
  vector<double> map_waypoints_sindir;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
    double dir = atan2(d_y, d_x);
    map_waypoints_dir.push_back(dir);
    map_waypoints_cosdir.push_back(cos(dir));
    map_waypoints_sindir.push_back(sin(dir));
  }

  SharedData shared;

  /**
   * Create splines mapping s to cartesian coordinates from the map data
   */
  shared.s_x.set_points(map_waypoints_s, map_waypoints_x);
  shared.s_y.set_points(map_waypoints_s, map_waypoints_y);
  shared.s_dir.set_points(map_waypoints_s, map_waypoints_cosdir);
  shared.s_cosdir.set_points(map_waypoints_s, map_waypoints_cosdir);
  shared.s_sindir.set_points(map_waypoints_s, map_waypoints_sindir);

  shared.map_waypoints_s = map_waypoints_s;
  shared.map_waypoints_x = map_waypoints_x;
  shared.map_waypoints_y = map_waypoints_y;

  h.onMessage(
      [&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &shared](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
          uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

          auto s = hasData(data);

          if (s != "") {
            auto j = json::parse(s);

            string event = j[0].get<string>();

            if (event == "telemetry") {
              // j[1] is the data JSON object

              // Main car's localization Data
              double car_x = j[1]["x"];
              double car_y = j[1]["y"];
              double car_s = j[1]["s"];
              double car_d = j[1]["d"];
              double car_yaw = j[1]["yaw"];
              double car_speed = j[1]["speed"];

              //cout << "\n\n\nVehicle = " << car_x << ", \t\t" << car_y << ", \t\t" << car_s << ", \t" << car_d << ", \t" << car_yaw << ", \t" << car_speed << endl;

              // Previous path data given to the Planner
              vector<double> previous_path_x = j[1]["previous_path_x"];
              vector<double> previous_path_y = j[1]["previous_path_y"];
              // Previous path's end s and d values
              double end_path_s = j[1]["end_path_s"];
              double end_path_d = j[1]["end_path_d"];

              // Sensor Fusion Data, a list of all other cars on the same side of the road.
              auto sensor_fusion = j[1]["sensor_fusion"];
              vector<vector<double>> sensor_data;

              /**
               * PATH PLANNING
               */
              VehiclePose ego_car;
              ego_car.x = car_x;
              ego_car.y = car_y;
              ego_car.s = car_s;
              ego_car.d = car_d;
              ego_car.yaw = deg2rad(car_yaw);
              ego_car.v = mph_to_mps(car_speed);
              ego_car.id =-1;
              int car_waypoint = ClosestWaypoint(car_x, car_y, map_waypoints_x, map_waypoints_y);
              ego_car.waypoint = car_waypoint;
              ego_car.lane = current_lane(car_d);
              cout << "\t\t\tWAYPOINT: " << car_waypoint << endl;
              json msgJson;
              vector<double> next_x_vals;
              vector<double> next_y_vals;
              /**
               * Sort traffic before sending to the behavior planner
                vector<vector<VehiclePose>> traffic = sort_traffic(ego_car, sensor_fusion.get<vector<vector<double>>>());
              */

              /**
               * lane_limits deduces the speed and distance of the closest traffic within PLAN_AHEAD
               * distance of the ego car. This is used by the trajectory planner to automatically
               * decide when to accelerate or brake depending on traffic conditions.
               */
              //TODO: Modify traffic by applying predictions of future state
              vector<Limit> limits = lane_limits(ego_car, sensor_fusion.get<vector<vector<double>>>());

              /**
               * Just the Trajectory Planner
              trajectory_planner.plan_trajectory(shared,
                  FSM::KE, ego_car, limits, end_path_s, end_path_d, previous_path_x, previous_path_y, trajectory);
              **/

              /**
               * Trajectory Planner wrapped with the Behaviour Planner
               **/
               behavior_planner.transition_function(shared, ego_car, limits, end_path_s,
                                                    end_path_d, previous_path_x, previous_path_y, trajectory);

              int i = 0;

              while (i < trajectory.x.size() ) {
                next_x_vals.push_back(trajectory.x[i]);
                next_y_vals.push_back(trajectory.y[i]);
                i++;
              }

              msgJson["next_y"] = next_y_vals;
              msgJson["next_x"] = next_x_vals;

              auto msg = "42[\"control\","+ msgJson.dump()+"]";

              //this_thread::sleep_for(chrono::milliseconds(1000));
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          } else {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
      size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
      char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
