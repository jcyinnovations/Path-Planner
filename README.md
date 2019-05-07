# Path Planning Project

Self-Driving Car Engineer Nanodegree Program
   
### Introduction.

Path Planning implementation using Behaviour Planner, Trajectory Generator and model-based prediction of traffic.

#### Architecture Description

The initial design employed A* Search to layout and constantly update the vehicle trajectory. Given that the track is fixed navigating it only requires decisions about traffic. Route selection is only important where there are several possible paths forward. Therefore the design was simplified to two basic components: the Trajectory generator and the Behavior Planner. 

#### Trajectory Generator

The Trajectory Generator acts as the driver; making basic (autonomic) decisions to determine the way forward as any driver would. The Trajectory Planner owns responsiblity for slowing down if needed, deciding how to execute a specific state request (keep lane, left lane, right lane) and halting the Behaviour Planner in critical manouvers (e.g executing a lane change).

This was the most involved design of the architecture. Several iterations were spent trying both the understand design failings and to specify proper constraints for the trajectories and how they would be maintained.

The basic design is implemented by the `TrajectoryPlanner` class in `Trajectory.cpp`. The `plan_trajectory()` method implements the planning process. This method takes as input shared data (more on that later), the requested state, vehicle pose, lane limits (described below), the previous trajectory and previously emmitted trajectory points. It then goes through the following process:

- delegate to `apply_requested_state()` to apply the requested state. This updates the Trajectory struct with the target parameters (lane, clearance, gap etc). It will cause the vehicle to automatically slow down to match traffic ahead if they are slower than the speed limit or speed up to clear close trailing vehicles.
- decide whether to halt Behaviour Planner if a lane-change is requested.
- setup the initial conditions for generating trajectories based on vehicle pose and current trajectory points.
- delegate the `solve_s_quintic` and `solve_d_quintic` to generate quintics of the trajectory. They also generate a trajectory `horizon` or number of intervals required to complete the trajectory.
- generate the full trajectory (all intervals on the required `horizon`). This is cached in a `queue` structure for later use to append to the controller cache of `PLAN_AHEAD` trajectory points (set to 50).
- append any remnant from the main controller plus additional points from the new trajectory up to `PLAN_AHEAD` (50 intervals) which equates to 1 second of trajectory.

##### Shared Data

Very early on, the biggest hurdle to successful trajectories was the odd behavior when using the waypoints to generate x,y coordinates required by the controller. There was excessive jerk in the trajectory as a side-effect of interpolation between widely spaced waypoints. Rather than manually interpolate, waypoints were used to create splines s_x, s_y, s_dir. These splines generated x,y,dir values that were then consumed by `getXY2()` in `common.h` to generate the smoothed x,y values. This resulted in significantly smoother trajectories. 

There is a bug that occurs around Waypoint 155 that is yet to be resolved. This bug caused erratic behavior at this point.

##### Handling Traffic

Traffic as outlined in the sensor fusion data is sorted according to lanes and then reduced to 3 parameters:

- speed: The speed of the slowest car ahead in that lane
- gap: The smallest gap between the ego car and the leading car in that lane
- clearance: the smallest clearance between the ego car and the trailing car in that lane.

All these parameters are deduced into the future by `INTERVAL * PLAN_AHEAD` seconds (INTERVAL = 0.02, PLAN_AHEAD = 50) or 1 second. Clearance causes a speed up of the ego car (x2) if it is too small (the tolerance is set in CLEARANCE = 10 m). Gap specifies the horizon distance within which the trajectory must be completed with the optimal gap being 40 m or more.

##### Quintic Constraints

The s quintic is planned on a HORIZON distance of 40 m assuming forward acceleration of 5 m/s/s. The d quintic assumes a lane-change should be executed in 3.5 seconds under optimal conditions or less if mandated by lane conditons (gap to traffic ahead). The assumption is that if the lane-change is too drastic, the Behavior Planner cost function will avoid using it. 

#### The Behavior Planner

The Behavior Planner (`Behavior` class in `Behavior.cpp`) would use a state machine and a set of pre-defined states to iterate through several trajectories; one per possible next state and decide at each interval, the best way forward by its choice of trajectory. 

##### State Machine
The chosen states are `KE`: KEep Lane, `CL`: Change Left, and `CR`: Change Right. Initially, 4 additional states were used but were removed to simplify the project and because they were deemed unnecessary after much trial and error.

For example, to change lanes, the Prepare Change states (`PL`, `PR`) would signal matching the lane speed and looking for an appropriate gap to enter. Instead of using those states, the architecture employs a cost function extension for Clearance Cost combined with a speed/acceleration penalty in the Trajectory Planner. To match speed in a lane-change, a human would usually speed up to be faster than the lane they wish to enter. The trajectory planner does exactly that by trying to double the speed of the slowest car in the lane it must enter; only in cases where the clearance is too low (the trailing vehicle is too close). This will likely result in a penalty for acceleration or speed in the Behaviour Planner if the speed or acceleration is too high. 

The Clearance cost accounts for the gap between the ego car and the closest leading vehicle in the target lane. The larger the clearance, the better the lane (lower the cost for adopting a trajectory to enter that lane).

##### Cost Function

The following costs were employed (shown with weights):

- Speed 			(0.05)
- Lane Keeping 		(0.00)
- Acceleration		(0.20)
- Lane Target		(0.10)
- Stay On Road		(0.50)
- Clearance			(0.15)

Notice Lane Keeping was removed from the cost-function. This function was delegated to the Trajectory Planner after some experimentation.

Also a significant proportion of the costs was allocated to staying on the road to highlight its supremacy over all other considerations.

### Caveats

Possible issues:

- generating the full trajectory on any change may introduce a time penalty at the start of a new trajectory that causes enough jitter to temporarily exceed max speed or accelaration. I spent countless hours trying to debug this unsuccessfully.

- The planner is erractic around waypoint 155.  


