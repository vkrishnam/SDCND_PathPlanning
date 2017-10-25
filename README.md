# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided, there is also a sparse map list of waypoints around the highway available as csv file. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

No incidents/accidents of:
	Acceleration exceeding maximum (10m/s^2)
	Jerk exceeding maximum (50m/s^2)
        Collisions with any vehicle on road


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Supporting Resource in the Project

A really helpful resource for doing this project and creating smooth trajectories is using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.


---
## Overall Approach for the solution 
Broadly follows the approach discussed in the Project walkthrough Q&A Video, but improvises the lane change with the logic of evaluation of multiple trajectories and their costs with different cost functions.

Broadly the Path Planning has two states: _Keep Lane_ or _Change Lane_.


Always looks for obstacles/vehicle ahead of the ego in the current lane from sensor fusion data. If no vehicle with in 30 meters vicinity in the forward direction, keep the current lane.
If an obstacle/vehicle is found ahead in short distance, reduce the ego vehicle speed to the speed of the vehicle ahead as the reference. 
Also enable the lane change evaluation (lane_change_evalaution state) logic.
If lane change evaluation logic is enabled, every time we see if lane change is done to go back to lane keep state and disable lane change evaluation logic.
If lane change evaluation logic is enabled, the multiple possible lanes are evaluated, or else only the current lane is evaluated (lane keep state).
When lane change logic is enabled, mutliple possble lanes are considered and trajectories are created with fitting different splines. 

Later on each of those trajectories are interpolated for the path points whose distance is covered in 0.02sec given the current vehicle speed. 

These different paths are further evaluated for possible collisions etc using different cost functions. Finally the path points whose cost is lowest is choosen and there by lane change is executed. When lane change is completed and detected, the FSM moves to keep_lane state.


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```



