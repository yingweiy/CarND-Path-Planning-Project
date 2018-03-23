# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


### Algorithmic Details

#### Trajectory generation
The trajectory is generated using spline. 

```python
line 11, code header 
#include "spline.h"

In the function h.onMessage()
//create a spline
tk::spline sp;

//set x,y points to the spline
sp.set_points(ptsx, ptsy);

// the trajectory points are then predicted by the x_point            
double y_point = sp(x_point);            
```

The spline is constructed with the ``lane`` parameter at 30m, 60m, and 90m:

```python
// In Frenet add evenly 30m spaced points ahead of the starting reference
vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

```

Then the spline is used to interpolate the points between these anchor points.

#### Acceleration and brake profile
The speed control is based on if the car is too close (too_close) to the car in front of it, as well as
the speed limit. In the acceleration, as the code shown below, the profile is based on the ref_vel. 
It is faster when the speed is slow, and slower when the car is already fast enough. 
```python
if (ref_vel < speed_limit)
{
    ref_vel += .224 * pow(50.0/ref_vel, 0.3); //accerlation is faster when the speed is slow
}
```

When the car is too close, the decrease of the speed is depending on the relative speed of the 
car to the car in front of it. If it is much faster than previous car, the speed decrease would be 
very fast, otherwise, it will, and may not decrease the speed.
```python
if (too_close)
{
    ref_vel -= .224 * pow((car_speed-too_close_check_speed)/30.0, 3); //decide the optimal speed to follow
}
```

#### States
The control is time-point-wise, and in order to have a memory, a state variable is necessary.
I only used two states in the implementation: keep in lane (0) and lane changing (1). 
However, for the lane changing, there is a parameter associated with this state, target_lane.

When the car decides to lane changing and the target lane is safe to do so, it will set the 
state variable to 1 (lane changing), and keep in the state until entered the target lane. Then the 
state is back to 0. 


#### Lane changing strategy

##### Safety
Gap checking with projected position of both car and checked cars in function below. A ```safe_distance```
variable is defined to allow the sufficient gap size (including the size of the car body's). 
A estimated time (2 second) of lane changing is used to project the location of the car.
```python
bool CheckLaneAvailability(int lane, double car_s, double car_speed, int prev_size, const vector<vector<double>> &sensor_fusion) {
    bool available = true;
    double safe_dist = 10;

    for (int i = 0; i<sensor_fusion.size(); i++)
    {
        // car is in my lane        
        float d = sensor_fusion[i][6];  //the i-th car's displacement (lane position)
        if ((d < 2 + 4 * lane + 2) && (d> 2+4*lane-2))   //if the car is in the lane
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            //double t = (double) prev_size * .02;
            double t = 2; //assume two seconds lane changing
            double check_car_sp; //projected check_car_s
            double car_sp;  //projected car_s
            check_car_sp = check_car_s + ( t * check_speed);
            car_sp = car_s + (t * car_speed);

            if ((check_car_sp >= car_sp) && (car_sp >= check_car_s-safe_dist))
            {
                available = false;
                break;
            }

            if ((check_car_sp >= car_s - safe_dist) && (car_s-safe_dist >= check_car_s - safe_dist))
            {
                available = false;
                break;
            }

            if ((car_sp >= check_car_sp) && (car_s <= check_car_s))
            {
                available = false;
                break;
            }

            if ((car_sp <= check_car_sp) && (car_s >= check_car_s))
            {
                available = false;
                break;
            }

        }
    }    
}
```

##### Optimization
Lane 1 has a choice function to choose from lane 0 or lane 2

   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).


#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

