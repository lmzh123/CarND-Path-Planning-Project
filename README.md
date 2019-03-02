# **Self-Driving Car Engineer Nanodegree** 

## Luis Miguel Zapata

---

**Path Planning**

This projects aims to develop a path planning algorithm for autonomous driving in a simulated highway.

[image1]: ./screenshots/simulator.png "Simulator"
[image2]: ./screenshots/trajectory.png "Trajectory"
[image3]: ./screenshots/fusion.png "Sensor Fusion"
[image4]: ./screenshots/result.png "Result"

### 1. Environment.
Simulator is based in Unity and mimics a car for autonomous driving in a highway. The vehicle is controlled from a c++ script using web sockets where the script specifies whih will be the nest positions in x and y coordinates.

![][image1]

The main goal of this project is to drive safely sorting the other vehicles when possible without exceeding the maximum allowed speed and jerk for vehicle's user control.

### 2. Trajectory generation.
In order to generate the trajectories for the vehicle the library spline is used. This tool allows to obtain a smooth trajectory using reference points. Fisrt step to generate the trajectory is to define 2 intial points based either on the previous path data given to the planer or the actual vehicle position if previous data is been already executed.

Next step is to define 3 points at 30, 60 and 90m ahead in frenet coordinates acoording to our current s and d that is given in terms of the actual lane.

```
vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
```

These 5 points in cartesian coordinates are referenced according to the previous final position and heading.

```
for(int i = 0; i < ptsx.size(); i++){
  // Shift car reference angle to 0 degrees
  double shift_x = ptsx[i]-ref_x;
  double shift_y = ptsy[i]-ref_y;

  ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
  ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
}
```
With this points a spline function is generated. This acts as an equation where according to a given x position the function returns the y coordinate  of the trajectory. Using this the last step is to generate 50 points using the reference velocity at which we want to travel and the delta time of our simulator (0.02s). Moreover as we are working in miles per hour these velocities have to be transformed from meters per second. Finally each x coordinate is incrementaly calculated and its corresponding y coordinate using the spline function.

```
// Calculate how to break up spline points so that we travel at our desired reference velocity
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

double x_add_on = 0;

// Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
for(int i =0; i <= 50-previous_path_x.size(); i++){
  double N = (target_dist/(0.02*ref_vel/2.24));
  double x_point = x_add_on+target_x/N;
  double y_point = s(x_point);
  x_add_on = x_point;

  double x_ref = x_point;
  double y_ref = y_point;

  // Rotate back to normal after rotating it earlier
  x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
  y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

  x_point += ref_x;
  y_point += ref_y;

  next_x_vals.push_back(x_point);
  next_y_vals.push_back(y_point);
}
```

The last step is to rotated back to its original orientation and feed to the simulator.

![][image2]

### 3. Sensor fusion.

![][image3]

### 4. Results.

![][image4]