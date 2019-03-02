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

![][image2]

### 3. Sensor fusion.

![][image3]

### 4. Results.

![][image4]