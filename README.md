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

The main goal of this project is to drive safely sorting the other vehicles when possible without exceeding the maximum allowed speed and jerk for vehicle's user comfort.

### 2. Trajectory generation.
In order to generate the trajectories for the vehicle the library spline is used. This tool allows to obtain a smooth trajectory using reference points. Fisrt step to generate the trajectory is to define 2 intial points based either on the previous path data given to the planer or the actual vehicle position if previous data has been already executed.

Next step is to define 3 points at 30, 60 and 90m ahead in s position in Frenet coordinates acoording to our current s and d (frenet coordinates) that is given in terms of the actual lane. This lane term will help us to move between lanes since the spline function will generate acoording to this, for instance, lane 0 corresponds to the left most lane, 1 to the center one and 2 is the right most lane.

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
With these points a spline function is generated. This function acts as an equation where according to a given x position the function returns the y coordinate  of the trajectory. Using the reference velocity at which we want to travel and the delta time of our simulator (0.02s) we generate the rest of the points of the trajectory which will always be of 50. 

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

The last step is to rotated back to its original orientation and feed to the simulator the path.

![][image2]

### 3. Sensor fusion.
The simulator give us all relevant information about surrounding vehicles:
* s position.
* d position.
* velocity in x.
* vlocity in y.

Using this information the planner will decide in which lane to drive and at what velocity to do it. The approach for driving safely according to the other vehicles will be the following:

Frist step is to determine in which lane is every vehicle using the d position.

```
// Check in wich lane is the car
int car_lane;
if(d >= 0 && d < 4){
  car_lane = 0;
}
else if(d >= 4 && d < 8){
  car_lane = 1;
}
else if(d >= 8 && d < 12){
  car_lane = 2;
}
```

Next thing is to determine which is the closest vehicle within 30m ahead or behind in s position in each of the surrounding anc actual lanes and gather their speeds. 

```
// velocity of the closest one
if(check_car_s > car_s - 30 && check_car_s <= car_s){
  if(car_lane == lane - 1){
    left_behind_flag = true;
    if(car_s - check_car_s < closest_left_behind){
      closest_left_behind = car_s - check_car_s;
      left_behind_vel = check_speed;
    }
  }
  // Check if it's on the right 
  else if(car_lane == lane + 1){
    right_behind_flag = true;
    if(car_s - check_car_s < closest_right_behind){
      closest_right_behind = car_s - check_car_s;
      right_behind_vel = check_speed;
    }
  }
}
// Check cars ahead
else if(check_car_s > car_s && check_car_s <= car_s + 30){
  if(car_lane == lane){
    ahead_flag = true;
  }
  // Check if it's on the left 
  else if(car_lane == lane - 1){
    left_ahead_flag = true;
    if(car_s - check_car_s < closest_left_ahead){
      closest_left_ahead = check_car_s - car_s;
      left_ahead_vel = check_speed;
    }
  }
  // Check if it's on the right 
  else if(car_lane == lane + 1){
    right_ahead_flag = true;
    if(car_s - check_car_s < closest_right_ahead){
      closest_right_ahead = check_car_s - car_s;
      right_ahead_vel = check_speed;
    }
  }
}
```

With this information we will determine if either we want to move to our right, to our left or simply stay in the same lane and also if we want to speed up or down incrementally.

```

if(ahead_flag){
  if(closest_left_behind >= 20 && closest_left_ahead >= 20 && 
     left_behind_vel - ref_vel < 15 && ref_vel - left_ahead_vel < 15 &&
     lane > 0){
    lane--;
  }
  else if(closest_right_behind >= 20 && closest_right_ahead >= 20 &&
          right_behind_vel - ref_vel < 15 && ref_vel - right_ahead_vel < 15
          && lane != 2){
    lane++;
  }
  else{
    ref_vel -= 0.224;
  }
}
else{
  if(ref_vel < 49.5){
    ref_vel += 0.224;
  }
} 
```

Simply if there is a car ahead us at less than 30m away we will look for a 40m gap on any lane where if there is a car behind us in that lane, this vehicle is not going 15mph faster than us and if there is a vehicle ahead in that lane, this is not going 15mph slower than us. If we can not merge we will we will reduce the speed 1mph. And if there is not a car ahead us we will speed up 1mph at a time, but being below the speed limit. 

![][image3]

### 4. Results.
The vehicle is able to navigate succesfuly for several miles and does not surpass the established limits. 
![][image4]