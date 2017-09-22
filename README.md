# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Implementation

### States

Car can be in one of three states:

* Keep current lane
* Change lane to the right
* Change lane to the left

In general any state can be chosen at any time except when car is in left
most or right most lane where states moving it outside drivable area cannot be chosen.

Ideally there should be one more state of emergency breaking.


### Finding the best next state

Each time the simulator provides new data a number of trajectories is generated.
Overall 9 trajectories are generated, 3 for each state.

So the result of each trajectory can be one of the following:

* staying in the current lane
* moving to the left lane
* moving to the right lane

plus one of the following:

* keeping the same speed
* increasing speed 
* decreasing speed

 
Cost is calculated for each possible trajectory. The best trajectory and its
corresponding state are chosen as the next state and trajectory.


### Trajectory generation

Result trajectory consists of 70 points which corresponds to 1.4 seconds horizon.
20 starting points are reused and 50 points are regenerated each time which means
that reaction time is 0.4 seconds. On a slow computer time span between simulator
calls may be 0.2 seconds and more so 0.4 sounds reasonable.

Trajectories are generated using spline. Spline is constructed based on two
last points of the previous reused trajectory (usually points 19 and 20) and
3 points lying in the center of target lane far away 
(usually 20th point + 30, 60, 90 meters). Most of the time this way of
generation provides smooth transition.

### Velocity, acceleration and jerk restrictions

Velocity restrictions are supported by cost function. Better velocities have
lower cost.

Total acceleration restriction is somewhat supported by lane construction rules. 
Distance between adjacent points is calculated using maximum possible speed change that doesn't
violate acceleration restriction. Having said that it is worth mentioning that
spline and frenet/cartesian conversions bring some error and all the calculations become approximate.
The only configuration that I observed acceleration violation in, was the case of fast target
 lane changes in case of sudden or simultaneous lane change by other car.
To avoid this situation I introduced suppression of fast lane changes.

Normal and tangent accelerations are not controlled for simplicity of costs
compositions. I never observed this kind of violations in this implementation. Although theoretically
construction of lane changing trajectory violates normal acceleration, in practice it doesn't happen,
may be thanks to spline smoothing or because simulator considers only averaged values.

Jerk is not controlled because I never observed its violation in this implementation.

### A word on jerk minimization

I tried to use quintic polynomial for jerk minimization but I could not get path smooth enough
probably because of frenet/cartesian conversion. Though in general trajectories looked great and provided
ability to analytically find maximum/minimum accelerations.

### Costs

The following costs are calculated for each trajectory.

* Velocity cost
* Distance to next car in the target lane cost
* Distance to previous car in the target lane cost
* Change lane cost

Velocity cost is a linear piecewise function with 0.7 cost for zero velocity,
0 for 45 mph and 1 for 50 mph. So it supports 45 mph speed.

Next car cost is applied to the closest car that has s coordinate more than the ego car
and whose s coordinate is closer than 15 meters to the end of trajectory. In other words
it indicates that something is in front of the ego car. The value is linear,
it starts as 0 at 15 meters ahead of the end of the trajectory and becomes 1 close to the
end of trajectory so it could override speed cost. In case car is right in front of
the ego car the cost may be about 4.

Previous car cost is applied to the closest car in another lane that has s coordinate less than the ego car
and whose s coordinate is less than 15 meters behind the ego car. In other words this
cost indicates that there is something behind the ego car in another lane and prevents it
from changing lanes. The value is linear, starts as 0 at 15 meters behind and reaches 4 when the car is right behind
the ego car.

Change lane cost has two modes. Most of the time it brings some small cost to prevent 
random lane change. Sometimes when no other vehicles are present and only velocity cost
is not zero, the cost is almost equal for all the lanes (here normal 
acceleration cost could probably help) which causes random lane changes. So small 
change lane cost should prevent random lane changes. 
But sometimes in emergency cases, e.g. when a car appeared
right in front of the ego car, the overall cost is large and this small weight is not enough.
So right after lane change this cost becomes huge for 1 second to prevent fast random lane changes.

In general lane change is driven by next car distance cost of the current lane
and previous/next car distance costs of all other lanes. Velocity almost doesn't affect
lane change because it is almost equal for all the generated trajectories. So probably
because distance costs has approximately the same magnitude this should prevent
lane change to lane where vehicles are closer to the ego car than on the current lane.
But fore better results we probably should consider speed, which is not implemented here.


### Behaviour

Car's behaviour is driven by costs described above. 

* In case there is no vehicle in front of the car it goes at full speed
* In case a vehicle is detected in front of the car it slows down
* In case a vehicle is detected in front of the car it changes the lane if possible
* Right after lane change it stays on the lane for some time.
* In case there vehicles in front of the car or behind the car in other lanes,
then lane change is not performed 

### Result

I was able to drive 20 miles at approximately 45 mph without an accident.

Car navigates making simple decisions. Sometimes when another car unexpectedly change
lanes and suddenly appears in front of the ego car it changes the lane which looks risky.
May be it makes sense to introduce a dedicated state for emergency breaking and break
in case of sudden drastic increase of weights instead of finding better trajectory.


## Reflections

* This is a simple solution, I assume that the real solution would be much more complicated. 

* The hardest thing is cost construction. It is quite hard to incorporate all the variables
into one number.

* Not clear how to use jerk minimization for this set up.

* Speaking of this implementation it probably makes sense to add a dedicated state
for emergency braking situation.

---
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
    
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
