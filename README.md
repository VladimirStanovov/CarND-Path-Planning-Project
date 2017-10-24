# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3. The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Go to the build directory: `cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Project pipeline

#### To succesfully perform path planning, the following steps should be completed at every iteration of the algorithm:
1. Determine current state, lane number and velocity,
2. Get the successor states for current state,
3. Create a trajectory for each state,
4. Calculate cost for each trajectory,
5. Change state to the state with the lowest trajectory cost.

#### The finite automata is used to define the successor states. In my solution, there were 5 states:
1. KL - keep (current) lane. The vehicle will do maximum safe acceleration in its lane,
2. PLCL - prepare lane change left. Vehicle changes to this state in case if the front vehicle is moving too slow. Changing to this state does not mean lane change, but the control algorithm calculates cost for the LCL state.
3. PLCR - prepare lane change right. Same as PLCL, but looks at the right lane,
4. LCL - lane change left - executes left lane change immediately,
5. LCR - lane change right - executes right lane change immediately.

#### The states have the following successors:

KL: PLCL, PLCR

PLCL: KL, PLCL, PLCR, LCL

PLCR: KL, PLCL, PLCR, LCR

LCL: KL

LCR: KL


Note that I added transition from PLCL to PLCR and back, as it seems important, for example, in case if one of the adjacent lanes was busy and less promising for a lane change, but then the situation changes and it becomes more promising than the opposite.

### Trajectory generation

Trajectory generation considered three main cases, i.e. KL, PLC*, LC*. For each of the cases, the lane number and the desired velocity was calculated (which is different from reference velocity). Reference velocity is used at the last step of the trajectory generation, whereas the desired velocity is the one to which reference velocity is adjusted.
In the case of "KL" state, the following steps are performed:

1. Set desired velocity to speed limit,
2. For each vehicle in the sensor fusion array:
3. Compare the vehicle's d position with current lane
4. If the vehicle is in the same lane:
5. Check its s position, and if it is in front, adjust desired speed to its velocity.
