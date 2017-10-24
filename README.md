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
#### In the case of "KL" state, the following steps are performed:

1. Set desired velocity to speed limit,
2. For each vehicle in the sensor fusion array:
3. Compare the vehicle's d position with current lane
4. If the vehicle is in the same lane:
5. Check its s position in the future, and if it is in front, adjust desired speed to its velocity.

The desired velocity was updated with the following equaion: desired_vel = (1-dist_coeff) * max_speed + dist_coeff * front_speed.
This allows gradual change of desired velocity between the max speed and the speed of the vehicle in front.
The dist_coeff if the coefficient in range [0,1], which shows how the velocity of the ego vehicle should be adjusted according to the distance from the front vehicle. The dist_coeff is calculated as follows: dist_coeff = 1/(1+exp((front_dist-16) * 0.4)). At the distance of 16 meters and lower the desired speed will be set to the front vehicle speed.
After calculating the desired velocity, we set the project velocity variable same as desired velocity. The project velocity meaning is described lower.

#### In the case of "PLC*" state, the following steps are performed:

1. Calculate desired velocity same as in "KL" state (we have to stay in our lane)
2. Calculate the "project velocity" which is the velocity that the vehicle will have if it will change into specific lane.

The project velocity is calculated in a similar manner to the desired velocity. The only change is the lane number - we pretend that we are in a different lane and see (only forward) if we can go faster there.

#### In case of "LC*" state, the following steps are performed:

1. Change lane number
2. Calculate desired speed and project speed in the other lane. The project speed is set to be the same as desired speed.
3. Check for collisions. If there is at least one vehicle in the lane we are going to change to in a range [-10,10] meters, mark the lane change as "unsafe". In this case the project velocity is set to 0, which results in maximum cost.

#### Calculating path points

After we have set our lane and desired velocity, we need to generate the path. Path generation actually uses the code from the project walkthrough with a couple of changes. The pipeline is as follows:

1. Get all the available points of the previous path from the simulator
2. Define 3 next waypoints at the distance of 50, 100 and 150 meters (larger distances make smoother path)
3. Use current vehicle positions and waypoints to generate a spline interpolation
4. Get new path points from the interpolation with the target_x set to 60 meters
5. For all the new waypoints: update reference velocity according to the difference between desired and reference velocity, as well as acceleration limit (0.224)

The generated path, as well as desired, project and reference velocity and lane number are returned from the trajectory generation function.

#### Calculating cost

The cost function relies on the lane number and project speed, as well as previous lane number, max speed and a number of constants. It is calculated as follows:

1. If the lane is changed, cost = cost + 0.5
2. If the new lane is invalid, cost = cost + 100
3. If the state is PLC*, cost = cost + max_speed + 1.5 - project_velocity
4. If the state is not PLC*, cost = cost + max_speed - project_velocity

The cost functuin is constructed so that keeping lane in the best choise if there are no obstacels, PLC* states are activated only if the project speed falls (we see that we have to brake), and the lane change is less preferable than keeping lane, unless the project speed in KL state is too small.

#### Additional vehicle logic

One additional feature is that if we perform a lane change, we should at least stay in this lane for some perioud of time (say, make at least 100 meters in this lane), because changing lanes instantly from one to another does not look like a nice, predictable driving behaviour for other drivers. So, after a lane change, further changes are blocked for the next 100m of the track.

#### Results and discussion

The resulting behaviour of the descirbed path planner appears to fulfill all requirements, i.e. the vehicle is capable of driving for up to 50 miles around the track without incidents. However, the weak part remains to be the collision detection method: in some cases, in a traffic jam, if front vehicles start accelerating and decelerating, as well as vehicle in other lanes, the ego vehicle may decide to make a lane change which could be unsafe due to the fact that the vehicle in the other lane would decide to accelerate. This may result in a collision, which can be avoided by setting the safe range for LC* states to more then 10 meters. However, this increase also results into more "fearful" lane changing, so that the ego vehicle does not perform a lane change, when it is obviously safe to do that.
Another disadvantage is that double lane changes are not in the model. Sometimes, it is better to make a double change to go faster, and the path planner won't see such opportunity.
