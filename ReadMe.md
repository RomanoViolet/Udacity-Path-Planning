— title: Udacity Path Planning Project abstract: | Implementation of the
"Path Planning" project towards the requirements of completing Semester
3 of the Udacity Self Driving Car Nanodegree program. —

# Context

This project implements the “Path Planning” project required in Semester
3 of the Udacity’s [“Self Driving Car NanoDegree
Program”](https://de.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)

# Problem Statement

**Given:**

1.  A simulated Ego Vehicle;

2.  A simulated test track, with three lanes in each direction, and
    simulated traffic on all lanes, and in each direction;

3.  A fixed number of waypoints along the test track:

4.  A trajectory control logic built into the ego vehicle

**Compute:**

The path to be travelled by the ego vehicle along the track

**Assume:**

Perfect trajectory control in the ego vehicle.

**Constraints:**

1.  No collision with any other vehicle;

2.  Maintain a minimum average speed of 40 km/hr. This constraint is
    enforced by the simulator.

3.  The ego vehicle must change lanes at least once during the entire
    journey along the track.

4.  go vehicle must travel according to right-hand-drive traffic rules
    (e.g., those in Germany) and obey all traffic laws.
    
    1.  Ego vehicle must not take too long to cross lanes (contraint
        enforced by the simulator)

# Solution Overview

The approach involves the following main steps:

1.  `updateNonEgoVehicleData`: Updating the positions of all Non-Ego
    vehicles sensed by the Ego vehicle at the current time-step;

2.  `updateEgoVehicleData`: Update the position, orientation, and
    velocity of the Ego vehicle to the current time-step;

3.  `provideTrajectory`: Based on the predicted positions of the Non-Ego
    vehicles over the planning horizon, compute a new trajectory of the
    Ego vehicle subject to the constraints listed in the problem
    description;

4.  `getCoordinatePairs`: Compute a sequence of waypoints that the Ego
    vehicle must follow based on the computed trajectory.

<!-- end list -->

``` c++
pathPlanner.updateNonEgoVehicleData(sensor_fusion);

// update ego-vehicle data
pathPlanner.updateEgoVehicleData(EgoVehicleData, false);

// choose an appropriate trajectory
optimalTrajectory = pathPlanner.provideTrajectory();

// extract the vector of coordinates
coordinatePairs = optimalTrajectory.getCoordinatePairs();
    
```
