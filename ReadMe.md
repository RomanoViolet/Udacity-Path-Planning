— title: Udacity Path Planning Project abstract: | Implementation of the
"Path Planning" project towards the requirements of completing Semester
3 of the Udacity Self Driving Car Nanodegree program. —

# Context

This project implements the “Path Planning” project required in Semester
3 of the Udacity’s [“Self Driving Car NanoDegree
Program”](https://de.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)

# Tools and Scripts

## uWebSocketIO

*Ad-verbatim from Udacity’s Instructions:*

**uWebSocketIO Starter Guide**

All of the projects in Term 2 and some in Term 3 involve using an open
source package called uWebSocketIO. This package facilitates the same
connection between the simulator and code that was used in the Term 1
Behavioral Cloning Project, but now with C++. The package does this by
setting up a web socket server connection from the C++ program to the
simulator, which acts as the host. In the project repository there are
two scripts for installing uWebSocketIO - one for Linux and the other
for macOS.

Note: Only uWebSocketIO branch e94b6e1, which the scripts reference, is
compatible with the package installation. Linux Installation:

From the project repository directory run the script: install-ubuntu.sh

## Term2 Simulator

Download from [Term3
Simulator](https://github.com/udacity/CarND-Path-Planning-Project).

## Building This Project

Execute the following in the `$root` directory of this project: ` mkdir
build cd build cmake .. make `

## Running This Project

Execute the following in the `$root` directory of this project: ` cd
build ./path_planning `

Thereafter, start the Term3 simulator, and choose the "Project 1: Path
Planning"

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

All steps are available as method `PathPlanner C++` class. The
`main.cpp` creates an instance `pathPlanner` of the class `PathPlanner`
and invokes the methods appropriately to solve the path planning
problem:

``` c++
pathPlanner.updateNonEgoVehicleData(sensor_fusion);

// update ego-vehicle data
pathPlanner.updateEgoVehicleData(EgoVehicleData, false);

// choose an appropriate trajectory
optimalTrajectory = pathPlanner.provideTrajectory();

// extract the vector of coordinates
coordinatePairs = optimalTrajectory.getCoordinatePairs();
    
```

# Solution Details

The `updateNonEgoVehicleData` and `updateEgoVehicleData` methods are
relatively straightforward. The method `updateNonEgoVehicleData` reads
the information from `sensor_fusion` function provided by the simulator,
and updates the position, velocity, and orientation of each non-ego
vehicle reported by `sensor_fusion`.

The method `updateEgoVehicleData` updates the internal knowledge of the
position, velocity, and orientation of the ego vehicle based on sensor
data.

The method `provideTrajectory` is composed of two private methods:

1.  `getLaneChangeSuggestion()`: Provides an overall cost of execution a
    change of lanes based on the costs associated to (a) staying in
    lane, (b) changing to the left lane, and (c) changing to the right
    lane.

# Assumptions

For the methods described below, the following information is assumed
given:

The cost \(c\) of a lane-change maneuver is a number in the range \[0,
1\]. That is: \[0 \leq c \leq 1 (highest \,cost)\]

Furthermore, it is given that the track has \(n \geq 1\) lanes in the
intended direction of travel, with lane \(n = 1\) being the left-most
lane. The final \(n = k\) lane which the ego vehicle must be in at the
end of the track is also provided.

The track is assumed to be \(l\) meters long.

A distance `returnToLaneBuffer` is used to force the ego vehicle to move
into lane \(l = k\) after travelling a maximum total distance
\(l -  \texttt{returnToLaneBuffer}\).

## Method `costOfLaneChangeLeft`

Assuming The decision to change the lane to the left is based on the
following criteria:

1.  The cost is set to \(c = 1\) if the ego is already traveling in the
    left most lane, i.e., \(n = 1\)

2.  The cost is set to \(c = 1\) during during the last
    `returnToLaneBuffer` meters from the end of the track, provided the
    ego vehicle is already in the intended final lane.

3.  For all other cases, the feasibility of staying in lane
    \(n' = n - 1\) is calculated using the method `costForLaneChange`.
    If this step yields the lowest cost, then the change to lane
    \(n' = n - 1\) is completed. It is assumed that if it is feasible to
    stay in the lane \(n' = n -1\) then the change of lanes
    \(n \rightarrow n'\) is also always feasible. However, this
    assumption may not hold for realistic driving scenarios.

All costs are computed by a common private method
`costForLaneChange`which operates are follows:

## Method `costForLaneChange`

First, the planning horizon in computed: a window of time starting at
the current instant, and the duration is set to the length of time into
the future for which trajectories are to be generated. Longer horizons
are not recommended since these reduce the agility of the ego vehicle to
respond to changes in the environment (e.g., new traffic pattern). On
the other hand, very short planning horizons are also not
computationally efficient (e.g., would need to plan multiple times to
execute one lane
change)

|                                                  |                                                                                                                                                |
| :----------------------------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------- |
| Symbol                                           | Semantic                                                                                                                                       |
| \(J\)                                            | Maximum Jerk limit, provided at design time.                                                                                                   |
|                                                  |                                                                                                                                                |
| \(A\)                                            | Maximum Acceleration limit, provided at design time.                                                                                           |
|                                                  |                                                                                                                                                |
| \(t\)                                            | Time at the beginning of the planning phase                                                                                                    |
|                                                  |                                                                                                                                                |
| \(\triangle t\)                                  | Length of the planning horizon                                                                                                                 |
|                                                  |                                                                                                                                                |
| \(v^{\text{ego}}(t)\)                            | Speed of the Ego vehicle at the beginning of the planning phase                                                                                |
|                                                  |                                                                                                                                                |
| \(\left|{\triangle v}^{\text{ego}}\right|\)      | Maximum change in the velocity of the ego vehicle by the end of the current planning phase                                                     |
|                                                  |                                                                                                                                                |
| \(v^{\text{ego}}_{\text{max}}( t+\triangle t )\) | Maximum possible speed of the ego vehicle at the end of the current planning phase subject to performance restrictions, and traffic conditions |
|                                                  |                                                                                                                                                |
| \(v^{\text{ego}}_{\text{min}}( t+\triangle t )\) | Minimum possible speed of the ego vehicle at the end of the current planning phase subject to performance restrictions, and traffic conditions |
|                                                  |                                                                                                                                                |
|                                                  |                                                                                                                                                |

Second, based on provided jerk and acceleration limits, the maximum
change of velocity \({\triangle v}^{\text{ego}}\) of the ego vehicle
within the time-horizon \(\triangle t\) being considered is computed.
That
is:

\[\left| v^{\text{ego}} ( t+\triangle t ) - v^{\text{ego}}(t) \right|   \leq \left| {\triangle v}^{\text{ego}} \right|\]

where:

\[\left| {\triangle v}^{\text{ego}} \right| = \text{min}\left\{ \intop\intop_{t=0}^{\triangle t}Jdt.dt,\intop_{t=0}^{\triangle t}Adt\right\}\]

where \(t\) is the time instant at the beginning of the planning phase.

In case there is no in-lane leading traffic in front of the ego vehicle,
the planner computes a trajectory which requires the ego vehicle to
speed up, upto the maximum allowed speed. In case, a leading in-lane
vehicle is detected, a maximum ego velocity
\(v^{\text{ego}}_{\text{max}}( t+\triangle t )\) necessary to avoid
rear-ending the leading vehicle is computed.
with:

\[v^{\text{ego}}_{\text{max}}( t+\triangle t ) \leq v^{\text{ego}}(t) +  \left| {\triangle v}^{\text{ego}} \right|\]

In case there a trailing in-lane non-ego vehicle, a minimum ego velocity
\(v^{\text{ego}}_{\text{min}}( t+\triangle t )\) in order to avoid being
rear-ended by the trailing vehicle is computed,
where:

\[v^{\text{ego}}_{\text{min}}( t+\triangle t ) \geq v^{\text{ego}}(t) -  \left| {\triangle v}^{\text{ego}} \right|\]

Obviously:
\[v^{\text{ego}}_{\text{min}}( t+\triangle t ) > v^{\text{ego}}_{\text{max}}( t+\triangle t ) \implies \text{Change Lane}\]

Under the condition
\(v^{\text{ego}}_{\text{min}}( t+\triangle t ) > v^{\text{ego}}_{\text{max}}( t+\triangle t )\),
the velocity associated with the planned trajectory over the planning
horizon is computed
as:

\[v^{\text{ego}}( t+\triangle t ) = \frac{v^{\text{ego}}_{\text{min}}( t+\triangle t ) + v^{\text{ego}}_{\text{max}}( t+\triangle t )}{2}\]

The \(v^{\text{ego}}\) will be followed in case a change of lanes is
also not possible.

In case that
\(v^{\text{ego}}_{\text{min}}( t+\triangle t ) \leq v^{\text{ego}}_{\text{max}}( t+\triangle t )\),
the ego velocity
is:

\[v^{\text{ego}}( t+\triangle t ) =  v^{\text{ego}}_{\text{max}}( t+\triangle t ), \text{if }  v^{\text{ego}}_{\text{min}}( t+\triangle t ) \leq v^{\text{ego}}_{\text{max}}( t+\triangle t )\]
