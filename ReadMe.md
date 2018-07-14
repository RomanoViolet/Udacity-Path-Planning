Udacity Path Planning Project

Context
=======

This project implements the “Path Planning” project required in Semester 3 of the Udacity’s “[Self Driving Car NanoDegree Program](https://de.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)”

Problem Statement
=================

Given

1.  A simulated Ego Vehicle;
2.  A simulated test track, with three lanes in each direction, and simulated traffic on all lanes, and in each direction;
3.  A fixed number of waypoints along the test track:
4.  A trajectory control logic built into the ego vehicle

Compute

1.  The path to be travelled by the ego vehicle along the track

Assume:

1.  Perfect trajectory control in the ego vehicle.

Constraints:

1.  No collision with any other vehicle;
2.  Maintain a minimum average speed of 40 km/hr. This contraint is enforced by the simulator.
3.  The ego vehicle must change lanes at least once during the entire journey along the track.
4.  Ego vehicle must travel according to right-hand-drive traffic rules (e.g., those in Germany) and obey all traffic laws.

    1.  Ego vehicle must not take too long to cross lanes (contraint enforced by the simulator)

Summary of Solution

**Algorithm Used:** A\*

**Heuristics:** Euclidean Measure

Wallclock Time:~7.16s. Measured only for computing all paths. Details below.

RAM Usage:Depends on the architecture of the target processor.

A summary of computation is provided at the terminal like so:

Note

1.  The time reported in the summary is the ‘nominal-time’ as mentioned in the problem statement (i.e., reported travel time is normalized to ‘island-time’)
2.  The distance is reported in terms of ‘nominal-units’ as mentioned in the problem statement.
3.  The reported computation time is for a tunable parameter “weight” set to 1.0 (from rover’s initial location to the location of the bachelor, and 10.0 for the path between the bachelor’s location to the wedding). See the section of Heuristics for more details. Weights greater than 1.0 leads to increasingly suboptimal paths (i.e., A\* optimality guarantee is violated increasingly), but decrease computation time.
4.  The final path also depends on the regions prohibited from travel, as decided by the masks (e.g., OF\_WATER\_BASIN) as provided in the setup.

Design Overview

**Entry Point for Execution:** main.cpp

Step 1

Loading of Elevation and Override data is performed in parallel in order to reduce the overall computation time:

Step 2

The following paths are computed in parallel:

1.  From Rover’s initial location to the location of the bachelor;
2.  From the location of the bachelor to the location of the wedding

Step 3

The overall time to compute the paths is estimated by a call to chrono library like so:

(before launching the first thread that computes the path from the Rover’s initial location to the location of the bachelor.

The overall time required to compute the paths is then deduced like so:

Detailed Design

Attempt was made to keep the overall solution as modular as possible.

Specifically:

1.  The call to the solver is made in “main.cpp” by launching the “SearchPath” function with the necessary paramters. See “SearchPath.hpp” for more details about parameters.
2.  The “SearchPath” itself uses the class “Astar” for computing the paths. See “AStar.hpp” for details about the class.

Therefore, the “SearchPath” function wrapper encapsulates the core A\* algorithm, enabling simultaneous searches for multiple paths, as implemented in this project.

Overview of Software Components

1.  ***Wrapper “SearchPath” ***encapsulates the A\* path search algorithm, and initializes a Astar object with the necessary parameters. See the “SearchPath.hpp” for details.
2.  Class AStar: Provides the core functions required to perform path search using A\* heuristic. The choice of A\* was motivated by Sebastian Thrun’s lecture in the third semester of “Self-Driving Car Nano-Degree Program”, and by the fact that the algorithm was successfully deployed on “Junior” autonomous vehicle from Stanford University in DARPA’s Urban Challenge. See “AStar.hpp” for details on the class itself.
3.  Class “Node”: Provides a convenient abstraction to express the location on the map. Currently supports only two coordinates &lt;x,y&gt;. Provides necessary operators (e.g., equality) to compare locations, as required by the A\* algorithm.
4.  Class Heuristics: Provides functions that may be used to compute the heuristic (“h”) as required by the A\* algorithm. Currently, only Euclidean Distance measure is provided. All algorithms are required to be optimistic (i.e., never overestimate the true distance between two points) by A\*. Therefore, it is possible to just recompute paths using a different heuristic cost function without requiring any modifications to either the wrapper “SearchPath”, or the core class “Astar”.
5.  Class CostGrid: Templated class provding matrix-like abstraction, together with convenient access functions. Used by “Astar” to store various data.
6.  Various Unit Tests as part of the development process.

Execution

1.  **Parallelism: **Search for paths from Rover’s initial position to the location of the bachelor, and from the location of the bachelor to the location of the wedding are computed in parallel.
2.  **Parallelism: **Search for all explorable neighbors are also in parallel:

Tunables

1.  Weight: A tunable parameter “weight” is introduced which forces the path-search to possibly select less favorable paths but with reduced computation time. The provided implementation has:

    1.  Weight = 1.0, for estimating the path between Rover’s initial position to the location of the bachelor;
    2.  Weight = 10.0, for estimating the path between the location of the bachelor and the wedding.
    3.  With these weights, the computation time is 65.61 seconds, compared to ~7 seconds when weights are kept as 100.0 for both path searches.

1.  Mask: The choice of regions where the Rover is prohibited can be supplied by setting the mask like so in main.cpp:

Heuristics

1.  Distance Estimation: Once the final path is computed as a sequence of coordinates, the total distance is estimated by adding the Euclidean distance between two successive coordinates, and accumulating the distance.
2.  Time Estimation:The following heuristic is followed:

    1.  For movement between two points on the map with no net change in elevation: t = d (island-seconds), where d is the island-distance travelled.
    2.  For movement between two points on the map with a net change in elevation of *h, *t = d/(1-sin(θ)), where θ = tan<sup>-1</sup>(h/d). This heuristic assumes that the force generated by the Rover’s engine remains constant throughout the journey. The scaling factor of  1-sin(θ) is based on the notion that effective force when going up an elevation with grade θ is F.sin(θ), where F is the constant force generated by the Rover’s engines.

Notes

1.  By intent,the time required to transform the path computed by AStar class to the format suitable for use with the visualizer is not counted, as this step does not form a core of the path-search. The provided transformation steps in “main.cpp” are mainly done to reduce the overall time to print the final bitmap.
2.  The A\* algorithm is described in Semester 3 of “Self Driving Car Nano-Degree Program”.

Final Result

(Provided for reference. Generated by the executable)

Related Summary:

Trip Distance Time

Rover to Bachelor 2508.59 3240.93

Bachelor to Wedding 1760.88 2287.81

Total Wallclock Time Required for Computation: 221.87 s.

Corresponding weights:

1. weight: 1.0: Rover’s Initial Position to the Bachelor’s Location

2. weight: 10.0: Bachelor’s location to the location of the wedding.
