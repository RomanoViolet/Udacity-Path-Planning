/*
 * PathPlanner.hpp
 *
 *  Created on: Mar 24, 2018
 *      Author: dumbledore
 */

#ifndef PATHPLANNER_HPP_
#define PATHPLANNER_HPP_

#include <vector>

#include "../src/Commons.hpp"
#include "../src/Curve.hpp"
#include "../src/Vehicle.hpp"
#include "../src/Trajectory.hpp"
#include "../src/WayPoint.hpp"


class PathPlanner {
 public:

  PathPlanner();

  virtual ~PathPlanner();



  void updateNonEgoVehicleData(const std::vector<std::vector<double>>& sensor_fusion);

  void updateEgoVehicleData(const std::vector<double>& EgoVehicleData, bool override);

  void setIntendedLane(unsigned lane);

  // method to choose a trajectory from a set of computed trajectories
  Trajectory provideTrajectory();

  // get length of plan in seconds
  double getPathPlanDuration();

  // Sample time
  double getSampleTime();

  // convert d-values to lane numbers
  unsigned dToLaneNumber(double d);

 private:


  // the number of elemenets used to desribe one waypoint
  const unsigned ElementsPerWayPoint = 5;

  // The sample time of the simulator in seconds
  const double SampleTime = 20E-3;

  // Width of each lane in meters
  const unsigned LaneWidth = 4;

  // Distance to be maintained between the Ego and the Non-Ego Vehicle
  const double bufferDistance = 35.0;  // meters

  // Number of steps to compute for into the future
  // ~ 1.8 second horizon @20ms per point.
  const unsigned nSteps = 50;

  // Maximum allowed acceleration
  const double maxAllowedAcceleration = 3; //m per squared seconds

  // Maximum Cost Value
  const double maximumCost = 1.0;

  // The number of lanes, starting at 1.
  const unsigned nLanes = 3;

  // The acceptable jerk limit
  const double JerkLimit = 7.0;

  // maximum speed the Ego can take: 72KPH ~ 44.7 MPH
  const double maxSpeedAllowed = (72.0*1000.0)/(60.0 * 60.0);

  // minimum velocity before lane change is considered
  const double minVelocityForLaneChange = 5.0;

  const double AccelerationLimit = 9.0;

  // compute the next required lane change
  // return 1: Suggestion
  // return 2: Final Velocity useful for computing trajectory
  std::pair<Commons::LaneChange, double> getLaneChangeSuggestion();

  // computes the cost of lane change to the left
  // return 1: cost
  // return 2: finalVelocity
  std::pair<double, double> costOfLaneChangeLeft();

  // computes the cost of lane chagne to the right
  // return 1: cost
  // return 2: finalVelocity
  std::pair<double, double> costOfLaneChangeRight();

  // computes the cost of remaining in the lane
  // return 1: cost
  // return 2: finalVelocity
  std::pair<double, double> costOfLaneRemainingInLane();

  // returns the non-ego vehicle ahead of ego vehicle.
  Vehicle getCarInFront(unsigned lane);

  // returns the non-ego vehicle behind the ego vehicle.
  Vehicle getCarAtBack(unsigned lane);

  // returns the cost to change to a specific lane
  // return 1: cost
  // return 2: finalVelocity
  std::pair<double, double> costForLaneChange(unsigned newLane);

  // compute the trajectory for the selected lane
  //@param 1: suggestion
  //@param 2: finalVelocity
  // returns: vector of s, d values.
  Trajectory computeTrajectory(std::pair<Commons::LaneChange, double>);

  // final lane the Ego must try to achieve
  const unsigned finalLane = 2;

  // Size of the track
  const double sizeOfTrack = 6945.554;

  // Buffer from end of track after which lane changes are prohibited
  // The simulator complains at 3mile mark (~65% track length)
  const double returnToLaneBuffer = (0.5 * sizeOfTrack);

  // Cost threshold above which the Ego attempts to switch lanes
  const double laneChangeThreshold = 0.35;

  // how much longer to plan a lane change, compared to going sraight; Useful to reducing jerks
  const double scaleForLaneChange = 2.2;

  // internal method to get the details of the non ego vehicle in front of the ego vehicle
  Vehicle getLeadingNonEgoVehicle();

  // internal method to get the details of the non ego vehicle trailing the ego vehicle
  Vehicle getTrailingNonEgoVehicle();



  // convert lane numbers to d-values
  double laneNumberToD(unsigned lane);



  // computes the Jerk Optimal Curve
  std::vector<double> getJerkOptimalCurve(std::vector<double> start, std::vector<double> end, double span, double sampleTime);


// all Non Ego Vehicles in the same directions as the Ego vehicle
  std::vector<Vehicle> allNonEgoVehicles;

  // Location data for the ego vehicle
  Vehicle egoVehicle;

  // intended driving Lane at the start
  unsigned intendedLane;


};

#endif /* PATHPLANNER_HPP_ */
