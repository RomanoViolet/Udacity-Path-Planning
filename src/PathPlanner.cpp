/*
 * PathPlanner.cpp
 *
 *  Created on: Mar 24, 2018
 *      Author: dumbledore
 */

#include <cstddef>
#include <limits>
#include <cmath>
#include <cassert>
#include <iostream>
#include "PathPlanner.hpp"
#include "Eigen-3.3/Dense"
//TODO: Possibly remove dx and dy waypoint data.

PathPlanner::PathPlanner() {

  // initialization conditions.
  this->intendedLane = 0;

}

PathPlanner::~PathPlanner() {

}


// get length of plan in seconds
double PathPlanner::getPathPlanDuration()
{
  return(this->SampleTime * this->nSteps);
}


// Sample time
double PathPlanner::getSampleTime()
{
  return(this->SampleTime);
}

void PathPlanner::setIntendedLane(unsigned lane)
{
  // Lanes Numbers: 1,2, or 3
  this->intendedLane = lane;
}

unsigned PathPlanner::dToLaneNumber(double d)
{
  //[0:4) --> 1
  //[4:8) --> 2
  //[8:12) --> 3
  return(1 + std::floor(d / this->LaneWidth));
}


double PathPlanner::laneNumberToD(unsigned lane)
{
    // 1 -> 2
    // 2 -> 6
    // 3 -> 10

  return((static_cast<double>(lane -1) * 4.0) + 2.0);
}




void PathPlanner::updateNonEgoVehicleData(const std::vector<std::vector<double>>& sensor_fusion)
{
  this->allNonEgoVehicles.clear();


  for (int i=0; i<sensor_fusion.size(); i++)
  {
    double XCoordinate_MapReference = sensor_fusion[i][1];
    double YCoordinate_MapReference = sensor_fusion[i][2];
    double Velocity = std::sqrt( std::pow(sensor_fusion[i][3], 2.0) + std::pow(sensor_fusion[i][4], 2.0) );
    double SCoordinate_LaneReference = sensor_fusion[i][5];
    double DCoordinate_LaneReference = sensor_fusion[i][6];
    unsigned Lane = this->dToLaneNumber(DCoordinate_LaneReference);

    this->allNonEgoVehicles.push_back(Vehicle(XCoordinate_MapReference,
                                                YCoordinate_MapReference,
                                                Velocity,
                                                SCoordinate_LaneReference,
                                                Lane,
                                                DCoordinate_LaneReference));

  }



}


//EgoVehicleData = {car_x, car_y, car_speed, car_s, car_d};
void PathPlanner::updateEgoVehicleData(
    const std::vector<double>& EgoVehicleData, bool override) {

// temporaries
  double XCoordinate_MapReference   = EgoVehicleData[0];
  double YCoordinate_MapReference   = EgoVehicleData[1];
  double Velocity                   = EgoVehicleData[2];
  double SCoordinate_LaneReference  = EgoVehicleData[3];
  unsigned Lane  = this->dToLaneNumber(EgoVehicleData[4]);
  //double DCoordinate_LaneReference  = EgoVehicleData[4];
  double DCoordinate_LaneReference  = this->laneNumberToD(Lane);

  if(! override)
  {
    // override with internally stored state
    Velocity = this->egoVehicle.getVelocity();
    SCoordinate_LaneReference = this->egoVehicle.getSCoordinatee();
    DCoordinate_LaneReference = this->egoVehicle.getDCoordinatee();
    Lane = this->dToLaneNumber(DCoordinate_LaneReference);
  }



  this->egoVehicle = Vehicle(XCoordinate_MapReference,
                             YCoordinate_MapReference,
                             Velocity,
                             SCoordinate_LaneReference,
                             Lane,
                             DCoordinate_LaneReference
                             );
}  //void PathPlanner::updateEgoVehicleData


Vehicle PathPlanner::getCarInFront(unsigned lane)
{
  Vehicle carInFront;
  double sTemp = std::numeric_limits<double>::infinity();

  for (Vehicle thisNonEgoVehicle : this->allNonEgoVehicles) {
    if (thisNonEgoVehicle.getLane() == lane) {
      if ((thisNonEgoVehicle.getSCoordinatee() > this->egoVehicle.getSCoordinatee()) && (thisNonEgoVehicle.getSCoordinatee() < sTemp))
      {
        carInFront = thisNonEgoVehicle;
        sTemp = thisNonEgoVehicle.getSCoordinatee();
      }

    }
  }

  return(carInFront);
}


Vehicle PathPlanner::getCarAtBack(unsigned lane)
{
  Vehicle carAtBack;
  double sTemp = -std::numeric_limits<double>::infinity();

  for (Vehicle thisNonEgoVehicle : this->allNonEgoVehicles) {
    if (thisNonEgoVehicle.getLane() == lane) {
      if ((thisNonEgoVehicle.getSCoordinatee() <= this->egoVehicle.getSCoordinatee()) && (thisNonEgoVehicle.getSCoordinatee() > sTemp))
      {
        carAtBack = thisNonEgoVehicle;
        sTemp = thisNonEgoVehicle.getSCoordinatee();
      }

    }
  }

  return(carAtBack);
}


std::pair<double, double> PathPlanner::costForLaneChange(unsigned newLane)
{

  double cost = -1.0;

  // temporaries
  double finalVelocity = -1.0;
  double distanceToLeadingNonEgoVehicle = 0;
  double velocityOfLeadingNonEgoVehicle = -1;
  double distanceToTrailingNonEgoVehicle = 0;
  double velocityOfTrailingNonEgoVehicle = -1;
  double minEgoVelocityToAvoidBeingRearEnded = -1;
  double maxEgoVelocityToAvoidRearEnding = -1;

  double currentEgoVelocity = this->egoVehicle.getVelocity();
  double planTime;

  if(newLane != this->egoVehicle.getLane())
  {
    planTime = this->nSteps * this->SampleTime * this->scaleForLaneChange;
  }
  else
  {
    planTime = this->nSteps * this->SampleTime;
  }



  double maximumChangeInVelocity = std::min(this->AccelerationLimit * planTime, this->JerkLimit * planTime * planTime * 0.5);

  // get the leading car data
  Vehicle carInFront = getCarInFront(newLane);

  if(!carInFront.isEmpty())
  {
    // a vehicle is ahead
    distanceToLeadingNonEgoVehicle = carInFront.getSCoordinatee() - this->egoVehicle.getSCoordinatee();
    velocityOfLeadingNonEgoVehicle = carInFront.getVelocity();
    maxEgoVelocityToAvoidRearEnding = (distanceToLeadingNonEgoVehicle + (velocityOfLeadingNonEgoVehicle * planTime) - this->bufferDistance) / planTime;
    //std::cout << "Vehicle Ahead in Lane" << newLane <<  "Ego in " << this->egoVehicle.getLane() << std::endl;
  }
  else
  {
    // no leading non-ego vehicle
    //std::cout << "No Vehicle Ahead in Lane " << newLane << "Ego in " << this->egoVehicle.getLane() << std::endl;
    maxEgoVelocityToAvoidRearEnding = this->maxSpeedAllowed;
  }

  // get the trailing car data
  Vehicle carAtBack = getCarAtBack(newLane);
  if(!carAtBack.isEmpty())
  {
    // trailing vehicle exists
    distanceToTrailingNonEgoVehicle = this->egoVehicle.getSCoordinatee() - carAtBack.getSCoordinatee();
    velocityOfTrailingNonEgoVehicle = carAtBack.getVelocity();

    // the minimum ego velocity required to avoid being rear-ended by the non-ego vehicle
    minEgoVelocityToAvoidBeingRearEnded = std::max((this->bufferDistance - distanceToTrailingNonEgoVehicle + (velocityOfTrailingNonEgoVehicle * planTime)) / planTime, 0.0);
    //std::cout << "Vehicle Behind in Lane" << newLane << "Ego in " << this->egoVehicle.getLane() << std::endl;
  }
  else
  {
    // no trailing vehicle
    minEgoVelocityToAvoidBeingRearEnded = 0.0;
    //std::cout << "No Vehicle Behind in Lane" << newLane << "Ego in " << this->egoVehicle.getLane() <<  std::endl;
  }

  // Is there a solution?
  if(minEgoVelocityToAvoidBeingRearEnded > maxEgoVelocityToAvoidRearEnding)
  {
    // Nope. Get out of this lane.
    // Return the least worst option
    //std::cout << "Lane "<< newLane << " Not Possible." << "Ego in " << this->egoVehicle.getLane() <<  std::endl;
    finalVelocity = std::min(0.5 * (minEgoVelocityToAvoidBeingRearEnded + maxEgoVelocityToAvoidRearEnding), this->maxSpeedAllowed);

    // cap to maximum possible change allowed
    finalVelocity = std::min(currentEgoVelocity + maximumChangeInVelocity, finalVelocity);

    cost = this->maximumCost;
  }
  else
  {

    // given the performance limitations, is is possible to change lane without rear-ending, and without being rear-ended?

    // First, whether we can avoid being rear-ended by a car from behind
    if(std::min(currentEgoVelocity + maximumChangeInVelocity, this->maxSpeedAllowed) < minEgoVelocityToAvoidBeingRearEnded)
    {
      // We will be rear-ended: Need to accelerate outside of performance limits.
      // Outside performance limit cases dominate, therefore, are written first in the decision tree.
      //std::cout << "Lane "<< newLane << " Not Possible." << "Ego in " << this->egoVehicle.getLane() <<  std::endl;
      finalVelocity = std::min(currentEgoVelocity + maximumChangeInVelocity, this->maxSpeedAllowed);
      cost = this->maximumCost;
    }
    else if(currentEgoVelocity - maximumChangeInVelocity > maxEgoVelocityToAvoidRearEnding)
    {
      // we would need to brake harder than allowed to avoid rear-ending the vehicle ahead.
      // Need to brake harder than performance limits.
      // Outside performance limit cases dominate, therefore, are written first in the decision tree.
      //std::cout << "Lane "<< newLane << " Not Possible." << "Ego in " << this->egoVehicle.getLane() <<  std::endl;
      finalVelocity = currentEgoVelocity - maximumChangeInVelocity;
      cost = this->maximumCost;
    }
    else
    {
      // maximum speed will be limited either by maxSpeedAllowed or maxEgoVelocityToAvoidRearEnding
      finalVelocity = std::min(std::min(currentEgoVelocity + maximumChangeInVelocity, this->maxSpeedAllowed), maxEgoVelocityToAvoidRearEnding);
      cost = 1.0 - std::exp(-1.0 * ((std::min(currentEgoVelocity + maximumChangeInVelocity, this->maxSpeedAllowed) - finalVelocity) / finalVelocity));
      //std::cout << "Lane "<< newLane << " Feasible. Cost: " << cost << "Ego in " << this->egoVehicle.getLane() <<  std::endl;
    }


  }

  // sanity check
  if(cost < 0 || finalVelocity > this->maxSpeedAllowed)
  {
    assert(cost >= 0 && "Lane change cost calculations not correct");
  }

  return(std::make_pair(cost, finalVelocity));
}




std::pair<double, double> PathPlanner::costOfLaneChangeLeft()
{
  std::pair<double, double> costAndFinalVelocity;

  // get the cost nevertheless in order to get finalVelocity information, if it is required.
  costAndFinalVelocity = costForLaneChange(this->egoVehicle.getLane() - 1);

  // modified cost, based on boundary conditions
  double cost = -1.0;

  if(this->egoVehicle.getLane() == 1)
  {
    // cannot move left.
    cost = this->maximumCost;
  }
  else if ((this->egoVehicle.getSCoordinatee() > this->returnToLaneBuffer) && (this->egoVehicle.getLane() < this->finalLane))
  {
    // should we prohibit lane movements?
    // Required if the ego is expected to end at a specific lane
    cost = this->maximumCost;

  }
  else
  {
    // going left needs to be investigated.
    cost = costAndFinalVelocity.first;
  }

  // update the cost.
  costAndFinalVelocity.first = cost;
  std::cout << "Cost for Switching to Left Lane: " << cost << " . Ego in " << this->egoVehicle.getLane() << std::endl;
  return(costAndFinalVelocity);
}


std::pair<double, double> PathPlanner::costOfLaneChangeRight()
{
  double cost = -1.0;

  std::pair<double, double> costAndFinalVelocity;

  // get the cost nevertheless in order to get finalVelocity information, if it is required.
  costAndFinalVelocity = costForLaneChange(this->egoVehicle.getLane() + 1);

  if(this->egoVehicle.getLane() == 3)
  {
    // modified cost, based on boundary conditions
    cost = this->maximumCost;
  }
  else if ((this->egoVehicle.getSCoordinatee() > this->returnToLaneBuffer) && (this->egoVehicle.getLane() > this->finalLane))
  {
    // should we prohibit lane movement?
    cost = this->maximumCost;
  }
  else
  {
    // going left needs to be investigated.
    cost = costAndFinalVelocity.first;
  }

  // update the cost.
  costAndFinalVelocity.first = cost;
  std::cout << "Cost for Switching to Right Lane: " << cost << " . Ego in " << this->egoVehicle.getLane() << std::endl;
  return(costAndFinalVelocity);
}


std::pair<double, double> PathPlanner::costOfLaneRemainingInLane()
{
  double cost = -1.0;

  std::pair<double, double> costAndFinalVelocity;

  // get the cost nevertheless in order to get finalVelocity information, if it is required.
  costAndFinalVelocity = costForLaneChange(this->egoVehicle.getLane());

  if(this->egoVehicle.getVelocity() < this->minVelocityForLaneChange)
  {
    // warmup. Lane change not allowed.
    cost = 0.0;
  }
  else if ((this->egoVehicle.getSCoordinatee() > this->returnToLaneBuffer) && (this->egoVehicle.getLane() == this->finalLane))
  {
    // update the cost based on boundary conditions
    // should we prohibit lane movement?
    cost = 0;
  }
  else
  {
    // going straight needs to be investigated.
    cost = costAndFinalVelocity.first;
  }

  // update the cost.
  costAndFinalVelocity.first = cost;
  std::cout << "Cost for Keeping Lane: " << cost << " . Ego in " << this->egoVehicle.getLane() << std::endl;
  return(costAndFinalVelocity);
}


std::pair<Commons::LaneChange, double> PathPlanner::getLaneChangeSuggestion() {

  Commons::LaneChange suggestedLaneChange;
  double finalVelocityForSuggestedLane = 0;
  std::pair<double, double> costKeepLane = costOfLaneRemainingInLane();

  // we would prefer to not change the lane as long as not necessary
  if(costKeepLane.first > this->laneChangeThreshold)
  {
    //attempt to change the lanes
    std::pair<double, double> costLaneChangeLeft = costOfLaneChangeLeft();
    std::pair<double, double> costLaneChangeRight = costOfLaneChangeRight();

    if((costLaneChangeRight.first < costLaneChangeLeft.first) && (costLaneChangeRight.first < costKeepLane.first))
    {
      suggestedLaneChange = Commons::LaneChange::Move_Right;
      finalVelocityForSuggestedLane = costLaneChangeRight.second;
    }
    else if((costLaneChangeLeft.first < costLaneChangeRight.first) && (costLaneChangeLeft.first < costKeepLane.first))
    {
      suggestedLaneChange = Commons::LaneChange::Move_Left;
      finalVelocityForSuggestedLane = costLaneChangeLeft.second;
    }
    else
    {
      suggestedLaneChange = Commons::LaneChange::KeepLane;
      finalVelocityForSuggestedLane = costKeepLane.second;
    }
  }
  else
  {
    suggestedLaneChange = Commons::LaneChange::KeepLane;
    finalVelocityForSuggestedLane = costKeepLane.second;
  }

  return (std::make_pair(suggestedLaneChange, finalVelocityForSuggestedLane));
}



// Calculates jerk minimizing path
 /*vector<double> computeMinimumJerkBAK(vector<double> start, vector<double> end, double max_time, double time_inc)*/
 std::vector<double> PathPlanner::getJerkOptimalCurve(std::vector<double> start, std::vector<double> end, double span, double sampleTime)
     {
   // alpha_0 through alpha_2 are known from the initial conditions
     double alpha_0 = start[0]; //si
     double alpha_1 = start[1]; // sidot
     double alpha_2 = 0.5 * start[2]; // sidotdot

     // initialize alpha_3, alpha_4, and alpha_5
     double alpha_3 = 0.0;
     double alpha_4 = 0.0;
     double alpha_5 = 0.0;

     Eigen::Vector3d B(alpha_3, alpha_4, alpha_5);
     Eigen::MatrixXd A(3, 3); // a 3x3 matrix

     double TPower2 = std::pow(span, 2.0);
     double TPower3 = TPower2 * span;
     double TPower4 = TPower3 * span;
     double TPower5 = TPower4 * span;

     A << TPower3, TPower4, TPower5, 3*TPower2, 4*TPower3, 5*TPower4, 6*span, 12*TPower2, 20*TPower3;

     double sf= end[0];
     double sfdot = end[1];
     double sfdotdot = end[2];

     Eigen::Vector3d C(3, 1); // a 3x1 matrix
     C << sf - ( alpha_0 + alpha_1 * span + alpha_2 *  TPower2 ), sfdot - (alpha_1 + 2 * alpha_2 * span), sfdotdot - 2*alpha_2 ;

     // Solve for B: A*B = C
     B = A.lu().solve(C);

     alpha_3 = B(0);
     alpha_4 = B(1);
     alpha_5 = B(2);



     /*std::vector<double> result{alpha_0, alpha_1, alpha_2, B(0), B(1), B(2)};*/
     std::vector<double> result;
     double currentS;
     unsigned nSteps = static_cast<unsigned>(span/static_cast<double>(sampleTime));
     double currentTimeStep = sampleTime;
     double tPower2;
     double tPower3;
     double tPower4;
     double tPower5;
     for(unsigned i = 0; i < nSteps; ++i)
     {

       tPower2 = std::pow(currentTimeStep, 2.0);
       tPower3 = tPower2 * currentTimeStep;
       tPower4 = tPower3 * currentTimeStep;
       tPower5 = tPower4 * currentTimeStep;

       currentS = alpha_0 + (alpha_1 * currentTimeStep) + (alpha_2 * tPower2) + (alpha_3 * tPower3) + (alpha_4 * tPower4) + (alpha_5 * tPower5);
       result.push_back(currentS);
       currentTimeStep = currentTimeStep + sampleTime;
     }


     // compute array of points in the domain of the problem, e.g., s[0], s[1], s[2], until the horizon

    return (result);
     }




Trajectory PathPlanner::computeTrajectory(std::pair<Commons::LaneChange, double> newLane)
{
  double currentS;
  double currentVelocity;
  double finalS;
  double finalVelocity;
  unsigned currentLane;
  unsigned finalLane;
  double span;

  switch(newLane.first)
  {
    case (Commons::LaneChange::KeepLane):
    {
        currentS = this->egoVehicle.getSCoordinatee();
        currentVelocity = this->egoVehicle.getVelocity();
        finalVelocity = newLane.second;
        currentLane = this->egoVehicle.getLane();
        finalLane = currentLane;
        span = this->nSteps * this->SampleTime;
        finalS = currentS + 0.5 * (currentVelocity + finalVelocity) * span;
        break;
    }
    case (Commons::LaneChange::Move_Left):
    {
        currentS = this->egoVehicle.getSCoordinatee();
        currentVelocity = this->egoVehicle.getVelocity();
        finalVelocity = newLane.second;
        currentLane = this->egoVehicle.getLane();
        finalLane = currentLane - 1;
        span = this->nSteps * this->SampleTime * this->scaleForLaneChange;
        finalS = currentS + 0.5 * (currentVelocity + finalVelocity) * span;
        break;
    }
    case (Commons::LaneChange::Move_Right):
    {
        currentS = this->egoVehicle.getSCoordinatee();
        currentVelocity = this->egoVehicle.getVelocity();
        finalVelocity = newLane.second;
        currentLane = this->egoVehicle.getLane();
        finalLane = currentLane + 1;
        span = this->nSteps * this->SampleTime * this->scaleForLaneChange;
        finalS = currentS + 0.5 * (currentVelocity + finalVelocity) * span;
        break;
    }
    default:
    {
      // keep lane
      currentS = this->egoVehicle.getSCoordinatee();
      currentVelocity = this->egoVehicle.getVelocity();
      finalVelocity = newLane.second;
      currentLane = this->egoVehicle.getLane();
      finalLane = currentLane;
      span = this->nSteps * this->SampleTime;
      finalS = currentS + 0.5 * (currentVelocity + finalVelocity) * span;
      break;
    }

  } // switch(newLane)

  // Now Compute the minimum jerk path

  // First for S
  std::vector<double> nextS;
  std::vector<double> nextD;
  // split the path when we are close to the target
  if(currentS < this->sizeOfTrack && finalS >= this->sizeOfTrack)
  {
    // two step process
    // span of first part: proportional to the distance to be driven.
    double spanFirst = this->SampleTime * std::round((span * ((this->sizeOfTrack - currentS) / (finalS - currentS)) / this->SampleTime));


    double finalSFirst = this->sizeOfTrack;
    nextS = getJerkOptimalCurve(
                                                 {currentS, currentVelocity, 0.0},
                                                 {finalSFirst,   finalVelocity,   0.0},
                                                 spanFirst,
                                                 this->SampleTime
                                                 );
    // step 2
    double finalSSecond = finalS - this->sizeOfTrack;
    std::vector<double> nextSSecond = getJerkOptimalCurve(
                                                 {0, currentVelocity, 0.0},
                                                 {finalSSecond,   finalVelocity,   0.0},
                                                 span - spanFirst,
                                                 this->SampleTime
                                                 );

    nextS.insert(nextS.end(), nextSSecond.begin(), nextSSecond.end());

    std::cout << "Funny Stuff About to Happen" << std::endl;

  }
  else
  {
    nextS = getJerkOptimalCurve(
                                                  {currentS, currentVelocity, 0.0},
                                                  {finalS,   finalVelocity,   0.0},
                                                  span,
                                                  this->SampleTime
                                                  );
  }




  nextD = getJerkOptimalCurve(
                                              {this->egoVehicle.getDCoordinatee(), 0.0, 0.0},
                                              {this->laneNumberToD(finalLane),   0.0, 0.0},
                                              span,
                                              this->SampleTime
                                              );

  // sanity check

  assert(nextS.size() == nextD.size() && "Computed Trajectory Lengths Not Equal");


  // convert to trajectory
  Trajectory trajectory;
  for (size_t i = 0; i < nextS.size(); ++i )
  {
    trajectory.insertPair(nextS[i], nextD[i]);
  }



  // save the state
  this->egoVehicle.setDCoordinate(this->laneNumberToD(finalLane));
  this->egoVehicle.setSCoordinate(finalS);
  this->egoVehicle.setVelocity(finalVelocity);

  return(trajectory);

}



Trajectory PathPlanner::provideTrajectory() {

  // suggests whether the Ego should change the lanes (left or right), or keep the lane.
  std::pair<Commons::LaneChange, double> laneChangeSuggestion = getLaneChangeSuggestion();

  // compute trajectory for the suggested Lane
  Trajectory trajectory = computeTrajectory(laneChangeSuggestion);

  return (trajectory);

// Find all next states wherein a collision
  /*std::vector<State> nextFeasibleStates = getSafeNextStates();*/

}

