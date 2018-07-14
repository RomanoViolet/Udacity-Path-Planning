/*
 * Vehicle.cpp
 *
 *  Created on: Mar 24, 2018
 *      Author: dumbledore
 */
#include <algorithm>
#include "Vehicle.hpp"

Vehicle::~Vehicle() {

}

// The copy constructor
Vehicle::Vehicle (const Vehicle& source)
{
  this->_XCoordinate_MapReference = source._XCoordinate_MapReference;
  this->_YCoordinate_MapReference = source._YCoordinate_MapReference;
  this->_carS = source._carS;
  this->_carD = source._carD;
  this->_carLane = source._carLane;
  this->_carVelocity = source._carVelocity;

  // the object is considered empty if the lane number is unreasonably high
  if (this->_carLane == std::numeric_limits<unsigned>::max())
  {
    this->_isEmpty = true;
  }
  else
  {
    this->_isEmpty = false;
  }


}

  // assignment operator
Vehicle& Vehicle::operator= (const Vehicle& source)
{
  // invoke the oopy constructor
  Vehicle tmp(source);

  // Swap. Use standard swap since this is not a deep copy and is not expected to throw
  std::swap(this->_XCoordinate_MapReference, tmp._XCoordinate_MapReference);
  std::swap(this->_YCoordinate_MapReference, tmp._YCoordinate_MapReference);
  std::swap(this->_carS, tmp._carS);
  std::swap(this->_carD, tmp._carD);
  std::swap(this->_carLane, tmp._carLane);
  std::swap(this->_carVelocity, tmp._carVelocity);

  // the object is considered empty if the lane number is unreasonably high
  if (this->_carLane == std::numeric_limits<unsigned>::max())
  {
    this->_isEmpty = true;
  }
  else
  {
    this->_isEmpty = false;
  }

  return *this;

}

void Vehicle::setVelocity(double velocity)
{
  this->_carVelocity = velocity;
}

void Vehicle::setDCoordinate(double D)
{
  this->_carD = D;
}

void Vehicle::setSCoordinate(double S)
{
  this->_carS = S;
}


double Vehicle::getDCoordinatee() const
{
  return(this->_carD);
}

bool Vehicle::isEmpty()
{
  return(this->_isEmpty);
}

double Vehicle::getXCoordinate() const
{
  return(this->_XCoordinate_MapReference);
}

double Vehicle::getYCoordinate() const
{
  return(this->_YCoordinate_MapReference);
}

double Vehicle::getSCoordinatee() const
{
  return(this->_carS);
}

unsigned Vehicle::getLane() const
{
  return(this->_carLane);
}

double Vehicle::getVelocity() const
{
  return(this->_carVelocity);
}

