/*
 * Vehicle.hpp
 *
 *  Created on: Mar 24, 2018
 *      Author: dumbledore
 */

#ifndef VEHICLE_HPP_
#define VEHICLE_HPP_
#include <limits>
class Vehicle {
 public:

  // default constructor
  Vehicle()
      : _XCoordinate_MapReference(0.0),
        _YCoordinate_MapReference(0.0),
        _carS(0.0),
        _carD(0.0),
        _carLane(std::numeric_limits<unsigned>::max()),
        _carVelocity(0.0),
        _isEmpty(true) {

  }

  Vehicle(double XCoordinate_MapReference, double YCoordinate_MapReference,
          double velocity, double carS, unsigned carLane, double DCoordinate_LaneReference)
      : _XCoordinate_MapReference(XCoordinate_MapReference),
        _YCoordinate_MapReference(YCoordinate_MapReference),
        _carS(carS),
        _carD(DCoordinate_LaneReference),
        _carLane(carLane),
        _carVelocity(velocity),
        _isEmpty(false) {

  }

  // The copy constructor
  Vehicle(const Vehicle& source);

  // assignment operator
  Vehicle& operator=(const Vehicle& source);

  // return whether this vehicle was ever meaningfully initialized
  bool isEmpty();

  // getter methods
  double getXCoordinate() const;
  double getYCoordinate() const;
  double getSCoordinatee() const;
  double getDCoordinatee() const;
  unsigned getLane() const;
  double getVelocity() const;

  // setters
  void setVelocity(double velocity);
  void setDCoordinate(double D);
  void setSCoordinate(double S);

  virtual ~Vehicle();

 private:

  double _XCoordinate_MapReference;
  double _YCoordinate_MapReference;
  double _carS;
  double _carD;
  unsigned _carLane;
  double _carVelocity;
  // whether this vehicle has never been assigned any meaningful data
  bool _isEmpty;

};

#endif /* VEHICLE_HPP_ */
