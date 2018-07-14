/*
 * WayPoint.cpp
 *
 *  Created on: Mar 24, 2018
 *      Author: dumbledore
 */

#include "WayPoint.hpp"
#include <cmath>

WayPoint::WayPoint(double XCoordinate, double YCoordinate, double SCoordinate,
                   double DCoordinate_XComponent,
                   double DCoordinate_YComponent) {

  this->XCoordinate_MapReference = XCoordinate;
  this->YCoordinate_MapReference = YCoordinate;
  this->SCoordinate_LaneReference = SCoordinate;
  this->DCoordinate_LaneReference = std::sqrt(
      std::pow(DCoordinate_XComponent, 2.0)
          + std::pow(DCoordinate_YComponent, 2.0));
}

WayPoint::~WayPoint() {

}

double WayPoint::getXCoordinate_MapReference() {
  return (this->XCoordinate_MapReference);
}

double WayPoint::getYCoordinate_MapReference() {
  return (this->YCoordinate_MapReference);
}
double WayPoint::getSCoordinate_LaneReference() {
  return (this->SCoordinate_LaneReference);
}
double WayPoint::getDCoordinate_LaneReference() {
  return (this->DCoordinate_LaneReference);
}
