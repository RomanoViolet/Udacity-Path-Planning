/*
 * WayPoint.hpp
 *
 *  Created on: Mar 24, 2018
 *      Author: dumbledore
 */

#ifndef WAYPOINT_HPP_
#define WAYPOINT_HPP_

class WayPoint {
 public:
  WayPoint(double XCoordinate, double YCoordinate, double SCoordinate,
           double DCoordinate_XComponent, double DCoordinate_YComponent);
  virtual ~WayPoint();

  double getXCoordinate_MapReference();
  double getYCoordinate_MapReference();
  double getSCoordinate_LaneReference();
  double getDCoordinate_LaneReference();

 private:
  double XCoordinate_MapReference;
  double YCoordinate_MapReference;
  double SCoordinate_LaneReference;
  double DCoordinate_LaneReference;

};

#endif /* WAYPOINT_HPP_ */
