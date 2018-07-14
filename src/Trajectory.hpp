/*
 * Trajectory.hpp
 *
 *  Created on: Mar 24, 2018
 *      Author: dumbledore
 */

#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include <vector>

#include "../src/Commons.hpp"
class PathPlanner;

class Trajectory {
 public:
  Trajectory();
  virtual ~Trajectory();

  void insertPair(double x, double y);
  std::vector<std::pair<double, double>> getCoordinatePairs();
 private:

  std::vector<std::pair<double, double>> trajectory;

};

#endif /* TRAJECTORY_HPP_ */
