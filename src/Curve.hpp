/*
 * Curve.hpp
 *
 *  Created on: Mar 25, 2018
 *      Author: dumbledore
 */

#ifndef CURVE_HPP_
#define CURVE_HPP_

#include <vector>

class Curve {
 public:
  Curve();
  virtual ~Curve();

  std::vector<double> getJerkOptimalCurve(std::vector<double> start, std::vector<double> end, double span, unsigned nSteps);


};

#endif /* CURVE_HPP_ */
