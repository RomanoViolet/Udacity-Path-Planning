/*
 * Trajectory.cpp
 *
 *  Created on: Mar 24, 2018
 *      Author: dumbledore
 */

#include "Trajectory.hpp"

Trajectory::Trajectory() {


}

Trajectory::~Trajectory() {

}

void Trajectory::insertPair(double x, double y)
{
  trajectory.push_back(std::make_pair(x, y));
}

std::vector<std::pair<double, double>> Trajectory::getCoordinatePairs()
{
  return(trajectory);
}
