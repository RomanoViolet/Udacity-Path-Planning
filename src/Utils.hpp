/*
 * Utils.hpp
 *
 *  Created on: Mar 26, 2018
 *      Author: dumbledore
 */

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <vector>
#include <string>

void upSampleWayPoints(std::vector<double>& map_waypoints_x,
                       std::vector<double>& map_waypoints_y,
                       std::vector<double>& map_waypoints_s,
                       std::vector<double>& map_waypoints_dx,
                       std::vector<double>& map_waypoints_dy,
                       unsigned upSampleFactor);

double pi();

double deg2rad(double x);

double rad2deg(double x);

std::string hasData(std::string s);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta,
                 const std::vector<double> &maps_x,
                 const std::vector<double> &maps_y);

std::vector<double> getFrenet(double x, double y, double theta,
                              const std::vector<double> &maps_x,
                              const std::vector<double> &maps_y);

std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s,
                          const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y);

#endif /* UTILS_HPP_ */
