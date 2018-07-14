/*
 * Utils.cpp
 *
 *  Created on: Mar 26, 2018
 *      Author: dumbledore
 */
#include "Utils.hpp"
#include "spline.h"

/*
 * Upsamples waypoint data by a factor of upSampleFactor.
 * Required for a smooth (s,d) -> (x,y) conversion by the getXY function.
 */

// Helper function
void upSampleVector(const std::vector<double>& input,
                    std::vector<double>& output, unsigned factor) {
  // The spline object
  tk::spline spline_x;

  /* create the index (or independent axis): 0, 1, 2, 3, ...
   * so that map_waypoints_x(1) = first way point, map_waypoints_x(2) = second way point, ...
   */
  std::vector<double> indices;
  for (size_t i = 0; i < input.size(); ++i) {
    indices.push_back(static_cast<double>(i));
  }

  // Set the points
  spline_x.set_points(indices, input);

  // Compute the indices again, with resampling.
  double incrementStep = 1.0 / factor;

  // For upSampled_map_waypoints_x
  output.clear();

  for (double i = 0; i < indices.size(); i = i + incrementStep) {
    // ask the spline to find values between the two original indices, e.g., at 0.01.
    output.push_back(spline_x(i));
  }
}

void upSampleWayPoints(std::vector<double>& map_waypoints_x,
                       std::vector<double>& map_waypoints_y,
                       std::vector<double>& map_waypoints_s,
                       std::vector<double>& map_waypoints_dx,
                       std::vector<double>& map_waypoints_dy,
                       unsigned upSampleFactor) {

  std::vector<double> upSampledData;
  upSampledData.clear();

  // delete the old data and replace it with the new one. Probably, we can use move semantics.
  //TODO Convert to std::move

  // upsample X-coordinates of the maps.
  upSampleVector(map_waypoints_x, upSampledData, upSampleFactor);
  map_waypoints_x.clear();
  map_waypoints_x = upSampledData;
  upSampledData.clear();

  // upsample Y-coordinates of the maps.
  upSampleVector(map_waypoints_y, upSampledData, upSampleFactor);
  map_waypoints_y.clear();
  map_waypoints_y = upSampledData;
  upSampledData.clear();

  // upsample S-coordinates of the maps.
  upSampleVector(map_waypoints_s, upSampledData, upSampleFactor);
  map_waypoints_s.clear();
  map_waypoints_s = upSampledData;
  upSampledData.clear();

  // upsample D-X-coordinates of the maps.
  upSampleVector(map_waypoints_dx, upSampledData, upSampleFactor);
  map_waypoints_dx.clear();
  map_waypoints_dx = upSampledData;
  upSampledData.clear();

  // upsample D-X-coordinates of the maps.
  upSampleVector(map_waypoints_dy, upSampledData, upSampleFactor);
  map_waypoints_dy.clear();
  map_waypoints_dy = upSampledData;
  upSampledData.clear();
}

// For converting back and forth between radians and degrees.
double pi() {
  return M_PI;
}
double deg2rad(double x) {
  return x * pi() / 180;
}
double rad2deg(double x) {
  return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y) {

  double closestLen = 100000;  //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta,
                 const std::vector<double> &maps_x,
                 const std::vector<double> &maps_y) {

  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta,
                              const std::vector<double> &maps_x,
                              const std::vector<double> &maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s,
                          const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                         (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x,y};

}

