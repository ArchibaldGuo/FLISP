#ifndef FLISP_PLANNER_GEOMETRY_UTILS_H
#define FLISP_PLANNER_GEOMETRY_UTILS_H

#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>

#include <cstddef>
#include <vector>

namespace flisp_planner {

double squaredDistance(const geometry_msgs::Point& a,
                       const geometry_msgs::Point& b);

double distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

double pointToSegmentDistance(const geometry_msgs::Point& point,
                              const geometry_msgs::Point& start,
                              const geometry_msgs::Point& end);

geometry_msgs::Point interpolatePathAtX(const nav_msgs::Path& path,
                                        double target_x);

geometry_msgs::Point clampToSphere(const geometry_msgs::Point& anchor,
                                   const geometry_msgs::Point& point,
                                   double radius);

nav_msgs::Path simplifyDouglasPeucker(const nav_msgs::Path& path,
                                      double epsilon);

nav_msgs::Path smoothMovingAverage(const nav_msgs::Path& path,
                                   std::size_t window_size);

void removeCollinearPoints(nav_msgs::Path& path, double distance_threshold);

}  // namespace flisp_planner

#endif  // FLISP_PLANNER_GEOMETRY_UTILS_H
