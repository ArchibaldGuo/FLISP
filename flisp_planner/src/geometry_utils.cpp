#include "flisp_planner/geometry_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace flisp_planner {
namespace {

void simplifyRecursive(const nav_msgs::Path& path,
                       std::size_t start,
                       std::size_t end,
                       std::vector<bool>& keep,
                       double epsilon) {
  if (end <= start + 1) {
    return;
  }

  const auto& first = path.poses[start].pose.position;
  const auto& last = path.poses[end].pose.position;

  double max_distance = 0.0;
  std::size_t max_index = start;
  for (std::size_t i = start + 1; i < end; ++i) {
    const double d = pointToSegmentDistance(path.poses[i].pose.position, first, last);
    if (d > max_distance) {
      max_distance = d;
      max_index = i;
    }
  }

  if (max_distance > epsilon) {
    keep[max_index] = true;
    simplifyRecursive(path, start, max_index, keep, epsilon);
    simplifyRecursive(path, max_index, end, keep, epsilon);
  }
}

}  // namespace

double squaredDistance(const geometry_msgs::Point& a,
                       const geometry_msgs::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

double distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
  return std::sqrt(squaredDistance(a, b));
}

double pointToSegmentDistance(const geometry_msgs::Point& point,
                              const geometry_msgs::Point& start,
                              const geometry_msgs::Point& end) {
  const double vx = end.x - start.x;
  const double vy = end.y - start.y;
  const double vz = end.z - start.z;
  const double len_sq = vx * vx + vy * vy + vz * vz;

  if (len_sq < 1.0e-12) {
    return distance(point, start);
  }

  const double raw_t = ((point.x - start.x) * vx +
                        (point.y - start.y) * vy +
                        (point.z - start.z) * vz) /
                       len_sq;
  const double t = std::clamp(raw_t, 0.0, 1.0);

  geometry_msgs::Point projection;
  projection.x = start.x + t * vx;
  projection.y = start.y + t * vy;
  projection.z = start.z + t * vz;
  return distance(point, projection);
}

geometry_msgs::Point interpolatePathAtX(const nav_msgs::Path& path,
                                        double target_x) {
  geometry_msgs::Point result;
  result.x = target_x;

  if (path.poses.empty()) {
    return result;
  }

  if (path.poses.size() == 1) {
    return path.poses.front().pose.position;
  }

  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto& a = path.poses[i].pose.position;
    const auto& b = path.poses[i + 1].pose.position;
    const double lo = std::min(a.x, b.x);
    const double hi = std::max(a.x, b.x);
    if (target_x < lo || target_x > hi) {
      continue;
    }

    const double denom = b.x - a.x;
    const double ratio = std::abs(denom) < 1.0e-9 ? 0.0 : (target_x - a.x) / denom;
    result.x = target_x;
    result.y = a.y + ratio * (b.y - a.y);
    result.z = a.z + ratio * (b.z - a.z);
    return result;
  }

  double best_dx = std::numeric_limits<double>::max();
  std::size_t best_index = 0;
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const double dx = std::abs(path.poses[i].pose.position.x - target_x);
    if (dx < best_dx) {
      best_dx = dx;
      best_index = i;
    }
  }
  return path.poses[best_index].pose.position;
}

geometry_msgs::Point clampToSphere(const geometry_msgs::Point& anchor,
                                   const geometry_msgs::Point& point,
                                   double radius) {
  if (radius <= 0.0) {
    return anchor;
  }

  const double d_sq = squaredDistance(anchor, point);
  if (d_sq <= radius * radius) {
    return point;
  }

  const double d = std::sqrt(d_sq);
  const double scale = radius / d;
  geometry_msgs::Point clamped;
  clamped.x = anchor.x + (point.x - anchor.x) * scale;
  clamped.y = anchor.y + (point.y - anchor.y) * scale;
  clamped.z = anchor.z + (point.z - anchor.z) * scale;
  return clamped;
}

nav_msgs::Path simplifyDouglasPeucker(const nav_msgs::Path& path,
                                      double epsilon) {
  if (path.poses.size() <= 2 || epsilon <= 0.0) {
    return path;
  }

  std::vector<bool> keep(path.poses.size(), false);
  keep.front() = true;
  keep.back() = true;
  simplifyRecursive(path, 0, path.poses.size() - 1, keep, epsilon);

  nav_msgs::Path result;
  result.header = path.header;
  result.poses.reserve(path.poses.size());
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    if (keep[i]) {
      result.poses.push_back(path.poses[i]);
    }
  }
  return result;
}

nav_msgs::Path smoothMovingAverage(const nav_msgs::Path& path,
                                   std::size_t window_size) {
  if (window_size < 3 || path.poses.size() <= window_size) {
    return path;
  }

  nav_msgs::Path result = path;
  const int half_window = static_cast<int>(window_size / 2);

  for (std::size_t i = 1; i + 1 < path.poses.size(); ++i) {
    const int begin = std::max(0, static_cast<int>(i) - half_window);
    const int end = std::min(static_cast<int>(path.poses.size()) - 1,
                             static_cast<int>(i) + half_window);

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    int count = 0;
    for (int j = begin; j <= end; ++j) {
      sum_x += path.poses[j].pose.position.x;
      sum_y += path.poses[j].pose.position.y;
      sum_z += path.poses[j].pose.position.z;
      ++count;
    }

    result.poses[i].pose.position.x = sum_x / count;
    result.poses[i].pose.position.y = sum_y / count;
    result.poses[i].pose.position.z = sum_z / count;
  }

  return result;
}

void removeCollinearPoints(nav_msgs::Path& path, double distance_threshold) {
  if (path.poses.size() < 3 || distance_threshold <= 0.0) {
    return;
  }

  std::vector<bool> keep(path.poses.size(), true);
  for (std::size_t i = 1; i + 1 < path.poses.size(); ++i) {
    const double d = pointToSegmentDistance(path.poses[i].pose.position,
                                            path.poses[i - 1].pose.position,
                                            path.poses[i + 1].pose.position);
    if (d < distance_threshold) {
      keep[i] = false;
    }
  }

  nav_msgs::Path compact;
  compact.header = path.header;
  compact.poses.reserve(path.poses.size());
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    if (keep[i]) {
      compact.poses.push_back(path.poses[i]);
    }
  }

  path = compact;
}

}  // namespace flisp_planner
