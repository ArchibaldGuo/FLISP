#include "flisp_planner/uav_path_planner.h"

#include "flisp_planner/geometry_utils.h"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace flisp_planner {
namespace {

constexpr double kPi = 3.14159265358979323846;

pcl::PointXYZ toPclPoint(const geometry_msgs::Point& point) {
  pcl::PointXYZ pcl_point;
  pcl_point.x = static_cast<float>(point.x);
  pcl_point.y = static_cast<float>(point.y);
  pcl_point.z = static_cast<float>(point.z);
  return pcl_point;
}

geometry_msgs::Point toRosPoint(const pcl::PointXYZ& point) {
  geometry_msgs::Point ros_point;
  ros_point.x = point.x;
  ros_point.y = point.y;
  ros_point.z = point.z;
  return ros_point;
}

}  // namespace

UavPathPlanner::UavPathPlanner(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(std::move(nh)),
      latest_cloud_(new PointCloud),
      kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>) {
  loadConfig(private_nh);

  ugv_path_sub_ = nh_.subscribe(config_.ugv_path_topic, 10,
                                &UavPathPlanner::ugvPathCallback, this);
  reference_path_sub_ = nh_.subscribe(config_.reference_path_topic, 10,
                                      &UavPathPlanner::referencePathCallback,
                                      this);
  cloud_sub_ = nh_.subscribe(config_.point_cloud_topic, 10,
                             &UavPathPlanner::pointCloudCallback, this);
  output_path_pub_ =
      nh_.advertise<nav_msgs::Path>(config_.output_path_topic, 10);
  runtime_pub_ = nh_.advertise<std_msgs::Float64>(config_.runtime_topic, 10);

  ROS_INFO("FLISP UAV path planner initialized.");
}

void UavPathPlanner::loadConfig(const ros::NodeHandle& private_nh) {
  private_nh.param("ugv_path_topic", config_.ugv_path_topic,
                   config_.ugv_path_topic);
  private_nh.param("reference_path_topic", config_.reference_path_topic,
                   config_.reference_path_topic);
  private_nh.param("point_cloud_topic", config_.point_cloud_topic,
                   config_.point_cloud_topic);
  private_nh.param("output_path_topic", config_.output_path_topic,
                   config_.output_path_topic);
  private_nh.param("runtime_topic", config_.runtime_topic,
                   config_.runtime_topic);
  private_nh.param("output_frame", config_.output_frame, config_.output_frame);

  private_nh.param("communication_radius", config_.communication_radius,
                   config_.communication_radius);
  private_nh.param("safe_radius", config_.safe_radius, config_.safe_radius);
  private_nh.param("min_altitude_offset", config_.min_altitude_offset,
                   config_.min_altitude_offset);
  private_nh.param("reference_extra_altitude_offset",
                   config_.reference_extra_altitude_offset,
                   config_.reference_extra_altitude_offset);

  private_nh.param("cloud_crop_margin_multiplier",
                   config_.cloud_crop_margin_multiplier,
                   config_.cloud_crop_margin_multiplier);
  private_nh.param("path_crop_margin", config_.path_crop_margin,
                   config_.path_crop_margin);
  private_nh.param("voxel_leaf_size_near_path",
                   config_.voxel_leaf_size_near_path,
                   config_.voxel_leaf_size_near_path);
  private_nh.param("voxel_leaf_size_global", config_.voxel_leaf_size_global,
                   config_.voxel_leaf_size_global);
  private_nh.param("downsample_threshold", config_.downsample_threshold,
                   config_.downsample_threshold);

  private_nh.param("enable_obstacle_avoidance",
                   config_.enable_obstacle_avoidance,
                   config_.enable_obstacle_avoidance);
  private_nh.param("sample_count", config_.sample_count, config_.sample_count);
  private_nh.param("optimization_iterations", config_.optimization_iterations,
                   config_.optimization_iterations);
  private_nh.param("sampling_radius_decay", config_.sampling_radius_decay,
                   config_.sampling_radius_decay);
  private_nh.param("safety_weight", config_.safety_weight,
                   config_.safety_weight);
  private_nh.param("smoothness_weight", config_.smoothness_weight,
                   config_.smoothness_weight);
  private_nh.param("progress_weight", config_.progress_weight,
                   config_.progress_weight);
  private_nh.param("height_weight", config_.height_weight,
                   config_.height_weight);
  private_nh.param("blocked_segment_penalty", config_.blocked_segment_penalty,
                   config_.blocked_segment_penalty);

  private_nh.param("enable_post_smoothing", config_.enable_post_smoothing,
                   config_.enable_post_smoothing);
  private_nh.param("simplify_epsilon", config_.simplify_epsilon,
                   config_.simplify_epsilon);
  private_nh.param("smooth_window_size", config_.smooth_window_size,
                   config_.smooth_window_size);
}

void UavPathPlanner::ugvPathCallback(const nav_msgs::Path::ConstPtr& msg) {
  ugv_path_ = *msg;
  have_ugv_path_ = true;
  processPaths();
}

void UavPathPlanner::referencePathCallback(const nav_msgs::Path::ConstPtr& msg) {
  reference_path_ = *msg;
  have_reference_path_ = true;
  processPaths();
}

void UavPathPlanner::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  PointCloud::Ptr raw_cloud(new PointCloud);
  pcl::fromROSMsg(*msg, *raw_cloud);
  if (raw_cloud->empty()) {
    latest_cloud_->clear();
    return;
  }

  PointCloud::Ptr filtered(new PointCloud);
  if (have_ugv_path_ && !ugv_path_.poses.empty()) {
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    double max_z = std::numeric_limits<double>::lowest();

    for (const auto& pose : ugv_path_.poses) {
      const auto& point = pose.pose.position;
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
      min_z = std::min(min_z, point.z);
      max_x = std::max(max_x, point.x);
      max_y = std::max(max_y, point.y);
      max_z = std::max(max_z, point.z);
    }

    const double margin = config_.path_crop_margin +
                          config_.cloud_crop_margin_multiplier *
                              config_.communication_radius;
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(raw_cloud);
    crop_box.setMin(Eigen::Vector4f(static_cast<float>(min_x - margin),
                                    static_cast<float>(min_y - margin),
                                    static_cast<float>(min_z - margin), 1.0F));
    crop_box.setMax(Eigen::Vector4f(static_cast<float>(max_x + margin),
                                    static_cast<float>(max_y + margin),
                                    static_cast<float>(max_z + margin), 1.0F));
    crop_box.filter(*filtered);
  } else {
    filtered = raw_cloud;
  }

  const float leaf_size = static_cast<float>(
      have_ugv_path_ ? config_.voxel_leaf_size_near_path
                     : config_.voxel_leaf_size_global);
  if (static_cast<int>(filtered->size()) > config_.downsample_threshold &&
      leaf_size > 0.0F) {
    PointCloud::Ptr downsampled(new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(filtered);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*downsampled);
    latest_cloud_ = downsampled;
  } else {
    latest_cloud_ = filtered;
  }

  if (!latest_cloud_->empty()) {
    kdtree_->setInputCloud(latest_cloud_);
  }
}

void UavPathPlanner::processPaths() {
  if (!have_ugv_path_ || !have_reference_path_ ||
      ugv_path_.poses.empty() || reference_path_.poses.empty()) {
    return;
  }

  const auto start_time = std::chrono::steady_clock::now();
  nav_msgs::Path path = buildInitialPath();
  if (path.poses.empty()) {
    return;
  }

  if (config_.enable_obstacle_avoidance) {
    const auto obstacles = collectObstacleSegments(path);
    for (const auto& obstacle : obstacles) {
      if (obstacle.index + 1 >= path.poses.size()) {
        continue;
      }
      const geometry_msgs::Point start = path.poses[obstacle.index].pose.position;
      const geometry_msgs::Point original =
          path.poses[obstacle.index + 1].pose.position;
      const geometry_msgs::Point anchor = communicationAnchor(obstacle.index + 1);
      path.poses[obstacle.index + 1].pose.position =
          optimizeWaypoint(start, original, anchor);
    }
  }

  nav_msgs::Path output_path = path;
  if (config_.enable_post_smoothing) {
    nav_msgs::Path simplified =
        simplifyDouglasPeucker(path, config_.simplify_epsilon);
    output_path = smoothMovingAverage(
        simplified, static_cast<std::size_t>(
                        std::max(0, config_.smooth_window_size)));
  }
  output_path_pub_.publish(output_path);

  const auto end_time = std::chrono::steady_clock::now();
  const std::chrono::duration<double, std::milli> elapsed_ms =
      end_time - start_time;

  std_msgs::Float64 runtime_msg;
  runtime_msg.data = elapsed_ms.count();
  runtime_pub_.publish(runtime_msg);
  ROS_INFO_THROTTLE(1.0, "FLISP UAV path: %zu points, %.3f ms",
                    output_path.poses.size(), elapsed_ms.count());
}

nav_msgs::Path UavPathPlanner::buildInitialPath() const {
  nav_msgs::Path path;
  path.header = ugv_path_.header;
  path.header.stamp = ros::Time::now();
  if (!config_.output_frame.empty()) {
    path.header.frame_id = config_.output_frame;
  }

  path.poses.reserve(ugv_path_.poses.size());
  for (std::size_t i = 0; i < ugv_path_.poses.size(); ++i) {
    geometry_msgs::Point lowest = ugv_path_.poses[i].pose.position;
    lowest.z += config_.min_altitude_offset;

    geometry_msgs::Point desired =
        interpolatePathAtX(reference_path_, ugv_path_.poses[i].pose.position.x);
    desired.z += config_.reference_extra_altitude_offset;

    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position =
        clampToSphere(lowest, desired, config_.communication_radius);
    pose.pose.orientation = ugv_path_.poses[i].pose.orientation;
    path.poses.push_back(pose);
  }

  return path;
}

std::vector<UavPathPlanner::ObstacleSegment>
UavPathPlanner::collectObstacleSegments(const nav_msgs::Path& path) const {
  std::vector<ObstacleSegment> obstacles;
  if (!latest_cloud_ || latest_cloud_->empty() || path.poses.size() < 2) {
    return obstacles;
  }

  obstacles.reserve(path.poses.size());
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    if (hasObstacleInCylinder(path.poses[i].pose.position,
                              path.poses[i + 1].pose.position)) {
      obstacles.push_back({i});
    }
  }
  return obstacles;
}

bool UavPathPlanner::hasObstacleInCylinder(
    const geometry_msgs::Point& start,
    const geometry_msgs::Point& end) const {
  return minDistanceToSegmentCloud(start, end) < config_.safe_radius;
}

double UavPathPlanner::minDistanceToSegmentCloud(
    const geometry_msgs::Point& start,
    const geometry_msgs::Point& end) const {
  if (!latest_cloud_ || latest_cloud_->empty()) {
    return std::numeric_limits<double>::infinity();
  }

  const double segment_length = distance(start, end);
  if (segment_length < 1.0e-9) {
    return std::numeric_limits<double>::infinity();
  }

  geometry_msgs::Point center;
  center.x = 0.5 * (start.x + end.x);
  center.y = 0.5 * (start.y + end.y);
  center.z = 0.5 * (start.z + end.z);

  std::vector<int> indices;
  std::vector<float> squared_distances;
  const double search_radius = 0.5 * segment_length + config_.safe_radius;
  if (kdtree_->radiusSearch(toPclPoint(center), search_radius, indices,
                            squared_distances) <= 0) {
    return std::numeric_limits<double>::infinity();
  }

  double min_distance = std::numeric_limits<double>::infinity();
  for (int index : indices) {
    const geometry_msgs::Point point =
        toRosPoint(latest_cloud_->points[static_cast<std::size_t>(index)]);
    min_distance = std::min(min_distance, pointToSegmentDistance(point, start, end));
  }
  return min_distance;
}

geometry_msgs::Point UavPathPlanner::optimizeWaypoint(
    const geometry_msgs::Point& start,
    const geometry_msgs::Point& original,
    const geometry_msgs::Point& anchor) {
  geometry_msgs::Point current = original;
  geometry_msgs::Point best = original;

  const int sample_count = std::max(8, config_.sample_count);
  std::vector<std::pair<double, double>> directions;
  directions.reserve(static_cast<std::size_t>(sample_count));
  for (int i = 0; i < sample_count; ++i) {
    const double angle = 2.0 * kPi * static_cast<double>(i) /
                         static_cast<double>(sample_count);
    directions.emplace_back(std::cos(angle), std::sin(angle));
  }

  double radius = std::max(config_.communication_radius, config_.safe_radius);
  for (int iteration = 0; iteration < config_.optimization_iterations; ++iteration) {
    double best_score = std::numeric_limits<double>::infinity();
    for (const auto& direction : directions) {
      geometry_msgs::Point candidate;
      candidate.x = original.x;
      candidate.y = current.y + radius * direction.first;
      candidate.z = current.z + radius * direction.second;
      candidate = clampToSphere(anchor, candidate, config_.communication_radius);

      const double score = candidateCost(candidate, start, original);
      if (score < best_score) {
        best_score = score;
        best = candidate;
      }
    }

    current = best;
    radius *= config_.sampling_radius_decay;
  }

  return clampToSphere(anchor, best, config_.communication_radius);
}

double UavPathPlanner::candidateCost(const geometry_msgs::Point& candidate,
                                     const geometry_msgs::Point& start,
                                     const geometry_msgs::Point& original) const {
  const double clearance = minDistanceToSegmentCloud(start, candidate);
  double safety_cost = 0.0;
  if (clearance < config_.safe_radius) {
    safety_cost = std::pow(config_.safe_radius - clearance, 2.0);
  }
  if (!std::isfinite(clearance)) {
    safety_cost = 0.0;
  }

  if (hasObstacleInCylinder(start, candidate)) {
    safety_cost += config_.blocked_segment_penalty;
  }

  const double smoothness_cost =
      std::pow(pointToSegmentDistance(candidate, start, original), 2.0);

  const double total_distance = distance(start, original);
  double progress_reward = 0.0;
  if (total_distance > 1.0e-9) {
    progress_reward =
        (total_distance - distance(candidate, original)) / total_distance;
    progress_reward = std::clamp(progress_reward, 0.0, 1.0);
  }

  const double height_cost = std::pow(candidate.z - start.z, 2.0);
  return config_.safety_weight * safety_cost +
         config_.smoothness_weight * smoothness_cost -
         config_.progress_weight * progress_reward +
         config_.height_weight * height_cost;
}

geometry_msgs::Point UavPathPlanner::communicationAnchor(std::size_t index) const {
  geometry_msgs::Point anchor;
  if (ugv_path_.poses.empty()) {
    return anchor;
  }

  const std::size_t clamped_index =
      std::min(index, ugv_path_.poses.size() - 1);
  anchor = ugv_path_.poses[clamped_index].pose.position;
  anchor.z += config_.min_altitude_offset;
  return anchor;
}

}  // namespace flisp_planner
