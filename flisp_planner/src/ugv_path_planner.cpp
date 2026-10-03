#include "flisp_planner/ugv_path_planner.h"

#include "flisp_planner/geometry_utils.h"

#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

namespace flisp_planner {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = 0.5 * kPi;

double gaussianLikelihood(double error, double sigma) {
  if (sigma <= 0.0) {
    return 0.0;
  }
  const double normalized = error / sigma;
  return std::exp(-0.5 * normalized * normalized) /
         (sigma * std::sqrt(2.0 * kPi));
}

std::vector<double> xValues(const std::vector<Point2>& points) {
  std::vector<double> x;
  x.reserve(points.size());
  for (const auto& point : points) {
    x.push_back(point[0]);
  }
  return x;
}

std::vector<double> yValues(const std::vector<Point2>& points) {
  std::vector<double> y;
  y.reserve(points.size());
  for (const auto& point : points) {
    y.push_back(point[1]);
  }
  return y;
}

}  // namespace

UgvPathPlanner::UgvPathPlanner(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(std::move(nh)), rng_(config_.random_seed) {
  loadConfig(private_nh);
  rng_.seed(config_.random_seed);

  point_cloud_sub_ = nh_.subscribe(config_.point_cloud_topic, 1,
                                   &UgvPathPlanner::pointCloudCallback, this);
  imu_sub_ = nh_.subscribe(config_.imu_topic, 1,
                           &UgvPathPlanner::imuCallback, this);
  ugv_path_pub_ = nh_.advertise<nav_msgs::Path>(config_.ugv_path_topic, 10);
  uav_reference_path_pub_ =
      nh_.advertise<nav_msgs::Path>(config_.uav_reference_path_topic, 10);
  runtime_pub_ = nh_.advertise<std_msgs::Float64>(config_.runtime_topic, 10);

  ROS_INFO("FLISP UGV path planner initialized.");
}

void UgvPathPlanner::loadConfig(const ros::NodeHandle& private_nh) {
  private_nh.param("point_cloud_topic", config_.point_cloud_topic,
                   config_.point_cloud_topic);
  private_nh.param("imu_topic", config_.imu_topic, config_.imu_topic);
  private_nh.param("ugv_path_topic", config_.ugv_path_topic,
                   config_.ugv_path_topic);
  private_nh.param("uav_reference_path_topic",
                   config_.uav_reference_path_topic,
                   config_.uav_reference_path_topic);
  private_nh.param("runtime_topic", config_.runtime_topic,
                   config_.runtime_topic);
  private_nh.param("output_frame", config_.output_frame, config_.output_frame);
  private_nh.param("uav_reference_frame", config_.uav_reference_frame,
                   config_.uav_reference_frame);

  private_nh.param("yaw_crop_min_x", config_.yaw_crop_min_x,
                   config_.yaw_crop_min_x);
  private_nh.param("yaw_crop_max_x", config_.yaw_crop_max_x,
                   config_.yaw_crop_max_x);
  private_nh.param("yaw_crop_min_y", config_.yaw_crop_min_y,
                   config_.yaw_crop_min_y);
  private_nh.param("yaw_crop_max_y", config_.yaw_crop_max_y,
                   config_.yaw_crop_max_y);
  private_nh.param("yaw_crop_min_z", config_.yaw_crop_min_z,
                   config_.yaw_crop_min_z);
  private_nh.param("yaw_crop_max_z", config_.yaw_crop_max_z,
                   config_.yaw_crop_max_z);
  private_nh.param("normal_neighbors", config_.normal_neighbors,
                   config_.normal_neighbors);
  private_nh.param("normal_samples", config_.normal_samples,
                   config_.normal_samples);

  private_nh.param("bin_c1", config_.bin_c1, config_.bin_c1);
  private_nh.param("bin_c2", config_.bin_c2, config_.bin_c2);
  private_nh.param("bin_kappa", config_.bin_kappa, config_.bin_kappa);
  private_nh.param("bin_lambda", config_.bin_lambda, config_.bin_lambda);
  private_nh.param("bin_c3", config_.bin_c3, config_.bin_c3);
  private_nh.param("bin_mu", config_.bin_mu, config_.bin_mu);
  private_nh.param("min_bin_step", config_.min_bin_step,
                   config_.min_bin_step);
  private_nh.param("max_bin_step", config_.max_bin_step,
                   config_.max_bin_step);
  private_nh.param("bin_step_model", config_.bin_step_model,
                   config_.bin_step_model);
  private_nh.param("preview_distance", config_.preview_distance,
                   config_.preview_distance);
  private_nh.param("edge_jump_threshold", config_.edge_jump_threshold,
                   config_.edge_jump_threshold);
  private_nh.param("curvature_variance_threshold",
                   config_.curvature_variance_threshold,
                   config_.curvature_variance_threshold);
  private_nh.param("polynomial_degree", config_.polynomial_degree,
                   config_.polynomial_degree);

  int linear_extension = static_cast<int>(config_.linear_extension_points);
  int polynomial_extension = static_cast<int>(config_.polynomial_extension_points);
  private_nh.param("linear_extension_points", linear_extension, linear_extension);
  private_nh.param("polynomial_extension_points", polynomial_extension,
                   polynomial_extension);
  config_.linear_extension_points =
      static_cast<std::size_t>(std::max(0, linear_extension));
  config_.polynomial_extension_points =
      static_cast<std::size_t>(std::max(0, polynomial_extension));

  private_nh.param("ground_fit_min_x", config_.ground_fit_min_x,
                   config_.ground_fit_min_x);
  private_nh.param("ground_fit_max_x", config_.ground_fit_max_x,
                   config_.ground_fit_max_x);
  private_nh.param("ground_fit_step", config_.ground_fit_step,
                   config_.ground_fit_step);
  private_nh.param("uav_reference_height", config_.uav_reference_height,
                   config_.uav_reference_height);

  private_nh.param("yaw_shrink_factor", config_.yaw_shrink_factor,
                   config_.yaw_shrink_factor);
  private_nh.param("roll_shrink_factor", config_.roll_shrink_factor,
                   config_.roll_shrink_factor);
  private_nh.param("outlier_prior", config_.outlier_prior,
                   config_.outlier_prior);
  private_nh.param("normal_noise_std", config_.normal_noise_std,
                   config_.normal_noise_std);
  private_nh.param("outlier_noise_std", config_.outlier_noise_std,
                   config_.outlier_noise_std);
  private_nh.param("outlier_probability_threshold",
                   config_.outlier_probability_threshold,
                   config_.outlier_probability_threshold);
  private_nh.param("outlier_mode", config_.outlier_mode,
                   config_.outlier_mode);
  private_nh.param("collinear_distance_threshold",
                   config_.collinear_distance_threshold,
                   config_.collinear_distance_threshold);

  private_nh.param("enable_obstacle_avoidance",
                   config_.enable_obstacle_avoidance,
                   config_.enable_obstacle_avoidance);
  private_nh.param("obstacle_detection_fraction",
                   config_.obstacle_detection_fraction,
                   config_.obstacle_detection_fraction);
  private_nh.param("obstacle_box_half_width", config_.obstacle_box_half_width,
                   config_.obstacle_box_half_width);
  private_nh.param("obstacle_box_z_offset", config_.obstacle_box_z_offset,
                   config_.obstacle_box_z_offset);
  private_nh.param("obstacle_box_min_z", config_.obstacle_box_min_z,
                   config_.obstacle_box_min_z);
  private_nh.param("obstacle_box_max_z", config_.obstacle_box_max_z,
                   config_.obstacle_box_max_z);
  private_nh.param("vehicle_width", config_.vehicle_width,
                   config_.vehicle_width);
  private_nh.param("safety_margin", config_.safety_margin,
                   config_.safety_margin);
  private_nh.param("max_tilt_ratio", config_.max_tilt_ratio,
                   config_.max_tilt_ratio);
  private_nh.param("firefly_count", config_.firefly_count,
                   config_.firefly_count);
  private_nh.param("firefly_iterations", config_.firefly_iterations,
                   config_.firefly_iterations);
  private_nh.param("firefly_alpha", config_.firefly_alpha,
                   config_.firefly_alpha);
  private_nh.param("firefly_beta0", config_.firefly_beta0,
                   config_.firefly_beta0);
  private_nh.param("firefly_gamma", config_.firefly_gamma,
                   config_.firefly_gamma);
  private_nh.param("firefly_alpha_decay", config_.firefly_alpha_decay,
                   config_.firefly_alpha_decay);
  private_nh.param("tilt_weight", config_.tilt_weight, config_.tilt_weight);
  private_nh.param("center_weight", config_.center_weight,
                   config_.center_weight);
  private_nh.param("collision_penalty_multiplier",
                   config_.collision_penalty_multiplier,
                   config_.collision_penalty_multiplier);
  private_nh.param("local_smooth_radius", config_.local_smooth_radius,
                   config_.local_smooth_radius);

  int smooth_transition =
      static_cast<int>(config_.smooth_transition_points);
  private_nh.param("smooth_transition_points", smooth_transition,
                   smooth_transition);
  config_.smooth_transition_points =
      static_cast<std::size_t>(std::max(0, smooth_transition));

  int seed = static_cast<int>(config_.random_seed);
  private_nh.param("random_seed", seed, seed);
  config_.random_seed = static_cast<unsigned int>(std::max(0, seed));
}

void UgvPathPlanner::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  const double ax = msg->linear_acceleration.x;
  const double ay = msg->linear_acceleration.y;
  const double az = msg->linear_acceleration.z;

  imu_roll_ = std::atan2(ay, az);
  imu_pitch_ = std::atan2(-ax, std::sqrt(ay * ay + az * az));
  imu_yaw_ = 0.0;
}

void UgvPathPlanner::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  const auto start_time = std::chrono::steady_clock::now();
  if (msg->width == 0 || msg->height == 0) {
    ROS_WARN_THROTTLE(1.0, "FLISP UGV received an empty point cloud.");
    return;
  }

  PointCloud::Ptr cloud(new PointCloud);
  pcl::fromROSMsg(*msg, *cloud);
  if (cloud->empty()) {
    ROS_WARN_THROTTLE(1.0, "FLISP UGV point cloud conversion produced no points.");
    return;
  }

  PointCloud::Ptr yaw_cloud(new PointCloud);
  pcl::CropBox<pcl::PointXYZ> yaw_crop;
  yaw_crop.setInputCloud(cloud);
  yaw_crop.setMin(Eigen::Vector4f(config_.yaw_crop_min_x,
                                  config_.yaw_crop_min_y,
                                  config_.yaw_crop_min_z, 1.0F));
  yaw_crop.setMax(Eigen::Vector4f(config_.yaw_crop_max_x,
                                  config_.yaw_crop_max_y,
                                  config_.yaw_crop_max_z, 1.0F));
  yaw_crop.filter(*yaw_cloud);

  std::tie(tunnel_yaw_, tunnel_diameter_) =
      estimateTunnelYawAndDiameter(yaw_cloud);
  std::tie(ground_slope_, ground_intercept_) = fitGroundLine(cloud);

  const auto points_2d = projectToXY(cloud);
  if (points_2d.empty()) {
    ROS_WARN_THROTTLE(1.0, "FLISP UGV found no valid finite points.");
    return;
  }

  const double step_size = dynamicBinStep(tunnel_yaw_);
  const BoundaryResult boundaries = extractBoundaryPoints(points_2d, step_size);
  if (boundaries.left.empty() || boundaries.right.empty()) {
    ROS_WARN_THROTTLE(1.0, "FLISP UGV could not extract both tunnel boundaries.");
    return;
  }

  const auto centerline = averageBoundaries(boundaries);
  nav_msgs::Path ugv_path = buildUgvPath(centerline, msg->header);
  if (ugv_path.poses.empty()) {
    ROS_WARN_THROTTLE(1.0, "FLISP UGV generated an empty path.");
    return;
  }

  correctBayesianOutliers(ugv_path);

  if (config_.enable_obstacle_avoidance) {
    const std::size_t max_detection_index =
        std::min<std::size_t>(ugv_path.poses.size() - 1,
                              static_cast<std::size_t>(
                                  ugv_path.poses.size() *
                                  std::clamp(config_.obstacle_detection_fraction,
                                             0.0, 1.0)));
    const auto obstacles =
        detectObstacleSegments(ugv_path, cloud, max_detection_index);
    applyFireflyAvoidance(ugv_path, obstacles, max_detection_index);
  }
  removeCollinearPoints(ugv_path, config_.collinear_distance_threshold);

  nav_msgs::Path uav_reference = buildUavReferencePath(centerline, msg->header);

  ugv_path_pub_.publish(ugv_path);
  uav_reference_path_pub_.publish(uav_reference);

  const auto end_time = std::chrono::steady_clock::now();
  const std::chrono::duration<double, std::milli> elapsed_ms =
      end_time - start_time;

  std_msgs::Float64 runtime_msg;
  runtime_msg.data = elapsed_ms.count();
  runtime_pub_.publish(runtime_msg);
  ROS_INFO_THROTTLE(1.0, "FLISP UGV path: %zu points, %.3f ms",
                    ugv_path.poses.size(), elapsed_ms.count());
}

std::pair<double, double> UgvPathPlanner::estimateTunnelYawAndDiameter(
    const PointCloud::ConstPtr& cloud) const {
  if (!cloud || static_cast<int>(cloud->size()) < config_.normal_samples) {
    return {0.0, tunnel_diameter_};
  }

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setSearchMethod(
      pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
  normal_estimator.setKSearch(std::max(3, config_.normal_neighbors));
  normal_estimator.setViewPoint(3.0F, 30.0F, 0.0F);
  normal_estimator.compute(*normals);

  if (normals->empty()) {
    return {0.0, tunnel_diameter_};
  }

  const auto compare_curvature = [](const pcl::Normal& a, const pcl::Normal& b) {
    return a.curvature < b.curvature;
  };
  const std::size_t sample_count =
      std::min<std::size_t>(normals->size(), config_.normal_samples);
  std::partial_sort(normals->begin(), normals->begin() + sample_count,
                    normals->end(), compare_curvature);

  Eigen::Vector3f normal_sum = Eigen::Vector3f::Zero();
  for (std::size_t i = 0; i < sample_count; ++i) {
    const auto& normal = normals->points[i];
    if (std::isfinite(normal.normal_x) && std::isfinite(normal.normal_y) &&
        std::isfinite(normal.normal_z)) {
      normal_sum += Eigen::Vector3f(normal.normal_x, normal.normal_y,
                                    normal.normal_z);
    }
  }

  if (normal_sum.norm() < 1.0e-6F) {
    return {0.0, tunnel_diameter_};
  }

  const Eigen::Vector3f wall_normal = normal_sum.normalized();
  const double cos_angle =
      std::clamp(static_cast<double>(wall_normal.dot(Eigen::Vector3f::UnitX())),
                 -1.0, 1.0);

  // The experimental LiDAR frame uses a wall-normal reference rotated by pi/2
  // from the paper's theta notation, so this is the same relative yaw error.
  const double yaw = kHalfPi - std::acos(cos_angle);

  Eigen::MatrixXf points(3, static_cast<int>(cloud->size()));
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    points(0, static_cast<int>(i)) = cloud->points[i].x;
    points(1, static_cast<int>(i)) = cloud->points[i].y;
    points(2, static_cast<int>(i)) = cloud->points[i].z;
  }
  const Eigen::VectorXf projections = (wall_normal.transpose() * points).transpose();
  const double diameter = projections.maxCoeff() - projections.minCoeff();
  return {yaw, diameter};
}

std::pair<double, double> UgvPathPlanner::fitGroundLine(
    const PointCloud::ConstPtr& cloud) const {
  if (!cloud || cloud->empty() || config_.ground_fit_step <= 0.0 ||
      config_.ground_fit_max_x <= config_.ground_fit_min_x) {
    return {ground_slope_, ground_intercept_};
  }

  const int bucket_count =
      static_cast<int>((config_.ground_fit_max_x - config_.ground_fit_min_x) /
                       config_.ground_fit_step) +
      1;
  std::vector<std::vector<pcl::PointXYZ>> buckets(bucket_count);

  for (const auto& point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.z)) {
      continue;
    }
    const int index = static_cast<int>((point.x - config_.ground_fit_min_x) /
                                       config_.ground_fit_step);
    if (index >= 0 && index < bucket_count) {
      buckets[static_cast<std::size_t>(index)].push_back(point);
    }
  }

  std::vector<double> x;
  std::vector<double> z;
  for (int i = 0; i < bucket_count; ++i) {
    const auto& bucket = buckets[static_cast<std::size_t>(i)];
    if (bucket.empty()) {
      continue;
    }
    const auto min_z_it = std::min_element(
        bucket.begin(), bucket.end(),
        [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) { return a.z < b.z; });
    x.push_back(config_.ground_fit_min_x + i * config_.ground_fit_step);
    z.push_back(min_z_it->z);
  }

  if (x.size() < 2) {
    return {ground_slope_, ground_intercept_};
  }

  const Eigen::VectorXd coeffs = linearFit(x, z);
  return {coeffs(1), coeffs(0)};
}

std::vector<std::pair<double, double>> UgvPathPlanner::projectToXY(
    const PointCloud::ConstPtr& cloud) const {
  std::vector<std::pair<double, double>> points;
  if (!cloud) {
    return points;
  }

  points.reserve(cloud->size());
  for (const auto& point : cloud->points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z) && point.x >= 0.0 &&
        point.x <= config_.preview_distance) {
      points.emplace_back(point.x, point.y);
    }
  }
  return points;
}

double UgvPathPlanner::dynamicBinStep(double yaw) const {
  const double abs_yaw = std::abs(yaw);
  if (config_.bin_step_model == "legacy_inverse") {
    const double step = config_.bin_c1 / (1.0 + 2.0 * abs_yaw);
    return std::clamp(step, config_.min_bin_step, config_.max_bin_step);
  }

  double step = config_.bin_c1;
  if (abs_yaw <= config_.bin_c2 && config_.bin_c2 > 0.0) {
    const double normalized = 1.0 - abs_yaw / config_.bin_c2;
    step = config_.bin_c1 * std::pow(std::max(0.0, normalized),
                                     config_.bin_kappa) +
           config_.bin_lambda * std::pow(abs_yaw, 3.0);
  } else {
    step = config_.bin_c3 + config_.bin_mu * (abs_yaw - config_.bin_c2);
  }
  return std::clamp(step, config_.min_bin_step, config_.max_bin_step);
}

UgvPathPlanner::BoundaryResult UgvPathPlanner::extractBoundaryPoints(
    const std::vector<std::pair<double, double>>& points_2d,
    double step_size) const {
  BoundaryResult result;
  if (points_2d.empty() || step_size <= 0.0) {
    return result;
  }

  bool stop_left = false;
  bool stop_right = false;
  const std::size_t bin_count =
      static_cast<std::size_t>(std::ceil(config_.preview_distance / step_size));

  for (std::size_t bin = 0; bin < bin_count; ++bin) {
    const double start_x = static_cast<double>(bin) * step_size;
    const double end_x = start_x + step_size;
    bool has_point = false;
    Point2 left{0.0, std::numeric_limits<double>::max()};
    Point2 right{0.0, std::numeric_limits<double>::lowest()};

    for (const auto& point : points_2d) {
      if (point.first < start_x || point.first >= end_x) {
        continue;
      }
      has_point = true;
      if (point.second < left[1]) {
        left = {point.first, point.second};
      }
      if (point.second > right[1]) {
        right = {point.first, point.second};
      }
    }

    if (!has_point) {
      continue;
    }

    if (!stop_left) {
      if (!result.left.empty() &&
          std::abs(left[1] - result.left.back()[1]) > config_.edge_jump_threshold) {
        stop_left = true;
      } else {
        result.left.push_back(left);
      }
    }

    if (!stop_right) {
      if (!result.right.empty() &&
          std::abs(right[1] - result.right.back()[1]) > config_.edge_jump_threshold) {
        stop_right = true;
      } else {
        result.right.push_back(right);
      }
    }
  }

  const std::size_t max_points =
      static_cast<std::size_t>(std::ceil(config_.preview_distance / step_size)) + 1;
  if (stop_left) {
    extendBoundary(result.left, step_size, max_points);
  }
  if (stop_right) {
    extendBoundary(result.right, step_size, max_points);
  }

  const std::size_t common_size = std::min(result.left.size(), result.right.size());
  result.left.resize(common_size);
  result.right.resize(common_size);
  return result;
}

void UgvPathPlanner::extendBoundary(std::vector<Point2>& edge,
                                    double step_size,
                                    std::size_t max_points) const {
  if (edge.size() < 2 || edge.size() >= max_points) {
    return;
  }

  const bool curved = isCurved(edge);
  const std::size_t extension_count =
      curved ? config_.polynomial_extension_points : config_.linear_extension_points;
  const std::size_t target_size =
      std::min(max_points, edge.size() + extension_count);

  const std::vector<double> x = xValues(edge);
  const std::vector<double> y = yValues(edge);
  Eigen::VectorXd coeffs =
      curved ? polynomialFit(x, y, config_.polynomial_degree) : linearFit(x, y);

  double next_x = edge.back()[0];
  while (edge.size() < target_size) {
    next_x += step_size;
    const double next_y =
        curved ? polynomialPredict(coeffs, next_x) : linearPredict(coeffs, next_x);
    edge.push_back({next_x, next_y});
  }
}

bool UgvPathPlanner::isCurved(const std::vector<Point2>& edge) const {
  if (edge.size() < 5) {
    return false;
  }

  std::vector<double> slopes;
  const std::size_t step =
      std::max<std::size_t>(1, static_cast<std::size_t>(edge.size() / 10));
  for (std::size_t i = step; i < edge.size(); i += step) {
    const double dx = edge[i][0] - edge[i - step][0];
    if (std::abs(dx) < 1.0e-9) {
      continue;
    }
    slopes.push_back((edge[i][1] - edge[i - step][1]) / dx);
  }

  if (slopes.size() < 3) {
    return false;
  }

  const double mean = std::accumulate(slopes.begin(), slopes.end(), 0.0) /
                      static_cast<double>(slopes.size());
  double variance = 0.0;
  for (double slope : slopes) {
    variance += (slope - mean) * (slope - mean);
  }
  variance /= static_cast<double>(slopes.size());
  return variance > config_.curvature_variance_threshold;
}

Eigen::VectorXd UgvPathPlanner::linearFit(const std::vector<double>& x,
                                          const std::vector<double>& y) const {
  Eigen::VectorXd coeffs(2);
  coeffs.setZero();
  if (x.empty() || y.empty() || x.size() != y.size()) {
    return coeffs;
  }
  if (x.size() == 1) {
    coeffs(0) = y.front();
    return coeffs;
  }

  const int n = static_cast<int>(x.size());
  Eigen::MatrixXd design(n, 2);
  Eigen::VectorXd target(n);
  for (int i = 0; i < n; ++i) {
    design(i, 0) = 1.0;
    design(i, 1) = x[static_cast<std::size_t>(i)];
    target(i) = y[static_cast<std::size_t>(i)];
  }
  return design.colPivHouseholderQr().solve(target);
}

double UgvPathPlanner::linearPredict(const Eigen::VectorXd& coeffs,
                                     double x) const {
  if (coeffs.size() < 2) {
    return 0.0;
  }
  return coeffs(0) + coeffs(1) * x;
}

Eigen::VectorXd UgvPathPlanner::polynomialFit(const std::vector<double>& x,
                                              const std::vector<double>& y,
                                              int degree) const {
  if (x.empty() || y.empty() || x.size() != y.size()) {
    Eigen::VectorXd coeffs(1);
    coeffs.setZero();
    return coeffs;
  }

  const int n = static_cast<int>(x.size());
  const int clamped_degree = std::max(0, std::min(degree, n - 1));
  const int terms = clamped_degree + 1;
  Eigen::MatrixXd design(n, terms);
  Eigen::VectorXd target(n);
  for (int i = 0; i < n; ++i) {
    double power = 1.0;
    for (int j = 0; j < terms; ++j) {
      design(i, j) = power;
      power *= x[static_cast<std::size_t>(i)];
    }
    target(i) = y[static_cast<std::size_t>(i)];
  }
  return design.colPivHouseholderQr().solve(target);
}

double UgvPathPlanner::polynomialPredict(const Eigen::VectorXd& coeffs,
                                         double x) const {
  double result = 0.0;
  double power = 1.0;
  for (int i = 0; i < coeffs.size(); ++i) {
    result += coeffs(i) * power;
    power *= x;
  }
  return result;
}

std::vector<std::pair<double, double>> UgvPathPlanner::averageBoundaries(
    const BoundaryResult& boundaries) const {
  std::vector<std::pair<double, double>> centerline;
  const std::size_t count = std::min(boundaries.left.size(), boundaries.right.size());
  centerline.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    centerline.emplace_back((boundaries.left[i][0] + boundaries.right[i][0]) * 0.5,
                            (boundaries.left[i][1] + boundaries.right[i][1]) * 0.5);
  }
  return centerline;
}

nav_msgs::Path UgvPathPlanner::buildUgvPath(
    const std::vector<std::pair<double, double>>& points,
    const std_msgs::Header& header) const {
  nav_msgs::Path path;
  path.header = header;
  if (!config_.output_frame.empty()) {
    path.header.frame_id = config_.output_frame;
  }

  if (points.size() < 3) {
    return path;
  }

  const double adjusted_yaw = tunnel_yaw_ * config_.yaw_shrink_factor;
  const double adjusted_roll = imu_roll_ * config_.roll_shrink_factor;
  const int total = static_cast<int>(points.size());
  const int transition_points =
      std::max(1, std::min(total, static_cast<int>(
                                      total / (1.0 + std::exp(-std::abs(adjusted_yaw))))));

  path.poses.reserve(points.size());
  for (int i = 0; i < total; ++i) {
    const double factor =
        i < transition_points ? 1.0 - static_cast<double>(i) / transition_points : 0.0;
    tf2::Quaternion q;
    q.setRPY(-(adjusted_roll * factor), 0.0, adjusted_yaw * factor);

    const auto& point = points[static_cast<std::size_t>(i)];
    tf2::Vector3 local_point(point.first, point.second,
                             ground_slope_ * point.first + ground_intercept_);
    const tf2::Vector3 global_point = tf2::quatRotate(q, local_point);

    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = global_point.x();
    pose.pose.position.y = global_point.y();
    pose.pose.position.z = global_point.z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    path.poses.push_back(pose);
  }

  return path;
}

nav_msgs::Path UgvPathPlanner::buildUavReferencePath(
    const std::vector<std::pair<double, double>>& points,
    const std_msgs::Header& header) const {
  nav_msgs::Path path;
  path.header = header;
  if (!config_.uav_reference_frame.empty()) {
    path.header.frame_id = config_.uav_reference_frame;
  }

  tf2::Quaternion q;
  q.setRPY(-imu_roll_, imu_pitch_, imu_yaw_);
  path.poses.reserve(points.size());

  for (const auto& point : points) {
    tf2::Vector3 local_point(point.first, point.second,
                             ground_slope_ * point.first + ground_intercept_ +
                                 config_.uav_reference_height);
    const tf2::Vector3 global_point = tf2::quatRotate(q, local_point);

    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = global_point.x();
    pose.pose.position.y = global_point.y();
    pose.pose.position.z = global_point.z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    path.poses.push_back(pose);
  }

  return path;
}

void UgvPathPlanner::correctBayesianOutliers(nav_msgs::Path& path) const {
  if (path.poses.size() < 5 || config_.outlier_mode == "off") {
    return;
  }

  const double prior_outlier =
      std::clamp(config_.outlier_prior, 1.0e-6, 1.0 - 1.0e-6);
  const double prior_normal = 1.0 - prior_outlier;

  for (std::size_t i = 2; i + 2 < path.poses.size(); ++i) {
    const auto& p_im2 = path.poses[i - 2].pose.position;
    const auto& p_im1 = path.poses[i - 1].pose.position;
    const auto& p_i = path.poses[i].pose.position;
    const auto& p_ip1 = path.poses[i + 1].pose.position;
    const auto& p_ip2 = path.poses[i + 2].pose.position;

    const double prev_denom = p_im1.x - p_im2.x;
    const double next_denom = p_ip1.x - p_ip2.x;
    if (std::abs(prev_denom) < 1.0e-9 || std::abs(next_denom) < 1.0e-9) {
      continue;
    }

    const double pred_prev =
        p_im1.y + (p_im1.y - p_im2.y) * (p_i.x - p_im1.x) / prev_denom;
    const double pred_next =
        p_ip1.y + (p_ip1.y - p_ip2.y) * (p_i.x - p_ip1.x) / next_denom;
    const double predicted_y = 0.5 * (pred_prev + pred_next);
    const double error = p_i.y - predicted_y;

    if (config_.outlier_mode == "legacy_smoothing") {
      path.poses[i].pose.position.y = predicted_y;
      continue;
    }

    const double normal_likelihood =
        gaussianLikelihood(error, config_.normal_noise_std);
    const double outlier_likelihood =
        gaussianLikelihood(error, config_.outlier_noise_std);
    const double denominator = normal_likelihood * prior_normal +
                               outlier_likelihood * prior_outlier;
    if (denominator > 0.0) {
      const double posterior =
          outlier_likelihood * prior_outlier / denominator;
      if (posterior > config_.outlier_probability_threshold) {
        path.poses[i].pose.position.y = predicted_y;
      }
    }
  }
}

std::vector<UgvPathPlanner::ObstacleSegment>
UgvPathPlanner::detectObstacleSegments(const nav_msgs::Path& path,
                                       const PointCloud::ConstPtr& cloud,
                                       std::size_t max_detection_index) const {
  std::vector<ObstacleSegment> obstacles;
  if (!cloud || cloud->empty() || path.poses.size() < 2 ||
      max_detection_index == 0) {
    return obstacles;
  }

  const std::size_t last =
      std::min<std::size_t>(path.poses.size() - 1, max_detection_index);
  for (std::size_t i = 1; i < last; ++i) {
    const auto& p1 = path.poses[i].pose.position;
    const auto& p2 = path.poses[i + 1].pose.position;
    const double dx = p2.x - p1.x;
    const double dy = p2.y - p1.y;
    const double segment_length = std::sqrt(dx * dx + dy * dy);
    if (segment_length < 1.0e-6) {
      continue;
    }

    geometry_msgs::Point center;
    center.x = 0.5 * (p1.x + p2.x);
    center.y = 0.5 * (p1.y + p2.y);
    center.z = p1.z + config_.obstacle_box_z_offset;

    PointCloud::Ptr filtered(new PointCloud);
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(cloud);
    crop_box.setMin(Eigen::Vector4f(-segment_length * 0.5,
                                    -config_.obstacle_box_half_width,
                                    config_.obstacle_box_min_z, 1.0F));
    crop_box.setMax(Eigen::Vector4f(segment_length * 0.5,
                                    config_.obstacle_box_half_width,
                                    config_.obstacle_box_max_z, 1.0F));
    crop_box.setRotation(Eigen::Vector3f(0.0F, 0.0F,
                                         static_cast<float>(std::atan2(dy, dx))));
    crop_box.setTranslation(Eigen::Vector3f(static_cast<float>(center.x),
                                            static_cast<float>(center.y),
                                            static_cast<float>(center.z)));
    crop_box.filter(*filtered);

    if (!filtered->empty()) {
      ObstacleSegment segment;
      segment.index = i;
      segment.obstacle_y_values.reserve(filtered->size());
      for (const auto& point : filtered->points) {
        if (std::isfinite(point.y)) {
          segment.obstacle_y_values.push_back(point.y);
        }
      }
      if (!segment.obstacle_y_values.empty()) {
        obstacles.push_back(std::move(segment));
      }
    }
  }

  return obstacles;
}

void UgvPathPlanner::applyFireflyAvoidance(
    nav_msgs::Path& path,
    const std::vector<ObstacleSegment>& obstacles,
    std::size_t max_detection_index) {
  if (obstacles.empty() || tunnel_diameter_ <= 0.0) {
    return;
  }

  for (const auto& obstacle : obstacles) {
    if (obstacle.index >= path.poses.size()) {
      continue;
    }

    auto& pose = path.poses[obstacle.index];
    const double old_y = pose.pose.position.y;
    const double new_y = runFireflySearch(old_y, obstacle.obstacle_y_values);
    if (std::abs(new_y - old_y) < 1.0e-6) {
      continue;
    }

    pose.pose.position.y = new_y;
    pose.pose.position.z += crossSectionRise(new_y) - crossSectionRise(old_y);
    smoothLocalAvoidance(path, obstacle.index, new_y, max_detection_index);
  }

  smoothAvoidancePath(path, max_detection_index);
}

double UgvPathPlanner::runFireflySearch(
    double original_y,
    const std::vector<double>& obstacle_y_values) {
  const double radius = 0.5 * tunnel_diameter_;
  const double y_min = -radius + config_.vehicle_width * 0.5 + config_.safety_margin;
  const double y_max = radius - config_.vehicle_width * 0.5 - config_.safety_margin;
  if (radius <= 0.0 || y_min >= y_max || config_.firefly_count <= 0 ||
      config_.firefly_iterations <= 0 || obstacle_y_values.empty()) {
    return original_y;
  }

  const int count = config_.firefly_count;
  std::uniform_real_distribution<double> unit_noise(-0.5, 0.5);
  std::vector<double> fireflies;
  std::vector<double> intensities(static_cast<std::size_t>(count), 0.0);
  fireflies.reserve(static_cast<std::size_t>(count));

  for (int i = 0; i < count; ++i) {
    const double ratio = count == 1 ? 0.5 : static_cast<double>(i) / (count - 1);
    double y = y_min + (y_max - y_min) * ratio;
    y += 0.1 * (y_max - y_min) * unit_noise(rng_);
    y = std::clamp(y, y_min, y_max);
    if (std::abs(y) / radius > config_.max_tilt_ratio) {
      y = std::copysign(config_.max_tilt_ratio * radius, y);
    }
    fireflies.push_back(y);
  }

  const auto brightness = [&](double y) {
    double min_obstacle_distance = std::numeric_limits<double>::max();
    for (double obstacle_y : obstacle_y_values) {
      double clearance = std::abs(y - obstacle_y) -
                         (config_.vehicle_width * 0.5 + config_.safety_margin);
      if (clearance < 0.0) {
        clearance *= config_.collision_penalty_multiplier;
      }
      min_obstacle_distance = std::min(min_obstacle_distance, clearance);
    }

    const double tilt = std::abs(y) / radius;
    const double dynamic_tilt_weight =
        config_.tilt_weight * (1.0 + tilt * tilt);
    const double tilt_penalty = dynamic_tilt_weight * tilt * tilt;
    const double center_penalty = config_.center_weight * std::abs(y);
    return min_obstacle_distance - tilt_penalty - center_penalty;
  };

  double alpha = config_.firefly_alpha;
  for (int iter = 0; iter < config_.firefly_iterations; ++iter) {
    for (int i = 0; i < count; ++i) {
      intensities[static_cast<std::size_t>(i)] =
          brightness(fireflies[static_cast<std::size_t>(i)]);
    }

    std::vector<double> next = fireflies;
    for (int i = 0; i < count; ++i) {
      for (int j = 0; j < count; ++j) {
        if (intensities[static_cast<std::size_t>(j)] <=
            intensities[static_cast<std::size_t>(i)]) {
          continue;
        }
        const double r = std::abs(fireflies[static_cast<std::size_t>(i)] -
                                  fireflies[static_cast<std::size_t>(j)]);
        const double beta = config_.firefly_beta0 *
                            std::exp(-config_.firefly_gamma * r * r);
        next[static_cast<std::size_t>(i)] +=
            beta * (fireflies[static_cast<std::size_t>(j)] -
                    fireflies[static_cast<std::size_t>(i)]);
      }

      next[static_cast<std::size_t>(i)] +=
          alpha * unit_noise(rng_) * (y_max - y_min);
      next[static_cast<std::size_t>(i)] =
          std::clamp(next[static_cast<std::size_t>(i)], y_min, y_max);
      if (std::abs(next[static_cast<std::size_t>(i)]) / radius >
          config_.max_tilt_ratio) {
        next[static_cast<std::size_t>(i)] =
            std::copysign(config_.max_tilt_ratio * radius,
                          next[static_cast<std::size_t>(i)]);
      }
    }
    fireflies = std::move(next);
    alpha *= config_.firefly_alpha_decay;
  }

  int best_index = 0;
  double best_brightness = brightness(fireflies.front());
  for (int i = 1; i < count; ++i) {
    const double value = brightness(fireflies[static_cast<std::size_t>(i)]);
    if (value > best_brightness) {
      best_brightness = value;
      best_index = i;
    }
  }

  return fireflies[static_cast<std::size_t>(best_index)];
}

void UgvPathPlanner::smoothLocalAvoidance(nav_msgs::Path& path,
                                          std::size_t center_index,
                                          double target_y,
                                          std::size_t max_detection_index) const {
  const int radius = std::max(0, config_.local_smooth_radius);
  for (int offset = 1; offset <= radius; ++offset) {
    const double weight =
        (static_cast<double>(radius - offset) + 1.0) /
        (static_cast<double>(radius) + 1.0);

    const std::size_t forward = center_index + static_cast<std::size_t>(offset);
    if (forward < path.poses.size() && forward < max_detection_index) {
      path.poses[forward].pose.position.y =
          (1.0 - weight) * path.poses[forward].pose.position.y +
          weight * target_y;
    }

    if (center_index >= static_cast<std::size_t>(offset)) {
      const std::size_t backward = center_index - static_cast<std::size_t>(offset);
      path.poses[backward].pose.position.y =
          (1.0 - weight) * path.poses[backward].pose.position.y +
          weight * target_y;
    }
  }
}

void UgvPathPlanner::smoothAvoidancePath(
    nav_msgs::Path& path,
    std::size_t max_detection_index) const {
  if (path.poses.size() < 5 || max_detection_index == 0) {
    return;
  }

  const std::vector<double> sg_coeffs{
      -3.0 / 35.0, 12.0 / 35.0, 17.0 / 35.0, 12.0 / 35.0, -3.0 / 35.0};
  const std::size_t half_window = sg_coeffs.size() / 2;
  const std::size_t transition =
      std::min(config_.smooth_transition_points, path.poses.size());
  const std::size_t smooth_end =
      std::min(path.poses.size(), max_detection_index + transition);
  if (smooth_end <= half_window * 2) {
    return;
  }

  nav_msgs::Path smoothed = path;
  for (std::size_t i = half_window; i + half_window < smooth_end; ++i) {
    double y = 0.0;
    for (int j = -static_cast<int>(half_window);
         j <= static_cast<int>(half_window); ++j) {
      const auto index = static_cast<std::size_t>(
          static_cast<long long>(i) + static_cast<long long>(j));
      y += path.poses[index].pose.position.y *
           sg_coeffs[static_cast<std::size_t>(j + static_cast<int>(half_window))];
    }
    smoothed.poses[i].pose.position.y = y;
  }

  if (transition > 0) {
    for (std::size_t i = max_detection_index; i < smooth_end; ++i) {
      const double blend = 1.0 -
                           static_cast<double>(i - max_detection_index) /
                               static_cast<double>(transition);
      smoothed.poses[i].pose.position.y =
          blend * smoothed.poses[i].pose.position.y +
          (1.0 - blend) * path.poses[i].pose.position.y;
    }
  }

  path = smoothed;
}

double UgvPathPlanner::crossSectionRise(double y) const {
  const double radius = 0.5 * tunnel_diameter_;
  if (radius <= 0.0) {
    return 0.0;
  }
  const double clamped_y = std::clamp(y, -radius, radius);
  return radius - std::sqrt(std::max(0.0, radius * radius - clamped_y * clamped_y));
}

}  // namespace flisp_planner
