#ifndef FLISP_PLANNER_UGV_PATH_PLANNER_H
#define FLISP_PLANNER_UGV_PATH_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace flisp_planner {

using Point2 = std::array<double, 2>;

struct UgvPlannerConfig {
  std::string point_cloud_topic{"/ouster/points"};
  std::string imu_topic{"/ouster/imu"};
  std::string ugv_path_topic{"/path"};
  std::string uav_reference_path_topic{"/path_ugv_simplist"};
  std::string runtime_topic{"/runtime_ugv"};
  std::string output_frame{"os_sensor"};
  std::string uav_reference_frame{"UGV/odom"};

  double yaw_crop_min_x{-2.0};
  double yaw_crop_max_x{2.0};
  double yaw_crop_min_y{-100.0};
  double yaw_crop_max_y{100.0};
  double yaw_crop_min_z{2.0};
  double yaw_crop_max_z{9.0};
  int normal_neighbors{15};
  int normal_samples{30};

  double bin_c1{3.0};
  double bin_c2{1.5};
  double bin_kappa{1.0};
  double bin_lambda{0.0};
  double bin_c3{0.5};
  double bin_mu{0.2};
  double min_bin_step{0.3};
  double max_bin_step{3.0};
  std::string bin_step_model{"legacy_inverse"};
  double preview_distance{80.0};
  double edge_jump_threshold{1.5};
  double curvature_variance_threshold{0.001};
  int polynomial_degree{2};
  std::size_t linear_extension_points{200};
  std::size_t polynomial_extension_points{50};

  double ground_fit_min_x{-5.0};
  double ground_fit_max_x{-3.0};
  double ground_fit_step{0.5};
  double uav_reference_height{5.0};

  double yaw_shrink_factor{0.3};
  double roll_shrink_factor{0.1};
  double outlier_prior{0.1};
  double normal_noise_std{0.05};
  double outlier_noise_std{0.5};
  double outlier_probability_threshold{0.75};
  std::string outlier_mode{"legacy_smoothing"};
  double collinear_distance_threshold{0.05};

  bool enable_obstacle_avoidance{false};
  double obstacle_detection_fraction{0.4};
  double obstacle_box_half_width{0.5};
  double obstacle_box_z_offset{0.8};
  double obstacle_box_min_z{0.0};
  double obstacle_box_max_z{0.5};
  double vehicle_width{0.6};
  double safety_margin{0.3};
  double max_tilt_ratio{0.35};
  int firefly_count{50};
  int firefly_iterations{80};
  double firefly_alpha{0.5};
  double firefly_beta0{2.8};
  double firefly_gamma{1.5};
  double firefly_alpha_decay{0.95};
  double tilt_weight{2.0};
  double center_weight{0.5};
  double collision_penalty_multiplier{5.0};
  int local_smooth_radius{2};
  std::size_t smooth_transition_points{10};
  unsigned int random_seed{7};
};

class UgvPathPlanner {
 public:
  UgvPathPlanner(ros::NodeHandle nh, ros::NodeHandle private_nh);

 private:
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

  struct BoundaryResult {
    std::vector<Point2> left;
    std::vector<Point2> right;
  };

  struct ObstacleSegment {
    std::size_t index{};
    std::vector<double> obstacle_y_values;
  };

  void loadConfig(const ros::NodeHandle& private_nh);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  std::pair<double, double> estimateTunnelYawAndDiameter(
      const PointCloud::ConstPtr& cloud) const;
  std::pair<double, double> fitGroundLine(const PointCloud::ConstPtr& cloud) const;
  std::vector<std::pair<double, double>> projectToXY(
      const PointCloud::ConstPtr& cloud) const;

  double dynamicBinStep(double yaw) const;
  BoundaryResult extractBoundaryPoints(
      const std::vector<std::pair<double, double>>& points_2d,
      double step_size) const;
  void extendBoundary(std::vector<Point2>& edge,
                      double step_size,
                      std::size_t max_points) const;
  bool isCurved(const std::vector<Point2>& edge) const;
  Eigen::VectorXd linearFit(const std::vector<double>& x,
                            const std::vector<double>& y) const;
  double linearPredict(const Eigen::VectorXd& coeffs, double x) const;
  Eigen::VectorXd polynomialFit(const std::vector<double>& x,
                                const std::vector<double>& y,
                                int degree) const;
  double polynomialPredict(const Eigen::VectorXd& coeffs, double x) const;
  std::vector<std::pair<double, double>> averageBoundaries(
      const BoundaryResult& boundaries) const;

  nav_msgs::Path buildUgvPath(const std::vector<std::pair<double, double>>& points,
                              const std_msgs::Header& header) const;
  nav_msgs::Path buildUavReferencePath(
      const std::vector<std::pair<double, double>>& points,
      const std_msgs::Header& header) const;
  void correctBayesianOutliers(nav_msgs::Path& path) const;

  std::vector<ObstacleSegment> detectObstacleSegments(
      const nav_msgs::Path& path,
      const PointCloud::ConstPtr& cloud,
      std::size_t max_detection_index) const;
  void applyFireflyAvoidance(nav_msgs::Path& path,
                             const std::vector<ObstacleSegment>& obstacles,
                             std::size_t max_detection_index);
  double runFireflySearch(double original_y,
                          const std::vector<double>& obstacle_y_values);
  void smoothLocalAvoidance(nav_msgs::Path& path,
                            std::size_t center_index,
                            double target_y,
                            std::size_t max_detection_index) const;
  void smoothAvoidancePath(nav_msgs::Path& path,
                           std::size_t max_detection_index) const;
  double crossSectionRise(double y) const;

  UgvPlannerConfig config_;
  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher ugv_path_pub_;
  ros::Publisher uav_reference_path_pub_;
  ros::Publisher runtime_pub_;

  double imu_roll_{0.0};
  double imu_pitch_{0.0};
  double imu_yaw_{0.0};
  double tunnel_yaw_{0.0};
  double tunnel_diameter_{0.0};
  double ground_slope_{0.0};
  double ground_intercept_{0.0};

  std::mt19937 rng_;
};

}  // namespace flisp_planner

#endif  // FLISP_PLANNER_UGV_PATH_PLANNER_H
