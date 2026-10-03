#ifndef FLISP_PLANNER_UAV_PATH_PLANNER_H
#define FLISP_PLANNER_UAV_PATH_PLANNER_H

#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>

#include <memory>
#include <cstddef>
#include <string>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace flisp_planner {

struct UavPlannerConfig {
  std::string ugv_path_topic{"/path"};
  std::string reference_path_topic{"/path_ugv_simplist"};
  std::string point_cloud_topic{"/ouster/points"};
  std::string output_path_topic{"/path_uav"};
  std::string runtime_topic{"/runtime_uav"};
  std::string output_frame{""};

  double communication_radius{2.5};
  double safe_radius{1.5};
  double min_altitude_offset{2.0};
  double reference_extra_altitude_offset{2.0};

  double cloud_crop_margin_multiplier{2.0};
  double path_crop_margin{0.0};
  double voxel_leaf_size_near_path{0.1};
  double voxel_leaf_size_global{0.2};
  int downsample_threshold{1000};

  bool enable_obstacle_avoidance{false};
  int sample_count{24};
  int optimization_iterations{5};
  double sampling_radius_decay{0.7};
  double safety_weight{2.5};
  double smoothness_weight{1.2};
  double progress_weight{0.8};
  double height_weight{0.5};
  double blocked_segment_penalty{25.0};

  bool enable_post_smoothing{false};
  double simplify_epsilon{0.05};
  int smooth_window_size{5};
};

class UavPathPlanner {
 public:
  UavPathPlanner(ros::NodeHandle nh, ros::NodeHandle private_nh);

 private:
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

  struct ObstacleSegment {
    std::size_t index{};
  };

  void loadConfig(const ros::NodeHandle& private_nh);
  void ugvPathCallback(const nav_msgs::Path::ConstPtr& msg);
  void referencePathCallback(const nav_msgs::Path::ConstPtr& msg);
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void processPaths();

  nav_msgs::Path buildInitialPath() const;
  std::vector<ObstacleSegment> collectObstacleSegments(
      const nav_msgs::Path& path) const;
  bool hasObstacleInCylinder(const geometry_msgs::Point& start,
                             const geometry_msgs::Point& end) const;
  double minDistanceToSegmentCloud(const geometry_msgs::Point& start,
                                   const geometry_msgs::Point& end) const;
  geometry_msgs::Point optimizeWaypoint(const geometry_msgs::Point& start,
                                        const geometry_msgs::Point& original,
                                        const geometry_msgs::Point& anchor);
  double candidateCost(const geometry_msgs::Point& candidate,
                       const geometry_msgs::Point& start,
                       const geometry_msgs::Point& original) const;
  geometry_msgs::Point communicationAnchor(std::size_t index) const;

  UavPlannerConfig config_;
  ros::NodeHandle nh_;
  ros::Subscriber ugv_path_sub_;
  ros::Subscriber reference_path_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher output_path_pub_;
  ros::Publisher runtime_pub_;

  nav_msgs::Path ugv_path_;
  nav_msgs::Path reference_path_;
  bool have_ugv_path_{false};
  bool have_reference_path_{false};

  PointCloud::Ptr latest_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
};

}  // namespace flisp_planner

#endif  // FLISP_PLANNER_UAV_PATH_PLANNER_H
