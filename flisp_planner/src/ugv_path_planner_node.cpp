#include "flisp_planner/ugv_path_planner.h"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "flisp_ugv_planner");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  flisp_planner::UgvPathPlanner planner(nh, private_nh);
  ros::spin();
  return 0;
}
