cmake_minimum_required(VERSION 3.0.2)
project(flisp_planner)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features filters kdtree search)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf2
    visualization_msgs
  DEPENDS EIGEN3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_library(flisp_planner_core
  src/geometry_utils.cpp
  src/uav_path_planner.cpp
  src/ugv_path_planner.cpp
)
target_link_libraries(flisp_planner_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(flisp_planner_core ${catkin_EXPORTED_TARGETS})

add_executable(flisp_ugv_planner_node src/ugv_path_planner_node.cpp)
add_dependencies(flisp_ugv_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(flisp_ugv_planner_node
  flisp_planner_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(flisp_uav_planner_node src/uav_path_planner_node.cpp)
add_dependencies(flisp_uav_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(flisp_uav_planner_node
  flisp_planner_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS flisp_planner_core flisp_ugv_planner_node flisp_uav_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
